/*
 * Copyright (C) 2026 NAVRobotec Pvt Ltd
 * Author: Ragnar Vallhala
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
// Changing the world in the Simulator must leave the vehicle ON the new ground.
//
// The World tab regenerates the terrain (preset / biome / seed ...) but the
// airframe used to stay where the OLD world had put it, so on a world with
// higher ground at the home pad it ended up inside the floor. This drives the
// real SimulatorWidget the way the World tab does (WorldEditorWidget::
// setConfig + worldMeshChanged) and asserts, for each change, that the
// vehicle the 3D view shows is at or above the new terrain height at its own
// position, level and at rest -- sim Stopped, and sim Running against the
// real engine module.
//
// Needs a real GL platform plugin (the widget owns GL views): run with
// QT_QPA_PLATFORM=xcb on an Xvfb and LIBGL_ALWAYS_SOFTWARE=1. On "offscreen"
// (ctest's default) it skips. The Running part also needs the engine module
// (VAYU_SITL_MODULE) and skips, saying why, without one. Give it private
// VAYU_VFS_DIR / VSIM_FIFO_SUFFIX / XDG_* when running it. Wall-clock paced.

#include "SimRendererWidget.h"
#include "SimulatorWidget.h"
#include "SitlModule.h"
#include "ProceduralWorld.h"
#include "WorldEditorWidget.h"

#include <QApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QGuiApplication>
#include <QStandardPaths>
#include <QTemporaryDir>
#include <QtTest>

#include <cmath>
#include <functional>

// Test seam: SimulatorWidget befriends this class so the test can reach the
// world editor, the terrain sampler and the 3D view without widening the
// widget's public API.
class SimulatorWidgetTestAccess {
public:
  explicit SimulatorWidgetTestAccess(SimulatorWidget &w) : w_(w) {}
  WorldEditorWidget *editor() const { return w_.m_worldEditor; }
  // Where the 3D view draws the vehicle (what the user sees and the camera
  // follows): the last snapshot while running, the parked pose when stopped.
  QVector3D shownPos() const { return w_.m_renderer->vehiclePos(); }
  // The camera: the free-roam eye when stopped in World mode, else the vehicle.
  QVector3D cameraPos() const { return w_.m_renderer->streamCenter(); }
  bool hasGround() const { return bool(w_.m_terrainHeightAt); }
  float groundAt(float x, float y) const { return w_.m_terrainHeightAt(x, y); }
  QVector3D lastPos() const { return w_.m_lastDronePos; }
  bool running() const { return w_.m_sim != nullptr; }
  bool shipBusy() const {
    return w_.m_shipWatcher || w_.m_shipPending || w_.m_collisionPending ||
           w_.m_spawnResetPending;
  }
  void worldMode() { w_.setMode(1); } // the World tab: free-roam camera
  // The deck the vehicle should rest on at the home pad, from the live terrain
  // sampler (NOT from the widget's cached pad list), and what the widget
  // itself thinks the landing surface there is.
  float trueDeckZ() const {
    const float g = w_.m_terrainHeightAt ? w_.m_terrainHeightAt(0.0f, 0.0f)
                                         : 0.0f;
    return -g - (w_.m_terrainHeightAt ? vsim::SimRendererWidget::kHelipadDeckM
                                      : 0.0f);
  }
  float widgetPadZ() const {
    return w_.landingSurfaceZ(w_.m_homePad.x(), w_.m_homePad.y());
  }
  float homePadTerrain() const { return w_.m_homePad.z(); }
  void persist() { w_.persistWorld(w_.m_worldEditor->config()); }
  void start() { w_.startInAppSim(); }
  void stop() { w_.stopInAppSim(); }
  // Change the world the way the World tab does.
  void changeWorld(const std::function<void(vsim::WorldConfig &)> &edit) {
    vsim::WorldConfig c = editor()->config();
    edit(c);
    editor()->setConfig(c);
    emit editor()->worldMeshChanged();
  }

private:
  SimulatorWidget &w_;
};

namespace {

void pump(int ms) {
  QElapsedTimer t;
  t.start();
  while (t.elapsed() < ms) {
    QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    QThread::msleep(5);
  }
}

// Wait (bounded) until `cond` holds; true if it did.
bool waitFor(const std::function<bool()> &cond, int ms) {
  QElapsedTimer t;
  t.start();
  while (t.elapsed() < ms) {
    if (cond())
      return true;
    pump(50);
  }
  return cond();
}

struct Change {
  const char *name;
  std::function<void(vsim::WorldConfig &)> edit;
};

// Metres above the new ground at the vehicle's own position (NED: up = -z).
float altAbove(const SimulatorWidgetTestAccess &a) {
  const QVector3D p = a.shownPos();
  const float g = a.hasGround() ? a.groundAt(p.x(), p.y()) : 0.0f;
  return -p.z() - g;
}

// Metres above the home pad's deck (up = -z), the deck taken from the live
// terrain sampler rather than from the widget's own pad bookkeeping.
float aboveDeck(const SimulatorWidgetTestAccess &a) {
  return a.trueDeckZ() - a.shownPos().z();
}

// Metres above the new ground at the camera's own position.
float camAbove(const SimulatorWidgetTestAccess &a) {
  const QVector3D p = a.cameraPos();
  const float g = a.hasGround() ? a.groundAt(p.x(), p.y()) : 0.0f;
  return -p.z() - g;
}

QVector<Change> changes() {
  auto preset = [](const char *id, quint32 seed) {
    return [id, seed](vsim::WorldConfig &c) {
      QVERIFY2(vsim::applyWorldPreset(c, QString::fromLatin1(id), seed), id);
    };
  };
  // Preset arenas whose ground at the origin is well above z=0, then a plain
  // seed change within one preset, then the finite meadow, then no world.
  return {
      {"preset alpine seed 3", preset("alpine", 3)},
      {"preset hills seed 11", preset("hills", 11)},
      {"preset hills seed 12 (seed only)", preset("hills", 12)},
      {"preset desert seed 5", preset("desert", 5)},
      {"preset alpine seed 99", preset("alpine", 99)},
      {"meadow seed 21",
       [](vsim::WorldConfig &c) {
         c.worldPreset.clear();
         c.proceduralBiome = QStringLiteral("meadow");
         c.proceduralSeed = 21;
       }},
      {"meadow seed 22 size 300",
       [](vsim::WorldConfig &c) {
         c.proceduralBiome = QStringLiteral("meadow");
         c.proceduralSeed = 22;
         c.proceduralSizeM = 300.0f;
       }},
      {"no world",
       [](vsim::WorldConfig &c) {
         c.worldPreset.clear();
         c.proceduralBiome.clear();
         c.worldMeshPath.clear();
       }},
  };
}

} // namespace

class TstWorldChangeFloor : public QObject {
  Q_OBJECT

  SimulatorWidget *w_ = nullptr;
  SimulatorWidgetTestAccess *a_ = nullptr;
  bool gl_ = false;
  QList<SimulatorWidget *> restored_;

private slots:
  void initTestCase() {
    gl_ = QGuiApplication::platformName() != QLatin1String("offscreen");
    if (!gl_)
      return;
    w_ = new SimulatorWidget();
    a_ = new SimulatorWidgetTestAccess(*w_);
    w_->resize(1200, 800);
    w_->show();
    QVERIFY(QTest::qWaitForWindowExposed(w_));
    a_->worldMode(); // editing the world: stopped, free-roam camera
    pump(500);
  }

  void cleanupTestCase() {
    if (a_ && a_->running())
      a_->stop();
    delete a_;
    delete w_;
    qDeleteAll(restored_);
  }

  // Sim Stopped: every world change parks the vehicle on the new ground.
  void stoppedChangesPlaceVehicleOnGround_data() {
    QTest::addColumn<int>("idx");
    const QVector<Change> cs = changes();
    for (int i = 0; i < cs.size(); ++i)
      QTest::newRow(cs[i].name) << i;
  }
  void stoppedChangesPlaceVehicleOnGround() {
    if (!gl_)
      QSKIP("needs a GL platform plugin (QT_QPA_PLATFORM=xcb on Xvfb)");
    QFETCH(int, idx);
    QVERIFY(!a_->running());
    a_->changeWorld(changes()[idx].edit);
    pump(200);
    const QVector3D p = a_->shownPos();
    const float alt = altAbove(*a_);
    const QVector3D c = a_->cameraPos();
    qInfo("stopped: pos=(%.2f %.2f %.2f) ground=%.2f alt=%.2f camera alt=%.2f",
          p.x(), p.y(), p.z(),
          a_->hasGround() ? a_->groundAt(p.x(), p.y()) : 0.0f, alt,
          camAbove(*a_));
    (void)c;
    QVERIFY2(alt >= 0.0f, "vehicle is below the new ground");
    QVERIFY2(camAbove(*a_) >= 0.0f, "camera is below the new ground");
    QVERIFY2(alt < 3.0f, "vehicle is not parked at the home pad");
  }

  // A launch with a saved world: the widget is built from QSettings (the
  // restore path) and the vehicle must be parked ON the home pad's deck -- not
  // at the terrain under it, which is inside the deck (the deck stands
  // kHelipadDeckM above the ground). Stopped only: the engine module is
  // one-shot per process, so the Running half is in runningChanges... below.
  void restoredWorldParksOnPad_data() {
    QTest::addColumn<int>("which");
    QTest::newRow("no terrain") << 0;
    QTest::newRow("alpine arena") << 1;
    QTest::newRow("endless meadow") << 2;
  }
  void restoredWorldParksOnPad() {
    if (!gl_)
      QSKIP("needs a GL platform plugin (QT_QPA_PLATFORM=xcb on Xvfb)");
    QFETCH(int, which);
    // Save the world through the first widget (it persists like the World tab).
    a_->changeWorld([which](vsim::WorldConfig &c) {
      if (which == 0) {
        c.worldPreset.clear();
        c.proceduralBiome.clear();
        c.worldMeshPath.clear();
      } else if (which == 1) {
        QVERIFY(vsim::applyWorldPreset(c, QStringLiteral("alpine"), 3));
      } else {
        c.worldPreset.clear();
        c.proceduralBiome = QStringLiteral("endless");
        c.proceduralSeed = 7;
      }
    });
    a_->persist();
    pump(300);

    // A second widget restores it, as the next app launch does. Kept alive to
    // the end: destroying a widget shuts the one-shot engine module down.
    auto *b = new SimulatorWidget();
    restored_.push_back(b);
    SimulatorWidgetTestAccess ab(*b);
    b->resize(1200, 800);
    b->show();
    QVERIFY(QTest::qWaitForWindowExposed(b));
    pump(500);
    const float above = aboveDeck(ab);
    qInfo("restored (%s): parked pos.z=%.3f deck.z=%.3f above deck=%.3f",
          QTest::currentDataTag(), ab.shownPos().z(), ab.trueDeckZ(), above);
    QVERIFY2(above >= 0.0f, "parked vehicle is inside the helipad");
    QVERIFY2(above < 1.0f, "parked vehicle is not on the home pad");
  }

  // Sim Running against the real engine: after each change (and the collision
  // build it triggers) the vehicle ends up on the new ground, at rest.
  void runningChangesPlaceVehicleOnGround() {
    if (!gl_)
      QSKIP("needs a GL platform plugin (QT_QPA_PLATFORM=xcb on Xvfb)");
    if (!SitlModule::instance().available())
      QSKIP("no engine module (set VAYU_SITL_MODULE)");
    a_->changeWorld(changes()[0].edit); // start on a raised world
    a_->start();
    QVERIFY(a_->running());
    QVERIFY(waitFor([&] { return !a_->shipBusy(); }, 20000));
    pump(3000); // settle on the pad
    qInfo("running: initial alt=%.2f above deck=%.2f", altAbove(*a_),
          aboveDeck(*a_));
    QVERIFY(altAbove(*a_) >= -0.05f);
    // On the pad deck, not sunk through it to the terrain under it.
    QVERIFY2(aboveDeck(*a_) >= -0.05f, "sank into the helipad after Start");

    const QVector<Change> cs = changes();
    for (int i = 1; i < cs.size(); ++i) {
      a_->changeWorld(cs[i].edit);
      QVERIFY2(waitFor([&] { return !a_->shipBusy(); }, 20000), cs[i].name);
      pump(3000);
      const QVector3D p = a_->shownPos();
      const float alt = altAbove(*a_);
      qInfo("running: %s pos=(%.2f %.2f %.2f) alt=%.2f", cs[i].name, p.x(),
            p.y(), p.z(), alt);
      QVERIFY2(alt >= -0.05f,
               qPrintable(QStringLiteral("below ground after '%1'")
                              .arg(QString::fromLatin1(cs[i].name))));
      QVERIFY2(aboveDeck(*a_) >= -0.05f,
               qPrintable(QStringLiteral("inside the helipad after '%1'")
                              .arg(QString::fromLatin1(cs[i].name))));
      // Respawned at the home pad (like Reset), not left hanging at the old
      // world's altitude.
      QVERIFY2(alt < 3.0f,
               qPrintable(QStringLiteral("not re-placed after '%1'")
                              .arg(QString::fromLatin1(cs[i].name))));
    }
  }

  // Stop, then change the world again with the sim stopped.
  void stoppedAfterRunPlacesVehicleOnGround() {
    if (!gl_)
      QSKIP("needs a GL platform plugin (QT_QPA_PLATFORM=xcb on Xvfb)");
    if (!a_->running())
      QSKIP("the Running test did not start the engine");
    a_->stop();
    QVERIFY(!a_->running());
    pump(500); // let the worker's last queued pose snapshots drain
    a_->changeWorld(changes()[0].edit);
    pump(200);
    const QVector3D p = a_->shownPos();
    qInfo("stopped after run: pos=(%.2f %.2f %.2f) alt=%.2f", p.x(), p.y(),
          p.z(), altAbove(*a_));
    QVERIFY2(altAbove(*a_) >= 0.0f, "below ground after a world change");
    QVERIFY2(camAbove(*a_) >= 0.0f, "camera below ground after a change");
  }
};

int main(int argc, char **argv) {
  // Private settings: the widget persists its world/geometry via QSettings.
  QTemporaryDir cfg;
  qputenv("XDG_CONFIG_HOME", cfg.path().toLocal8Bit());
  QApplication::setOrganizationName(QStringLiteral("navigator-test"));
  QApplication::setApplicationName(QStringLiteral("tst_world_change_floor"));
  QApplication app(argc, argv);
  TstWorldChangeFloor t;
  return QTest::qExec(&t, argc, argv);
}

#include "tst_world_change_floor.moc"
