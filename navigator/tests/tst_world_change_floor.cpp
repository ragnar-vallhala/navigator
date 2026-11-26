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
// (VAYU_SITL_MODULE) and skips, saying why, without one. Every case is
// self-contained (CI runs each in its own process). Give it private
// VAYU_VFS_DIR / VSIM_FIFO_SUFFIX / XDG_* when running it. Wall-clock paced.

#include <atomic>
#include "SimRendererWidget.h"
#include "SimulatorWidget.h"
#include "SitlModule.h"
#include "ProceduralWorld.h"
#include "procgen/TerrainField.h"
#include "WorldEditorWidget.h"
#include "SimWorker.h"
#include "procgen/TreeColliderWindow.h"

#include <QScopeGuard>
#include <QApplication>
#include <QCheckBox>
#include <QSettings>
#include <QDir>
#include <QFile>
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
  // Drop the streamed world (chunks, flora, caches) but keep the widget.
  void releaseStreamedWorld() { w_.releaseStreamedWorld(); }
  // The camera: the free-roam eye when stopped in World mode, else the vehicle.
  QVector3D cameraPos() const { return w_.m_renderer->streamCenter(); }
  bool hasGround() const { return bool(w_.m_terrainHeightAt); }
  float groundAt(float x, float y) const { return w_.m_terrainHeightAt(x, y); }
  // Where the vehicle is, as the product decides it (lift, stream arrival,
  // collision cover): the engine's own newest pose, not the GUI's queued copy.
  QVector3D lastPos() const { return w_.dronePos(); }
  QVector3D lastVel() const { return w_.droneVel(); }
  // The GUI's copy, which lags a busy GUI thread (what the HUD was last given).
  QVector3D cachedPos() const { return w_.m_lastDronePos; }
  // Streaming state: meshed chunks cached around the vehicle, and the pads.
  size_t chunksCached() const { return w_.m_chunkCache.size(); }
  const std::vector<QVector3D> &pads() const { return w_.m_helipads; }
  // Teleport the running vehicle to (x, y), `up` metres above the terrain
  // sampler there, level and at rest (what a long flight would deliver).
  void teleport(float x, float y, float up) {
    w_.placeVehicle(x, y, -(w_.m_terrainHeightAt(x, y) + up));
  }
  // Place the running vehicle at an explicit NED pose, as the UI does.
  void placeAt(float x, float y, float z) { w_.placeVehicle(x, y, z); }
  // Make every collision build take at least `ms` (a slow runner, on demand).
  void shipDelay(int ms) { w_.m_testShipDelayMs = ms; }
  // Run `f` wherever the collision's pad search runs (it must not be the GUI thread).
  void onPadSearch(std::function<void()> f) { w_.m_testOnPadSearch = std::move(f); }
  // Recentre the collision window on cell (cx, cy), as the streamer does when
  // the vehicle crosses into it (the chunks around it must already be cached).
  void recentre(int cx, int cy) {
    w_.m_collisionPending = true;
    w_.m_colCx = cx;
    w_.m_colCy = cy;
    w_.tryBuildCollision();
  }
  // Chunk meshing: finished by workers so far, and still waiting for one.
  int chunksPosted() const { return w_.m_chunkSink->posted(); }
  size_t chunksQueued() const { return w_.m_chunkQueue->pending(); }
  // The 3D view's chunk bookkeeping and the streamed window's size.
  size_t viewChunks() const { return w_.m_renderer->worldChunkCount(); }
  size_t viewPendingUploads() const {
    return w_.m_renderer->pendingChunkUploadCount();
  }
  size_t viewPendingRemovals() const {
    return w_.m_renderer->pendingChunkRemovalCount();
  }
  size_t windowChunks() const {
    const size_t side = 2 * size_t(w_.m_chunkStreamer.config().renderRadius) + 1;
    return side * side;
  }
  bool shipRunning() const { return w_.m_shipWatcher != nullptr; }
  bool shipQueued() const { return w_.m_shipPending.has_value(); }
  bool collisionPending() const { return w_.m_collisionPending; }
  bool running() const { return w_.m_sim != nullptr; }
  QString logText() const { return w_.m_log->toPlainText(); }
  void clearLog() { w_.m_log->clear(); }
  bool treesShown() const { return w_.m_renderer->vegetationTreesEnabled(); }
  bool gridShown() const { return w_.m_renderer->surveyGrid(); }
  bool downGridShown() const {
    return !w_.m_downRenderer || w_.m_downRenderer->surveyGrid();
  }
  bool downTreesShown() const {
    return !w_.m_downRenderer || w_.m_downRenderer->vegetationTreesEnabled();
  }
  bool shipBusy() const {
    return w_.m_shipWatcher || w_.m_shipPending || w_.m_collisionPending ||
           w_.m_spawnResetPending || w_.m_heldPose.has_value();
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
  vsim::WorldConfig restore() { return w_.restoreWorld(); }
  // Trunk-collider path (updateTreeColliders). A SimWorker that is never
  // started stands in for a running sim: sendTreeColliders only queues, and the
  // test sink sees each window. dropFakeSim must run before the widget dies.
  void fakeSim() { w_.m_sim = new vsim::SimWorker(&w_); }
  void dropFakeSim() {
    delete w_.m_sim;
    w_.m_sim = nullptr;
  }
  // The streamed chunks' trunks, as onChunkMeshed stores them (one chunk).
  void setColliders(std::vector<vsim::procgen::VegCollider> c) {
    w_.m_chunkColliders.clear();
    w_.m_chunkColliders[1] = std::move(c);
    ++w_.m_colliderRev;
  }
  // A pose from the engine, through the widget's own pose handler.
  void pose(float x, float y, float z) {
    vsim::SimSnapshot s;
    s.pos_w = QVector3D(x, y, z);
    w_.updateHud(s);
  }
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
  // Presets whose ground at the origin is well above z=0, then a plain seed
  // change within one preset, then the legacy finite "meadow" of old saves
  // (streams now), then no world.
  return {
      {"preset alpine seed 3", preset("alpine", 3)},
      {"preset hills seed 11", preset("hills", 11)},
      {"preset hills seed 12 (seed only)", preset("hills", 12)},
      {"preset desert seed 5", preset("desert", 5)},
      {"preset alpine seed 99", preset("alpine", 99)},
      {"legacy meadow seed 21",
       [](vsim::WorldConfig &c) {
         c.worldPreset.clear();
         c.proceduralBiome = QStringLiteral("meadow");
         c.proceduralSeed = 21;
       }},
      {"legacy meadow seed 22 size 300",
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

  // Every case starts from the same state: sim stopped, log empty. A case that
  // leaves the sim running (inFlightChangesPlaceVehicleOnGround does) would
  // otherwise hand the next case its "World changed" lines, and
  // runningChangesPlaceVehicleOnGround asserts there are none after Start. CI
  // runs each case in its own process, so only a one-process run showed it.
  void init() {
    if (!gl_)
      return;
    if (a_->running()) {
      a_->stop();
      QVERIFY(waitFor([&] { return !a_->running(); }, 10000));
    }
    a_->clearLog();
  }

  void cleanupTestCase() {
    if (a_ && a_->running())
      a_->stop();
    delete a_;
    delete w_;
    qDeleteAll(restored_);
  }

  // The Sim controls "Trees" checkbox switches trees in the 3D views live and
  // stores the choice; switching back on needs no world rebuild.
  void treesCheckboxTogglesLiveAndPersists() {
    if (!gl_)
      QSKIP("needs a GL platform plugin (QT_QPA_PLATFORM=xcb on Xvfb)");
    QCheckBox *box = nullptr;
    for (QCheckBox *c : w_->findChildren<QCheckBox *>())
      if (c->text() == QLatin1String("Trees"))
        box = c;
    QVERIFY2(box, "no Trees checkbox in the Sim controls");
    if (!box->isEnabled())
      QSKIP("NAV_VEGETATION_TREES is set; the variable beats the checkbox");
    const bool before = box->isChecked();
    QCOMPARE(a_->treesShown(), before);
    box->setChecked(!before);
    QCOMPARE(a_->treesShown(), !before);
    QCOMPARE(a_->downTreesShown(), !before);
    QCOMPARE(QSettings().value("sim/vegetationTrees").toBool(), !before);
    box->setChecked(before);
    QCOMPARE(a_->treesShown(), before);
    QCOMPARE(QSettings().value("sim/vegetationTrees").toBool(), before);
  }

  // Trees checkbox -> trunk colliders. Drives the real checkbox and the real
  // pose handler; the engine side is the SimWorker test sink (what the GUI
  // asked sendTreeColliders to send), so it runs on an SDK without ENV too. The
  // chunk trunks are injected the way onChunkMeshed stores them.
  void treesToggleSendsClearThenRebuild() {
    if (!gl_)
      QSKIP("needs a GL platform plugin (QT_QPA_PLATFORM=xcb on Xvfb)");
    using namespace vsim::procgen;
    QCheckBox *box = nullptr;
    for (QCheckBox *c : w_->findChildren<QCheckBox *>())
      if (c->text() == QLatin1String("Trees"))
        box = c;
    QVERIFY2(box, "no Trees checkbox in the Sim controls");
    if (!box->isEnabled())
      QSKIP("NAV_VEGETATION_TREES is set; the variable beats the checkbox");
    const bool before = box->isChecked();
    struct Sent {
      bool clear;
      std::vector<EngineCylinder> cyl;
    };
    std::vector<Sent> sent;
    vsim::SimWorker::setTreeColliderSinkForTest([&](const ColliderWindow &w) {
      sent.push_back({w.cylinders.empty(), w.cylinders});
    });
    a_->fakeSim();
    auto cleanup = qScopeGuard([&] {
      vsim::SimWorker::setTreeColliderSinkForTest(nullptr);
      a_->dropFakeSim();
      box->setChecked(before);
    });
    if (!box->isChecked())
      box->setChecked(true);
    const auto tree = [](float x, float y) {
      return VegCollider{PgVec3{x, y, 0.0f}, 0.3f, 10.0f, false};
    };
    // Three trunks in the 150 m window and one far outside it.
    a_->setColliders({tree(10, 0), tree(40, 5), tree(-30, 20), tree(400, 0)});
    sent.clear();

    a_->pose(0, 0, -5); // Trees on, trunks in the window: a set goes out
    QCOMPARE(int(sent.size()), 1);
    QVERIFY(!sent[0].clear);
    QCOMPARE(int(sent[0].cyl.size()), 3);
    const std::vector<EngineCylinder> first = sent[0].cyl;
    a_->pose(5, 0, -5); // inside the hysteresis: nothing
    QCOMPARE(int(sent.size()), 1);
    a_->pose(30, 0, -5); // moved more than a quarter of the margin: rebuild
    QCOMPARE(int(sent.size()), 2);
    a_->pose(0, 0, -5); // and back: the window is the first one again
    QCOMPARE(int(sent.size()), 3);
    QVERIFY(sent[2].cyl == first);

    sent.clear();
    box->setChecked(false); // the real toggle: exactly one clear
    QCOMPARE(int(sent.size()), 1);
    QVERIFY(sent[0].clear);
    for (float x : {0.0f, 30.0f, 200.0f, 0.0f, 90.0f}) // moving sends nothing
      a_->pose(x, 0, -5);
    QCOMPARE(int(sent.size()), 1);

    box->setChecked(true); // back on: the rebuild is the same set again
    QCOMPARE(int(sent.size()), 2);
    QVERIFY(!sent[1].clear);
    QVERIFY(sent[1].cyl == first);
  }

  // A different world (seed) forces a rebuild even with the drone standing still.
  void worldChangeRebuildsTheColliders() {
    if (!gl_)
      QSKIP("needs a GL platform plugin (QT_QPA_PLATFORM=xcb on Xvfb)");
    using namespace vsim::procgen;
    QCheckBox *box = nullptr;
    for (QCheckBox *c : w_->findChildren<QCheckBox *>())
      if (c->text() == QLatin1String("Trees"))
        box = c;
    QVERIFY2(box, "no Trees checkbox in the Sim controls");
    if (!box->isEnabled())
      QSKIP("NAV_VEGETATION_TREES is set; the variable beats the checkbox");
    const bool before = box->isChecked();
    int sends = 0;
    vsim::SimWorker::setTreeColliderSinkForTest(
        [&](const ColliderWindow &) { ++sends; });
    // The world is changed first (stopped, as the other cases do), then a sim
    // stand-in appears, so only the colliders path sees it.
    a_->changeWorld([](vsim::WorldConfig &c) {
      QVERIFY(vsim::applyWorldPreset(c, QStringLiteral("hills"), 11));
    });
    pump(100);
    a_->fakeSim();
    auto cleanup = qScopeGuard([&] {
      vsim::SimWorker::setTreeColliderSinkForTest(nullptr);
      a_->dropFakeSim();
      box->setChecked(before);
    });
    if (!box->isChecked())
      box->setChecked(true);
    a_->setColliders({VegCollider{PgVec3{10, 0, 0}, 0.3f, 10.0f, false}});
    sends = 0;
    a_->pose(0, 0, -5);
    QCOMPARE(sends, 1);
    a_->pose(1, 0, -5); // standing still: nothing
    a_->pose(0, 0, -5);
    QCOMPARE(sends, 1);
    // Same preset, another seed, as the World tab does it.
    a_->dropFakeSim();
    a_->changeWorld([](vsim::WorldConfig &c) {
      QVERIFY(vsim::applyWorldPreset(c, QStringLiteral("hills"), 12));
    });
    a_->fakeSim();
    sends = 0;
    a_->pose(0, 0, -5); // same position, new world: rebuilt
    QCOMPARE(sends, 1);
  }

  // The Sim controls "Survey grid" checkbox is off by default, switches the
  // grid in the 3D views live and stores the choice.
  void surveyGridCheckboxTogglesLiveAndPersists() {
    if (!gl_)
      QSKIP("needs a GL platform plugin (QT_QPA_PLATFORM=xcb on Xvfb)");
    QCheckBox *box = nullptr;
    for (QCheckBox *c : w_->findChildren<QCheckBox *>())
      if (c->text() == QLatin1String("Survey grid"))
        box = c;
    QVERIFY2(box, "no Survey grid checkbox in the Sim controls");
    QVERIFY(!box->isChecked()); // off unless the user stored otherwise
    QCOMPARE(a_->gridShown(), box->isChecked());
    box->setChecked(true);
    QVERIFY(a_->gridShown());
    QVERIFY(a_->downGridShown());
    QCOMPARE(QSettings().value("sim/surveyGrid").toBool(), true);
    box->setChecked(false);
    QVERIFY(!a_->gridShown());
    QCOMPARE(QSettings().value("sim/surveyGrid").toBool(), false);
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
    // The widget stays alive to the end (above), its streamed world does not:
    // each one holds about 1 GB under software GL, and a one-process run kept
    // them all (12-13 GB over the suite, then the QtTest function timeout).
    const auto release = qScopeGuard([&ab] { ab.releaseStreamedWorld(); });
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


  // The prop look (Obstacle::style: tree trunk, canopy, container, ...) is part
  // of the saved world, so a preset's trees and containers must not turn into
  // plain shapes after a restart. Round-trips the settings store only.
  void obstacleStyleSurvivesRestart_data() {
    QTest::addColumn<QString>("preset");
    QTest::newRow("forest") << QStringLiteral("forest");
    QTest::newRow("yard") << QStringLiteral("yard");
  }
  void obstacleStyleSurvivesRestart() {
    if (!gl_)
      QSKIP("needs a GL platform plugin (QT_QPA_PLATFORM=xcb on Xvfb)");
    QFETCH(QString, preset);
    a_->changeWorld([&](vsim::WorldConfig &c) {
      QVERIFY(vsim::applyWorldPreset(c, preset, 5));
    });
    const QVector<vsim::Obstacle> want = a_->editor()->config().obstacles;
    int styled = 0;
    for (const vsim::Obstacle &o : want)
      styled += o.style != 0;
    QVERIFY2(styled > 0, "the preset has no styled obstacle to check");
    a_->persist();
    const QVector<vsim::Obstacle> got = a_->restore().obstacles;
    QCOMPARE(got.size(), want.size());
    for (int i = 0; i < want.size(); ++i)
      QVERIFY2(got[i].style == want[i].style,
               qPrintable(QStringLiteral("obstacle %1 style %2 -> %3")
                              .arg(i).arg(want[i].style).arg(got[i].style)));
  }

  // Every shipped preset (not just a sample), stopped, then restored by a fresh
  // widget as the next launch does.
  void everyPresetStoppedAndRestored_data() {
    QTest::addColumn<QString>("preset");
    for (const auto &p : vsim::worldPresetList())
      QTest::newRow(qPrintable(p.first)) << p.first;
  }
  void everyPresetStoppedAndRestored() {
    if (!gl_)
      QSKIP("needs a GL platform plugin (QT_QPA_PLATFORM=xcb on Xvfb)");
    QFETCH(QString, preset);
    bool applied = false;
    a_->changeWorld([&](vsim::WorldConfig &c) {
      applied = vsim::applyWorldPreset(c, preset, 17);
      c.worldPreset = preset;
    });
    QVERIFY2(applied, "unknown preset");
    pump(200);
    qInfo("%s stopped: alt=%.2f cam=%.2f above deck=%.2f", qPrintable(preset),
          altAbove(*a_), camAbove(*a_), aboveDeck(*a_));
    QVERIFY2(altAbove(*a_) >= 0.0f, "vehicle below the new ground");
    QVERIFY2(aboveDeck(*a_) >= 0.0f, "vehicle inside the helipad");
    QVERIFY2(camAbove(*a_) >= 0.0f, "camera below the new ground");
    a_->persist();
    pump(300);
    auto *b = new SimulatorWidget();
    restored_.push_back(b);
    SimulatorWidgetTestAccess ab(*b);
    // The widget stays alive to the end (above), its streamed world does not:
    // each one holds about 1 GB under software GL, and a one-process run kept
    // them all (12-13 GB over the suite, then the QtTest function timeout).
    const auto release = qScopeGuard([&ab] { ab.releaseStreamedWorld(); });
    b->resize(1200, 800);
    b->show();
    QVERIFY(QTest::qWaitForWindowExposed(b));
    pump(500);
    qInfo("%s restored: above deck=%.2f", qPrintable(preset), aboveDeck(ab));
    QVERIFY2(aboveDeck(ab) >= 0.0f, "restored vehicle is inside the helipad");
    QVERIFY2(aboveDeck(ab) < 1.0f, "restored vehicle is not on the pad");
  }

  // A widget that is never painted (a minimised window, or the sim view in a
  // hidden dock tab) does not run paintGL, which is where chunk uploads and
  // removals are applied. World changes in that state must not pile up the old
  // worlds' meshes: what waits for a paint is at most the current window.
  void aHiddenWidgetKeepsAtMostOneWindowAcrossWorldChanges() {
    if (!gl_)
      QSKIP("needs a GL platform plugin (QT_QPA_PLATFORM=xcb on Xvfb)");
    auto *h = new SimulatorWidget(); // never shown
    restored_.push_back(h);          // kept to the end, as the others
    SimulatorWidgetTestAccess hv(*h);
    // The first world is a forest: its chunks are slow to build, so it is still
    // being built when the next change comes. While the GUI thread is blocked
    // the workers go on and their results wait in the sink; they are of the
    // old generation when the GUI reads them.
    const char *presets[] = {"forest", "hills", "desert"};
    const size_t last = sizeof(presets) / sizeof(presets[0]) - 1;
    for (size_t i = 0; i <= last; ++i) {
      const QString preset = QString::fromLatin1(presets[i]);
      if (i > 0)
        QThread::msleep(2000);
      bool applied = false;
      const int base = hv.chunksPosted();
      hv.changeWorld([&](vsim::WorldConfig &c) {
        applied = vsim::applyWorldPreset(c, preset, 17);
        c.worldPreset = preset;
      });
      QVERIFY2(applied, "unknown preset");
      const size_t window = hv.windowChunks();
      if (i < last) // leave it half built
        QVERIFY(waitFor([&] { return hv.chunksPosted() - base >= 4; }, 120000));
      else
        QVERIFY(waitFor([&] { return hv.chunksCached() >= window; }, 180000));
      pump(300);
      qInfo("hidden, %s: window=%zu cached=%zu viewChunks=%zu "
            "pendingUploads=%zu pendingRemovals=%zu",
            presets[i], window, hv.chunksCached(), hv.viewChunks(),
            hv.viewPendingUploads(), hv.viewPendingRemovals());
      // What waits for a paint is at most the current window, each chunk once.
      QVERIFY2(hv.viewPendingUploads() <= window,
               qPrintable(QStringLiteral("%1 uploads wait for a paint that "
                                         "never comes, window is %2")
                              .arg(hv.viewPendingUploads())
                              .arg(window)));
      QVERIFY2(hv.viewPendingRemovals() <= window, "removals pile up unpainted");
    }
  }

  // Removing a chunk from a view that is not painted must also drop its queued
  // upload: the flush erases first and uploads after, so the upload would bring
  // the chunk back.
  void removingAChunkDropsItsQueuedUpload() {
    if (!gl_)
      QSKIP("needs a GL platform plugin (QT_QPA_PLATFORM=xcb on Xvfb)");
    vsim::SimRendererWidget r; // never shown: nothing paints, nothing flushes
    const std::vector<QVector3D> tri = {QVector3D(0, 0, 0), QVector3D(1, 0, 0),
                                        QVector3D(0, 1, 0)};
    const std::vector<QVector3D> nrm(3, QVector3D(0, 0, -1));
    const std::vector<QVector3D> col(3, QVector3D(0.4f, 0.5f, 0.3f));
    r.setWorldChunk(11, tri, nrm, col);
    r.setWorldChunk(12, tri, nrm, col);
    QCOMPARE(r.pendingChunkUploadCount(), size_t(2));
    r.removeWorldChunk(11);
    QCOMPARE(r.pendingChunkUploadCount(), size_t(1));
    r.clearWorldChunks();
    QCOMPARE(r.pendingChunkUploadCount(), size_t(0));
  }

  // Sim Running and the vehicle airborne (not landed): each preset change
  // brings it back down onto the new ground's pad, never into it.
  void inFlightChangesPlaceVehicleOnGround() {
    if (!gl_)
      QSKIP("needs a GL platform plugin (QT_QPA_PLATFORM=xcb on Xvfb)");
    if (!SitlModule::instance().available())
      QSKIP("no engine module (set VAYU_SITL_MODULE)");
    a_->changeWorld(changes()[1].edit);
    if (!a_->running()) {
      a_->start();
      QVERIFY(a_->running());
    }
    // Online and placed: the engine's own pose, on the pad (the stopped-state
    // park is also "on the pad", so wait for the collision to have shipped).
    QVERIFY(waitFor(
        [&] {
          return a_->logText().contains(QStringLiteral("world mesh \u2192 daemon")) &&
                 !a_->shipBusy() && altAbove(*a_) > 0.0f && altAbove(*a_) < 3.0f;
        },
        30000));
    for (const auto &pr : vsim::worldPresetList()) {
      pump(1500);
      QVERIFY(waitFor([&] { return !a_->shipBusy(); }, 30000));
      a_->teleport(0.0f, 0.0f, 40.0f); // airborne, 40 m up
      // Nothing holds it there: it falls and lands in about 3 s. The GUI's copy
      // of the pose lags the engine by seconds on a slow runner and then plays
      // the whole fall back in one burst, so it can miss the airborne window
      // altogether. Read the engine's own pose, polling without processing
      // events, and change the world while it is in the air.
      const auto engineAlt = [&] {
        const QVector3D p = a_->lastPos();
        return -p.z() - (a_->hasGround() ? a_->groundAt(p.x(), p.y()) : 0.0f);
      };
      QElapsedTimer fall;
      fall.start();
      bool airborne = false;
      while (fall.elapsed() < 5000 && !(airborne = engineAlt() > 5.0f))
        QThread::msleep(5);
      QVERIFY2(airborne, qPrintable(pr.first));
      bool applied = false;
      a_->changeWorld([&](vsim::WorldConfig &c) {
        applied = vsim::applyWorldPreset(c, pr.first, 5);
        c.worldPreset = pr.first;
      });
      QVERIFY2(applied, qPrintable(pr.first));
      pump(300);
      QVERIFY2(waitFor([&] { return !a_->shipBusy(); }, 30000),
               qPrintable(pr.first));
      pump(3000);
      qInfo("in flight -> %s: alt=%.2f cam=%.2f above deck=%.2f",
            qPrintable(pr.first), altAbove(*a_), camAbove(*a_), aboveDeck(*a_));
      // On the new ground's pad: not below it, not still up at 40 m, and not
      // re-seated somewhere off the pad. The home pad sits about 0.9 m above
      // the ground and the vehicle about 0.04 m above its deck.
      QVERIFY2(altAbove(*a_) >= -0.05f, qPrintable(pr.first));
      QVERIFY2(altAbove(*a_) < 1.5f, qPrintable(pr.first));
      QVERIFY2(aboveDeck(*a_) >= -0.05f, qPrintable(pr.first));
      QVERIFY2(aboveDeck(*a_) < 1.0f, qPrintable(pr.first));
      QVERIFY2(camAbove(*a_) >= 0.0f, qPrintable(pr.first));
    }
  }

  // Sim Running against the real engine: after each change (and the collision
  // build it triggers) the vehicle ends up on the new ground, at rest.
  void runningChangesPlaceVehicleOnGround() {
    if (!gl_)
      QSKIP("needs a GL platform plugin (QT_QPA_PLATFORM=xcb on Xvfb)");
    if (!SitlModule::instance().available())
      QSKIP("no engine module (set VAYU_SITL_MODULE)");
    a_->changeWorld(changes()[0].edit); // start on a raised world
    a_->shipDelay(4000); // a slow runner: the collision build outlasts a fixed settle
    a_->start();
    QVERIFY(a_->running());
    // shipBusy() is false right after start(): the engine is not online yet, so
    // nothing is queued. Waiting for it to go false alone passes at once and
    // leaves the vehicle at the engine's default pose (far below a raised
    // world's ground) until the collision has shipped and Start's reset has
    // placed it. Wait for that ship, then for the placement, then settle.
    QVERIFY(waitFor(
        [&] {
          return a_->logText().contains(QStringLiteral("world mesh \u2192 daemon")) &&
                 !a_->shipBusy();
        },
        30000));
    QVERIFY(waitFor([&] { return altAbove(*a_) > 0.0f && altAbove(*a_) < 3.0f; },
                    10000));
    pump(3000); // settle on the pad
    qInfo("running: initial alt=%.2f above deck=%.2f", altAbove(*a_),
          aboveDeck(*a_));
    QVERIFY(altAbove(*a_) >= -0.05f);
    // On the pad deck, not sunk through it to the terrain under it.
    QVERIFY2(aboveDeck(*a_) >= -0.05f, "sank into the helipad after Start");

    QVERIFY(!a_->logText().contains(QStringLiteral("World changed")));
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
    // The in-flight respawn is announced to the user, once (not per change).
    QCOMPARE(a_->logText().count(QStringLiteral(
                 "World changed: vehicle respawned on the home pad")),
             1);
  }

  // Procedural worlds are endless: far past the old +-200 m arena edge the
  // terrain still streams and still collides (the vehicle teleported there
  // comes to rest on the ground instead of falling), and the helipads around it
  // sit on the same surface an independent sampler of the world's field gives.
  void farFromOriginStreamsAndCollides_data() {
    QTest::addColumn<QString>("preset");
    QTest::addColumn<float>("dist");
    for (const char *id : {"alpine", "hills", "desert", "forest", "racing"})
      for (float d : {600.0f, 2000.0f})
        QTest::newRow(qPrintable(QStringLiteral("%1 %2 m").arg(id).arg(d)))
            << QString::fromLatin1(id) << d;
    for (float d : {600.0f, 2000.0f})
      QTest::newRow(qPrintable(QStringLiteral("endless %1 m").arg(d)))
          << QStringLiteral("endless") << d;
  }
  void farFromOriginStreamsAndCollides() {
    if (!gl_)
      QSKIP("needs a GL platform plugin (QT_QPA_PLATFORM=xcb on Xvfb)");
    if (!SitlModule::instance().available())
      QSKIP("no engine module (set VAYU_SITL_MODULE)");
    QFETCH(QString, preset);
    QFETCH(float, dist);
    // Per-phase wall time, logged so a CI timeout shows where it went.
    QElapsedTimer row;
    row.start();
    if (!a_->running()) {
      a_->start();
      QVERIFY(a_->running());
    }
    const qint64 tStart = row.elapsed();
    a_->changeWorld([&](vsim::WorldConfig &c) {
      if (preset == QLatin1String("endless")) {
        c.worldPreset.clear();
        c.proceduralBiome = preset;
        c.proceduralSeed = 31;
      } else {
        QVERIFY(vsim::applyWorldPreset(c, preset, 31));
      }
    });
    QVERIFY(waitFor([&] { return !a_->shipBusy(); }, 30000));
    const qint64 tWorld = row.elapsed();
    pump(2000);

    const float x = dist * 0.8f, y = dist * 0.6f; // |(x, y)| == dist
    a_->teleport(x, y, 8.0f);
    // The streamer follows the vehicle: wait for the collision there to ship.
    QVERIFY(waitFor(
        [&] {
          const QVector3D p = a_->lastPos();
          return std::hypot(p.x() - x, p.y() - y) < 30.0f && !a_->shipBusy();
        },
        60000));
    const qint64 tShip = row.elapsed();
    // Fall (8 m) and settle, sampling the pose so a failure shows whether the
    // vehicle was still falling, resting on the engine's z=0 floor, or lifted.
    QString trace;
    {
      QElapsedTimer tl;
      tl.start();
      qint64 next = 0;
      // Up to 6 s, but done as soon as the vehicle has lain still for 1 s
      // after at least 2 s (the fall is 8 m, about 1.3 s): a fixed 6 s per
      // row was most of the test's wall time on a slow runner.
      qint64 stillSince = -1;
      while (tl.elapsed() < 6000) {
        pump(20);
        if (std::fabs(a_->lastVel().z()) < 0.05f && std::fabs(a_->lastVel().x()) < 0.3f &&
            std::fabs(a_->lastVel().y()) < 0.3f) {
          if (stillSince < 0)
            stillSince = tl.elapsed();
          if (tl.elapsed() >= 2000 && tl.elapsed() - stillSince >= 1000)
            break;
        } else {
          stillSince = -1;
        }
        if (tl.elapsed() >= next) {
          next += 500;
          trace += QStringLiteral(" t=%1:z=%2,vz=%3,busy=%4")
                       .arg(tl.elapsed())
                       .arg(a_->lastPos().z(), 0, 'f', 1)
                       .arg(a_->lastVel().z(), 0, 'f', 1)
                       .arg(a_->shipBusy() ? 1 : 0);
        }
      }
    }
    const QVector3D p = a_->lastPos();
    const float g = a_->groundAt(p.x(), p.y());
    const float alt = -p.z() - g;
    long rssMb = -1; // resident set, Linux only; informational
#ifdef __linux__
    {
      QFile sm(QStringLiteral("/proc/self/statm"));
      if (sm.open(QIODevice::ReadOnly)) {
        const QList<QByteArray> f = sm.readAll().split(' ');
        if (f.size() > 1)
          rssMb = f[1].toLong() * 4 / 1024; // 4 KiB pages
      }
    }
#endif
    qInfo("far %s %.0f m phases: start=%lld world=%lld teleport-ship=%lld "
          "settle=%lld total=%lld ms",
          qPrintable(preset), dist, static_cast<long long>(tStart),
          static_cast<long long>(tWorld - tStart),
          static_cast<long long>(tShip - tWorld - 2000),
          static_cast<long long>(row.elapsed() - tShip),
          static_cast<long long>(row.elapsed()));
    qInfo("far %s %.0f m: vehicle=(%.1f %.1f %.2f) ground=%.2f alt=%.2f "
          "vz=%.2f chunks=%zu pads=%zu rss=%ld MB",
          qPrintable(preset), dist, p.x(), p.y(), p.z(), g, alt,
          a_->lastVel().z(), a_->chunksCached(), a_->pads().size(), rssMb);
    if (alt < -0.3f)
      qWarning("pose trace after the teleport:%s", qPrintable(trace));
    QVERIFY2(alt >= -0.3f, "vehicle fell through the terrain far from origin");
    QVERIFY2(alt < 3.0f, "vehicle did not come to rest on the terrain");
    // (Vertical speed is only logged: a vehicle at rest on a steep dune can
    // still slide, so a hard bound would be flaky. A fall through shows in alt.)
    QVERIFY2(a_->chunksCached() >= 9, "collision chunks were not streamed");
    QVERIFY2(a_->chunksCached() <= 49, "chunk cache is not bounded");

    // Optional pause so a screenshot of the window can be taken (manual use).
    pump(qEnvironmentVariableIntValue("NAV_TEST_HOLD_MS"));

    // Pads near the vehicle sit on the streamed surface of THIS world's field
    // (independent sampler, not the widget's own lambda), not above/below it.
    vsim::procgen::FieldParams fp = a_->editor()->config().field;
    fp.seed = static_cast<uint32_t>(a_->editor()->config().proceduralSeed);
    const vsim::procgen::TerrainField field(fp);
    int nearPads = 0;
    for (const QVector3D &pad : a_->pads()) {
      const float ref = vsim::procgen::streamSurfaceHeight(field, pad.x(), pad.y());
      QVERIFY2(std::fabs(pad.z() - ref) < 0.02f, "a helipad floats or sinks");
      if (std::hypot(pad.x() - x, pad.y() - y) < 260.0f)
        ++nearPads;
    }
    QVERIFY2(nearPads >= 1, "no helipad was placed around the far vehicle");
  }

  // A vehicle teleported/reset far away is held until the collision there has
  // shipped: it is never released into terrain the engine lacks, so it does not
  // fall through to the engine's z=0 floor (which a slow machine, or a slow
  // collision build as forced here, would otherwise make it do). Sampled every
  // 50 ms from the teleport to 3 s after the vehicle is released and settled.
  void farTeleportNeverFallsThrough_data() {
    QTest::addColumn<QString>("preset");
    QTest::newRow("alpine") << QStringLiteral("alpine");
    QTest::newRow("endless") << QStringLiteral("endless");
  }
  void farTeleportNeverFallsThrough() {
    if (!gl_)
      QSKIP("needs a GL platform plugin (QT_QPA_PLATFORM=xcb on Xvfb)");
    if (!SitlModule::instance().available())
      QSKIP("no engine module (set VAYU_SITL_MODULE)");
    QFETCH(QString, preset);
    if (!a_->running()) {
      a_->start();
      QVERIFY(a_->running());
    }
    a_->changeWorld([&](vsim::WorldConfig &c) {
      if (preset == QLatin1String("endless")) {
        c.worldPreset.clear();
        c.proceduralBiome = preset;
        c.proceduralSeed = 31;
      } else {
        QVERIFY(vsim::applyWorldPreset(c, preset, 31));
      }
    });
    QVERIFY(waitFor([&] { return !a_->shipBusy(); }, 30000));
    pump(2000);

    const float x = 480.0f, y = 360.0f;
    a_->shipDelay(3000); // longer than the 8 m fall (~1.3 s)
    a_->teleport(x, y, 8.0f);
    float minAlt = 1e9f;
    int samples = 0;
    qint64 settledAt = -1;
    QElapsedTimer t;
    t.start();
    while (t.elapsed() < 60000) {
      pump(50);
      const QVector3D p = a_->lastPos();
      const float alt = -p.z() - a_->groundAt(p.x(), p.y());
      minAlt = std::min(minAlt, alt);
      ++samples;
      const bool there = std::hypot(p.x() - x, p.y() - y) < 30.0f;
      if (there && !a_->shipBusy() && alt < 3.0f && settledAt < 0)
        settledAt = t.elapsed();
      if (settledAt >= 0 && t.elapsed() - settledAt >= 3000)
        break;
    }
    a_->shipDelay(0);
    qInfo("hold %s: samples=%d minAlt=%.2f settledAt=%lld ms", qPrintable(preset),
          samples, minAlt, static_cast<long long>(settledAt));
    QVERIFY2(settledAt >= 0, "the vehicle never arrived and came to rest");
    QVERIFY2(minAlt >= -0.3f, "the vehicle sank below the ground on the way");
    const QVector3D p = a_->lastPos();
    const float alt = -p.z() - a_->groundAt(p.x(), p.y());
    QVERIFY2(alt >= -0.3f && alt < 3.0f, "not resting on the terrain");
  }

  // A vehicle dropped on ANY helipad (not just the home one) must rest on its
  // deck. The pad collision used to be built from the pad list as it stood
  // before the view followed the placement, so a pad the list did not yet hold
  // had no collision in the engine and the vehicle sank through its deck to the
  // terrain (about 0.9 m low). Hills seed 5: pads taken from the live list,
  // each dropped on 1 m above its deck.
  // Needs xcb and the engine module: it skips offscreen (plain ctest), and
  // CI's "tst_world_change_floor (xcb on Xvfb ...)" step runs it, one process
  // per case (about a minute).
  // The pose the product decides on must not wait for the GUI thread. The GUI's
  // copy is a queued event, so a render or a terrain build ahead of it shows the
  // vehicle where it was seconds ago (1-5 s measured on a loaded CI runner);
  // lift, stream arrival and collision cover must use the engine's newest pose.
  void decisionsSeeTheEnginesPoseWhileTheGuiThreadIsBlocked() {
    if (!gl_)
      QSKIP("needs a GL platform plugin (QT_QPA_PLATFORM=xcb on Xvfb)");
    if (!SitlModule::instance().available())
      QSKIP("no engine module (set VAYU_SITL_MODULE)");
    a_->changeWorld([](vsim::WorldConfig &c) {
      QVERIFY(vsim::applyWorldPreset(c, QStringLiteral("hills"), 5));
    });
    if (!a_->running()) {
      a_->start();
      QVERIFY(a_->running());
    }
    QVERIFY(waitFor(
        [&] {
          return a_->logText().contains(
                     QStringLiteral("world mesh \u2192 daemon")) &&
                 !a_->shipBusy() && altAbove(*a_) > 0.0f &&
                 altAbove(*a_) < 3.0f;
        },
        30000));
    pump(1500);
    // Park on a pad, then move 25 m along x inside the collision window already
    // in force (so the engine takes the reset at once), and block the GUI thread.
    const std::vector<QVector3D> pads = a_->pads();
    QVERIFY2(pads.size() > 1, "the world has no pads");
    const QVector3D pad = pads[1];
    const float kDeckM = vsim::SimRendererWidget::kHelipadDeckM;
    const float deck = -pad.z() - kDeckM;
    a_->placeAt(pad.x(), pad.y(), deck - 1.0f);
    QVERIFY(waitFor([&] { return !a_->shipBusy(); }, 30000));
    pump(2500);
    // The first wait can pass before the build for this move has started (it
    // starts on the next stream tick), and a slow runner is still building
    // after the pump: wait for it again, then let a following tick confirm
    // that nothing else starts.
    QVERIFY(waitFor([&] { return !a_->shipBusy(); }, 30000));
    pump(500);
    QVERIFY(waitFor([&] { return !a_->shipBusy(); }, 30000));
    const QVector3D before = a_->cachedPos();
    const float nx = pad.x() + 25.0f;
    a_->placeAt(nx, pad.y(), deck - 1.0f);
    QVERIFY2(!a_->shipBusy(), "the move needed a collision ship; pick a nearer spot");
    // No event processing: the GUI thread is "busy" for two seconds.
    QThread::msleep(2000);
    const QVector3D fresh = a_->lastPos();
    const QVector3D cached = a_->cachedPos();
    QVERIFY2(std::abs(cached.x() - before.x()) < 1.0f,
             "the GUI copy moved while its thread was blocked: the test is not "
             "exercising a lagging copy");
    QVERIFY2(std::hypot(fresh.x() - nx, fresh.y() - pad.y()) < 3.0f,
             qPrintable(QStringLiteral("the decision pose is (%1, %2), not at the "
                                       "new spot (%3, %4)")
                            .arg(fresh.x()).arg(fresh.y()).arg(nx).arg(pad.y())));
  }

  // The pad search (dozens of candidates, each sampling the exact surface 17
  // times) cost the GUI thread 150-650 ms per collision build, and the pose and
  // the repaints waited behind it. It runs on the collision worker.
  void padSearchRunsOffTheGuiThread() {
    if (!gl_)
      QSKIP("needs a GL platform plugin (QT_QPA_PLATFORM=xcb on Xvfb)");
    if (!SitlModule::instance().available())
      QSKIP("no engine module (set VAYU_SITL_MODULE)");
    a_->changeWorld([](vsim::WorldConfig &c) {
      QVERIFY(vsim::applyWorldPreset(c, QStringLiteral("hills"), 5));
    });
    if (!a_->running()) {
      a_->start();
      QVERIFY(a_->running());
    }
    QVERIFY(waitFor([&] { return !a_->shipBusy() && altAbove(*a_) > 0.0f; },
                    30000));
    std::atomic<int> onGui{0}, offGui{0};
    QThread *const gui = QCoreApplication::instance()->thread();
    a_->onPadSearch([&] {
      (QThread::currentThread() == gui ? onGui : offGui).fetch_add(1);
    });
    // A far teleport recentres the collision window: a new build, so a new
    // pad search.
    a_->teleport(480.0f, 360.0f, 8.0f);
    // Two searches: the new collision window's pads, and the view's pad list,
    // refreshed because the view moved far.
    QVERIFY(waitFor([&] { return offGui.load() >= 2; }, 60000));
    QVERIFY(waitFor([&] { return !a_->shipBusy(); }, 30000));
    a_->onPadSearch({});
    QCOMPARE(onGui.load(), 0);
  }

  // On a slow machine the collision window moves again before any build lands.
  // Newest-wins then starved every build and the engine kept collision that did
  // not reach the vehicle. A build that already covers the vehicle must land
  // even when a newer one is queued behind it.
  void aRecentreShipThatCoversTheVehicleStillLands() {
    if (!gl_)
      QSKIP("needs a GL platform plugin (QT_QPA_PLATFORM=xcb on Xvfb)");
    if (!SitlModule::instance().available())
      QSKIP("no engine module (set VAYU_SITL_MODULE)");
    a_->changeWorld([](vsim::WorldConfig &c) {
      QVERIFY(vsim::applyWorldPreset(c, QStringLiteral("hills"), 5));
    });
    if (!a_->running()) {
      a_->start();
      QVERIFY(a_->running());
    }
    const QString landed = QStringLiteral("world mesh \u2192 daemon");
    // The first collision is in the engine, the chunks around the vehicle are
    // cached (a recentre needs them) and the vehicle is on the ground.
    QVERIFY(waitFor(
        [&] {
          return a_->logText().contains(landed) && !a_->shipBusy() &&
                 a_->chunksCached() >= 25 && altAbove(*a_) > 0.0f;
        },
        60000));
    a_->clearLog();
    a_->shipDelay(1500); // a build in flight long enough to be superseded
    // The window around the vehicle's own cell: it covers the vehicle.
    a_->recentre(0, 0);
    QVERIFY2(a_->shipRunning(),
             qPrintable(QStringLiteral("pending=%1 cached=%2 pos=(%3,%4)")
                            .arg(a_->collisionPending())
                            .arg(a_->chunksCached())
                            .arg(a_->lastPos().x())
                            .arg(a_->lastPos().y())));
    // The window moves on while that build is running (the next cell over still
    // holds the vehicle's cell, but the engine has not been given it yet).
    a_->recentre(1, 0);
    QVERIFY(a_->shipQueued());
    QVERIFY(waitFor([&] { return !a_->shipBusy(); }, 30000));
    a_->shipDelay(0);
    // Both landed: the covering one was not dropped for the newer one.
    QCOMPARE(a_->logText().count(landed), 2);
  }

  // The GUI thread hands chunk jobs to the mesh pool once per stream tick; with
  // software GL a single frame took 11-12 s, the pool ran dry after its first
  // few chunks and the terrain stopped growing until the frame ended. Workers
  // now pull from a queue, so a blocked GUI thread costs nothing until the
  // whole needed set is built.
  void meshingContinuesWhileTheGuiThreadIsBlocked() {
    if (!gl_)
      QSKIP("needs a GL platform plugin (QT_QPA_PLATFORM=xcb on Xvfb)");
    if (!SitlModule::instance().available())
      QSKIP("no engine module (set VAYU_SITL_MODULE)");
    a_->changeWorld([](vsim::WorldConfig &c) {
      QVERIFY(vsim::applyWorldPreset(c, QStringLiteral("hills"), 5));
    });
    if (!a_->running()) {
      a_->start();
      QVERIFY(a_->running());
    }
    QVERIFY(waitFor(
        [&] {
          return a_->logText().contains(
                     QStringLiteral("world mesh \u2192 daemon")) &&
                 !a_->shipBusy() && a_->chunksCached() >= 25 &&
                 a_->chunksQueued() == 0;
        },
        60000));
    pump(500);
    // A far spot: its whole 7x7 window is missing, far more than one tick's
    // hand-out used to be (6). Poll in small steps so only ONE tick has planned
    // when the GUI thread blocks.
    const int base = a_->chunksPosted();
    a_->teleport(20000.0f, 20000.0f, 5.0f);
    QElapsedTimer t;
    t.start();
    while (a_->chunksQueued() == 0 && a_->chunksPosted() == base &&
           t.elapsed() < 20000)
      pump(5);
    QVERIFY2(a_->chunksQueued() > 0 || a_->chunksPosted() > base,
             "the far spot was never planned");
    // No event processing: the GUI thread is busy for four seconds, as a
    // software-GL frame on a starved machine keeps it.
    QThread::msleep(4000);
    const int builtWhileBlocked = a_->chunksPosted() - base;
    QVERIFY2(builtWhileBlocked >= 10,
             qPrintable(QStringLiteral("only %1 chunks were meshed while the "
                                       "GUI thread was blocked (one tick used "
                                       "to hand out 6)")
                            .arg(builtWhileBlocked)));
  }

  void dropOnEveryPadRestsOnDeck() {
    if (!gl_)
      QSKIP("needs a GL platform plugin (QT_QPA_PLATFORM=xcb on Xvfb)");
    if (!SitlModule::instance().available())
      QSKIP("no engine module (set VAYU_SITL_MODULE)");
    a_->changeWorld([](vsim::WorldConfig &c) {
      QVERIFY(vsim::applyWorldPreset(c, QStringLiteral("hills"), 5));
    });
    if (!a_->running()) {
      a_->start();
      QVERIFY(a_->running());
    }
    // Online and placed on the home pad first (the engine's own pose), as the
    // other cases do: a first drop sent before the engine is posting is lost.
    QVERIFY(waitFor(
        [&] {
          return a_->logText().contains(
                     QStringLiteral("world mesh \u2192 daemon")) &&
                 !a_->shipBusy() && altAbove(*a_) > 0.0f &&
                 altAbove(*a_) < 3.0f;
        },
        30000));
    pump(1500);
    const float kDeckM = vsim::SimRendererWidget::kHelipadDeckM;
    int failed = 0;
    // Two passes over the first ten non-home entries of the pad list, read
    // afresh before every drop: the view (and with it the list) follows the
    // vehicle, so the pads dropped on are not the ones known at Start and the
    // collision window in force was planned around a different list.
    const int kPasses = 2, kPerPass = 10;
    int hops = 0;
    for (int i = 0; i < kPasses * kPerPass; ++i) {
      const std::vector<QVector3D> pads = a_->pads(); // a copy: the list moves
      QVERIFY2(pads.size() > 3, "the world has no pads to drop on");
      const QVector3D pad = pads[size_t(1 + i % kPerPass) % pads.size()];
      ++hops;
      const float deck = -pad.z() - kDeckM; // NED z of this pad's deck top
      a_->placeAt(pad.x(), pad.y(), deck - 1.0f);
      QVERIFY(waitFor([&] { return !a_->shipBusy(); }, 30000));
      pump(2500); // fall (~0.5 s) and settle
      const QVector3D p = a_->lastPos();
      const float rest = p.z() - deck; // negative: above the deck
      qInfo("hop %d (%.1f %.1f): rest z - deck = %.3f", i,
            static_cast<double>(pad.x()), static_cast<double>(pad.y()),
            static_cast<double>(rest));
      if (std::hypot(p.x() - pad.x(), p.y() - pad.y()) > 3.0f ||
          std::fabs(rest + 0.04f) > 0.06f) {
        ++failed;
        qWarning("hop %d (%.1f %.1f): vehicle at (%.1f %.1f) rest %.3f", i,
                 static_cast<double>(pad.x()), static_cast<double>(pad.y()),
                 static_cast<double>(p.x()), static_cast<double>(p.y()),
                 static_cast<double>(rest));
      }
    }
    QVERIFY2(failed == 0,
             qPrintable(QStringLiteral("%1 of %2 pad drops did not rest on "
                                       "the deck")
                            .arg(failed)
                            .arg(hops)));
  }

  // Stop, then change the world again with the sim stopped.
  void stoppedAfterRunPlacesVehicleOnGround() {
    if (!gl_)
      QSKIP("needs a GL platform plugin (QT_QPA_PLATFORM=xcb on Xvfb)");
    if (!SitlModule::instance().available())
      QSKIP("no engine module (set VAYU_SITL_MODULE)");
    // Self-contained (CI runs each case in its own process): start a run here
    // if no earlier case left one going.
    if (!a_->running()) {
      a_->changeWorld(changes()[0].edit);
      a_->start();
      QVERIFY(a_->running());
      QVERIFY(waitFor([&] { return !a_->shipBusy(); }, 20000));
      pump(1000);
    }
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
