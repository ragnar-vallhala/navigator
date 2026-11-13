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
// tst_sim_gl_context.cpp - the 3D sim view survives being undocked and docked
// again, and its OpenGL context being destroyed and created again.
//
// The view is a QOpenGLWindow hosted by QWidget::createWindowContainer, the way
// the Simulator page and a docking system use it. Needs a real GL platform
// plugin: run with QT_QPA_PLATFORM=xcb on an Xvfb and LIBGL_ALWAYS_SOFTWARE=1.
// On "offscreen" (ctest's default) it skips, because QOpenGLWindow crashes there.
//
// It loads a small world (imported mesh + streamed chunk + obstacles + helipad +
// a training gate) and a drone mesh, renders a frame, recreates the context,
// renders again, and checks: the scene is not blank and looks the same, no GL
// error was raised, no GL object leaks, and no stray top-level window remains.
#include "SimHudWidget.h"
#include "SimRendererWidget.h"

#include <QGuiApplication>
#include <QSurfaceFormat>
#include <QVBoxLayout>
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QtTest>

#include <cmath>
#include <set>

using vsim::SimRendererWidget;

namespace {

// A rolling NED height field on [-half, half]^2 (z up is negative), as a
// triangle soup with per-vertex normals and colours.
struct Soup {
  std::vector<QVector3D> pos, nrm, col;
};
Soup heightField(float cx, float cy, float half, int n) {
  auto h = [](float x, float y) { return -0.6f * std::sin(x * 0.5f) * std::cos(y * 0.5f); };
  auto vert = [&](Soup &s, float x, float y) {
    const float e = 0.01f;
    const QVector3D dx(2 * e, 0, h(x + e, y) - h(x - e, y));
    const QVector3D dy(0, 2 * e, h(x, y + e) - h(x, y - e));
    QVector3D nn = QVector3D::crossProduct(dy, dx).normalized(); // up = -z
    if (nn.z() > 0)
      nn = -nn;
    s.pos.push_back({x, y, h(x, y)});
    s.nrm.push_back(nn);
    const float t = 0.5f + 0.5f * std::sin(x * 0.3f + y * 0.2f);
    s.col.push_back({0.25f + 0.3f * t, 0.45f + 0.2f * t, 0.2f});
  };
  Soup s;
  const float st = 2.0f * half / n;
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) {
      const float x0 = cx - half + i * st, y0 = cy - half + j * st;
      const float x1 = x0 + st, y1 = y0 + st;
      for (auto [x, y] : {std::pair{x0, y0}, {x1, y0}, {x1, y1}, {x0, y0}, {x1, y1}, {x0, y1}})
        vert(s, x, y);
    }
  return s;
}

// A small wedge-shaped "airframe" so the imported-mesh path is exercised.
Soup droneBox() {
  Soup s;
  const QVector3D v[8] = {{.1f, .06f, .02f},  {.1f, -.06f, .02f},  {-.1f, -.06f, .02f},
                          {-.1f, .06f, .02f}, {.1f, .06f, -.02f},  {.1f, -.06f, -.02f},
                          {-.1f, -.06f, -.02f}, {-.1f, .06f, -.02f}};
  const int f[12][3] = {{0, 1, 2}, {0, 2, 3}, {4, 6, 5}, {4, 7, 6}, {0, 4, 5}, {0, 5, 1},
                        {1, 5, 6}, {1, 6, 2}, {2, 6, 7}, {2, 7, 3}, {3, 7, 4}, {3, 4, 0}};
  for (auto &t : f) {
    const QVector3D n =
        QVector3D::crossProduct(v[t[1]] - v[t[0]], v[t[2]] - v[t[0]]).normalized();
    for (int k : t) {
      s.pos.push_back(v[k]);
      s.nrm.push_back(n);
    }
  }
  return s;
}

void loadScene(SimRendererWidget *r) {
  const Soup w = heightField(0, 0, 12, 24);
  r->setWorldMesh(w.pos, w.nrm, w.col);
  const Soup c = heightField(30, 0, 12, 16);
  r->setWorldChunk(0x100000001LL, c.pos, c.nrm, c.col);
  const Soup c2 = heightField(0, 30, 12, 16);
  r->setWorldChunk(0x100000002LL, c2.pos, c2.nrm, c2.col);
  r->setWorldChunk(0x100000003LL, c2.pos, c2.nrm, c2.col);
  r->removeWorldChunk(0x100000003LL); // a removal in the stream must stay removed
#ifdef VAYU_SIM_GRASS
  { // flora on a chunk (blades: pos xyz, yaw, height, flower, tint rgb)
    std::vector<float> bl;
    for (int i = 0; i < 64; ++i) {
      const int col = i % 8, row = i / 8; // grid cell, integer on purpose
      const float x = 26.0f + float(col) * 1.5f, y = -5.0f + float(row) * 1.5f;
      for (float v : {x, y, 0.0f, 0.3f * float(i), 0.5f, float(i & 1), 0.3f, 0.6f, 0.2f})
        bl.push_back(v);
    }
    r->setChunkFlora(0x100000001LL, bl, 64, 30.0f, 0.0f, 12.0f);
    r->setChunkFlora(0x100000003LL, bl, 64, 30.0f, 0.0f, 12.0f);
    r->removeChunkFlora(0x100000003LL);
  }
#endif
  vsim::Obstacle box;
  box.pos = {3, 2, -0.5f};
  vsim::Obstacle cyl;
  cyl.type = vsim::Obstacle::Cylinder;
  cyl.pos = {-3, 2, -1};
  cyl.size = {0.5f, 0.5f, 2.0f};
  r->setObstacles({box, cyl});
  r->setHelipads({QVector3D(-2, -3, 0.0f)});
  r->setTrainingGates({}); // gates hide the world; the course is exercised elsewhere
  const Soup d = droneBox();
  r->setDroneMesh(d.pos, d.nrm);
  vsim::SimSnapshot snap;
  snap.pos_w = {0, 0, -0.6f};
  r->setSnapshot(snap);
  r->setMotorsEditable(false);
}

// A coarse signature of a frame: 16x12 block means. Same scene -> same blocks
// (the sky glow animates a little, hence a tolerance rather than equality).
std::vector<float> signature(const QImage &img) {
  const QImage q = img.convertToFormat(QImage::Format_RGB32)
                       .scaled(16, 12, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
  std::vector<float> s;
  for (int y = 0; y < q.height(); ++y)
    for (int x = 0; x < q.width(); ++x) {
      const QColor c = q.pixelColor(x, y);
      s.insert(s.end(), {float(c.red()), float(c.green()), float(c.blue())});
    }
  return s;
}
float meanDiff(const std::vector<float> &a, const std::vector<float> &b) {
  float d = 0;
  for (size_t i = 0; i < a.size(); ++i)
    d += std::fabs(a[i] - b[i]);
  return d / float(a.size());
}
int distinctColours(const QImage &img) {
  std::set<QRgb> s;
  for (int y = 0; y < img.height(); y += 3)
    for (int x = 0; x < img.width(); x += 3)
      s.insert(img.pixel(x, y));
  return int(s.size());
}
// Lowest GL buffer name the driver hands out now: with every freed name reused,
// a leak per reset shows up as this number growing.
GLuint probeBufferName(SimRendererWidget *r) {
  r->makeCurrent();
  GLuint id = 0;
  QOpenGLContext::currentContext()->functions()->glGenBuffers(1, &id);
  QOpenGLContext::currentContext()->functions()->glDeleteBuffers(1, &id);
  r->doneCurrent();
  return id;
}

// Two views of the scene: over the imported world mesh, and over the streamed
// chunk (whose only copy lives in the VBO until it is read back).
struct Views {
  QImage mesh, chunk;
};
Views capture(SimRendererWidget *r) {
  Views v;
  vsim::SimSnapshot s;
  s.pos_w = {0, 0, -0.6f};
  r->setSnapshot(s);
  v.mesh = r->grabFrame();
  s.pos_w = {30, 0, -0.6f};
  r->setSnapshot(s);
  v.chunk = r->grabFrame();
  s.pos_w = {0, 0, -0.6f};
  r->setSnapshot(s);
  return v;
}
bool sameScene(const Views &ref, const Views &v, const char **why) {
  *why = nullptr;
  if (distinctColours(v.mesh) < 40 || distinctColours(v.chunk) < 40)
    *why = "blank frame";
  else if (meanDiff(signature(ref.mesh), signature(v.mesh)) > 8.0f)
    *why = "imported world view changed";
  else if (meanDiff(signature(ref.chunk), signature(v.chunk)) > 8.0f)
    *why = "streamed chunk view changed";
  return !*why;
}

// A docked-looking host (a window with a layout) and the renderer inside its
// window container; "float" makes the container a top-level of its own and
// "dock" puts it back, which is what a docking system does to a panel.
struct Rig {
  QWidget host;
  QVBoxLayout *lay = nullptr;
  SimRendererWidget *r = nullptr;
  QWidget *box = nullptr;
  SimHudWidget *hud = nullptr;
  explicit Rig(bool cpuChunks = false, bool withHud = false) {
    lay = new QVBoxLayout(&host);
    lay->setContentsMargins(0, 0, 0, 0);
    r = new SimRendererWidget;
    r->setGlErrorCheck(true);
    r->setForceChunkCpuCopy(cpuChunks);
    if (withHud) {
      hud = new SimHudWidget(&host);
      hud->hide();
      r->setHudWidget(hud);
      r->setHudVisible(true);
    }
    loadScene(r);
    box = QWidget::createWindowContainer(r, &host);
    box->setFixedSize(640, 480);
    lay->addWidget(box);
  }
  bool showDocked() {
    host.show();
    return QTest::qWaitForWindowExposed(r);
  }
  bool floatOut() {
    box->setParent(nullptr);
    box->setFixedSize(640, 480);
    box->show();
    return QTest::qWaitForWindowExposed(r);
  }
  bool dock() {
    lay->addWidget(box); // re-parents into host
    host.show();
    box->show();
    return QTest::qWaitForWindowExposed(r);
  }
};

int visibleWindows() {
  int n = 0;
  for (QWindow *w : QGuiApplication::allWindows())
    if (w->isVisible())
      ++n;
  return n;
}

} // namespace

class TstSimGlContext : public QObject {
  Q_OBJECT
private slots:
  void initTestCase() {
    if (QGuiApplication::platformName() == "offscreen")
      QSKIP("the GL view needs a real platform: set QT_QPA_PLATFORM=xcb, DISPLAY "
            "to an Xvfb and LIBGL_ALWAYS_SOFTWARE=1");
    QSurfaceFormat fmt; // what the app asks for in main()
    fmt.setVersion(4, 6);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    fmt.setSamples(4);
    QSurfaceFormat::setDefaultFormat(fmt);
  }

  // Undock (float) and re-dock the view twice; every step must render the same
  // scene with no GL error, and no window may be left behind.
  void survivesFloatAndDock() {
    const int windowsBefore = visibleWindows();
    {
      Rig rig;
      QVERIFY(rig.showDocked());
      const Views ref = capture(rig.r);
      const char *why = nullptr;
      QVERIFY2(sameScene(ref, ref, &why), why);
      QVERIFY(ref.mesh != ref.chunk);
      const int live = rig.r->liveGlObjects();
      QVERIFY(live > 20);
      for (int round = 0; round < 2; ++round) {
        QVERIFY(rig.floatOut());
        QVERIFY2(sameScene(ref, capture(rig.r), &why), why);
        QCOMPARE(rig.r->liveGlObjects(), live);
        QVERIFY(rig.dock());
        QVERIFY2(sameScene(ref, capture(rig.r), &why), why);
        QCOMPARE(rig.r->liveGlObjects(), live);
      }
      qInfo("GL initialisations over 2 float/dock rounds: %d", rig.r->glInitCount());
      QCOMPARE(rig.r->glErrorCount(), 0);
    }
    QCoreApplication::processEvents();
    QCOMPARE(visibleWindows(), windowsBefore);
  }

  // The view follows its container: resized while shown, resized while hidden
  // (a workspace switch re-places panels with their windows hidden) and
  // masked/unmasked, every next frame is drawn for the window's current size.
  void viewportFollowsSize() {
    Rig rig;
    rig.box->setMinimumSize(100, 80);
    rig.box->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
    rig.box->setFixedSize(400, 300);
    rig.box->setMinimumSize(100, 80);
    rig.box->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
    QVERIFY(rig.showDocked());
    auto settle = [&] {
      rig.r->update();
      QTest::qWait(150);
      rig.r->update();
      QTest::qWait(150);
    };
    auto check = [&](const char *what) {
      const QSize want = rig.r->size() * rig.r->devicePixelRatio();
      // a frame is drawn asynchronously: give a loaded machine a few seconds
      QElapsedTimer waited;
      waited.start();
      while (rig.r->viewportSize() != want && waited.elapsed() < 4000) {
        rig.r->update();
        QTest::qWait(50);
      }
      const QSize got = rig.r->viewportSize();
      if (got != want)
        qWarning("%s: viewport %dx%d, window %dx%d", what, got.width(),
                 got.height(), want.width(), want.height());
      return got == want;
    };
    settle();
    QVERIFY(check("initial"));
    rig.host.resize(560, 420);
    settle();
    QCOMPARE(rig.r->size(), rig.box->size());
    QVERIFY(check("resized while shown"));
    // hidden: the way a switch re-places a panel
    rig.r->hide();
    rig.host.resize(700, 500);
    QTest::qWait(100);
    rig.r->show();
    settle();
    QCOMPARE(rig.r->size(), rig.box->size());
    QVERIFY(check("resized while hidden"));
    // a whole-container hide/resize/show
    rig.box->hide();
    rig.host.resize(480, 360);
    QTest::qWait(100);
    rig.box->show();
    settle();
    QCOMPARE(rig.r->size(), rig.box->size());
    QVERIFY(check("container hidden and resized"));
    // mask on and off
    rig.r->setMask(QRegion(0, 0, 200, 150));
    settle();
    rig.r->setMask(QRegion());
    settle();
    QVERIFY(check("mask changes"));
    // a cached size that went stale (a resize that never reached resizeGL)
    rig.r->setCachedViewportForTest(QSize(123, 456));
    settle();
    QVERIFY(check("stale cached size"));
  }

  // The context is torn down and rebuilt for real: the renderer frees every GL
  // object and builds them again (what Qt does when it recreates the context),
  // repeatedly and on the same context, and nothing leaks.
  void resetIsIdempotent() {
    Rig rig;
    QVERIFY(rig.showDocked());
    SimRendererWidget *r = rig.r;
    const Views ref = capture(r);
    const char *why = nullptr;
    const int live = r->liveGlObjects();
    const int inits = r->glInitCount();
    const GLuint name0 = probeBufferName(r);
    for (int i = 0; i < 6; ++i) {
      r->resetGlResources();
      QVERIFY2(sameScene(ref, capture(r), &why), why);
      QCOMPARE(r->liveGlObjects(), live);
    }
    QCOMPARE(r->glInitCount(), inits + 6);
    QVERIFY2(probeBufferName(r) <= name0 + 2, "GL buffer names keep growing: leak");
    QCOMPARE(r->glErrorCount(), 0);
  }

  // The context-destroyed path. Qt 6.4.2 does not destroy a QOpenGLWindow's
  // context on QWindow::destroy()/hide-show, float or dock (tried: gone and
  // init counts stay 0 / 1), so a native recreation cannot be provoked here.
  // What this does instead is fire the genuine QOpenGLContext::
  // aboutToBeDestroyed signal of the live context, which runs the renderer's
  // real connected slot (onContextGone) - the same code Qt runs when it does
  // destroy the context - then lets the next frame bring the renderer back up.
  // The streamed terrain / flora live only in the old VBOs, so the scene only
  // matches if the slot read them back.
  void contextGoneReadsBack() {
    const int base = SimRendererWidget::chunkReadBacks();
    {
      Rig rig;
      QVERIFY(rig.showDocked());
      const Views ref = capture(rig.r);
      QCOMPARE(rig.r->glInitCount(), 1);
      QCOMPARE(rig.r->contextGoneCount(), 0);
      QCOMPARE(SimRendererWidget::chunkReadBacks(), base);
      for (int round = 1; round <= 2; ++round) {
        QOpenGLContext *ctx = rig.r->context();
        QVERIFY(ctx);
        emit ctx->aboutToBeDestroyed(); // the real signal, real connection
        QCOMPARE(rig.r->contextGoneCount(), round); // the slot ran
        QVERIFY(rig.r->contextWasCurrentWhenGone()); // with the context current
        QCOMPARE(rig.r->liveGlObjects(), 0);          // everything released
        rig.r->update();
        rig.r->grabFrame(); // next frame re-creates the resources
        qInfo("round %d: gone=%d inits=%d readbacks=%d", round,
              rig.r->contextGoneCount(), rig.r->glInitCount(),
              SimRendererWidget::chunkReadBacks() - base);
        QCOMPARE(rig.r->glInitCount(), round + 1);
        QVERIFY(SimRendererWidget::chunkReadBacks() > base);
        const char *why = nullptr;
        QVERIFY2(sameScene(ref, capture(rig.r), &why), why);
      }
    }
    QCoreApplication::processEvents();
    const int afterRecreate = SimRendererWidget::chunkReadBacks();
    {
      Rig rig; // destroying a live renderer reads nothing
      QVERIFY(rig.showDocked());
      capture(rig.r);
    }
    QCoreApplication::processEvents();
    QCOMPARE(SimRendererWidget::chunkReadBacks(), afterRecreate);
  }

  // Teardown reads nothing back from the GPU; a reset (which a following
  // context needs) does, one per loaded chunk, unless the platform cannot
  // (GLES) - then a CPU copy made at upload time stands in for it.
  void readBackOnlyOnReset() {
    const int base = SimRendererWidget::chunkReadBacks();
    {
      Rig rig;
      QVERIFY(rig.showDocked());
      const Views ref = capture(rig.r);
      const char *why = nullptr;
      QCOMPARE(SimRendererWidget::chunkReadBacks(), base);
      rig.r->resetGlResources();
      QVERIFY2(sameScene(ref, capture(rig.r), &why), why);
      QVERIFY(SimRendererWidget::chunkReadBacks() > base); // reset reads back
    }
    QCoreApplication::processEvents();
    const int afterReset = SimRendererWidget::chunkReadBacks();
    {
      Rig rig; // destroying a live renderer must read nothing
      QVERIFY(rig.showDocked());
      capture(rig.r);
    }
    QCoreApplication::processEvents();
    QCOMPARE(SimRendererWidget::chunkReadBacks(), afterReset);
    {
      Rig rig(true); // the GLES path: copies kept, no read-back
      QVERIFY(rig.showDocked());
      const Views ref = capture(rig.r);
      const char *why = nullptr;
      rig.r->resetGlResources();
      rig.r->resetGlResources();
      QVERIFY2(sameScene(ref, capture(rig.r), &why), why);
    }
    QCOMPARE(SimRendererWidget::chunkReadBacks(), afterReset);
  }

  // The HUD image is re-rendered only when something changed, and this is the
  // cost of that on software GL (not a GPU number).
  void hudIsCached() {
    Rig rig(false, true);
    QVERIFY(rig.showDocked());
    SimRendererWidget *r = rig.r;
    r->grabFrame();
    r->resetHudStats();
    for (int i = 0; i < 5; ++i)
      r->grabFrame();
    QCOMPARE(r->hudRenderCount(), 0); // nothing changed: image reused

    rig.hud->setStatus(QStringLiteral("ARMED"));
    r->grabFrame();
    QCOMPARE(r->hudRenderCount(), 1);
    rig.hud->setStatus(QStringLiteral("ARMED")); // same value: not a change
    r->grabFrame();
    QCOMPARE(r->hudRenderCount(), 1);

    vsim::SimSnapshot s;
    s.pos_w = {0, 0, -3.0f};
    rig.hud->setSnapshot(s);
    r->grabFrame();
    QCOMPARE(r->hudRenderCount(), 2);
    rig.hud->setSnapshot(s); // identical telemetry: not a change
    r->grabFrame();
    QCOMPARE(r->hudRenderCount(), 2);

    rig.box->setFixedSize(600, 440); // size change: re-render
    QTest::qWait(100);
    r->grabFrame();
    QVERIFY(r->hudRenderCount() >= 3);

    r->setHudVisible(false); // hidden: not painted, not rendered
    r->resetHudStats();
    r->grabFrame();
    QCOMPARE(r->hudRenderCount(), 0);
    rig.hud->setStatus(QStringLiteral("DISARMED"));
    r->setHudVisible(true); // shown again: content may have moved meanwhile
    r->grabFrame();
    QCOMPARE(r->hudRenderCount(), 1);

    // Cost of paintHud per frame over 300 frames, cached vs rendering every
    // frame, with the HUD static and with it changing every frame.
    auto bench = [&](bool cache, bool changing) {
      r->setHudCache(cache);
      r->grabFrame();
      r->resetHudStats();
      for (int i = 0; i < 300; ++i) {
        if (changing) {
          s.pos_w = {0, 0, -3.0f - 0.01f * float(i)};
          rig.hud->setSnapshot(s);
        }
        r->grabFrame();
      }
      return std::pair<double, int>(double(r->hudPaintNs()) / 300.0 / 1e3,
                                    r->hudRenderCount());
    };
    const auto offStatic = bench(false, false);
    const auto onStatic = bench(true, false);
    const auto offMoving = bench(false, true);
    const auto onMoving = bench(true, true);
    r->setHudCache(true);
    qInfo("paintHud per grabFrame (2 paints), software GL, 300 grabs (us; HUD renders):");
    qInfo("  static HUD:   every-frame %.0f (%d)   cached %.0f (%d)", offStatic.first,
          offStatic.second, onStatic.first, onStatic.second);
    qInfo("  changing HUD: every-frame %.0f (%d)   cached %.0f (%d)", offMoving.first,
          offMoving.second, onMoving.first, onMoving.second);
    QCOMPARE(onStatic.second, 0);
    QVERIFY(onStatic.first < offStatic.first);
    QCOMPARE(onMoving.second, 300);
  }

  // Hiding the container hides the native window; showing it again renders.
  void survivesHideShow() {
    Rig rig;
    QVERIFY(rig.showDocked());
    const Views ref = capture(rig.r);
    const char *why = nullptr;
    rig.box->hide();
    QTest::qWait(100);
    rig.box->show();
    QVERIFY(QTest::qWaitForWindowExposed(rig.r));
    QVERIFY2(sameScene(ref, capture(rig.r), &why), why);
    QCOMPARE(rig.r->glErrorCount(), 0);
  }
};

QTEST_MAIN(TstSimGlContext)
#include "tst_sim_gl_context.moc"
