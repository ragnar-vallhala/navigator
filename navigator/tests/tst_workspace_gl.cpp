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
// Does a QOpenGLWidget keep rendering while ADS floats it, re-docks it and
// rebuilds the layout around it? Needs a real GL platform plugin: run with
// QT_QPA_PLATFORM=xcb on an Xvfb (LIBGL_ALWAYS_SOFTWARE=1); on "offscreen"
// (ctest's default) it skips, because QOpenGLWidget crashes there.
#include "AdsBackend.h"
#include "AttitudeWidget.h"
#include "Drone3DWidget.h"
#include "GlPanel.h"
#include "WorkspaceManager.h"

#include <QGuiApplication>
#include <QLabel>
#include <QMainWindow>
#include <QSettings>
#include <QTemporaryDir>
#include <QtTest>
#include <functional>

using namespace ws;

namespace {
// Qt reports a main window that cannot present its GL content as these two
// warnings, once per frame.
int g_glWarnings = 0;
QtMessageHandler g_prev = nullptr;
void countGlWarnings(QtMsgType t, const QMessageLogContext &c, const QString &m) {
  if (m.contains("non-opengl surface") || m.contains("Failed to make context current"))
    ++g_glWarnings;
  else if (g_prev)
    g_prev(t, c, m);
}

struct Fixture {
  QMainWindow win;
  WorkspaceManager *wm;
  QWidget *gl = nullptr;
  std::function<int()> frames, inits, destroyed;
  std::function<QImage()> grab;
  template <class P> QWidget *make() {
    auto *p = new P;
    gl = p;
    frames = [p] { return p->frames(); };
    inits = [p] { return p->initCount(); };
    destroyed = [p] { return p->ctxDestroyed(); };
    grab = [p] { return p->grabFramebuffer(); };
    return p;
  }
  explicit Fixture(bool window) {
    win.resize(1000, 700);
    wm = new WorkspaceManager(&win, std::make_unique<AdsBackend>(&win), &win);
    PanelRegistry r;
    r.add("gl", "GL", [this, window]() -> QWidget * {
      return window ? this->make<GlWindowPanel>() : this->make<GlPanel>();
    });
    for (const char *id : {"a", "b", "c"})
      r.add(id, QString(id).toUpper(), [id] { return new QLabel(id); });
    wm->registerPanels(r);
    using N = LayoutNode;
    wm->setBuiltins(
        {{"One", {N::split(Qt::Horizontal, {N::tabs({"gl"}), N::tabs({"a", "b"})})}},
         {"Two", {N::split(Qt::Vertical, {N::tabs({"c"}), N::tabs({"gl", "a"}, "gl")})}}});
    wm->setScreenProvider([] { return QList<QRect>{QRect(0, 0, 1600, 900)}; });
    win.show();
    wm->load("One");
  }
  // Let the animation run; true if frames advanced and pixels are real.
  bool renders(const char *what) {
    const int before = frames();
    QTest::qWait(400);
    const QImage img = grab();
    const bool moved = frames() > before + 3;
    const bool real = !img.isNull() && img.width() > 10 && img.pixelColor(img.width() / 2, img.height() - 8) != QColor(0, 0, 0);
    int tops = 0; // visible top-level windows other than the main one
    for (QWidget *w : QApplication::topLevelWidgets())
      if (w != &win && w->isVisible())
        ++tops;
    qInfo("%-18s frames %d -> %d  init %d  ctx-destroyed %d  %dx%d  in-main-window %d  extra-top-levels %d  %s",
          what, before, frames(), inits(), destroyed(),
          img.width(), img.height(), gl->window() == &win, tops,
          (moved && real) ? "OK" : "NOT RENDERING");
    return moved && real;
  }
};
} // namespace

class TstWorkspaceGl : public QObject {
  Q_OBJECT
  QTemporaryDir m_dir;
private slots:
  void initTestCase() {
    if (QGuiApplication::platformName() == "offscreen")
      QSKIP("QOpenGLWidget needs a real platform: set QT_QPA_PLATFORM=xcb, "
            "DISPLAY to an Xvfb and LIBGL_ALWAYS_SOFTWARE=1");
    QCoreApplication::setOrganizationName("NAVRobotecTest");
    QCoreApplication::setApplicationName("tst_workspace_gl");
    QSettings::setDefaultFormat(QSettings::IniFormat);
    QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, m_dir.path());
  }

  // Runs the full float / move / re-dock / switch / load / reset sequence and
  // reports the number of GL presentation warnings.
  void runSequence(bool window, int &warnings) {
    g_prev = qInstallMessageHandler(countGlWarnings);
    g_glWarnings = 0;
    Fixture f(window);
    auto *b = f.wm->backend();
    QVERIFY(f.renders("docked"));
    b->floatPanel("gl");
    QVERIFY(b->panelState("gl").floating);
    QVERIFY(f.renders("floated"));
    b->setFloatGeometry("gl", QRect(300, 200, 420, 320));
    QVERIFY(f.renders("floated+moved"));
    b->tabPanel("gl", "a"); // re-dock by tabbing into another group
    QVERIFY(!b->panelState("gl").floating);
    QVERIFY(f.renders("re-docked (tab)"));
    b->floatPanel("gl");
    QVERIFY(f.renders("floated again"));
    for (int i = 0; i < 6; ++i) { // workspace switches rebuild the manager
      f.wm->load(i % 2 ? "One" : "Two");
      QVERIFY(f.renders(i % 2 ? "switch -> One" : "switch -> Two"));
    }
    QVERIFY(f.wm->saveCurrentAs("Mine"));
    b->floatPanel("gl");
    f.wm->load("Mine");
    QVERIFY(f.renders("load saved"));
    f.wm->resetLayout();
    QVERIFY(f.renders("reset"));
    qInstallMessageHandler(g_prev);
    qInfo("GL presentation warnings (%s): %d", window ? "window container" : "QOpenGLWidget", g_glWarnings);
    warnings = g_glWarnings;
  }

  // The attitude panel of the real window: the real AttitudeWidget and
  // Drone3DWidget inside docks. Workspace switches, a reset and a restore (what
  // a startup does) rebuild the dock manager; each widget must keep its one GL
  // context (initializeGL once, no context destroyed).
  void realAttitudeWidgetsKeepGlContext() {
    QMainWindow win;
    win.resize(1000, 700);
    auto *wm = new WorkspaceManager(&win, std::make_unique<AdsBackend>(&win), &win);
    auto *att = new AttitudeWidget;
    auto *d3 = new Drone3DWidget;
    PanelRegistry r;
    r.add("att", "Att", [att]() -> QWidget * { return att; });
    r.add("d3", "3D", [d3]() -> QWidget * { return d3; });
    for (const char *id : {"a", "b"})
      r.add(id, QString(id).toUpper(), [id] { return new QLabel(id); });
    wm->registerPanels(r);
    using N = LayoutNode;
    wm->setBuiltins({{"One", {N::split(Qt::Horizontal, {N::tabs({"att", "d3"}), N::tabs({"a"})})}},
                     {"Two", {N::split(Qt::Vertical, {N::tabs({"b"}), N::tabs({"d3", "att"}, "att")})}}});
    wm->setScreenProvider([] { return QList<QRect>{QRect(0, 0, 1600, 900)}; });
    win.show();
    wm->load("One");
    QTest::qWait(400);
    // d3 is the hidden tab at first: show it so both contexts exist.
    wm->backend()->showPanel("d3");
    QTest::qWait(300);
    wm->backend()->showPanel("att");
    QTest::qWait(300);
    QCOMPARE(att->glInitCount(), 1);
    QCOMPARE(d3->glInitCount(), 1);
    for (int i = 0; i < 4; ++i) {
      wm->load(i % 2 ? "One" : "Two");
      QTest::qWait(150);
    }
    wm->resetLayout();
    wm->persistLast();
    wm->restoreLast();
    wm->backend()->showPanel("d3");
    QTest::qWait(300);
    wm->backend()->showPanel("att");
    QTest::qWait(300);
    QCOMPARE(att->glInitCount(), 1);
    QCOMPARE(d3->glInitCount(), 1);
  }

  // The supported combination: ADS + QOpenGLWindow in a window container.
  void windowContainerKeepsRendering() {
    int w = -1;
    runSequence(true, w);
    QCOMPARE(w, 0);
  }

  // A QOpenGLWidget under ADS re-runs initializeGL() on every re-parent and the
  // main window then stops presenting (a screenshot shows stale pixels and a
  // ghost of the floating window; also with Qt::AA_ShareOpenGLContexts and with
  // a window manager). Plain QDockWidget with the same panel is clean, and ADS
  // 5.1.1 shows it in a 30-line program with no WorkspaceManager. Flip this
  // when it is fixed upstream.
  void openGlWidgetStopsPresenting() {
    int w = -1;
    runSequence(false, w);
    QEXPECT_FAIL("", "ADS + QOpenGLWidget: main window stops presenting (Qt 6.4.2, xcb, software GL)", Continue);
    QCOMPARE(w, 0);
  }
};

QTEST_MAIN(TstWorkspaceGl)
#include "tst_workspace_gl.moc"
