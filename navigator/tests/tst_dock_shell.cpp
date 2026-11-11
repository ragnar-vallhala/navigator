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
// The dock host of the main window (DockShell), without the real pages: the
// panel ids and built-in workspaces are complete and consistent, the switch
// reads env + setting, showPanel is a no-op for an unknown id, shows without
// floating, and the panel that holds QOpenGLWidgets cannot be floated.
#include "DockShell.h"

#include <QLabel>
#include <QMainWindow>
#include <QSettings>
#include <QSignalSpy>
#include <QTemporaryDir>
#include <QtTest>

namespace {
QStringList allIds() {
  QStringList l;
  for (const auto &p : DockShell::panelInfos())
    l << p.id;
  return l;
}

// A window with a started shell whose panels are labels (as many as asked).
struct Fixture {
  QMainWindow win;
  DockShell *shell;
  explicit Fixture(const QStringList &ids = allIds()) {
    win.resize(1200, 800);
    shell = new DockShell(&win);
    for (const QString &id : ids)
      shell->addPanel(id, new QLabel(id));
    shell->start();
    win.show();
  }
};
} // namespace

class TstDockShell : public QObject {
  Q_OBJECT
  QTemporaryDir m_dir;

private slots:
  void initTestCase() {
    QVERIFY(m_dir.isValid());
    QCoreApplication::setOrganizationName("NAVRobotecTest");
    QCoreApplication::setApplicationName("tst_dock_shell");
    QSettings::setDefaultFormat(QSettings::IniFormat);
    QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, m_dir.path());
  }
  void init() {
    QSettings().clear();
    qunsetenv("NAVIGATOR_DOCKING");
  }

  void panelIdsAreTheAgreedOnes() {
    const QStringList want = {"view3d", "imu",         "logs",       "rc",
                              "packets", "calibration", "motors",    "control_loop",
                              "gyro_notch", "perf",     "sim",       "settings"};
    QCOMPARE(allIds(), want);
    QCOMPARE(QSet<QString>(want.begin(), want.end()).size(), want.size());
    for (const auto &p : DockShell::panelInfos())
      QVERIFY2(!p.title.isEmpty(), qPrintable(p.id));
  }

  void builtinWorkspacesAreCompleteAndConsistent() {
    const auto b = DockShell::builtins();
    QStringList names;
    for (const auto &w : b)
      names << w.name;
    QCOMPARE(names, (QStringList{"Fly", "Tune", "Debug link", "Sim"})); // Fly = default

    const QStringList ids = allIds();
    QMap<QString, QStringList> want = {
        {"Fly", {"view3d", "imu", "logs"}},
        {"Tune", {"control_loop", "gyro_notch", "motors", "calibration", "logs"}},
        {"Debug link", {"packets", "logs", "perf", "rc"}},
        {"Sim", {"sim", "logs"}}};
    for (const auto &w : b) {
      QStringList ref = DockShell::referencedIds(w.spec);
      for (const QString &id : ref)
        QVERIFY2(ids.contains(id), qPrintable(w.name + " names unknown " + id));
      QSet<QString> uniq(ref.begin(), ref.end());
      QCOMPARE(uniq.size(), ref.size()); // no panel placed twice
      ref.sort();
      QStringList exp = want.value(w.name);
      exp.sort();
      QCOMPARE(ref, exp);
    }
  }

  void switchReadsEnvAndSetting() {
    QVERIFY(DockShell::compiledIn());
    QVERIFY(!DockShell::enabled());
    for (const char *v : {"1", "true", "ON", "yes"}) {
      qputenv("NAVIGATOR_DOCKING", v);
      QVERIFY2(DockShell::enabled(), v);
    }
    for (const char *v : {"", "0", "off", "no", "2"}) {
      qputenv("NAVIGATOR_DOCKING", v);
      QVERIFY2(!DockShell::enabled(), v);
    }
    qunsetenv("NAVIGATOR_DOCKING");
    DockShell::setPersisted(true);
    QVERIFY(DockShell::persistedRequested());
    QVERIFY(DockShell::enabled());
    DockShell::setPersisted(false);
    QVERIFY(!DockShell::enabled());
  }

  void everyBuiltinAppliesAndShowsItsPanels() {
    Fixture f;
    auto *wm = f.shell->workspaces();
    for (const auto &w : DockShell::builtins()) {
      QVERIFY(wm->load(w.name));
      for (const QString &id : DockShell::referencedIds(w.spec)) {
        const auto st = wm->backend()->panelState(id);
        QVERIFY2(st.visible && !st.floating, qPrintable(w.name + "/" + id));
      }
    }
  }

  void showPanelUnknownIdIsNoOp() {
    Fixture f;
    QSignalSpy spy(f.shell, &DockShell::panelShown);
    const auto before = f.shell->workspaces()->backend()->saveState();
    QVERIFY(!f.shell->showPanel("no_such_panel"));
    QVERIFY(!f.shell->showPanel(""));
    QCOMPARE(spy.count(), 0);
    QCOMPARE(f.shell->workspaces()->backend()->saveState(), before);
    // a known id of this build that was never registered (no simulator)
    Fixture g(QStringList{"view3d", "logs"});
    QVERIFY(!g.shell->hasPanel("sim"));
    QVERIFY(!g.shell->showPanel("sim"));
  }

  void showPanelOpensWithoutFloating() {
    Fixture f;
    auto *b = f.shell->workspaces()->backend();
    QVERIFY(!b->panelState("settings").visible); // not in Fly
    QSignalSpy spy(f.shell, &DockShell::panelShown);
    QVERIFY(f.shell->showPanel("settings"));
    QCOMPARE(spy.count(), 1);
    QCOMPARE(spy.first().first().toString(), QString("settings"));
    QVERIFY(b->panelState("settings").visible);
    QVERIFY(!b->panelState("settings").floating);
    QVERIFY(f.shell->showPanel("logs")); // already visible: stays put
    QVERIFY(!b->panelState("logs").floating);
    b->hidePanel("rc");
    QVERIFY(f.shell->showPanel("rc"));
    QVERIFY(b->panelState("rc").visible);
  }

  void panelsMenuActionsFollowVisibility() {
    Fixture f;
    auto *b = f.shell->workspaces()->backend();
    QVERIFY(b->viewAction("logs")->isChecked());
    QVERIFY(!b->viewAction("settings")->isChecked());
    f.shell->showPanel("settings");
    QVERIFY(b->viewAction("settings")->isChecked());
    b->viewAction("logs")->trigger(); // unchecks: hides
    QVERIFY(!b->panelState("logs").visible);
    QVERIFY(f.shell->workspaces()->load("Tune"));
    QVERIFY(b->viewAction("logs")->isChecked());
    QVERIFY(!b->viewAction("settings")->isChecked());
  }

  // The Panels menu opening a panel counts as a show (MRU list in the window).
  void panelsMenuOpenEmitsPanelShown() {
    Fixture f;
    auto *b = f.shell->workspaces()->backend();
    QSignalSpy spy(f.shell, &DockShell::panelShown);
    b->viewAction("settings")->trigger(); // opens
    QCOMPARE(spy.count(), 1);
    QCOMPARE(spy.at(0).at(0).toString(), QString("settings"));
    b->viewAction("settings")->trigger(); // closes: not a show
    QCOMPARE(spy.count(), 1);
  }

  // start(false) builds nothing; the one restoreLast() then applies the layout
  // (startup: the saved layout once, not default and then saved).
  void startWithoutLayoutThenRestoreLastBuildsOnce() {
    QMainWindow win;
    DockShell shell(&win);
    for (const QString &id : allIds())
      shell.addPanel(id, new QLabel(id));
    shell.start(false);
    QVERIFY(!shell.workspaces()->backend()->panelState("logs").visible);
    shell.workspaces()->persistLast(); // nothing built: must not save junk
    win.show();
    shell.restoreLast(); // nothing saved: the default
    QCOMPARE(shell.workspaces()->currentName(), shell.workspaces()->defaultName());
    QVERIFY(shell.workspaces()->backend()->panelState("view3d").visible);
  }

  void attitudePanelCannotFloat() {
    Fixture f;
    auto *b = f.shell->workspaces()->backend();
    QVERIFY(b->panelState("view3d").fixed);
    QVERIFY(!b->panelState("logs").fixed);
    b->floatPanel("view3d");
    QVERIFY(b->panelState("view3d").visible);
    QVERIFY(!b->panelState("view3d").floating);
    b->floatPanel("logs"); // others still float
    QVERIFY(b->panelState("logs").floating);
    // a rebuild (workspace switch) keeps it pinned
    QVERIFY(f.shell->workspaces()->load("Fly"));
    b->floatPanel("view3d");
    QVERIFY(!b->panelState("view3d").floating);
  }
};

QTEST_MAIN(TstDockShell)
#include "tst_dock_shell.moc"
