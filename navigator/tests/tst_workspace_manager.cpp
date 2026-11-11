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
// WorkspaceManager on the real AdsBackend (offscreen): built-ins, save / load /
// delete / rename, persistence across a "restart", corrupt or older saved
// state falling back to the default, and the off-screen clamp.
#include "AdsBackend.h"
#include "ScreenClamp.h"
#include "WorkspaceManager.h"

#include <QLabel>
#include <QMainWindow>
#include <QPointer>
#include <QSettings>
#include <QSignalSpy>
#include <QTemporaryDir>
#include <QtTest>

using namespace ws;

namespace {
PanelRegistry panels() {
  PanelRegistry r;
  for (const char *id : {"a", "b", "c", "d", "e"}) {
    const QString s = QString::fromLatin1(id);
    r.add(s, s.toUpper(), [s] { return new QLabel(s); });
  }
  return r;
}

std::vector<WorkspaceManager::Builtin> builtins() {
  using N = LayoutNode;
  std::vector<WorkspaceManager::Builtin> b;
  b.push_back({"Fly", {N::split(Qt::Horizontal,
                                {N::tabs({"a", "b"}, "a"), N::tabs({"c"})})}});
  b.push_back({"Sim", {N::split(Qt::Vertical, {N::tabs({"d"}), N::tabs({"e"})})}});
  return b;
}

// One window + manager, as the app would build them at start.
struct Fixture {
  QMainWindow win;
  WorkspaceManager *wm;
  Fixture() {
    win.resize(900, 600);
    wm = new WorkspaceManager(&win, std::make_unique<AdsBackend>(&win), &win);
    wm->registerPanels(panels());
    wm->setBuiltins(builtins());
    wm->setScreenProvider([] { return QList<QRect>{QRect(0, 0, 1600, 900)}; });
    win.show();
  }
};
} // namespace

class TstWorkspaceManager : public QObject {
  Q_OBJECT
  QTemporaryDir m_dir;

private slots:
  void initTestCase() {
    QVERIFY(m_dir.isValid());
    QCoreApplication::setOrganizationName("NAVRobotecTest");
    QCoreApplication::setApplicationName("tst_workspace_manager");
    QSettings::setDefaultFormat(QSettings::IniFormat);
    QSettings::setPath(QSettings::IniFormat, QSettings::UserScope,
                       m_dir.path());
  }
  void init() { QSettings().clear(); }

  void builtinsAreDefaultsAndReadOnly() {
    Fixture f;
    QCOMPARE(f.wm->builtinNames(), (QStringList{"Fly", "Sim"}));
    QCOMPARE(f.wm->defaultName(), QString("Fly"));
    QVERIFY(f.wm->load("Fly"));
    QCOMPARE(f.wm->currentName(), QString("Fly"));
    auto *b = f.wm->backend();
    QVERIFY(b->panelState("a").visible);
    QCOMPARE(b->panelState("a").tabGroup, (QStringList{"a", "b"}));
    QVERIFY(b->panelState("c").visible);
    QVERIFY(!b->panelState("d").visible); // not in Fly: hidden
    QVERIFY(!b->panelState("a").floating);
    QVERIFY(f.wm->load("Sim"));
    QVERIFY(b->panelState("d").visible);
    QVERIFY(!b->panelState("a").visible);
    // built-ins cannot be overwritten, deleted or renamed
    QVERIFY(!f.wm->saveCurrentAs("Fly"));
    QVERIFY(!f.wm->saveCurrentAs("fly"));
    QVERIFY(!f.wm->remove("Fly"));
    QVERIFY(!f.wm->rename("Fly", "Mine"));
    QVERIFY(f.wm->userNames().isEmpty());
  }

  // The manager may outlive the window (it is not always a child of it). The
  // host then takes the dock manager and every dock with it; the backend must
  // notice and neither touch the dead host nor delete the manager twice.
  void hostDeletedBeforeManager() {
    auto *win = new QMainWindow;
    win->resize(900, 600);
    auto *wm = new WorkspaceManager(win, std::make_unique<AdsBackend>(win),
                                    nullptr);
    wm->registerPanels(panels());
    wm->setBuiltins(builtins());
    wm->setScreenProvider([] { return QList<QRect>{QRect(0, 0, 1600, 900)}; });
    win->show();
    QVERIFY(wm->load("Fly"));
    QPointer<QWidget> dockManager = win->centralWidget();
    QVERIFY(dockManager); // the layout really lives under the host
    delete win;           // the host goes first, taking the dock manager along
    QVERIFY(dockManager.isNull());
    delete wm; // must not crash: no dead-host call, no double delete
  }

  void saveLoadRestoresDockState() {
    Fixture f;
    auto *b = f.wm->backend();
    f.wm->load("Fly");
    b->floatPanel("c");
    b->setFloatGeometry("c", QRect(100, 120, 400, 300));
    b->tabPanel("e", "a");
    QVERIFY(b->panelState("c").floating);
    QCOMPARE(b->panelState("a").tabGroup, (QStringList{"a", "b", "e"}));
    QVERIFY(f.wm->saveCurrentAs("Mine"));
    QCOMPARE(f.wm->currentName(), QString("Mine"));
    QCOMPARE(f.wm->userNames(), QStringList{"Mine"});

    f.wm->load("Sim"); // something clearly different
    QVERIFY(!b->panelState("c").floating);

    QVERIFY(f.wm->load("Mine"));
    QCOMPARE(f.wm->currentName(), QString("Mine"));
    QVERIFY(b->panelState("c").floating);
    QCOMPARE(b->panelState("c").floatGeometry.size(), QSize(400, 300));
    QCOMPARE(b->panelState("a").tabGroup, (QStringList{"a", "b", "e"}));
    QVERIFY(!b->panelState("d").visible);
    QVERIFY(!b->panelState("a").floating);
  }

  void overwriteDeleteRename() {
    Fixture f;
    auto *b = f.wm->backend();
    f.wm->load("Fly");
    QVERIFY(f.wm->saveCurrentAs("One"));
    b->hidePanel("c");
    QVERIFY(f.wm->saveCurrentAs("One")); // overwrite
    QCOMPARE(f.wm->userNames(), QStringList{"One"});
    f.wm->load("Sim");
    QVERIFY(f.wm->load("One"));
    QVERIFY(!b->panelState("c").visible);

    QVERIFY(!f.wm->saveCurrentAs(""));
    QVERIFY(!f.wm->saveCurrentAs("  "));
    QVERIFY(!f.wm->saveCurrentAs(" padded"));

    f.wm->saveCurrentAs("Two");
    QVERIFY(!f.wm->rename("One", "Two")); // taken
    QVERIFY(!f.wm->rename("One", "Sim")); // built-in name
    QVERIFY(!f.wm->rename("Nope", "X"));
    QVERIFY(f.wm->rename("One", "Uno"));
    QCOMPARE(f.wm->userNames(), (QStringList{"Two", "Uno"}));
    QVERIFY(!f.wm->load("One"));
    QVERIFY(f.wm->load("Uno"));

    QVERIFY(f.wm->remove("Uno"));
    QVERIFY(!f.wm->remove("Uno"));
    QVERIFY(!f.wm->load("Uno"));
    QCOMPARE(f.wm->userNames(), QStringList{"Two"});
    QVERIFY(f.wm->currentName().isEmpty()); // the active one was deleted
    // names that look like paths or unicode survive
    QVERIFY(f.wm->saveCurrentAs("a/b\\c ✈"));
    QVERIFY(f.wm->userNames().contains("a/b\\c ✈"));
  }

  void resetAlwaysWorks() {
    Fixture f;
    auto *b = f.wm->backend();
    f.wm->load("Sim");
    b->floatPanel("d");
    b->hidePanel("e");
    f.wm->resetLayout(); // a built-in resets itself
    QCOMPARE(f.wm->currentName(), QString("Sim"));
    QVERIFY(!b->panelState("d").floating);
    QVERIFY(b->panelState("e").visible);
    f.wm->load("Sim");
    f.wm->saveCurrentAs("Mine");
    f.wm->resetLayout(); // a user one resets to the default
    QCOMPARE(f.wm->currentName(), QString("Fly"));
    QVERIFY(b->panelState("a").visible);
    // a window with nothing applied yet: reset still produces a layout
    Fixture g;
    g.wm->resetLayout();
    QVERIFY(g.wm->backend()->panelState("a").visible);
  }

  void lastLayoutSurvivesRestart() {
    {
      Fixture f;
      auto *b = f.wm->backend();
      f.wm->load("Fly");
      b->floatPanel("b");
      b->setFloatGeometry("b", QRect(60, 70, 333, 222));
      f.wm->persistLast();
    }
    Fixture g; // "restart"
    QVERIFY(g.wm->restoreLast());
    auto *b = g.wm->backend();
    QVERIFY(b->panelState("b").floating);
    QCOMPARE(b->panelState("b").floatGeometry.size(), QSize(333, 222));
    QVERIFY(b->panelState("a").visible);
    QCOMPARE(g.wm->currentName(), QString("Fly"));
  }

  void firstStartUsesDefault() {
    Fixture f;
    QSignalSpy failed(f.wm, &WorkspaceManager::loadFailed);
    QVERIFY(!f.wm->restoreLast()); // nothing saved
    QCOMPARE(failed.count(), 0);   // not an error
    QCOMPARE(f.wm->currentName(), QString("Fly"));
    QVERIFY(f.wm->backend()->panelState("a").visible);
  }

  // ---- corrupt / older / foreign saved state: default, no crash ----------
  void corruptOrOldSavedStateFallsBack_data() {
    QTest::addColumn<QByteArray>("state");
    QTest::addColumn<int>("format");
    QTest::addColumn<QString>("backend");
    const QString ok = "ads-v1";
    QTest::newRow("garbage") << QByteArray("not a layout at all") << 1 << ok;
    QTest::newRow("binary noise")
        << QByteArray("\x00\xff\x01\x02\x03\x04\x80\x7f", 8) << 1 << ok;
    QTest::newRow("truncated xml") << QByteArray("<?xml version=\"1.0\"?><ADS") << 1 << ok;
    QTest::newRow("empty") << QByteArray() << 1 << ok;
    QTest::newRow("older format") << QByteArray("whatever") << 0 << ok;
    QTest::newRow("newer format") << QByteArray("whatever") << 99 << ok;
    QTest::newRow("other backend") << QByteArray("whatever") << 1 << QString("qdock-v1");
  }
  void corruptOrOldSavedStateFallsBack() {
    QFETCH(QByteArray, state);
    QFETCH(int, format);
    QFETCH(QString, backend);
    Fixture f;
    f.wm->load("Fly");
    f.wm->saveCurrentAs("Good");
    const QString grp = "workspaces/user/" + QString::fromLatin1(QByteArray("Good").toHex());
    {
      QSettings q;
      q.setValue(grp + "/state", state);
      q.setValue(grp + "/format", format);
      q.setValue(grp + "/backend", backend);
      q.sync();
    }
    f.wm->load("Sim");
    QSignalSpy failed(f.wm, &WorkspaceManager::loadFailed);
    QVERIFY(!f.wm->load("Good")); // reports failure ...
    QCOMPARE(failed.count(), 1);
    QCOMPARE(f.wm->currentName(), QString("Fly")); // ... and shows the default
    QVERIFY(f.wm->backend()->panelState("a").visible);
    QVERIFY(f.wm->backend()->panelState("c").visible);

    // the same blob as the "last layout" at start
    {
      QSettings q;
      q.setValue("workspaces/last/state", state);
      q.setValue("workspaces/last/format", format);
      q.setValue("workspaces/last/backend", backend);
      q.setValue("workspaces/last/geometry", QByteArray("junk geometry"));
      q.sync();
    }
    Fixture g;
    QVERIFY(!g.wm->restoreLast());
    QCOMPARE(g.wm->currentName(), QString("Fly"));
    QVERIFY(g.wm->backend()->panelState("a").visible);
    QVERIFY(g.wm->backend()->panelState("c").visible);
    // and the app can carry on: a new layout can be saved and loaded
    g.wm->backend()->floatPanel("c");
    QVERIFY(g.wm->saveCurrentAs("After"));
    QVERIFY(g.wm->load("After"));
    QVERIFY(g.wm->backend()->panelState("c").floating);
  }

  void oldWindowStateKeysAreIgnored() {
    // Navigator today stores ui/windowState (QMainWindow::saveState); a manager
    // starting on such a profile must just use the default.
    QSettings q;
    q.setValue("ui/windowState", QByteArray("\x00\x00\x00\xff\x00\x00", 6));
    q.setValue("ui/geometry", QByteArray("junk"));
    q.sync();
    Fixture f;
    QVERIFY(!f.wm->restoreLast());
    QVERIFY(f.wm->backend()->panelState("a").visible);
  }

  // ---- off-screen windows -------------------------------------------------
  void clampRect() {
    const QList<QRect> one{QRect(0, 0, 1600, 900)};
    const QRect ok(100, 100, 400, 300);
    QCOMPARE(clampRectToScreens(ok, one), ok);
    // far right/below, far left, above the top (title bar unreachable)
    for (const QRect &bad : {QRect(5000, 5000, 400, 300), QRect(-3000, 50, 400, 300),
                             QRect(200, -500, 400, 300), QRect(1590, 880, 400, 300)}) {
      const QRect c = clampRectToScreens(bad, one);
      QVERIFY2(one[0].contains(c), qPrintable(QStringLiteral("%1,%2").arg(c.x()).arg(c.y())));
      QCOMPARE(c.size(), bad.size());
    }
    // larger than the screen: shrunk to fit
    QVERIFY(one[0].contains(clampRectToScreens(QRect(-50, -50, 4000, 3000), one)));
    // second monitor unplugged: a rect on it lands on the remaining one
    const QList<QRect> two{QRect(0, 0, 1600, 900), QRect(1600, 0, 1920, 1080)};
    const QRect onSecond(2000, 100, 500, 400);
    QCOMPARE(clampRectToScreens(onSecond, two), onSecond);
    QVERIFY(one[0].contains(clampRectToScreens(onSecond, one)));
    // no screen info at all: leave it alone
    QCOMPARE(clampRectToScreens(onSecond, {}), onSecond);
  }

  void offscreenFloatingWindowIsPulledBack() {
    Fixture f;
    auto *b = f.wm->backend();
    f.wm->load("Fly");
    b->floatPanel("c");
    b->setFloatGeometry("c", QRect(9000, 7000, 400, 300));
    QVERIFY(f.wm->clampToScreens() >= 1);
    const QRect g = b->panelState("c").floatGeometry;
    QVERIFY2(QRect(0, 0, 1600, 900).contains(g),
             qPrintable(QStringLiteral("%1,%2 %3x%4").arg(g.x()).arg(g.y()).arg(g.width()).arg(g.height())));
    QCOMPARE(g.size(), QSize(400, 300));
  }

  void offscreenWindowRestoredFromSavedLayoutIsPulledBack() {
    Fixture f;
    auto *b = f.wm->backend();
    f.wm->load("Fly");
    b->floatPanel("c");
    b->setFloatGeometry("c", QRect(2500, 100, 400, 300)); // on a 2nd monitor
    f.wm->saveCurrentAs("TwoMon");
    // restart on a machine with one smaller screen
    Fixture g;
    g.wm->setScreenProvider([] { return QList<QRect>{QRect(0, 0, 1280, 720)}; });
    QVERIFY(g.wm->load("TwoMon"));
    const QRect r = g.wm->backend()->panelState("c").floatGeometry;
    QVERIFY2(QRect(0, 0, 1280, 720).contains(r),
             qPrintable(QStringLiteral("%1,%2").arg(r.x()).arg(r.y())));
  }
};

QTEST_MAIN(TstWorkspaceManager)
#include "tst_workspace_manager.moc"
