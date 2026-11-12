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
#include <QSplitter>
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
// The stored layout blob of a user view, read straight from the settings.
QByteArray storedState(const QString &name) {
  QSettings q;
  return q.value("workspaces/user/" + QString::fromLatin1(name.toUtf8().toHex()) +
                 "/state")
      .toByteArray();
}

QByteArray storedLast() {
  return QSettings().value("workspaces/last/state").toByteArray();
}

// A backend whose capture can be made empty (a failed capture).
class FlakyBackend : public AdsBackend {
public:
  using AdsBackend::AdsBackend;
  bool empty = false;
  QByteArray saveState() const override {
    return empty ? QByteArray() : AdsBackend::saveState();
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

  // A saved layout is restored onto the same dock manager and the same panel
  // widgets, with the identical state blob.
  void savedLayoutRestoresOnTheSameManager() {
    Fixture f;
    QVERIFY(f.wm->load("Fly"));
    QWidget *dm = f.win.centralWidget();
    QPointer<QWidget> a;
    for (QLabel *l : f.win.findChildren<QLabel *>())
      if (l->text() == "a")
        a = l;
    QVERIFY(a);
    QVERIFY(f.wm->saveCurrentAs("Mine"));
    const QByteArray mine = f.wm->backend()->saveState();
    QVERIFY(f.wm->load("Sim"));
    QVERIFY(!f.wm->backend()->panelState("a").visible);
    QVERIFY(f.wm->load("Mine"));
    QCOMPARE(f.win.centralWidget(), dm);
    QCOMPARE(f.wm->backend()->saveState(), mine);
    QVERIFY(a);
    QVERIFY(f.wm->backend()->panelState("a").visible);
  }

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
    const QString ok = "ads-v2";
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

  // A perfectly valid layout saved as format 1 (what the mock wrote with its own
  // panel ids) must be refused: the format number was bumped for the real window.
  void validLayoutOfOlderFormatIsRefused() {
    Fixture f;
    f.wm->load("Fly");
    f.wm->saveCurrentAs("Good");
    const QString grp = "workspaces/user/" + QString::fromLatin1(QByteArray("Good").toHex());
    QSettings q;
    QCOMPARE(q.value(grp + "/format").toInt(), WorkspaceManager::kFormat);
    QVERIFY(WorkspaceManager::kFormat > 1);
    q.setValue(grp + "/format", 1);
    q.sync();
    f.wm->load("Sim");
    QSignalSpy failed(f.wm, &WorkspaceManager::loadFailed);
    QVERIFY(!f.wm->load("Good"));
    QCOMPARE(failed.count(), 1);
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

  // ---- the last layout is kept (debounced) and restored at start ----------

  // (a) an edit made through the real backend is stored with no save call, and
  // a new manager on the same settings ("restart") restores it as edited.
  void editedLayoutIsDebouncedAndRestored() {
    {
      Fixture f;
      auto *b = f.wm->backend();
      f.wm->setLastDelay(50);
      f.wm->load("Fly");
      QSignalSpy saved(f.wm, &WorkspaceManager::lastSaved);
      b->floatPanel("c");
      b->setFloatGeometry("c", QRect(100, 120, 400, 300));
      b->hidePanel("b");
      QTRY_VERIFY_WITH_TIMEOUT(saved.count() >= 1, 3000);
      QTest::qWait(200); // settled: the slot holds the final layout
      QCOMPARE(storedLast(), b->saveState());
      // no persistLast() / close here: only the debounce wrote it
    }
    Fixture g;
    QVERIFY(g.wm->restoreLast());
    auto *b = g.wm->backend();
    QCOMPARE(g.wm->currentName(), QString("Fly")); // the menu tick
    QVERIFY(b->panelState("c").floating);
    QCOMPARE(b->panelState("c").floatGeometry.size(), QSize(400, 300));
    QVERIFY(!b->panelState("b").visible);
    QVERIFY(b->panelState("a").visible);
  }

  // A splitter dragged with the mouse is a layout change too (ADS has no signal
  // for it).
  void lastLayoutSeesSplitterDrag() {
    Fixture f;
    auto *b = f.wm->backend();
    f.wm->setLastDelay(50);
    f.wm->load("Fly");
    QTest::qWait(150);
    QSplitter *sp = nullptr; // one with two visible sides
    for (QSplitter *c : f.win.findChildren<QSplitter *>())
      if (c->count() > 1 && !c->sizes().contains(0))
        sp = c;
    QVERIFY(sp);
    const QByteArray beforeDrag = storedLast();
    QWidget *h = sp->handle(1);
    const QPoint c = h->rect().center();
    const QPoint d = sp->orientation() == Qt::Horizontal ? QPoint(-60, 0)
                                                         : QPoint(0, -60);
    QTest::mousePress(h, Qt::LeftButton, {}, c);
    QTest::mouseMove(h, c + d / 2);
    QTest::mouseMove(h, c + d);
    QTest::mouseRelease(h, Qt::LeftButton, {}, c + d);
    QTRY_VERIFY_WITH_TIMEOUT(storedLast() != beforeDrag, 3000);
    QCOMPARE(storedLast(), b->saveState());
  }

  // The active user view is what the menu ticks after a restart, and the edit
  // is in the last slot while the saved view itself is untouched.
  void editedUserViewIsRestoredButNotOverwritten() {
    QByteArray mine;
    {
      Fixture f;
      f.wm->setLastDelay(50);
      f.wm->load("Fly");
      QVERIFY(f.wm->saveCurrentAs("Mine"));
      mine = storedState("Mine");
      f.wm->backend()->hidePanel("c");
      QSignalSpy saved(f.wm, &WorkspaceManager::lastSaved);
      QTRY_VERIFY_WITH_TIMEOUT(saved.count() >= 1, 3000);
    }
    QCOMPARE(storedState("Mine"), mine);
    Fixture g;
    QVERIFY(g.wm->restoreLast());
    QCOMPARE(g.wm->currentName(), QString("Mine"));
    QVERIFY(!g.wm->backend()->panelState("c").visible); // as edited
    QVERIFY(g.wm->load("Mine")); // the saved definition is still the original
    QVERIFY(g.wm->backend()->panelState("c").visible);
  }

  // (b) switching right after an edit still records it.
  void switchFlushesPendingEdit() {
    {
      Fixture f;
      f.wm->setLastDelay(60000); // the debounce can not fire in this test
      f.wm->load("Fly");
      QSignalSpy saved(f.wm, &WorkspaceManager::lastSaved);
      f.wm->backend()->hidePanel("c");
      QCOMPARE(saved.count(), 0);
      QVERIFY(f.wm->load("Sim")); // flush happens here
      QCOMPARE(saved.count(), 1);
    }
    Fixture g; // restarted before the new layout was ever recorded
    QVERIFY(g.wm->restoreLast());
    QCOMPARE(g.wm->currentName(), QString("Fly"));
    QVERIFY(!g.wm->backend()->panelState("c").visible);
  }

  // (c) applying or loading never loops: at most one record per load, then
  // quiet; a restore records nothing.
  void loadingDoesNotLoopWrite() {
    Fixture f;
    f.wm->setLastDelay(50);
    f.wm->load("Fly");
    f.wm->backend()->floatPanel("c");
    QVERIFY(f.wm->saveCurrentAs("Mine"));
    QSignalSpy saved(f.wm, &WorkspaceManager::lastSaved);
    for (int i = 0; i < 3; ++i) {
      f.wm->load("Mine");
      f.wm->load("Fly");
      f.wm->resetLayout();
    }
    QTest::qWait(400);
    const int n = saved.count();
    QVERIFY(n <= 9); // at most one record per switch (9 of them), no more
    QTest::qWait(500);
    QCOMPARE(saved.count(), n); // quiet
    f.wm->persistLast();
    QSignalSpy again(f.wm, &WorkspaceManager::lastSaved);
    QVERIFY(f.wm->restoreLast());
    QTest::qWait(400);
    QCOMPARE(again.count(), 0);
  }

  // (d) edits never change a built-in or a saved workspace.
  void editsLeaveDefinitionsAlone() {
    Fixture f;
    auto *b = f.wm->backend();
    f.wm->setLastDelay(50);
    f.wm->load("Fly");
    QVERIFY(f.wm->saveCurrentAs("Mine"));
    const QByteArray mine = storedState("Mine");
    f.wm->load("Fly");
    b->hidePanel("c");
    b->floatPanel("a");
    QTest::qWait(300);
    f.wm->load("Mine");
    b->hidePanel("a");
    QTest::qWait(300);
    f.wm->load("Sim");
    QCOMPARE(f.wm->userNames(), QStringList{"Mine"});
    QCOMPARE(storedState("Mine"), mine);
    QVERIFY(f.wm->load("Fly")); // the built-in definition is intact
    QVERIFY(b->panelState("c").visible);
    QVERIFY(!b->panelState("a").floating);
  }

  // (e) a last layout that cannot be used falls back to the default.
  void unusableLastFallsBackToDefault() {
    {
      Fixture f; // a layout naming a panel the next run does not have
      f.wm->backend()->addPanel("zz", "ZZ", new QLabel("zz"));
      f.wm->load("Sim");
      f.wm->backend()->showPanel("zz");
      f.wm->persistLast();
    }
    {
      Fixture g;
      QSignalSpy failed(g.wm, &WorkspaceManager::loadFailed);
      QVERIFY(!g.wm->restoreLast());
      QCOMPARE(failed.count(), 1);
      QCOMPARE(g.wm->currentName(), QString("Fly"));
      QVERIFY(g.wm->backend()->panelState("a").visible);
    }
    QSettings q; // corrupt bytes
    q.setValue("workspaces/last/state", QByteArray("garbage"));
    q.sync();
    Fixture h;
    QVERIFY(!h.wm->restoreLast());
    QCOMPARE(h.wm->currentName(), QString("Fly"));
  }

  // (f) a failed (empty) capture is never stored.
  void emptyCaptureIsNeverSaved() {
    QMainWindow win;
    win.resize(900, 600);
    auto be = std::make_unique<FlakyBackend>(&win);
    FlakyBackend *fb = be.get();
    auto *wm = new WorkspaceManager(&win, std::move(be), &win);
    wm->registerPanels(panels());
    wm->setBuiltins(builtins());
    wm->setScreenProvider([] { return QList<QRect>{QRect(0, 0, 1600, 900)}; });
    wm->setLastDelay(50);
    win.show();
    wm->load("Fly");
    wm->persistLast();
    const QByteArray last = storedLast();
    QVERIFY(!last.isEmpty());
    QSignalSpy saved(wm, &WorkspaceManager::lastSaved);
    fb->empty = true;
    fb->hidePanel("c");
    QTest::qWait(300);
    wm->flushLast();
    wm->load("Sim");
    wm->persistLast();
    QCOMPARE(saved.count(), 0);
    QCOMPARE(storedLast(), last);
  }
};

QTEST_MAIN(TstWorkspaceManager)
#include "tst_workspace_manager.moc"
