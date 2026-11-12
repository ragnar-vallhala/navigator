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
#include "AdsBackend.h"
#include "Theme.h"

#include <QFile>
#include <QImage>
#include <QLabel>
#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QSettings>
#include <QStyle>
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

QMenu *menuNamed(QMenuBar *bar, const QString &title) {
  for (QAction *a : bar->actions())
    if (a->menu() && a->text() == title)
      return a->menu();
  return nullptr;
}

// A panel that counts its reparents (a switch must cause none: that would
// re-create its native GL windows) and notes whether the host was painting
// while it was shown or hidden (a switch freezes the host).
class ProbeLabel : public QLabel {
public:
  struct Stats {
    int reparents = 0, shownOrHidden = 0, unfrozen = 0;
  };
  ProbeLabel(const QString &t, QMainWindow *host, Stats *st)
      : QLabel(t), m_host(host), m_st(st) {}
  bool event(QEvent *e) override {
    if (e->type() == QEvent::ParentChange)
      ++m_st->reparents;
    if (e->type() == QEvent::Show || e->type() == QEvent::Hide) {
      ++m_st->shownOrHidden;
      if (m_host->updatesEnabled())
        ++m_st->unfrozen;
    }
    return QLabel::event(e);
  }

private:
  QMainWindow *m_host;
  Stats *m_st;
};

int areaCount(QWidget *dm) {
  int n = 0;
  QCoreApplication::sendPostedEvents(nullptr, QEvent::DeferredDelete);
  for (QWidget *w : dm->findChildren<QWidget *>())
    if (QByteArray(w->metaObject()->className()) == "ads::CDockAreaWidget")
      ++n;
  return n;
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
  // The docked layout must wear the dark theme: the dock manager's own
  // palette()-based sheet used to paint Qt's default grey around the panels
  // and under the tab titles. Checks pixels the way a user sees them.
  void dockSurfacesWearTheDarkTheme() {
    // Put the app palette and sheet back whatever happens below.
    struct Restore {
      QPalette pal = qApp->palette();
      QString sheet = qApp->styleSheet();
      QString style = qApp->style()->objectName();
      ~Restore() {
        qApp->setStyleSheet(sheet);
        qApp->setPalette(pal);
        if (style != qApp->style()->objectName())
          qApp->setStyle(style);
      }
    } restore;
    // The palette and the real dark.qss, as main() applies them.
    Theme::apply();
    QFile qss(QStringLiteral(DARK_QSS_PATH));
    QVERIFY(qss.open(QIODevice::ReadOnly));
    qApp->setStyleSheet(QString::fromUtf8(qss.readAll()));
    Fixture f;
    QVERIFY(QTest::qWaitForWindowExposed(&f.win));
    QTest::qWait(300); // the queued splitter sizing and polish
    auto pixelAt = [](QWidget *w, QPoint p) {
      return w->grab().toImage().pixelColor(p);
    };
    int areas = 0, docks = 0, tabs = 0;
    const auto widgets = f.win.findChildren<QWidget *>();
    for (QWidget *w : widgets) {
      const QByteArray cls = w->metaObject()->className();
      if (!w->isVisible() || w->width() < 8 || w->height() < 8)
        continue;
      if (cls == "ads::CDockWidget" || cls == "ads::CDockAreaWidget") {
        (cls == "ads::CDockWidget" ? docks : areas)++;
        // bottom-right corner: clear of any text, inside the panel
        const QColor got = pixelAt(w, QPoint(w->width() - 3, w->height() - 3));
        QVERIFY2(got == Theme::kBg, qPrintable(QStringLiteral("%1 is %2")
                                                   .arg(QString::fromLatin1(cls),
                                                        got.name())));
      } else if (cls == "ads::CDockWidgetTab") {
        tabs++;
        QVERIFY(w->property("activeTab").toBool()); // one panel per area here
        QCOMPARE(pixelAt(w, QPoint(w->width() - 3, 2)), Theme::kSurfaceAlt);
        const auto labels = w->findChildren<QLabel *>();
        QVERIFY(!labels.isEmpty());
        for (QLabel *l : labels)
          QCOMPARE(l->palette().color(QPalette::WindowText), Theme::kText);
      }
    }
    QVERIFY(areas > 0);
    QVERIFY(docks > 0);
    QVERIFY(tabs > 0);

    // A floated panel lives in its own top-level window: same theme there.
    f.shell->workspaces()->backend()->floatPanel("imu");
    QTest::qWait(400);
    int floated = 0;
    for (QWidget *tw : QApplication::topLevelWidgets()) {
      if (!QByteArray(tw->metaObject()->className()).contains("Floating"))
        continue;
      QVERIFY(tw->isVisible());
      const QImage im = tw->grab().toImage();
      QCOMPARE(im.pixelColor(im.width() - 3, im.height() - 3), Theme::kBg);
      QCOMPARE(im.pixelColor(im.width() / 2, 2), Theme::kBg); // tab strip
      floated++;
    }
    QVERIFY(floated > 0);

    // Setting the same extra sheet again leaves one copy, not two.
    auto *ads = dynamic_cast<ws::AdsBackend *>(f.shell->workspaces()->backend());
    QVERIFY(ads);
    QWidget *dm = f.win.centralWidget();
    const QString once = dm->styleSheet();
    ads->setExtraStyleSheet(Theme::dockStyleSheet());
    ads->setExtraStyleSheet(Theme::dockStyleSheet());
    QCOMPARE(dm->styleSheet(), once);
  }

  void init() {
    QSettings().clear();
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
    // Fly = default; the rest after Sim are the old full-window pages.
    QCOMPARE(names, (QStringList{"Fly", "Tune", "Debug link", "Sim", "RC channels",
                                 "Motors", "Control loop", "Packets", "Kernel perf"}));

    const QStringList ids = allIds();
    QMap<QString, QStringList> want = {
        {"Fly", {"view3d", "imu", "logs"}},
        {"Tune", {"control_loop", "gyro_notch", "motors", "calibration", "logs"}},
        {"Debug link", {"packets", "logs", "perf", "rc"}},
        {"Sim", {"sim", "logs"}},
        {"RC channels", {"rc"}},
        {"Motors", {"motors"}},
        {"Control loop", {"control_loop"}},
        {"Packets", {"packets"}},
        {"Kernel perf", {"perf"}}};
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

  // Ctrl+1..6, 8 of the old View menu: each key is a built-in workspace, the
  // Workspaces menu hosts the shortcut, and pressing the key switches to it.
  void shortcutsMapToTheOldViewOrder() {
    const QStringList order = {"Fly", "RC channels", "Motors", "Control loop",
                               "Packets", "Sim", "Kernel perf"};
    const QStringList keys = {"Ctrl+1", "Ctrl+2", "Ctrl+3", "Ctrl+4",
                              "Ctrl+5", "Ctrl+6", "Ctrl+8"};
    const auto &map = DockShell::workspaceShortcuts();
    QCOMPARE(int(map.size()), int(order.size()));
    QStringList names;
    for (const auto &w : DockShell::builtins())
      names << w.name;
    for (size_t i = 0; i < map.size(); ++i) {
      QCOMPARE(map[i].workspace, order[int(i)]);
      QCOMPARE(map[i].key, keys[int(i)]);
      QVERIFY(names.contains(map[i].workspace));
    }

    Fixture f;
    auto *bar = new QMenuBar; // owned by the window
    f.win.setMenuBar(bar);
    f.shell->addMenus(bar, nullptr);
    QMenu *ws = menuNamed(bar, "&Workspaces");
    QVERIFY(ws);
    f.win.show();
    QVERIFY(QTest::qWaitForWindowExposed(&f.win));
    f.win.activateWindow();
    QVERIFY(QTest::qWaitForWindowActive(&f.win));
    for (int i = 0; i < order.size(); ++i) {
      QAction *a = nullptr;
      for (QAction *c : ws->actions())
        if (c->text() == order[i])
          a = c;
      QVERIFY2(a, qPrintable(order[i]));
      QCOMPARE(a->shortcut(), QKeySequence(keys[i]));
      // Through the real key path, not trigger().
      QTest::keyClick(&f.win, QKeySequence(keys[i])[0].key(), Qt::ControlModifier);
      QCOMPARE(f.shell->workspaces()->currentName(), order[i]);
    }
    // Ctrl+7 (Calibration) and Ctrl+9 stay outside the workspace keys.
    for (QAction *c : ws->actions())
      QVERIFY(c->shortcut() != QKeySequence("Ctrl+7") && c->shortcut() != QKeySequence("Ctrl+9"));
  }

  // Without the simulator panel (no SITL build) Ctrl+6 is not bound.
  void simShortcutOnlyWithSimulatorPanel() {
    Fixture f(QStringList{"view3d", "logs", "rc"});
    auto *bar = new QMenuBar; // owned by the window
    f.win.setMenuBar(bar);
    f.shell->addMenus(bar, nullptr);
    QMenu *ws = menuNamed(bar, "&Workspaces");
    QVERIFY(ws);
    for (QAction *c : ws->actions())
      if (c->text() == "Sim")
        QVERIFY(c->shortcut().isEmpty());
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

  // One dock manager and the same panel widgets for the window's life: a
  // switch re-places them, never re-parents (no native window is re-created),
  // and the host does not paint while it happens.
  void switchKeepsManagerAndPanelsAndFreezesHost() {
    QMainWindow win;
    win.resize(1200, 800);
    ProbeLabel::Stats st;
    QHash<QString, QPointer<QWidget>> w;
    auto *shell = new DockShell(&win);
    for (const QString &id : allIds()) {
      auto *l = new ProbeLabel(id, &win, &st);
      w.insert(id, l);
      shell->addPanel(id, l);
    }
    shell->start(false);
    win.show();
    auto *wm = shell->workspaces();
    QVERIFY(wm->load("Fly"));
    QWidget *dm = win.centralWidget();
    QVERIFY(dm);
    st = {};
    int areas = -1;
    for (int round = 0; round < 2; ++round) {
      for (const auto &b : DockShell::builtins()) {
        QVERIFY(wm->load(b.name));
        QCOMPARE(win.centralWidget(), dm);
        QVERIFY(win.updatesEnabled());
        QVERIFY(win.centralWidget()->isVisible());
        for (const QString &id : allIds())
          QVERIFY2(w.value(id), qPrintable(id)); // never destroyed
      }
      if (round == 0)
        areas = areaCount(dm);
    }
    QCOMPARE(st.reparents, 0);
    QVERIFY(st.shownOrHidden > 0);
    QCOMPARE(st.unfrozen, 0);
    QCOMPARE(areaCount(dm), areas); // emptied areas do not pile up
    QVERIFY(areas <= 8);
  }

  // A panel the new layout lacks is hidden, not destroyed, and comes back with
  // its state; a floated panel is docked again by the next switch.
  void droppedPanelsAreHiddenAndComeBack() {
    Fixture f;
    auto *wm = f.shell->workspaces();
    auto *b = wm->backend();
    QVERIFY(wm->load("Fly"));
    QWidget *imu = nullptr;
    for (QWidget *c : f.win.findChildren<QLabel *>())
      if (static_cast<QLabel *>(c)->text() == "imu")
        imu = c;
    QVERIFY(imu);
    imu->setProperty("keep", 42);
    b->floatPanel("imu");
    QVERIFY(b->panelState("imu").floating);
    QVERIFY(wm->load("Motors"));
    QVERIFY(!b->panelState("imu").visible);
    QVERIFY(!b->panelState("view3d").visible);
    QVERIFY(b->panelState("motors").visible);
    QTest::qWait(30);
    QVERIFY(!imu->isVisible());
    QVERIFY(wm->load("Fly"));
    QVERIFY(b->panelState("imu").visible);
    QVERIFY(!b->panelState("imu").floating);
    QCOMPARE(imu->property("keep").toInt(), 42);
    QTest::qWait(30);
    QVERIFY(imu->isVisible());
  }

  void switchNeverResizesTheWindow() {
    Fixture f;
    f.win.resize(1300, 820);
    QTest::qWait(50);
    const QSize before = f.win.size();
    auto *wm = f.shell->workspaces();
    for (int round = 0; round < 2; ++round)
      for (const auto &w : DockShell::builtins()) {
        QVERIFY(wm->load(w.name));
        QTest::qWait(10);
        QCOMPARE(f.win.size(), before);
        QVERIFY2(f.win.minimumSizeHint().width() <= before.width(),
                 qPrintable(w.name));
        QVERIFY2(f.win.minimumSizeHint().height() <= before.height(),
                 qPrintable(w.name));
      }
  }

  void quickSuccessiveLoadsEndConsistent() {
    Fixture f;
    auto *wm = f.shell->workspaces();
    QVERIFY(wm->load("Sim"));
    QVERIFY(wm->load("Fly"));
    QVERIFY(wm->load("Packets"));
    QTest::qWait(50);
    QCOMPARE(wm->currentName(), QString("Packets"));
    auto *b = wm->backend();
    QVERIFY(b->panelState("packets").visible);
    QVERIFY(!b->panelState("sim").visible);
    QVERIFY(!b->panelState("view3d").visible);
    QVERIFY(f.win.updatesEnabled());
    QVERIFY(f.win.centralWidget()->isVisible());
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

  // The Panels menu: one submenu per group, panels in table order, no flat
  // leftovers; the checkable actions are the backend's own.
  void panelsMenuIsGrouped() {
    Fixture f;
    QMenuBar bar;
    f.shell->addMenus(&bar, nullptr);
    QMenu *panels = menuNamed(&bar, "&Panels");
    QVERIFY(panels);
    const QList<QAction *> top = panels->actions();
    QStringList got;
    for (QAction *a : top) {
      QVERIFY2(a->menu(), qPrintable("flat leftover: " + a->text()));
      QStringList ids;
      for (QAction *c : a->menu()->actions())
        ids << c->text();
      got << a->text() + ": " + ids.join(", ");
    }
    auto *b = f.shell->workspaces()->backend();
    auto t = [&](const char *id) { return b->viewAction(id)->text(); };
    const QStringList want = {
        "&Flight: " + QStringList{t("view3d"), t("rc"), t("motors")}.join(", "),
        "&Sensors: " + t("imu"),
        "&Tuning: " + QStringList{t("calibration"), t("control_loop"), t("gyro_notch")}.join(", "),
        "&Debug: " + QStringList{t("logs"), t("packets"), t("perf")}.join(", "),
        "S&imulation: " + t("sim"),
        "&App: " + t("settings"),
    };
    QCOMPARE(got, want);
  }

  void panelsMenuToggleOpensClosesAndStaysInSync() {
    Fixture f;
    QMenuBar bar;
    f.shell->addMenus(&bar, nullptr);
    auto *b = f.shell->workspaces()->backend();
    QMenu *panels = menuNamed(&bar, "&Panels");
    QAction *act = nullptr; // Settings, from the App submenu
    for (QAction *g : panels->actions())
      if (g->text() == "&App")
        act = g->menu()->actions().value(0);
    QVERIFY(act);
    QCOMPARE(act, b->viewAction("settings"));
    QVERIFY(!act->isChecked() && !b->panelState("settings").visible);
    act->trigger();
    QVERIFY(act->isChecked() && b->panelState("settings").visible);
    act->trigger();
    QVERIFY(!act->isChecked() && !b->panelState("settings").visible);
    b->showPanel("settings"); // shown elsewhere: the check follows
    QVERIFY(act->isChecked());
  }

  // An ungrouped (or unknown-group) panel lands in a trailing "Other"; a group
  // nobody uses is not shown.
  void ungroupedPanelsGoToOtherAndEmptyGroupsAreHidden() {
    const std::vector<DockShell::PanelInfo> infos = {
        {"a", "A", "debug"}, {"b", "B"}, {"c", "C", "nonsense"}, {"d", "D", "debug"}};
    QMenu menu;
    QHash<QString, QAction *> acts;
    for (const auto &i : infos)
      acts[i.id] = new QAction(i.title, &menu);
    DockShell::buildPanelsMenu(&menu, infos, [&](const QString &id) { return acts.value(id); });
    QStringList got;
    for (QAction *a : menu.actions()) {
      QStringList ids;
      for (QAction *c : a->menu()->actions())
        ids << c->text();
      got << a->text() + ": " + ids.join(",");
    }
    QCOMPARE(got, (QStringList{"&Debug: A,D", "&Other: B,C"}));
    // nothing ungrouped: no Other
    QMenu m2;
    DockShell::buildPanelsMenu(&m2, {infos[0]}, [&](const QString &id) { return acts.value(id); });
    QCOMPARE(m2.actions().size(), 1);
    // every real panel has a known group
    QStringList known;
    for (const auto &g : DockShell::panelGroups())
      known << g.id;
    for (const auto &p : DockShell::panelInfos())
      QVERIFY2(known.contains(p.group), qPrintable(p.id));
  }

  void panelsMenuHasNoDuplicateMnemonics() {
    Fixture f;
    QMenuBar bar;
    f.shell->addMenus(&bar, nullptr);
    QMenu *panels = menuNamed(&bar, "&Panels");
    auto check = [](QMenu *m) {
      QSet<QChar> seen;
      for (QAction *a : m->actions()) {
        const int i = a->text().indexOf('&');
        if (i < 0 || i + 1 >= a->text().size())
          continue;
        const QChar c = a->text().at(i + 1).toLower();
        QVERIFY2(!seen.contains(c), qPrintable(m->title() + ": " + a->text()));
        seen.insert(c);
      }
    };
    check(panels);
    for (QAction *g : panels->actions())
      check(g->menu());
    QMenu all; // with a group of its own: the Other mnemonic must not clash either
    DockShell::buildPanelsMenu(&all, {{"x", "X"}}, [&](const QString &) { return new QAction("X", &all); });
    check(&all);
    // the full set incl. Other
    QMenu full;
    std::vector<DockShell::PanelInfo> infos = DockShell::panelInfos();
    infos.push_back({"zz", "ZZ"});
    DockShell::buildPanelsMenu(&full, infos, [&](const QString &id) { return new QAction(id, &full); });
    check(&full);
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
