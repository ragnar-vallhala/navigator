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
// The real panels (built by the real MainWindow, not stand-ins) report a
// content minimum through PanelFrame, keep their rows apart at that minimum,
// and the dock layout never asks the main window for more than a 1366x768
// screen, for the default workspace and every built-in one. Nothing is shown:
// several panels hold GL windows that cannot be created without GL.
#include "AdsBackend.h"
#include "DockShell.h"
#include "MainWindow.h"
#include "PanelFrame.h"
#include "WorkspaceManager.h"

#include <DockManager.h>
#include <DockWidget.h>

#include <QLayout>
#include <QMainWindow>
#include <QResizeEvent>
#include <QScrollArea>
#include <QStackedWidget>
#include <QTabBar>
#include <QTabWidget>
#include <QSettings>
#include <QTemporaryDir>
#include <QtTest>

using namespace ws;

namespace {
constexpr int kScreenW = 1366;
constexpr int kScreenH = 768;
// Room a window manager keeps for the title bar, panels and borders.
constexpr int kChromeH = 40;

QWidget *contentOf(ads::CDockWidget *d) {
  auto *sa = qobject_cast<QScrollArea *>(d->widget());
  return sa ? sa->widget() : d->widget();
}

// Lay out every layout under `root` (hidden widgets are laid out lazily) from
// the top down so child geometries are real at the size given to `root`.
void activateAll(QWidget *root) {
  const auto lay = [](QWidget *w) {
    w->ensurePolished();
    if (QLayout *l = w->layout())
      l->activate();
    // No resize event reaches a hidden widget (layouts and QTabWidget lay out
    // their children from it), so deliver one by hand.
    QResizeEvent ev(w->size(), w->size());
    QCoreApplication::sendEvent(w, &ev);
    // A hidden QTabWidget defers its own layout until it is shown: give its
    // page stack the room below the tab bar.
    if (auto *tw = qobject_cast<QTabWidget *>(w))
      if (auto *st = tw->findChild<QStackedWidget *>()) {
        const int bar = tw->tabBar()->sizeHint().height();
        st->setGeometry(0, bar, tw->width(), qMax(0, tw->height() - bar));
        QResizeEvent sev(st->size(), st->size());
        QCoreApplication::sendEvent(st, &sev);
      }
  };
  lay(root);
  const auto kids = root->findChildren<QWidget *>();
  for (QWidget *k : kids)
    lay(k);
}

// One layout item that is visible and whose rect cuts into another's, or that
// got less than its own minimum: both read as overlapping rows on screen.
QStringList overlaps(QWidget *root) {
  QStringList bad;
  const auto layouts = root->findChildren<QLayout *>();
  for (QLayout *l : layouts) {
    if (!l->parentWidget() || !l->parentWidget()->isVisibleTo(root))
      continue;
    QList<QRect> rects;
    QList<const QLayoutItem *> items;
    for (int i = 0; i < l->count(); ++i) {
      const QLayoutItem *it = l->itemAt(i);
      if (it->isEmpty() || (it->widget() && !it->widget()->isVisibleTo(root)))
        continue;
      items << it;
      rects << it->geometry();
    }
    for (int i = 0; i < rects.size(); ++i) {
      const QSize min = items[i]->minimumSize();
      if (rects[i].width() < min.width() || rects[i].height() < min.height())
        bad << QString("%1: item %2 is %3x%4, needs %5x%6")
                   .arg(l->parentWidget()->metaObject()->className())
                   .arg(i).arg(rects[i].width()).arg(rects[i].height())
                   .arg(min.width()).arg(min.height());
      for (int j = i + 1; j < rects.size(); ++j)
        if (rects[i].intersects(rects[j]))
          bad << QString("%1: items %2 and %3 overlap")
                     .arg(l->parentWidget()->metaObject()->className())
                     .arg(i).arg(j);
    }
  }
  return bad;
}
} // namespace

class TstPanelRealMin : public QObject {
  Q_OBJECT
  QTemporaryDir m_dir;
  MainWindow *m_win = nullptr;
  DockShell *m_shell = nullptr;

  ads::CDockWidget *dock(const QString &id) const {
    auto *dm = m_win->findChild<ads::CDockManager *>();
    return dm ? dm->findDockWidget(id) : nullptr;
  }
  // Open every panel the build has, one workspace after another, so each has a
  // dock widget to look at.
  void openAll() {
    for (const DockShell::PanelInfo &p : DockShell::panelInfos())
      if (m_shell->hasPanel(p.id))
        m_shell->showPanel(p.id);
    QTest::qWait(50);
  }

private slots:
  void initTestCase() {
    QVERIFY(m_dir.isValid());
    QCoreApplication::setOrganizationName("NAVRobotecTest");
    QCoreApplication::setApplicationName("tst_panel_real_min");
    QSettings::setDefaultFormat(QSettings::IniFormat);
    QSettings::setPath(QSettings::IniFormat, QSettings::UserScope,
                       m_dir.path());
    m_win = new MainWindow;
    m_shell = m_win->findChild<DockShell *>();
    QVERIFY(m_shell);
    QVERIFY(m_shell->workspaces());
    QVERIFY(m_shell->workspaces()->load(m_shell->workspaces()->defaultName()));
    QTest::qWait(50);
    openAll();
  }
  void cleanupTestCase() {
    if (m_win) {
      m_win->close(); // stops the telemetry worker thread
      delete m_win;
    }
  }

  // Every real panel is hosted so that its dock widget reports the panel's own
  // minimum (capped), not ADS's 60x40.
  void everyPanelReportsItsContentMinimum() {
    int checked = 0;
    for (const DockShell::PanelInfo &p : DockShell::panelInfos()) {
      ads::CDockWidget *d = dock(p.id);
      if (!d)
        continue; // a panel this build lacks (no simulator)
      ++checked;
      QWidget *c = contentOf(d);
      QVERIFY2(c, qPrintable(p.id));
      const QSize cm = PanelFrame::minimumOf(c);
      const QSize want = cappedMinimum(cm);
      const QSize got = d->minimumSizeHint();
      qInfo("MEASURE %-12s sizeHint %4dx%-4d content min %4dx%-4d dock min "
            "%4dx%-4d",
            qPrintable(p.id), c->sizeHint().width(), c->sizeHint().height(),
            cm.width(), cm.height(), got.width(), got.height());
      QVERIFY2(got.width() >= want.width() && got.height() >= want.height(),
               qPrintable(QString("%1: dock min %2x%3 < content %4x%5")
                              .arg(p.id).arg(got.width()).arg(got.height())
                              .arg(want.width()).arg(want.height())));
      QVERIFY2(got.width() <= kMaxMinW && got.height() <= kMaxMinH,
               qPrintable(p.id));
    }
    QVERIFY(checked >= 11);
  }

  // At its own minimum size no panel's rows overlap or get less room than they
  // need (Gyro Notch's rows used to when the panel was squeezed).
  void noRowsOverlapAtTheMinimum() {
    for (const DockShell::PanelInfo &p : DockShell::panelInfos()) {
      ads::CDockWidget *d = dock(p.id);
      if (!d)
        continue;
      QWidget *c = contentOf(d);
      const QSize cm = PanelFrame::minimumOf(c);
      c->resize(cm);
      activateAll(c);
      const QStringList bad = overlaps(c);
      QVERIFY2(bad.isEmpty(),
               qPrintable(p.id + " at " + QString::number(cm.width()) + "x" +
                          QString::number(cm.height()) + ": " +
                          bad.join("; ")));
    }
  }

  // The dock layout of every built-in workspace needs no more than a 1366x768
  // screen: the capped panel minimums keep the main window inside it.
  void mainWindowMinimumFitsTheScreen() {
    WorkspaceManager *wm = m_shell->workspaces();
    const QStringList names = wm->builtinNames();
    QVERIFY(names.size() >= 5);
    for (const QString &n : names) {
      QVERIFY2(wm->load(n), qPrintable(n));
      QTest::qWait(20);
      QWidget *central = m_win->centralWidget();
      QVERIFY(central);
      const QSize cmin = central->minimumSizeHint();
      const QSize wmin = m_win->minimumSize().expandedTo(
          m_win->minimumSizeHint());
      qInfo("MEASURE workspace %-14s central min %4dx%-4d window min %4dx%-4d",
            qPrintable(n), cmin.width(), cmin.height(), wmin.width(),
            wmin.height());
      QVERIFY2(cmin.width() <= kScreenW && cmin.height() <= kScreenH - kChromeH,
               qPrintable(QString("%1: central min %2x%3").arg(n)
                              .arg(cmin.width()).arg(cmin.height())));
      QVERIFY2(wmin.width() <= kScreenW && wmin.height() <= kScreenH - kChromeH,
               qPrintable(QString("%1: window min %2x%3").arg(n)
                              .arg(wmin.width()).arg(wmin.height())));
    }
  }

  // Quit the real app with panels floated and start it again: they come back
  // floating, visible and where they were. Built from the real MainWindow, so
  // the real close path (layout saved, then the floats closed) and the real
  // startup restore (queued until the window is up) are the ones tested. Last in
  // the file: it retires the shared window first and starts from empty settings.
  void restartRestoresFloatedPanelsInTheRealWindow() {
    m_win->close();
    delete m_win;
    m_win = nullptr;
    m_shell = nullptr;
    {
      QSettings st;
      st.clear();
      st.sync();
    }
    // Asked for; a real panel may be given more than that (its content minimum) and a
    // platform may place the frame differently, so what is compared after the restart is
    // what each float had when the app quit.
    const QRect imuAsk(40, 60, 300, 220), logsAsk(380, 150, 320, 240);
    QRect imuAt, logsAt;
    // A float's geometry has settled once two reads 100 ms apart agree.
    const auto settled = [](ws::DockBackend *b, const char *id) {
      QRect prev = b->panelState(id).floatGeometry;
      for (int i = 0; i < 100; ++i) {
        QTest::qWait(100);
        const QRect now = b->panelState(id).floatGeometry;
        if (now == prev)
          return now;
        prev = now;
      }
      return prev;
    };
    const auto anyPanelVisible = [](DockShell *sh) {
      for (const DockShell::PanelInfo &p : DockShell::panelInfos())
        if (sh->hasPanel(p.id) && sh->workspaces()->backend()->panelState(p.id).visible)
          return true;
      return false;
    };
    const auto shellOf = [](MainWindow &w) { return w.findChild<DockShell *>(); };
    const auto floatsShown = [] {
      int n = 0;
      for (QWidget *tw : QApplication::topLevelWidgets())
        if (tw->isVisible() &&
            QByteArray(tw->metaObject()->className()).contains("Floating"))
          ++n;
      return n;
    };
    {
      MainWindow w;
      w.resize(1200, 800);
      w.show();
      QVERIFY(QTest::qWaitForWindowExposed(&w));
      DockShell *sh = shellOf(w);
      QVERIFY(sh);
      auto *b = sh->workspaces()->backend();
      // The queued startup restore has run once a layout shows some panel.
      QTRY_VERIFY_WITH_TIMEOUT(anyPanelVisible(sh), 10000);
      b->floatPanel("imu");
      b->floatPanel("logs");
      b->floatPanel("rc");
      QTRY_VERIFY_WITH_TIMEOUT(b->panelState("imu").floating && b->panelState("logs").floating, 10000);
      for (QWidget *tw : QApplication::topLevelWidgets())
        if (QByteArray(tw->metaObject()->className()).contains("Floating"))
          tw->show();
      b->setFloatGeometry("imu", imuAsk);
      b->setFloatGeometry("logs", logsAsk);
      b->hidePanel("rc"); // floated, then closed: stays closed
      QTRY_VERIFY_WITH_TIMEOUT(!b->panelState("rc").visible, 10000);
      QVERIFY(b->panelState("imu").floating);
      QVERIFY(b->panelState("logs").floating);
      imuAt = settled(b, "imu");
      logsAt = settled(b, "logs");
      w.close(); // MainWindow::closeEvent: save, then close the floats
      // Nothing to poll: closing is not an edit, so no autosave may fire; give one the time to.
      QTest::qWait(900);
    }
    MainWindow w2;
    w2.resize(1200, 800);
    w2.show();
    QVERIFY(QTest::qWaitForWindowExposed(&w2));
    DockShell *sh2 = shellOf(w2);
    QVERIFY(sh2);
    auto *b2 = sh2->workspaces()->backend();
    QTRY_VERIFY_WITH_TIMEOUT(b2->panelState("imu").floating && b2->panelState("imu").visible, 10000);
    QTRY_VERIFY_WITH_TIMEOUT(b2->panelState("logs").floating && b2->panelState("logs").visible, 10000);
    settled(b2, "imu");
    settled(b2, "logs");
    const auto near = [](const QRect &a, const QRect &e) {
      return qAbs(a.x() - e.x()) <= 4 && qAbs(a.y() - e.y()) <= 4 &&
             qAbs(a.width() - e.width()) <= 4 && qAbs(a.height() - e.height()) <= 4;
    };
    const ws::PanelState imu = b2->panelState("imu"), logs = b2->panelState("logs");
    const auto geo = [](const ws::PanelState &p) {
      return QStringLiteral("%1,%2 %3x%4").arg(p.floatGeometry.x()).arg(p.floatGeometry.y())
          .arg(p.floatGeometry.width()).arg(p.floatGeometry.height());
    };
    QVERIFY2(imu.visible && imu.floating,
             qPrintable(QStringLiteral("imu visible=%1 floating=%2").arg(imu.visible).arg(imu.floating)));
    QVERIFY2(logs.visible && logs.floating,
             qPrintable(QStringLiteral("logs visible=%1 floating=%2").arg(logs.visible).arg(logs.floating)));
    QVERIFY2(near(imu.floatGeometry, imuAt), qPrintable("imu at " + geo(imu)));
    QVERIFY2(near(logs.floatGeometry, logsAt), qPrintable("logs at " + geo(logs)));
    QVERIFY(!b2->panelState("rc").visible);
    // The offscreen platform hides floats once the dock manager's window is activated (its
    // stay-on-top emulation re-sets a window flag; X11 and Wayland do not), so the count of
    // shown windows is only meaningful on a real one.
    if (QGuiApplication::platformName() != QLatin1String("offscreen"))
      QCOMPARE(floatsShown(), 2);
    w2.close();
  }
};

QTEST_MAIN(TstPanelRealMin)
#include "tst_panel_real_min.moc"
