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
    m_win->close(); // stops the telemetry worker thread
    delete m_win;
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
};

QTEST_MAIN(TstPanelRealMin)
#include "tst_panel_real_min.moc"
