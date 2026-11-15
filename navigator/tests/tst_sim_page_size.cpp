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
// The Simulator page must not demand more width than a laptop screen has.
//
// Its minimumSizeHint used to be ~1477 px wide (the sum of the 3D view's
// floor and the widest row of the properties column), so a dock holding it
// could push Navigator's window past a 1366 px screen. This builds the real
// SimulatorWidget and asserts
//   - the page's minimumSizeHint().width() is under 1100 px, and so is every
//     tab (Vehicle / World / Autotune);
//   - the page can be resized to 1000x600 and stays that wide;
//   - no section title shows a stray '&' (a lone '&' in a button text is a
//     mnemonic marker and is swallowed: "Wind & Turbulence" read
//     "Wind  Turbulence") or an underscore.
// The measurements are printed (QINFO) so a regression shows what grew.
// Offscreen is enough; give it private XDG_* so settings stay private.

#include "SimulatorWidget.h"

#include <QCheckBox>
#include <QComboBox>
#include <QGroupBox>
#include <QLayout>
#include <QPushButton>
#include <QApplication>
#include <QScrollArea>
#include <QSplitter>
#include <QStackedWidget>
#include <QTemporaryDir>
#include <QToolButton>
#include <QtTest>


// Test seam (SimulatorWidget befriends this name).
class SimulatorWidgetTestAccess {
public:
  explicit SimulatorWidgetTestAccess(SimulatorWidget &w) : w_(w) {}
  QStackedWidget *stack() const { return w_.m_rightStack; }
  bool imuDropout() const { return w_.m_imuDropout; }

private:
  SimulatorWidget &w_;
};

namespace {
constexpr int kMaxMinWidth = 1100;

// Report every row / grid layout under `root` that needs more than 300 px, with
// the width each of its items asks for: those rows set the floor. Only reported.
void wideRows(const QWidget *root) {
  const auto layouts = root->findChildren<QLayout *>();
  for (const QLayout *l : layouts) {
    const int w = l->minimumSize().width();
    if (w < 300 || !l->parentWidget() || !l->parentWidget()->isVisibleTo(root))
      continue;
    QStringList items;
    for (int i = 0; i < l->count(); ++i) {
      const QLayoutItem *it = l->itemAt(i);
      const QString name =
          it->widget() ? QString::fromLatin1(it->widget()->metaObject()->className())
                       : QStringLiteral("layout");
      items << QString("%1:%2").arg(name).arg(it->minimumSize().width());
    }
    qInfo("   %s min %d: %s", l->metaObject()->className(), w,
          qPrintable(items.join(' ')));
  }
}
} // namespace

class TstSimPageSize : public QObject {
  Q_OBJECT

  SimulatorWidget *w_ = nullptr;
  SimulatorWidgetTestAccess *a_ = nullptr;

private slots:
  void initTestCase() {
    w_ = new SimulatorWidget();
    a_ = new SimulatorWidgetTestAccess(*w_);
  }
  void cleanupTestCase() {
    delete a_;
    delete w_;
  }

  void pageMinimumWidthFitsALaptop() {
    const QSize m = w_->minimumSizeHint();
    qInfo("page minimumSizeHint %dx%d", m.width(), m.height());
    QStackedWidget *st = a_->stack();
    QVERIFY(st);
    qInfo("3D view side:");
    wideRows(w_->findChild<QSplitter *>()->widget(0));
    for (int i = 0; i < st->count(); ++i) {
      const QSize t = st->widget(i)->minimumSizeHint();
      qInfo("tab %d minimumSizeHint %dx%d", i, t.width(), t.height());
      wideRows(st->widget(i));
      QVERIFY2(t.width() < kMaxMinWidth, "a tab needs too much width");
    }
    QVERIFY2(m.width() < kMaxMinWidth, "the Simulator page needs too much width");
  }

  void pageResizesToASmallWindow() {
    // Not shown: the page holds a GL window that cannot be created on a
    // runner without GL. A resize is clamped to the minimum size either way.
    // Activate first so the minimum size is applied to the hidden page.
    if (w_->layout())
      w_->layout()->activate();
    w_->resize(1000, 600);
    QVERIFY2(w_->width() <= 1000,
             qPrintable(QString("page forced to %1 px wide").arg(w_->width())));
  }

  // Fault injection is meant for use in flight. Starting the sim locks the
  // Vehicle page (and tab) and forces the World page, so the Fault Injection
  // panel must live on a page that stays reachable, not on the Vehicle page.
  // Runs offscreen (no engine needed): checks where the panel lives and that
  // its controls are enabled; the running-sim behaviour is a manual xcb check.
  void faultPanelLivesOnTheFlightPage() {
    QStackedWidget *st = a_->stack();
    QVERIFY(st && st->count() >= 2);
    auto *fault = w_->findChild<QGroupBox *>(QStringLiteral("FaultPanel"));
    QVERIFY2(fault, "Fault Injection panel not found");
    QVERIFY2(st->widget(1)->isAncestorOf(fault),
             "fault panel is not on the World page (the page forced while "
             "the sim runs)");
    QVERIFY2(!st->widget(0)->isAncestorOf(fault),
             "fault panel is on the Vehicle page, which is locked in flight");
    QVERIFY(fault->isEnabled());
    for (int i = 1; i <= 4; ++i) {
      const QString t = QStringLiteral("M%1").arg(i);
      QPushButton *kill = nullptr;
      for (auto *b : fault->findChildren<QPushButton *>())
        if (b->text() == t)
          kill = b;
      QVERIFY2(kill, qPrintable("no kill button " + t));
      QVERIFY2(kill->isEnabled(), qPrintable(t + " disabled"));
      QVERIFY(!kill->isHidden());
    }
  }

  // The IMU dropout toggle must drive the flag that pushFaults() sends to the
  // engine (vsim_ctl_faults_t.imu_dropout); it used to be dead code. Offscreen
  // check of the flag only; that the engine reacts is a manual run (see PR).
  void imuDropoutToggleSetsTheFaultFlag() {
    auto *box = w_->findChild<QCheckBox *>(QStringLiteral("ImuDropoutCheck"));
    QVERIFY2(box, "IMU dropout checkbox not found");
    QVERIFY(box->isEnabled());
    QVERIFY(!a_->imuDropout());
    box->setChecked(true);
    QVERIFY2(a_->imuDropout(), "toggle did not set the dropout flag");
    box->setChecked(false);
    QVERIFY(!a_->imuDropout());
  }

  // The IMU/Physics/Render rate combos have no effect on the engine, so they
  // are hidden. Offscreen: no combo listing "NNN Hz" may be visible on the page.
  void nonFunctionalRateCombosAreHidden() {
    int hz = 0;
    // Judge visibility relative to the World page: the stack hides the pages
    // that are not current, which would make this pass vacuously.
    QWidget *world = a_->stack()->widget(1);
    for (auto *c : world->findChildren<QComboBox *>()) {
      if (c->count() == 0 || !c->itemText(0).endsWith(QStringLiteral(" Hz")))
        continue;
      ++hz;
      QVERIFY2(!c->isVisibleTo(world), "a rate combo is still shown");
    }
    QCOMPARE(hz, 3); // they still exist (settings stay), just hidden
  }

  void sectionTitlesAreClean() {
    int sections = 0;
    const auto btns = w_->findChildren<QToolButton *>();
    for (const QToolButton *b : btns) {
      const QString t = b->text();
      if (t.isEmpty() || !b->isCheckable() ||
          b->toolButtonStyle() != Qt::ToolButtonTextBesideIcon)
        continue;
      ++sections;
      QString plain = t;
      plain.replace(QStringLiteral("&&"), QString());
      QVERIFY2(!t.contains('_'), qPrintable("underscore in title: " + t));
      QVERIFY2(!plain.contains('&'),
               qPrintable("lone '&' (a mnemonic marker) in title: " + t));
    }
    QVERIFY2(sections > 0, "no section titles found");
  }
};

int main(int argc, char **argv) {
  QTemporaryDir cfg;
  qputenv("XDG_CONFIG_HOME", cfg.path().toLocal8Bit());
  QApplication::setOrganizationName(QStringLiteral("navigator-test"));
  QApplication::setApplicationName(QStringLiteral("tst_sim_page_size"));
  QApplication app(argc, argv);
  TstSimPageSize t;
  return QTest::qExec(&t, argc, argv);
}

#include "tst_sim_page_size.moc"
