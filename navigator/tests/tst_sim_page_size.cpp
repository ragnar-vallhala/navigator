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
    w_->resize(1000, 600);
    QVERIFY2(w_->width() <= 1000,
             qPrintable(QString("page forced to %1 px wide").arg(w_->width())));
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
