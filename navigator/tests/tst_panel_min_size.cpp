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
// Panels hosted by AdsBackend report a content minimum size (PanelFrame): ADS
// alone says 60x40, so a panel docked in a short strip or floated small was
// squashed (Gyro Notch rows overlapped). Each registered panel here stands in
// for a real one with the content minimum measured in the app (offscreen,
// Fly workspace); gyro_notch is a real row layout so overlap can be checked.
// The reported minimum is the content's, capped (kMaxMinW x kMaxMinH) so tall
// or wide panels cannot push the main window past a small screen; below the
// content minimum the frame scrolls instead of squeezing the content.
#include "AdsBackend.h"
#include "PanelFrame.h"
#include "WorkspaceManager.h"

#include <DockAreaWidget.h>
#include <DockManager.h>
#include <DockWidget.h>
#include <FloatingDockContainer.h>

#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QScrollArea>
#include <QSettings>
#include <QSplitter>
#include <QTemporaryDir>
#include <QVBoxLayout>
#include <QtTest>

using namespace ws;

namespace {
struct PanelMin {
  const char *id;
  int w, h; // content minimum (setMinimumSize / layout minimumSizeHint)
};
// Measured from the real panels (see the commit message).
const PanelMin kPanels[] = {
    {"view3d", 280, 472},  {"imu", 490, 432},       {"logs", 311, 180},
    {"rc", 400, 600},      {"packets", 1168, 411},  {"calibration", 596, 549},
    {"motors", 670, 468},  {"control_loop", 719, 134}, {"gyro_notch", 256, 470},
    {"perf", 651, 325},    {"sim", 1423, 595},      {"settings", 170, 180},
};

// Gyro Notch's shape: a title, form rows and a button in a QVBoxLayout.
class RowsPanel : public QWidget {
public:
  QList<QWidget *> rows;
  RowsPanel() {
    auto *v = new QVBoxLayout(this);
    v->setContentsMargins(8, 8, 8, 8);
    v->setSpacing(6);
    for (int i = 0; i < 10; ++i) {
      QWidget *r = (i % 3 == 2) ? static_cast<QWidget *>(new QPushButton("row"))
                                : static_cast<QWidget *>(new QLabel("row"));
      r->setMinimumSize(240, 40);
      v->addWidget(r);
      rows << r;
    }
  }
};

QWidget *makePanel(const PanelMin &p) {
  if (QByteArray(p.id) == "gyro_notch")
    return new RowsPanel;
  auto *w = new QWidget;
  w->setMinimumSize(p.w, p.h);
  return w;
}

PanelRegistry registry() {
  PanelRegistry r;
  for (const PanelMin &p : kPanels) {
    const QString id = QString::fromLatin1(p.id);
    r.add(id, id, [p] { return makePanel(p); });
  }
  return r;
}

struct Fixture {
  QMainWindow win;
  WorkspaceManager *wm;
  AdsBackend *ads;
  Fixture() {
    win.resize(1100, 700);
    auto b = std::make_unique<AdsBackend>(&win);
    ads = b.get();
    wm = new WorkspaceManager(&win, std::move(b), &win);
    wm->registerPanels(registry());
    using N = LayoutNode;
    std::vector<WorkspaceManager::Builtin> bi;
    // A tall-enough anchor above every panel: the panel is the bottom strip.
    for (const PanelMin &p : kPanels)
      if (QByteArray(p.id) != "view3d")
        bi.push_back({QString("S_") + p.id,
                      {N::split(Qt::Vertical,
                                {N::tabs({"view3d"}), N::tabs({p.id})})}});
    wm->setBuiltins(bi);
    wm->setScreenProvider([] { return QList<QRect>{QRect(0, 0, 1600, 900)}; });
    win.show();
  }
  ads::CDockWidget *dock(const char *id) const {
    return win.findChild<ads::CDockWidget *>(QString::fromLatin1(id));
  }
};

QWidget *contentOf(ads::CDockWidget *d) {
  auto *sa = qobject_cast<QScrollArea *>(d->widget());
  return sa ? sa->widget() : d->widget();
}

// A panel's own minimum, the way a layout sees it.
QSize contentMin(const QWidget *w) { return PanelFrame::minimumOf(w); }

bool rowsOverlap(RowsPanel *p) {
  for (int i = 1; i < p->rows.size(); ++i)
    if (p->rows[i]->geometry().top() <= p->rows[i - 1]->geometry().bottom())
      return true;
  return false;
}
} // namespace

class TstPanelMinSize : public QObject {
  Q_OBJECT
  QTemporaryDir m_dir;

private slots:
  void initTestCase() {
    QVERIFY(m_dir.isValid());
    QCoreApplication::setOrganizationName("NAVRobotecTest");
    QCoreApplication::setApplicationName("tst_panel_min_size");
    QSettings::setDefaultFormat(QSettings::IniFormat);
    QSettings::setPath(QSettings::IniFormat, QSettings::UserScope,
                       m_dir.path());
  }

  // Only checks that the stand-ins were built to the table: the table holds the
  // sizes measured in the real app at one point in time, so this proves nothing
  // about the real panels, and the numbers go stale as panels change.
  void standInsMatchTable() {
    for (const PanelMin &p : kPanels) {
      QScopedPointer<QWidget> w(makePanel(p));
      QCOMPARE(contentMin(w.data()), QSize(p.w, p.h));
    }
  }

  void cappedMinimumFormula() {
    QCOMPARE(cappedMinimum({256, 477}), QSize(256, kMaxMinH));
    QCOMPARE(cappedMinimum({1423, 595}), QSize(kMaxMinW, kMaxMinH));
    QCOMPARE(cappedMinimum({170, 100}), QSize(170, 100));
    // The cap must keep the 1100x700 window workable: two stacked areas.
    QVERIFY(2 * kMaxMinH < 700 - 150);
  }

  // The dock widget (what ADS's splitters and floating windows read) reports
  // the panel's content minimum, capped; never ADS's default 60x40.
  void dockReportsContentMinimum() {
    for (const PanelMin &p : kPanels) {
      Fixture f;
      f.wm->load(QString("S_") + (QByteArray(p.id) == "imu" ? "logs" : "imu"));
      f.wm->backend()->showPanel(p.id);
      ads::CDockWidget *d = f.dock(p.id);
      QVERIFY2(d, p.id);
      const QSize want = cappedMinimum({p.w, p.h});
      const QSize got = d->minimumSizeHint();
      QVERIFY2(got.width() >= want.width() && got.height() >= want.height(),
               qPrintable(QString("%1: dock min %2x%3 < %4x%5")
                              .arg(p.id).arg(got.width()).arg(got.height())
                              .arg(want.width()).arg(want.height())));
      // ... and never past the cap
      QVERIFY2(got.width() <= kMaxMinW && got.height() <= kMaxMinH, p.id);
    }
  }

  // Docked in the smallest strip its splitter allows: the area keeps the
  // panel's minimum and the content is never given less than its own minimum.
  void shortStripKeepsContent() {
    for (const PanelMin &p : kPanels) {
      if (QByteArray(p.id) == "view3d")
        continue;
      Fixture f;
      f.wm->load(QString("S_") + p.id);
      ads::CDockWidget *d = f.dock(p.id);
      QVERIFY2(d && d->dockAreaWidget(), p.id);
      ads::CDockAreaWidget *area = d->dockAreaWidget();
      QWidget *par = area->parentWidget();
      while (par && !qobject_cast<QSplitter *>(par))
        par = par->parentWidget();
      auto *sp = qobject_cast<QSplitter *>(par);
      QVERIFY2(sp && sp->orientation() == Qt::Vertical, p.id);
      sp->setSizes({100000, 1});
      QTest::qWait(50);
      const QSize want = cappedMinimum({p.w, p.h});
      QVERIFY2(area->height() >= want.height(),
               qPrintable(QString("%1: strip %2 < %3")
                              .arg(p.id).arg(area->height()).arg(want.height())));
      const QSize cs = contentOf(d)->size(), cm = contentMin(contentOf(d));
      QVERIFY2(cs.width() >= cm.width() && cs.height() >= cm.height(),
               qPrintable(QString("%1: content %2x%3 < min %4x%5")
                              .arg(p.id).arg(cs.width()).arg(cs.height())
                              .arg(cm.width()).arg(cm.height())));
      if (auto *rp = dynamic_cast<RowsPanel *>(contentOf(d)))
        QVERIFY2(!rowsOverlap(rp), "rows overlap in the strip");
    }
  }

  // Floated and asked to be tiny: the window keeps the minimum, the content
  // keeps its own minimum (the frame scrolls), rows never overlap.
  void floatedSmallKeepsContent() {
    for (const PanelMin &p : kPanels) {
      if (QByteArray(p.id) == "view3d")
        continue;
      Fixture f;
      f.wm->load(QString("S_") + p.id);
      auto *b = f.wm->backend();
      b->floatPanel(p.id);
      for (QSize ask : {QSize(300, 90), QSize(200, 200), QSize(420, 300)}) {
        b->setFloatGeometry(p.id, QRect(20, 20, ask.width(), ask.height()));
        QTest::qWait(50);
        ads::CDockWidget *d = f.dock(p.id);
        QVERIFY2(d && d->isInFloatingContainer(), p.id);
        const QSize want = cappedMinimum({p.w, p.h});
        auto *fw = d->floatingDockContainer();
        // The window's minimum is what a window manager enforces (the
        // offscreen platform ignores size hints, so apply it here).
        fw->show();
        QTest::qWait(100);
        const QSize wmin = fw->minimumSizeHint();
        const QSize dmin = d->minimumSizeHint();
        QVERIFY2(wmin.width() >= want.width() && wmin.height() >= want.height(),
                 qPrintable(QString("%1: floating window min %2x%3 < %4x%5")
                                .arg(p.id).arg(wmin.width()).arg(wmin.height())
                                .arg(want.width()).arg(want.height())));
        QVERIFY2(dmin.width() >= want.width() && dmin.height() >= want.height(),
                 p.id);
        fw->resize(ask.expandedTo(wmin));
        QTest::qWait(50);
        const QSize cs = contentOf(d)->size(), cm = contentMin(contentOf(d));
        QVERIFY2(cs.width() >= cm.width() && cs.height() >= cm.height(),
                 qPrintable(QString("%1 asked %2x%3: content %4x%5 < min %6x%7")
                                .arg(p.id).arg(ask.width()).arg(ask.height())
                                .arg(cs.width()).arg(cs.height())
                                .arg(cm.width()).arg(cm.height())));
        if (auto *rp = dynamic_cast<RowsPanel *>(contentOf(d)))
          QVERIFY2(!rowsOverlap(rp), "rows overlap when floated");
      }
    }
  }
};

QTEST_MAIN(TstPanelMinSize)
#include "tst_panel_min_size.moc"
