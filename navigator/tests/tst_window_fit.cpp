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
// windowfit::fitToScreen: the main window never opens larger than the screen
// (down to its own minimum size). Pure, no widget, no display.

#include "WindowFit.h"

#include <QtTest>

using windowfit::fitRect;
using windowfit::fitToScreen;

class TstWindowFit : public QObject {
  Q_OBJECT
private slots:
  void fittingSizeIsKept() {
    QCOMPARE(fitToScreen({1300, 820}, {1920, 1080}, {1100, 700}),
             QSize(1300, 820));
    QCOMPARE(fitToScreen({1500, 800}, {1920, 1040}, {1100, 700}),
             QSize(1500, 800));
  }
  void tooWideIsCutToTheScreen() {
    // A 1500 px wide saved window on a 1366 px screen.
    const QSize s = fitToScreen({1500, 700}, {1366, 768}, {1100, 700});
    QVERIFY(s.width() <= 1366);
    QCOMPARE(s.height(), 700);
  }
  void tooTallIsCutToTheScreen() {
    const QSize s = fitToScreen({1300, 820}, {1920, 720}, {1100, 650});
    QCOMPARE(s.width(), 1300);
    QVERIFY(s.height() <= 720 - windowfit::kFrameAllowance);
  }
  void defaultOnA1280x720ScreenFits() {
    const QSize s = fitToScreen({1300, 820}, {1280, 720}, {1100, 700});
    QVERIFY(s.width() <= 1280);
    QVERIFY(s.height() >= 700); // the minimum wins over the frame allowance
    QVERIFY(s.width() >= 1100);
  }
  void neverBelowTheMinimum() {
    // Screen smaller than the minimum: stay at the minimum.
    QCOMPARE(fitToScreen({1300, 820}, {1024, 600}, {1100, 700}),
             QSize(1100, 700));
  }
  void exactlyTheScreenIsStillCut() {
    // A window as tall as the screen has no room for its title bar.
    const QSize s = fitToScreen({1280, 720}, {1280, 720}, {1100, 600});
    QVERIFY(s.height() < 720);
  }
  // ---- fitRect: position as well as size --------------------------------
  void rectThatFitsStaysWhereItIs() {
    const QRect want(QPoint(100, 60), QSize(1300, 820));
    QCOMPARE(fitRect(want, QRect(0, 0, 1920, 1080), {1100, 700}), want);
  }
  void rightEdgePulledInsideTheScreen() {
    // Saved at x=200, 1300 wide: cut off on a 1280 screen once narrowed.
    const QRect r =
        fitRect(QRect(QPoint(200, 0), QSize(1300, 700)), QRect(0, 0, 1280, 720), {1100, 600});
    QCOMPARE(r.width(), 1280);
    QCOMPARE(r.x(), 0);
    QVERIFY(r.right() <= 1279);
  }
  void narrowedWindowKeepsAsMuchOfItsPlaceAsFits() {
    // 1000 wide at x=500 on a 1280 screen: only the overhang is pulled back.
    const QRect r =
        fitRect(QRect(QPoint(500, 20), QSize(1000, 600)), QRect(0, 0, 1280, 720), {800, 500});
    QCOMPARE(r.size(), QSize(1000, 600));
    QCOMPARE(r.x(), 280);
    QCOMPARE(r.y(), 20);
  }
  void topPanelKeepsTheFrameBelowIt() {
    // A 30 px top panel: available area starts at y=30; a window at y=0 would
    // have its title bar under the panel.
    const QRect r = fitRect(QRect(QPoint(0, 0), QSize(1100, 600)),
                            QRect(0, 30, 1280, 690), {1100, 500});
    QCOMPARE(r.y(), 30);
  }
  void bottomEdgePulledUp() {
    const QRect avail(0, 0, 1280, 720);
    const QRect r = fitRect(QRect(QPoint(0, 400), QSize(1100, 600)), avail, {1100, 500});
    QVERIFY(r.y() + r.height() + windowfit::kFrameAllowance <= avail.bottom() + 1);
  }
  void minimumBiggerThanTheAreaPinsTopLeft() {
    const QRect r = fitRect(QRect(QPoint(300, 200), QSize(1300, 820)),
                            QRect(10, 20, 1024, 700), {1100, 700});
    QCOMPARE(r.size(), QSize(1100, 700));
    QCOMPARE(r.topLeft(), QPoint(10, 20));
  }
};

QTEST_MAIN(TstWindowFit)
#include "tst_window_fit.moc"
