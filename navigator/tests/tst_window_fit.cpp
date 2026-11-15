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
};

QTEST_MAIN(TstWindowFit)
#include "tst_window_fit.moc"
