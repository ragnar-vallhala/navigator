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
// rcframe: the RC bridge's failsafe policy -- link loss sends the feeder's
// NO_SIGNAL (never channels / arm), and the arm override cannot arm while the
// link is lost or RC is disabled.

#include "RcFrame.h"

#include <QtTest>

class TstRcFrame : public QObject {
  Q_OBJECT
private slots:
  void normalFrameIsCsv();
  void linkLostSendsNoSignalOnly();
  void armOverrideSuppressedWhenLinkLost();
  void armOverrideIgnoredWhenDisabled();
  void armOverrideIgnoredWithNoSource();
  void openFailureIsLostSource();
  void reopenTimer();
  void armOverrideAppliesWhenLive();
  void deviceLossErrnoClassification();
  void uartLineClassification();
  void fullRangeThrottleUnchanged();
  void centredThrottleCannotArmInFullRange();
  void halfRangeCentreAndBelowAreMinimum();
  void halfRangeFullUpIsMaximum();
  void throttleDeadzoneSnapsToMin();
  void throttleInvertAppliesFirst();
  void axisToUsEndpoints();
};

void TstRcFrame::normalFrameIsCsv() {
  QCOMPARE(QString::fromStdString(
               rcframe::formatFrame(false, 1500, 1500, 1000, 1500, 2000, 1000)),
           QStringLiteral("1500,1500,1000,1500,2000,1000\n"));
}

void TstRcFrame::linkLostSendsNoSignalOnly() {
  const std::string f =
      rcframe::formatFrame(true, 1500, 1500, 1000, 1500, 2000, 1000);
  QCOMPARE(QString::fromStdString(f), QStringLiteral("NO_SIGNAL\n"));
  QVERIFY(f.find(',') == std::string::npos); // no channel data at all
}

void TstRcFrame::armOverrideSuppressedWhenLinkLost() {
  for (int ov : {-1, 0, 1})
    QCOMPARE(rcframe::resolveArm(true, true, true, ov, 1500), -1);
  QCOMPARE(rcframe::resolveArm(true, false, true, 1, 1500), -1);
}

void TstRcFrame::armOverrideIgnoredWhenDisabled() {
  // Disabled RC stays neutral: the override must not force arm=2000.
  QCOMPARE(rcframe::resolveArm(false, false, true, 1, 1500), 1500);
  QCOMPARE(rcframe::resolveArm(false, false, true, 0, 1500), 1500);
}

void TstRcFrame::armOverrideIgnoredWithNoSource() {
  // No input device: live neutral link, but the override must not arm.
  for (int ov : {-1, 0, 1})
    QCOMPARE(rcframe::resolveArm(false, true, false, ov, 1500), 1500);
  // A lost source still wins (no frame at all).
  QCOMPARE(rcframe::resolveArm(true, true, false, 1, 1500), -1);
}

void TstRcFrame::openFailureIsLostSource() {
  QVERIFY(rcframe::sourceLostAtOpen(true, false));  // selected, won't open
  QVERIFY(!rcframe::sourceLostAtOpen(true, true));  // selected, opened
  QVERIFY(!rcframe::sourceLostAtOpen(false, false)); // nothing selected
}

void TstRcFrame::reopenTimer() {
  int w = 500;
  int fires = 0;
  for (int t = 0; t < 1000; t += 20)
    fires += rcframe::reopenDue(w, 20, 500);
  QCOMPARE(fires, 2); // every 500 ms
  int z = 0;
  QVERIFY(rcframe::reopenDue(z, 20, 500)); // immediate when wait is 0
  QCOMPARE(z, 500);
}

void TstRcFrame::armOverrideAppliesWhenLive() {
  QCOMPARE(rcframe::resolveArm(false, true, true, -1, 1234), 1234);
  QCOMPARE(rcframe::resolveArm(false, true, true, 1, 1234), 2000);
  QCOMPARE(rcframe::resolveArm(false, true, true, 0, 1234), 1000);
}

void TstRcFrame::deviceLossErrnoClassification() {
  QVERIFY(rcframe::deviceLostErrno(ENODEV)); // unplugged joystick
  QVERIFY(rcframe::deviceLostErrno(EIO));
  QVERIFY(!rcframe::deviceLostErrno(EAGAIN)); // no event right now
  QVERIFY(!rcframe::deviceLostErrno(EINTR));
}

void TstRcFrame::uartLineClassification() {
  using rcframe::Line;
  QCOMPARE(rcframe::classifyLine("1500,1500,1000,1500,1000,1000"), Line::Frame);
  QCOMPARE(rcframe::classifyLine("NO_SIGNAL"), Line::NoSignal);
  QCOMPARE(rcframe::classifyLine("  NO_SIGNAL"), Line::NoSignal);
  QCOMPARE(rcframe::classifyLine("boot v1.2"), Line::Junk);
  QCOMPARE(rcframe::classifyLine(""), Line::Junk);
}

// Firmware arms only below this throttle value (us).
static constexpr int kArmMaxUs = 1100;

void TstRcFrame::fullRangeThrottleUnchanged() {
  // A throttle that rests fully down, as before: -32767 -> 1000, 0 -> 1500.
  QCOMPARE(rcframe::throttleToUs(-32767, false, false), 1000);
  QCOMPARE(rcframe::throttleToUs(0, false, false), 1500);
  QCOMPARE(rcframe::throttleToUs(32767, false, false), 2000);
  QCOMPARE(rcframe::throttleToUs(16383, false, false), 1749); // integer truncation
}

void TstRcFrame::centredThrottleCannotArmInFullRange() {
  // The reported failure: a spring-centred stick gives 1500 us, never below
  // the 1100 us arm limit. Half-range mode fixes exactly this.
  QVERIFY(rcframe::throttleToUs(0, false, false) >= kArmMaxUs);
  QVERIFY(rcframe::throttleToUs(0, false, true) < kArmMaxUs);
}

void TstRcFrame::halfRangeCentreAndBelowAreMinimum() {
  for (int v : {0, -1, -1000, -20000, -32767})
    QCOMPARE(rcframe::throttleToUs(v, false, true), 1000);
}

void TstRcFrame::halfRangeFullUpIsMaximum() {
  QCOMPARE(rcframe::throttleToUs(32767, false, true), 2000);
  const int mid = rcframe::throttleToUs(16383, false, true);
  QVERIFY(mid > 1450 && mid < 1550);
  QVERIFY(rcframe::throttleToUs(8000, false, true) <
          rcframe::throttleToUs(24000, false, true)); // monotonic
}

void TstRcFrame::throttleDeadzoneSnapsToMin() {
  // 1000 + 4% of the range is inside the dead band, 1000 + 6% is not.
  QCOMPARE(rcframe::throttleToUs(1300, false, true), 1000); // ~1040 us
  QVERIFY(rcframe::throttleToUs(2300, false, true) > 1050);  // ~1070 us
  // Full range: a stick resting a little above the bottom still arms.
  QCOMPARE(rcframe::throttleToUs(-30500, false, false), 1000); // ~1035 us
}

void TstRcFrame::throttleInvertAppliesFirst() {
  // Inverted axis: stick fully up reads -32767 and means minimum throttle.
  QCOMPARE(rcframe::throttleToUs(32767, true, false), 1000);
  QCOMPARE(rcframe::throttleToUs(-32767, true, false), 2000);
  QCOMPARE(rcframe::throttleToUs(-32767, true, true), 2000);
  QCOMPARE(rcframe::throttleToUs(32767, true, true), 1000);
}

void TstRcFrame::axisToUsEndpoints() {
  // Non-throttle channels keep the plain mapping, deadzone-free.
  QCOMPARE(rcframe::axisToUs(-32767, false), 1000);
  QCOMPARE(rcframe::axisToUs(0, false), 1500);
  QCOMPARE(rcframe::axisToUs(32767, false), 2000);
  QCOMPARE(rcframe::axisToUs(-30500, false), 1035); // not snapped
  QCOMPARE(rcframe::axisToUs(32767, true), 1000);
}

QTEST_MAIN(TstRcFrame)
#include "tst_rc_frame.moc"
