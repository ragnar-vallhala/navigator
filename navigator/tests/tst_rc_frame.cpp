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
  void armOverrideAppliesWhenLive();
  void deviceLossErrnoClassification();
  void uartLineClassification();
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
    QCOMPARE(rcframe::resolveArm(true, true, ov, 1500), -1);
  QCOMPARE(rcframe::resolveArm(true, false, 1, 1500), -1);
}

void TstRcFrame::armOverrideIgnoredWhenDisabled() {
  // Disabled RC stays neutral: the override must not force arm=2000.
  QCOMPARE(rcframe::resolveArm(false, false, 1, 1500), 1500);
  QCOMPARE(rcframe::resolveArm(false, false, 0, 1500), 1500);
}

void TstRcFrame::armOverrideAppliesWhenLive() {
  QCOMPARE(rcframe::resolveArm(false, true, -1, 1234), 1234);
  QCOMPARE(rcframe::resolveArm(false, true, 1, 1234), 2000);
  QCOMPARE(rcframe::resolveArm(false, true, 0, 1234), 1000);
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

QTEST_MAIN(TstRcFrame)
#include "tst_rc_frame.moc"
