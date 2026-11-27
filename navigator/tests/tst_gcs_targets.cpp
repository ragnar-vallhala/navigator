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
// tst_gcs_targets.cpp - which vehicle commands go to, what the geometry
// encoder refuses, and what a refused command says (todo 84).
#include "AckText.h"
#include "CommandCodec.h"

#include <QtTest>
#include <cmath>
#include <limits>

// target_sys is the first payload byte (header is 10 bytes).
static quint8 targetOf(const QByteArray &f) { return f.size() > 10 ? quint8(f[10]) : 0xEE; }

class TstGcsTargets : public QObject {
  Q_OBJECT
private slots:
  void init() { CommandCodec::setTargetSys(42); }
  void cleanup() { CommandCodec::setTargetSys(42); }

  void defaultsTo42UntilAHeartbeat() {
    QCOMPARE(CommandCodec::targetSys(), quint8(42));
    QCOMPARE(targetOf(CommandCodec::encodeArm()), quint8(42));
  }
  void commandsFollowTheHeartbeatSysid() {
    CommandCodec::setTargetSys(7);
    QCOMPARE(targetOf(CommandCodec::encodeArm()), quint8(7));
    QCOMPARE(targetOf(CommandCodec::encodeDisarm()), quint8(7));
    QCOMPARE(targetOf(CommandCodec::encodeSetFlightMode(2)), quint8(7));
    QCOMPARE(targetOf(CommandCodec::encodeCalibrate(0xFF)), quint8(7));
    // An explicit target still wins.
    QCOMPARE(targetOf(CommandCodec::encodeArm(9)), quint8(9));
  }
  void unsetAndBroadcastAreNotAVehicle() {
    CommandCodec::setTargetSys(7);
    CommandCodec::setTargetSys(0);
    CommandCodec::setTargetSys(0xFF);
    QCOMPARE(CommandCodec::targetSys(), quint8(7));
  }

  void geometryRefusesWhatItCannotSend() {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const float x[4] = {0.1f, -0.1f, -0.1f, 0.1f};
    const float y[4] = {0.1f, 0.1f, -0.1f, -0.1f};
    const float sp[4] = {1.f, -1.f, 1.f, -1.f};
    QVERIFY(!CommandCodec::encodeSetMotorGeometry(x, y, sp).isEmpty());
    float bx[4] = {0.1f, -0.1f, -0.1f, 0.1f};
    bx[2] = nan;
    QVERIFY(CommandCodec::encodeSetMotorGeometry(bx, y, sp).isEmpty());
    float by[4] = {0.1f, 0.1f, -0.1f, -0.1f};
    by[0] = inf;
    QVERIFY(CommandCodec::encodeSetMotorGeometry(x, by, sp).isEmpty());
    for (float bad : {0.0f, 0.5f, 2.0f, -0.5f, nan}) {
      float bs[4] = {1.f, -1.f, 1.f, -1.f};
      bs[3] = bad;
      QVERIFY(CommandCodec::encodeSetMotorGeometry(x, y, bs).isEmpty());
    }
  }

  void refusedCommandsSayWhy() {
    QCOMPARE(acktext::ackFailure(NAVLINK_MSGID_CMD_ARM, 2),
             QStringLiteral("ARM refused by the vehicle (denied)"));
    QCOMPARE(acktext::ackFailure(NAVLINK_MSGID_CMD_DISARM, 1),
             QStringLiteral("DISARM refused by the vehicle (temporarily rejected)"));
    QCOMPARE(acktext::ackFailure(NAVLINK_MSGID_CMD_SET_FLIGHT_MODE, 4),
             QStringLiteral("Flight mode change refused by the vehicle (failed)"));
    QCOMPARE(acktext::ackFailure(NAVLINK_MSGID_CMD_ARM, 9),
             QStringLiteral("ARM refused by the vehicle (result 9)"));
    // Nothing to say for success, a deferred answer, or commands that have
    // their own feedback.
    QVERIFY(acktext::ackFailure(NAVLINK_MSGID_CMD_ARM, 0).isEmpty());
    QVERIFY(acktext::ackFailure(NAVLINK_MSGID_CMD_ARM, 5).isEmpty());
    QVERIFY(acktext::ackFailure(NAVLINK_MSGID_CMD_CALIBRATE_IMU, 2).isEmpty());
  }
};

QTEST_APPLESS_MAIN(TstGcsTargets)
#include "tst_gcs_targets.moc"
