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
// FAILSAFE_STATUS decode: a real frame, built with the generated encoder, goes
// through NavlinkRouter and arrives with every field, and only on its own
// callback. No handler set must be harmless.

#include "NavlinkRouter.h"

#include <QtTest>

extern "C" {
#include "navlink_msgs.h"
}

namespace {
QByteArray failsafeFrame(uint8_t level, uint8_t cause, uint8_t flags,
                         uint8_t refusal, uint8_t cells) {
  navlink_failsafe_status_t m{};
  m.batt_level = level;
  m.land_cause = cause;
  m.flags = flags;
  m.arm_refusal = refusal;
  m.cells = cells;
  uint8_t buf[NAVLINK_MAX_FRAME];
  const size_t n = navlink_failsafe_status_encode(buf, &m, 0, 1, 1);
  return QByteArray(reinterpret_cast<const char *>(buf), int(n));
}
} // namespace

class TstFailsafeDecode : public QObject {
  Q_OBJECT
private slots:
  void everyFieldArrives();
  void onlyItsOwnCallbackFires();
  void noHandlerIsHarmless();
  void twoFramesGiveTwoCallbacks();
};

void TstFailsafeDecode::everyFieldArrives() {
  NavlinkRouter r;
  int calls = 0;
  FailsafeStatusData got;
  r.onFailsafeStatus = [&](const FailsafeStatusData &d) {
    ++calls;
    got = d;
  };
  r.feed(failsafeFrame(NAVLINK_BATT_LEVEL_LAND,
                       NAVLINK_LANDING_CAUSE_BATT | NAVLINK_LANDING_CAUSE_RC_LOSS,
                       NAVLINK_FAILSAFE_FLAGS_LANDING |
                           NAVLINK_FAILSAFE_FLAGS_PILOT_TOOK_OVER,
                       NAVLINK_ARM_REFUSAL_BATTERY_LOW, 4));
  QCOMPARE(calls, 1);
  QCOMPARE(int(got.battLevel), int(NAVLINK_BATT_LEVEL_LAND));
  QCOMPARE(int(got.landCause), int(NAVLINK_LANDING_CAUSE_BATT |
                                   NAVLINK_LANDING_CAUSE_RC_LOSS));
  QCOMPARE(int(got.flags), int(NAVLINK_FAILSAFE_FLAGS_LANDING |
                               NAVLINK_FAILSAFE_FLAGS_PILOT_TOOK_OVER));
  QCOMPARE(int(got.armRefusal), int(NAVLINK_ARM_REFUSAL_BATTERY_LOW));
  QCOMPARE(int(got.cells), 4);
}

void TstFailsafeDecode::onlyItsOwnCallbackFires() {
  NavlinkRouter r;
  int battery = 0, failsafe = 0;
  r.onBattery = [&](const BatteryData &) { ++battery; };
  r.onFailsafeStatus = [&](const FailsafeStatusData &) { ++failsafe; };
  r.feed(failsafeFrame(0, 0, 0, 0, 3));
  QCOMPARE(failsafe, 1);
  QCOMPARE(battery, 0);
}

void TstFailsafeDecode::noHandlerIsHarmless() {
  NavlinkRouter r; // onFailsafeStatus left empty
  r.feed(failsafeFrame(NAVLINK_BATT_LEVEL_CRITICAL, 8, 1, 0, 3));
  QVERIFY(true); // reaching here without a crash is the check
}

void TstFailsafeDecode::twoFramesGiveTwoCallbacks() {
  NavlinkRouter r;
  QVector<int> levels;
  r.onFailsafeStatus = [&](const FailsafeStatusData &d) {
    levels.append(d.battLevel);
  };
  r.feed(failsafeFrame(NAVLINK_BATT_LEVEL_OK, 0, 0, 0, 3));
  r.feed(failsafeFrame(NAVLINK_BATT_LEVEL_WARN, 0, 0, 0, 3));
  QCOMPARE(levels.size(), 2);
  QCOMPARE(levels.at(0), int(NAVLINK_BATT_LEVEL_OK));
  QCOMPARE(levels.at(1), int(NAVLINK_BATT_LEVEL_WARN));
}

QTEST_MAIN(TstFailsafeDecode)
#include "tst_failsafe_decode.moc"
