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
// FAILSAFE_STATUS display rules: "unknown" until the first message, the
// arm-refusal reason only while disarmed, an unknown refusal code is still a
// refusal, and every landing cause is named.

#include "FailsafeStatus.h"

#include <QtTest>

class TstFailsafeStatus : public QObject {
  Q_OBJECT
private slots:
  void unknownUntilFirstMessage();
  void okWhenNothingIsGoingOn();
  void battLevelTexts();
  void unknownBattLevelIsNotOk();
  void everyLandingCauseIsNamed();
  void landingFlagWithoutCauseStillSaysLanding();
  void armRefusalShownOnlyWhileDisarmed();
  void armRefusalAloneIsNotAnAlarm();
  void unknownRefusalCodeIsStillARefusal();
  void everyKnownRefusalHasText();
  void pilotTookOverAndUntrustedAreShown();
  void alarmFollowsLevelOrLanding();
};

void TstFailsafeStatus::unknownUntilFirstMessage() {
  // Even a status that reads "all fine" must not show OK before one arrives.
  const FailsafeStatusData ok{};
  QCOMPARE(failsafe::statusLine(false, false, ok),
           QStringLiteral("Failsafe: unknown"));
  QCOMPARE(failsafe::statusLine(false, true, ok),
           QStringLiteral("Failsafe: unknown"));
  // And a refusal in the data is not shown while unknown either.
  FailsafeStatusData r{};
  r.armRefusal = NAVLINK_ARM_REFUSAL_NO_RC_SIGNAL;
  QCOMPARE(failsafe::statusLine(false, false, r),
           QStringLiteral("Failsafe: unknown"));
}

void TstFailsafeStatus::okWhenNothingIsGoingOn() {
  const FailsafeStatusData ok{};
  QCOMPARE(failsafe::statusLine(true, false, ok),
           QStringLiteral("Failsafe: OK"));
  QCOMPARE(failsafe::statusLine(true, true, ok),
           QStringLiteral("Failsafe: OK"));
}

void TstFailsafeStatus::battLevelTexts() {
  QCOMPARE(failsafe::battLevelText(NAVLINK_BATT_LEVEL_OK),
           QStringLiteral("Battery OK"));
  QCOMPARE(failsafe::battLevelText(NAVLINK_BATT_LEVEL_WARN),
           QStringLiteral("Battery low"));
  QCOMPARE(failsafe::battLevelText(NAVLINK_BATT_LEVEL_LAND),
           QStringLiteral("Battery low, landing"));
  QCOMPARE(failsafe::battLevelText(NAVLINK_BATT_LEVEL_CRITICAL),
           QStringLiteral("Battery critical, landing"));
  FailsafeStatusData w{};
  w.battLevel = NAVLINK_BATT_LEVEL_WARN;
  QCOMPARE(failsafe::statusLine(true, true, w),
           QStringLiteral("Battery low"));
}

void TstFailsafeStatus::unknownBattLevelIsNotOk() {
  // A level a newer FC could add must not read as "Battery OK".
  const QString t = failsafe::battLevelText(9);
  QVERIFY(t != failsafe::battLevelText(NAVLINK_BATT_LEVEL_OK));
  QVERIFY(t.contains(QStringLiteral("unknown")));
  FailsafeStatusData s{};
  s.battLevel = 9;
  QVERIFY(failsafe::isAlarm(s));
}

void TstFailsafeStatus::everyLandingCauseIsNamed() {
  QCOMPARE(failsafe::landCauseText(0), QString());
  QCOMPARE(failsafe::landCauseText(NAVLINK_LANDING_CAUSE_RC_LOSS),
           QStringLiteral("RC link lost"));
  QCOMPARE(failsafe::landCauseText(NAVLINK_LANDING_CAUSE_ESTIMATOR),
           QStringLiteral("estimator fault"));
  QCOMPARE(failsafe::landCauseText(NAVLINK_LANDING_CAUSE_BATT),
           QStringLiteral("low battery"));
  QCOMPARE(failsafe::landCauseText(NAVLINK_LANDING_CAUSE_BATT_CRIT),
           QStringLiteral("critical battery"));
  // Several at once: all of them, in a fixed order.
  QCOMPARE(failsafe::landCauseText(NAVLINK_LANDING_CAUSE_RC_LOSS |
                                   NAVLINK_LANDING_CAUSE_BATT_CRIT),
           QStringLiteral("RC link lost, critical battery"));
  FailsafeStatusData s{};
  s.landCause = NAVLINK_LANDING_CAUSE_RC_LOSS;
  s.flags = NAVLINK_FAILSAFE_FLAGS_LANDING;
  QCOMPARE(failsafe::statusLine(true, true, s),
           QStringLiteral("Landing: RC link lost"));
}

void TstFailsafeStatus::landingFlagWithoutCauseStillSaysLanding() {
  FailsafeStatusData s{};
  s.flags = NAVLINK_FAILSAFE_FLAGS_LANDING;
  QVERIFY(failsafe::isLanding(s));
  QCOMPARE(failsafe::statusLine(true, true, s), QStringLiteral("Landing"));
  // And a cause bit alone counts too, even if the flag were missing.
  FailsafeStatusData c{};
  c.landCause = NAVLINK_LANDING_CAUSE_ESTIMATOR;
  QVERIFY(failsafe::isLanding(c));
}

void TstFailsafeStatus::armRefusalShownOnlyWhileDisarmed() {
  FailsafeStatusData s{};
  s.armRefusal = NAVLINK_ARM_REFUSAL_THROTTLE_NOT_MIN;
  // Disarmed: the pilot needs the reason.
  QCOMPARE(failsafe::statusLine(true, false, s),
           QStringLiteral("Arm refused: throttle not at minimum"));
  // Armed or in flight the FC always reads THROTTLE_NOT_MIN: true, useless.
  QCOMPARE(failsafe::statusLine(true, true, s),
           QStringLiteral("Failsafe: OK"));
}

void TstFailsafeStatus::armRefusalAloneIsNotAnAlarm() {
  FailsafeStatusData s{};
  s.armRefusal = NAVLINK_ARM_REFUSAL_NO_IMU_SAMPLES;
  QVERIFY(!failsafe::isAlarm(s));
}

void TstFailsafeStatus::unknownRefusalCodeIsStillARefusal() {
  FailsafeStatusData s{};
  s.armRefusal = 200; // a reason a newer FC added at the end of the list
  const QString line = failsafe::statusLine(true, false, s);
  QVERIFY(line != QStringLiteral("Failsafe: OK"));
  QVERIFY(line.contains(QStringLiteral("reason unknown")));
}

void TstFailsafeStatus::everyKnownRefusalHasText() {
  for (uint8_t c = NAVLINK_ARM_REFUSAL_SWITCH_NOT_DOWN;
       c <= NAVLINK_ARM_REFUSAL_BATTERY_LOW; ++c) {
    const QString t = failsafe::armRefusalText(c);
    QVERIFY2(!t.isEmpty(), qPrintable(QString::number(c)));
    QVERIFY2(!t.contains(QStringLiteral("unknown")), qPrintable(t));
  }
  QCOMPARE(failsafe::armRefusalText(NAVLINK_ARM_REFUSAL_NONE), QString());
}

void TstFailsafeStatus::pilotTookOverAndUntrustedAreShown() {
  FailsafeStatusData s{};
  s.battLevel = NAVLINK_BATT_LEVEL_LAND;
  s.landCause = NAVLINK_LANDING_CAUSE_BATT;
  s.flags = NAVLINK_FAILSAFE_FLAGS_LANDING |
            NAVLINK_FAILSAFE_FLAGS_PILOT_TOOK_OVER |
            NAVLINK_FAILSAFE_FLAGS_BATT_UNTRUSTED;
  const QString line = failsafe::statusLine(true, true, s);
  QVERIFY(line.contains(QStringLiteral("Landing: low battery")));
  QVERIFY(line.contains(QStringLiteral("pilot took over")));
  QVERIFY(line.contains(QStringLiteral("battery reading not trusted")));
}

void TstFailsafeStatus::alarmFollowsLevelOrLanding() {
  FailsafeStatusData s{};
  QVERIFY(!failsafe::isAlarm(s));
  s.battLevel = NAVLINK_BATT_LEVEL_WARN;
  QVERIFY(failsafe::isAlarm(s));
  s = {};
  s.landCause = NAVLINK_LANDING_CAUSE_RC_LOSS;
  QVERIFY(failsafe::isAlarm(s));
}

QTEST_MAIN(TstFailsafeStatus)
#include "tst_failsafe_status.moc"
