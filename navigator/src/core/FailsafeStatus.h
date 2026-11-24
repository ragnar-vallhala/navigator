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
#pragma once

// What the flight controller's own safety logic has decided (NavLink
// FAILSAFE_STATUS), as text for the pilot. Pure: no GUI, no engine, so the
// display rules are unit tested.
//
// The rules that matter:
//   * Nothing received yet (or nothing recent) reads "unknown", never "OK": an
//     older FC does not send this message, and a pilot must not read silence as
//     "all clear".
//   * The arm-refusal reason is shown only while DISARMED. In flight the FC's
//     reason reads THROTTLE_NOT_MIN, which is true but meaningless there.
//   * A refusal code this build does not know is "refused, reason unknown",
//     never "OK": the list can grow at the end.

#include "Types.h"
#include "navlink_msgs.h" // NAVLINK_BATT_LEVEL_*, NAVLINK_LANDING_CAUSE_*, ...

#include <QString>
#include <QStringList>

#include <cstdint>

namespace failsafe {

// The battery level the FC decided on, as a short phrase.
inline QString battLevelText(uint8_t level) {
  switch (level) {
  case NAVLINK_BATT_LEVEL_OK:
    return QStringLiteral("Battery OK");
  case NAVLINK_BATT_LEVEL_WARN:
    return QStringLiteral("Battery low");
  case NAVLINK_BATT_LEVEL_LAND:
    return QStringLiteral("Battery low, landing");
  case NAVLINK_BATT_LEVEL_CRITICAL:
    return QStringLiteral("Battery critical, landing");
  default:
    return QStringLiteral("Battery level unknown");
  }
}

// Why the FC is landing by itself: every set bit, in a fixed order. Empty when
// it is not landing. Several can be set at once.
inline QString landCauseText(uint8_t mask) {
  QStringList c;
  if (mask & NAVLINK_LANDING_CAUSE_RC_LOSS)
    c << QStringLiteral("RC link lost");
  if (mask & NAVLINK_LANDING_CAUSE_ESTIMATOR)
    c << QStringLiteral("estimator fault");
  if (mask & NAVLINK_LANDING_CAUSE_BATT)
    c << QStringLiteral("low battery");
  if (mask & NAVLINK_LANDING_CAUSE_BATT_CRIT)
    c << QStringLiteral("critical battery");
  return c.join(QStringLiteral(", "));
}

// Why an arm request would be refused. Empty for "nothing refuses it". A code
// this build does not know is still a refusal.
inline QString armRefusalText(uint8_t code) {
  switch (code) {
  case NAVLINK_ARM_REFUSAL_NONE:
    return QString();
  case NAVLINK_ARM_REFUSAL_SWITCH_NOT_DOWN:
    return QStringLiteral("arm switch not down");
  case NAVLINK_ARM_REFUSAL_THROTTLE_NOT_MIN:
    return QStringLiteral("throttle not at minimum");
  case NAVLINK_ARM_REFUSAL_NO_RC_SIGNAL:
    return QStringLiteral("no RC signal");
  case NAVLINK_ARM_REFUSAL_ESTIMATOR_DEGRADED:
    return QStringLiteral("estimator degraded");
  case NAVLINK_ARM_REFUSAL_NO_INITIAL_ESTIMATE:
    return QStringLiteral("no initial estimate yet");
  case NAVLINK_ARM_REFUSAL_NO_IMU_SAMPLES:
    return QStringLiteral("no IMU samples");
  case NAVLINK_ARM_REFUSAL_BATTERY_LOW:
    return QStringLiteral("battery low");
  default:
    return QStringLiteral("refused, reason unknown");
  }
}

// True when the FC is landing by itself, from either the flag or a cause bit.
inline bool isLanding(const FailsafeStatusData &s) {
  return (s.flags & NAVLINK_FAILSAFE_FLAGS_LANDING) != 0 || s.landCause != 0;
}

// True when something the pilot should notice is going on: a battery level
// above OK or an automatic landing. An arm refusal alone is not an alarm.
inline bool isAlarm(const FailsafeStatusData &s) {
  return s.battLevel != NAVLINK_BATT_LEVEL_OK || isLanding(s);
}

// One line for the status area.
//   fresh : a FAILSAFE_STATUS arrived recently (false until the first one).
//   armed : the vehicle is armed or in flight (arm refusal is hidden then).
inline QString statusLine(bool fresh, bool armed, const FailsafeStatusData &s) {
  if (!fresh)
    return QStringLiteral("Failsafe: unknown");

  QStringList parts;
  if (isLanding(s)) {
    const QString why = landCauseText(s.landCause);
    parts << (why.isEmpty() ? QStringLiteral("Landing")
                            : QStringLiteral("Landing: ") + why);
  }
  if (s.battLevel != NAVLINK_BATT_LEVEL_OK)
    parts << battLevelText(s.battLevel);
  if (s.flags & NAVLINK_FAILSAFE_FLAGS_PILOT_TOOK_OVER)
    parts << QStringLiteral("pilot took over");
  if (s.flags & NAVLINK_FAILSAFE_FLAGS_BATT_UNTRUSTED)
    parts << QStringLiteral("battery reading not trusted");
  if (!armed && s.armRefusal != NAVLINK_ARM_REFUSAL_NONE)
    parts << QStringLiteral("Arm refused: ") + armRefusalText(s.armRefusal);

  if (parts.isEmpty())
    return QStringLiteral("Failsafe: OK");
  return parts.join(QStringLiteral(" | "));
}

} // namespace failsafe
