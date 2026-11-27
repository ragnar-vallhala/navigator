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

// Which in-flight commands the operator may send in which vehicle state.
// Pure (state name in, answer out) so the rules are unit-testable without a
// window. State names are the ones TelemetryEngine accepts from the HEARTBEAT
// nav_state; an empty or unrecognised name means the state is unknown.
#include <QString>
#include <cmath>

namespace armgate {

// How a DISARM request must be handled.
enum class DisarmPolicy {
  Send,    // on the ground with motors off or idle: send at once
  Confirm, // the vehicle may be flying: ask first, default Cancel
};

// Disarm cuts the motors. In ARMED (the firmware stays there for the first
// second of a climb or a slow low hover), IN_AIR or FAILSAFE, or when the state
// is unknown, that can drop the aircraft, so it needs an explicit
// confirmation; a single click or chord must never do it.
inline DisarmPolicy disarmPolicy(const QString &state) {
  static const char *const kSafe[] = {"UNINITIALIZED", "INIT",       "STANDBY",
                                      "PREARM",        "TERMINATED", "CALIBRATING"};
  for (const char *s : kSafe)
    if (state == QLatin1String(s))
      return DisarmPolicy::Send;
  return DisarmPolicy::Confirm;
}

// Calibration moves the sensors' trims and the firmware reads its stop frame
// (CMD_CALIBRATE_IMU which=0xFF) as a disarm on some builds (todo 36), so a
// calibration may start only when the vehicle is idle on the ground.
inline bool calibrationStartAllowed(const QString &state) {
  return state == QLatin1String("STANDBY") || state == QLatin1String("PREARM");
}

// Cancel is also offered while the routine runs (state CALIBRATING), but never
// while the vehicle is armed or flying.
inline bool calibrationCancelAllowed(const QString &state) {
  return calibrationStartAllowed(state) ||
         state == QLatin1String("CALIBRATING");
}

// Legacy CMD_SET_PID gains: reject what can never be a gain (NaN, infinity,
// negative). Per-controller ranges are the firmware's and the parameter
// catalog's to enforce; this only keeps garbage off the wire.
inline bool pidGainsValid(float kp, float ki, float kd, float kff) {
  for (float g : {kp, ki, kd, kff})
    if (!std::isfinite(g) || g < 0.0f)
      return false;
  return true;
}

} // namespace armgate
