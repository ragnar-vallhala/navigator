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

// HEARTBEAT.nav_state -> state name and "armed", in ONE place. The UI
// (TelemetryEngine), the parameter service and the command gates all used to
// derive "armed" separately, from the string or from the number, and disagreed
// on a state they did not know: the UI froze on the old state while the
// parameters treated it as armed.
//
// Rule: ARMED, IN_AIR and FAILSAFE are armed; UNINITIALIZED, INIT, STANDBY,
// PREARM, TERMINATED and CALIBRATING are not; any other value is UNKNOWN and
// counts as armed (fail safe: locks stay on, a disarm needs confirmation).
#include <QString>
#include <cstdint>

namespace navstate {

constexpr int kKnownCount = 9;
inline const char *const kNames[kKnownCount] = {
    "UNINITIALIZED", "INIT",       "STANDBY",    "PREARM",     "ARMED",
    "IN_AIR",        "FAILSAFE",   "TERMINATED", "CALIBRATING"};
inline const char *const kUnknown = "UNKNOWN";

inline bool known(uint8_t navState) { return navState < kKnownCount; }

// The state name shown to the operator; "UNKNOWN" for a value this build does
// not know.
inline const char *name(uint8_t navState) {
  return known(navState) ? kNames[navState] : kUnknown;
}

inline bool armed(uint8_t navState) {
  if (!known(navState))
    return true;
  return navState == 4 || navState == 5 || navState == 6;
}

// Same rule from a state name; `ok` is false for text that is not a state name
// at all (the status channel also carries non-state bytes), which callers must
// ignore rather than treat as armed.
inline bool armedByName(const QString &state, bool *ok = nullptr) {
  for (int i = 0; i < kKnownCount; ++i)
    if (state == QLatin1String(kNames[i])) {
      if (ok)
        *ok = true;
      return armed(uint8_t(i));
    }
  const bool unk = state == QLatin1String(kUnknown);
  if (ok)
    *ok = unk;
  return unk; // UNKNOWN is armed; anything else is not a state
}

} // namespace navstate
