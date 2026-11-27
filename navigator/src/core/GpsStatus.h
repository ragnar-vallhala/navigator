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

// The GPS receiver's report (NavLink GPS_RAW) as text for the pilot. Pure: no
// GUI, no engine, so the display rules are unit tested.
//
//   * Nothing received (or nothing recent) reads "no GPS", never a fix: only an
//     FC with a receiver sends GPS_RAW, and silence is not "locked".
//   * The text is the receiver's claim. When the FC says it does not believe the
//     fix (FAILSAFE_STATUS flags GPS_UNTRUSTED) the line says so and alarms.
//   * A fix type this build does not know is shown as its number, never as a fix.

#include "Types.h"
#include "navlink_msgs.h" // NAVLINK_GPS_FIX_TYPE_*, NAVLINK_FAILSAFE_FLAGS_GPS_UNTRUSTED

#include <QString>

#include <cstdint>

namespace gpsstatus {

inline QString fixText(uint8_t fixType) {
  switch (fixType) {
  case NAVLINK_GPS_FIX_TYPE_NO_FIX:
    return QStringLiteral("no fix");
  case NAVLINK_GPS_FIX_TYPE_DEAD_RECKONING:
    return QStringLiteral("dead reckoning");
  case NAVLINK_GPS_FIX_TYPE_FIX_2D:
    return QStringLiteral("2D fix");
  case NAVLINK_GPS_FIX_TYPE_FIX_3D:
    return QStringLiteral("3D fix");
  case NAVLINK_GPS_FIX_TYPE_GNSS_DR:
    return QStringLiteral("3D fix + dead reckoning");
  case NAVLINK_GPS_FIX_TYPE_TIME_ONLY:
    return QStringLiteral("time only");
  default:
    return QStringLiteral("fix type %1").arg(fixType);
  }
}

// The receiver gives a usable position: a 2D or better fix with gnssFixOK set.
inline bool hasPosition(const GpsRawData &d) {
  const bool kind = d.fixType == NAVLINK_GPS_FIX_TYPE_FIX_2D ||
                    d.fixType == NAVLINK_GPS_FIX_TYPE_FIX_3D ||
                    d.fixType == NAVLINK_GPS_FIX_TYPE_GNSS_DR;
  return kind && (d.navFlags & 1u);
}

// Colour the field for attention: no usable position, or the FC distrusts it.
inline bool isAlarm(bool fresh, const GpsRawData &d, uint8_t failsafeFlags) {
  if (!fresh)
    return false; // "no GPS" is not an alarm: many craft have none
  return !hasPosition(d) || (failsafeFlags & NAVLINK_FAILSAFE_FLAGS_GPS_UNTRUSTED);
}

// One line for the status area.
//   fresh         : a GPS_RAW arrived recently (false until the first one).
//   failsafeFlags : FAILSAFE_STATUS.flags (0 if none was received).
inline QString statusLine(bool fresh, const GpsRawData &d, uint8_t failsafeFlags) {
  if (!fresh)
    return QStringLiteral("no GPS");
  QString s = fixText(d.fixType) + QStringLiteral(", %1 sats").arg(d.sats);
  if (d.fixType != NAVLINK_GPS_FIX_TYPE_NO_FIX && !(d.navFlags & 1u))
    s += QStringLiteral(", fix not OK");
  if (hasPosition(d) && d.hAccMm != 0xFFFFFFFFu)
    s += QStringLiteral(", ±%1 m").arg(double(d.hAccMm) / 1000.0, 0, 'f', 1);
  if (failsafeFlags & NAVLINK_FAILSAFE_FLAGS_GPS_UNTRUSTED)
    s += QStringLiteral(", FC does not trust it");
  return s;
}

} // namespace gpsstatus
