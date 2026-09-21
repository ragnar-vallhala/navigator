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
#include "Units.h"

#include <QtMath>

#include <algorithm>

namespace {
Units::AngleUnit g_angle = Units::AngleUnit::Degrees;
Units::AltUnit g_alt = Units::AltUnit::Meters;
Units::SpeedUnit g_speed = Units::SpeedUnit::Mps;
int g_decimals = 2;
} // namespace

namespace Units {

void setAngleUnit(AngleUnit u) { g_angle = u; }
void setAltUnit(AltUnit u) { g_alt = u; }
void setSpeedUnit(SpeedUnit u) { g_speed = u; }
void setDecimals(int n) { g_decimals = std::clamp(n, 0, 6); }

int decimals() { return g_decimals; }

double toAngle(double deg) {
  return g_angle == AngleUnit::Radians ? qDegreesToRadians(deg) : deg;
}

double toAltitude(double metres) {
  return g_alt == AltUnit::Feet ? metres * 3.280839895 : metres;
}

double toSpeed(double mps) {
  switch (g_speed) {
  case SpeedUnit::Kmh:
    return mps * 3.6;
  case SpeedUnit::Mph:
    return mps * 2.236936292;
  case SpeedUnit::Mps:
    break;
  }
  return mps;
}

QString angleSuffix() {
  return g_angle == AngleUnit::Radians ? QStringLiteral(" rad")
                                       : QStringLiteral("°");
}

QString altSuffix() {
  return g_alt == AltUnit::Feet ? QStringLiteral("ft") : QStringLiteral("m");
}

QString speedSuffix() {
  switch (g_speed) {
  case SpeedUnit::Kmh:
    return QStringLiteral("km/h");
  case SpeedUnit::Mph:
    return QStringLiteral("mph");
  case SpeedUnit::Mps:
    break;
  }
  return QStringLiteral("m/s");
}

QString angle(double deg, int fieldWidth) {
  return QStringLiteral("%1%2")
      .arg(toAngle(deg), fieldWidth, 'f', g_decimals)
      .arg(angleSuffix());
}

QString altitude(double metres) {
  return QStringLiteral("%1 %2")
      .arg(toAltitude(metres), 0, 'f', g_decimals)
      .arg(altSuffix());
}

QString speed(double mps) {
  return QStringLiteral("%1 %2")
      .arg(toSpeed(mps), 0, 'f', g_decimals)
      .arg(speedSuffix());
}

} // namespace Units
