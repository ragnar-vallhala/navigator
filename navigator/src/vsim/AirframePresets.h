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

#include "../ui/widgets/AutotuneGains.h" // PidSetCmd
#include "VsimTypes.h"

#include <QString>
#include <QVector>

namespace vsim {

// A built-in airframe: the geometry the Vehicle page loads and the firmware
// gain set the sim sends at Start (the sim runs the real firmware with one set
// of gains, so a light or a heavy airframe needs its own).
//
// Numbers and their sources / ESTIMATE flags: the airframe-preset research
// note; the table is in AirframePresets.cpp.
struct AirframePreset {
  QString id;   // stable key, persisted (e.g. "5in")
  QString name; // combo label
  QString pack; // the real airframe's battery, informational ("6S")
  GeometryConfig geometry;
  // CMD_SET_PID applies (controller 0=angle / 1=rate, axis 0=roll 1=pitch
  // 2=yaw), sent through the same path as the autotuner's Apply.
  QVector<PidSetCmd> gains;
};

// The five presets, 5-inch first (the default airframe).
const QVector<AirframePreset> &airframePresets();
// nullptr if id is unknown.
const AirframePreset *findAirframePreset(const QString &id);
// The firmware's compiled-in gain defaults (vayu control/tuning.h).
QVector<PidSetCmd> stockPidGains();

// Quantities derived from the motor fields, for sanity checks and display.
struct AirframeDerived {
  double thrustToWeight; // all 4 motors at duty 1, supply as specified
  double hoverDuty;      // sqrt(m g / 4 k) / max_omega, linear duty -> omega
};
AirframeDerived deriveAirframe(const GeometryConfig &g);

} // namespace vsim
