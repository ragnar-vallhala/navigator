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
#include "AirframePresets.h"

#include <cmath>

namespace vsim {

namespace {

struct Row {
  const char *id;
  const char *name;
  const char *pack;
  float mass, propRadius, p; // kg, m, motor offset +-p in x and y [m]
  float ixx, iyy, izz;
  float kThrust, kMoment, maxOmega, tau;
  // Rate-loop gains {kp, ki, kd} for roll, pitch, yaw (firmware units, per
  // deg/s); the angle loop keeps the stock gains.
  float gains[3][3];
};

// Sourced numbers, ESTIMATE flags and derivations: the airframe-preset
// research note. max_omega assumes the real pack's loaded voltage (3.7 V per
// cell).
//
// Battery: the geometry message carries no battery, and the engine's pack model
// takes max_omega as "duty 1 on a 11.1 V (3S) supply" and scales rotor speed by
// pack voltage / 11.1 V. A preset's max_omega already includes its real pack,
// so VSIM_BATT_CELLS must be left at the engine default (3S): setting it to the
// real 6S/4S/1S would apply the voltage a second time (6S: 2x the speed). The
// sim therefore flies every preset on its table max_omega, scaled by the 3S
// pack's own voltage (about 1.0-1.13) and sagging as that pack
// drains; the HUD battery readout is the engine's 3S pack, not the real one.
constexpr Row kRows[] = {
    {"5in", "5-inch freestyle (V2207 1950KV)", "6S", 0.682f, 0.0622f, 0.0866f,
     1.54e-3f, 1.80e-3f, 3.06e-3f, 1.234e-6f, 1.59e-8f, 3365.0f, 0.0125f,
     {{4.5e-4f, 1.2e-3f, 7.5e-6f}, {3.8e-4f, 4.5e-4f, 7.5e-6f}, {6.0e-4f, 1.2e-3f, 0.0f}}},
    {"3in", "3-inch light (F1404 3800KV)", "4S", 0.180f, 0.0381f, 0.0502f,
     1.38e-4f, 1.57e-4f, 2.70e-4f, 3.46e-7f, 2.73e-9f, 2753.0f, 0.010f,
     {{3.3e-4f, 9.3e-4f, 5.5e-6f}, {2.7e-4f, 3.2e-4f, 5.4e-6f}, {4.2e-4f, 8.2e-4f, 0.0f}}},
    {"7in", "7-inch long range (F90 1300KV)", "6S", 1.165f, 0.0889f, 0.1156f,
     3.78e-3f, 4.62e-3f, 7.70e-3f, 5.49e-6f, 9.32e-8f, 1942.0f, 0.020f,
     {{3.3e-4f, 8.8e-4f, 5.4e-6f}, {2.8e-4f, 3.4e-4f, 5.7e-6f}, {4.5e-4f, 9.0e-4f, 0.0f}}},
    {"10in", "10-inch heavy lift (MN3110 470KV)", "6S", 1.520f, 0.127f,
     0.1485f, 1.12e-2f, 1.30e-2f, 2.29e-2f, 1.076e-5f, 2.41e-7f, 901.0f,
     0.030f,
     {{6.7e-4f, 1.8e-3f, 1.1e-5f}, {5.6e-4f, 6.7e-4f, 1.1e-5f}, {9.1e-4f, 1.8e-3f, 0.0f}}},
    {"whoop65", "65 mm whoop (0802 25000KV)", "1S", 0.029f, 0.0155f, 0.0230f,
     6.2e-6f, 7.1e-6f, 1.27e-5f, 8.15e-9f, 2.71e-11f, 5668.0f, 0.020f,
     {{6.0e-4f, 1.6e-3f, 1.0e-5f}, {4.8e-4f, 6.0e-4f, 1.0e-5f}, {8.4e-4f, 1.7e-3f, 0.0f}}},
};

GeometryConfig geometryFromRow(const Row &r) {
  GeometryConfig g;
  g.mass = r.mass;
  g.propRadius = r.propRadius;
  g.inertia = {r.ixx, 0, 0, 0, r.iyy, 0, 0, 0, r.izz};
  // Fixed by the firmware mixer; it derives its mix from the signs only.
  // [0] front-right, [1] rear-right, [2] rear-left, [3] front-left.
  const float sx[4] = {+1, -1, -1, +1};
  const float sy[4] = {+1, +1, -1, -1};
  const int spin[4] = {+1, -1, +1, -1};
  for (int i = 0; i < 4; ++i) {
    MotorConfig &m = g.motors[i];
    m.pos = QVector3D(sx[i] * r.p, sy[i] * r.p, 0.0f);
    m.axis = QVector3D(0, 0, -1);
    m.spin = spin[i];
    m.k_thrust = r.kThrust;
    m.k_moment = r.kMoment;
    m.max_omega = r.maxOmega;
    m.tau = r.tau;
  }
  return g;
}

// Rate-loop gains from the row, angle loop stock (3.0 / 3.0 / 1.2 1/s).
//
// Why not the stock gains: the firmware defaults are a hardware tune. Flown in
// the sim on a clean gain store (VAYU_VFS_DIR empty, so no stale 0:pid.bin) the
// stock gains put the rate loop of the 5in, 3in, 7in and whoop into a limit
// cycle (motors slamming between idle and full, which rectifies into a runaway
// climb), and the 10in is marginal (22 deg attitude excursions in a hover).
//
// The 5-inch set is the stock rate gains x0.15; flown, it is stable up to
// x2.5 of itself and limit-cycles at x4 (onset x0.4-0.6 of stock). The others
// start from the same rule, which keeps the rate-loop crossover in place
// against the motor lag:
//     kp_i = kp_5in * (K_5in / K_i) * (tau_5in / tau_i)
// with K the roll/pitch/yaw plant gain (torque per unit mixer output over
// inertia) from each airframe's own k_thrust, max_omega, arm and inertia, and
// were then checked by flight. Margin to the limit cycle, as a multiple of the
// table gains: 5in 2.5-4, 7in and 10in over 4 (not found), whoop 2.5-4 (spread
// 0.65 at x4). The 3in limit-cycled at x2.5 with the rule's value, so its set
// is that value x0.85 (stable at x2 of the old value), for a margin of about 2.5.
// The whoop's set is 4x the rule's value: the rule's own left its step
// under-damped.
QVector<PidSetCmd> gainsFor(const Row &r) {
  QVector<PidSetCmd> g;
  for (int ax = 0; ax < 3; ++ax)
    g.push_back({1, ax, r.gains[ax][0], r.gains[ax][1], r.gains[ax][2], 0.0f});
  for (const PidSetCmd &c : stockPidGains())
    if (c.controller == 0)
      g.push_back(c);
  return g;
}

QVector<AirframePreset> buildPresets() {
  QVector<AirframePreset> out;
  for (const Row &r : kRows) {
    AirframePreset p;
    p.id = QString::fromLatin1(r.id);
    p.name = QString::fromUtf8(r.name);
    p.pack = QString::fromLatin1(r.pack);
    p.geometry = geometryFromRow(r);
    p.gains = gainsFor(r);
    out.push_back(p);
  }
  return out;
}

} // namespace

const QVector<AirframePreset> &airframePresets() {
  static const QVector<AirframePreset> presets = buildPresets();
  return presets;
}

const AirframePreset *findAirframePreset(const QString &id) {
  for (const AirframePreset &p : airframePresets())
    if (p.id == id)
      return &p;
  return nullptr;
}

QVector<PidSetCmd> stockPidGains() {
  // vayu firmware/include/control/tuning.h (DEAFULT_*).
  return {
      {1, 0, 0.003f, 0.00811f, 0.00005f, 0.0f},  // roll rate
      {1, 1, 0.0025f, 0.003f, 0.00005f, 0.0f},   // pitch rate
      {1, 2, 0.004f, 0.008f, 0.0f, 0.0f},        // yaw rate
      {0, 0, 3.0f, 0.0f, 0.0f, 0.0f},            // roll angle
      {0, 1, 3.0f, 0.0f, 0.0f, 0.0f},            // pitch angle
      {0, 2, 1.2f, 0.0f, 0.0f, 0.0f},            // yaw angle
  };
}

AirframeDerived deriveAirframe(const GeometryConfig &g) {
  constexpr double kG = 9.81;
  double thrustMax = 0.0;
  for (const MotorConfig &m : g.motors)
    thrustMax += double(m.k_thrust) * m.max_omega * m.max_omega;
  const double weight = double(g.mass) * kG;
  const MotorConfig &m0 = g.motors[0];
  AirframeDerived d;
  d.thrustToWeight = thrustMax / weight;
  d.hoverDuty = std::sqrt(weight / 4.0 / m0.k_thrust) / m0.max_omega;
  return d;
}

} // namespace vsim
