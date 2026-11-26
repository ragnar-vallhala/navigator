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

#include "procgen/Flora.h"        // procgen::FloraParams (procedural tuning)
#include "procgen/TerrainField.h" // procgen::FieldParams

#include <QQuaternion>
#include <QString>
#include <QVector>
#include <QVector3D>

#include <array>
#include <cmath>

// Navigator-side type aliases for SimSnapshot fields. The full physics
// state structures (RigidBodyState, DroneParams, MotorParams,
// SensorNoise, ImuSample) live in sim/vsim/include/vsim_types.h, linked into
// the in-process engine. Navigator only consumes pose snapshots, so all it
// needs is the Qt-typed Vec3 / Quat the renderer reads off SimSnapshot.
namespace vsim {

using Vec3 = QVector3D;
using Quat = QQuaternion;

// Aerospace Tait-Bryan (ZYX) roll/pitch/yaw [deg] from a body->world quaternion
// in NED (X north, Y east, Z down). Qt's QQuaternion::getEulerAngles assumes a
// Y-up frame and permutes the axes for an NED airframe — the (drifting) heading
// leaks into "roll" — so use this for any attitude readout off a SimSnapshot.
inline void quatToEulerNED(const Quat &q, float *rollDeg, float *pitchDeg,
                           float *yawDeg) {
  const double w = q.scalar(), x = q.x(), y = q.y(), z = q.z();
  const double roll =
      std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  double sinp = 2.0 * (w * y - z * x);
  sinp = sinp > 1.0 ? 1.0 : (sinp < -1.0 ? -1.0 : sinp);
  const double pitch = std::asin(sinp);
  const double yaw =
      std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  constexpr double kRad2Deg = 57.29577951308232;
  *rollDeg = static_cast<float>(roll * kRad2Deg);
  *pitchDeg = static_cast<float>(pitch * kRad2Deg);
  *yawDeg = static_cast<float>(yaw * kRad2Deg);
}

// Per-rotor configuration as edited in the GeometryEditorWidget. Mirrors
// (a subset of) the physics-side MotorParams; pushed to the engine via
// SimWorker::sendGeometry -> VSIM_CTL_SET_GEOMETRY.
struct MotorConfig {
  QVector3D pos{0, 0, 0};     // body-frame [m]
  QVector3D axis{0, 0, -1};   // unit thrust axis (default -Z = up in NED)
  int spin = +1;              // +1 CCW / -1 CW (seen from above)
  float k_thrust = 1.234e-6f; // thrust_N = k_thrust * omega^2
  float k_moment = 1.59e-8f;  // reaction torque magnitude
  float max_omega = 3365.0f;  // duty=1 omega [rad/s]
  float tau = 0.0125f;        // first-order spin-up time constant [s]
};

// The full airframe geometry the editor produces: a mesh (GCS-side only,
// for rendering + inertia computation) plus the mass properties and motor
// layout pushed to the engine. meshPath/scale/com never cross the wire.
//
// The defaults are the "5-inch freestyle" airframe preset (AirframePresets.h:
// iFlight Nazgul5-class build, T-Motor V2207 1950KV on 6S); a test pins the two
// together.
struct GeometryConfig {
  QString meshPath;
  float scale = 1.0f;           // meters per mesh unit
  QVector3D translate{0, 0, 0}; // body-frame offset applied to the mesh [m]
  QVector3D rotate{0, 0, 0};    // body-frame orientation, XYZ Euler [deg]
  float mass = 0.682f;          // kg
  // Rotor radius [m] (half the prop diameter): sizes ground effect and the
  // contact footprint in the sim.
  float propRadius = 0.0622f;
  // Inertia tensor about the CoM, body frame [kg*m^2], row-major.
  std::array<float, 9> inertia = {1.54e-3f, 0,       0, 0, 1.80e-3f,
                                  0,        0,       0, 3.06e-3f};
  QVector3D com{0, 0, 0}; // CoM offset from model origin [m]
  // Motor order and spin are fixed by the firmware mixer: [0] front-right
  // (+x,+y) CCW, [1] rear-right (-x,+y) CW, [2] rear-left (-x,-y) CCW,
  // [3] front-left (+x,-y) CW; body NED, x forward, y right.
  std::array<MotorConfig, 4> motors = {{
      {{+0.0866f, +0.0866f, 0}, {0, 0, -1}, +1, 1.234e-6f, 1.59e-8f, 3365.0f,
       0.0125f},
      {{-0.0866f, +0.0866f, 0}, {0, 0, -1}, -1, 1.234e-6f, 1.59e-8f, 3365.0f,
       0.0125f},
      {{-0.0866f, -0.0866f, 0}, {0, 0, -1}, +1, 1.234e-6f, 1.59e-8f, 3365.0f,
       0.0125f},
      {{+0.0866f, -0.0866f, 0}, {0, 0, -1}, -1, 1.234e-6f, 1.59e-8f, 3365.0f,
       0.0125f},
  }};
};

// Environment + aerodynamics edited in the World tab; pushed to the engine via
// SimWorker::sendWorld -> VSIM_CTL_SET_WORLD. NED: +Z is down, so gravity
// is positive and the ground plane sits at ground_z.
// A static world obstacle. Rendered GCS-side now; collision in the physics is a
// later phase (restitution carried so the wire format won't change then).
// All in the NED world frame the drone pose uses (z down → on-ground center
// sits at z = -size.z/2). size: box = full extents; sphere = x is radius;
// cylinder = x is radius, z is height.
struct Obstacle {
  enum Type { Box = 0, Sphere = 1, Cylinder = 2 };
  int type = Box;
  QVector3D pos{2.0f, 0.0f, -0.5f};
  QVector3D size{1.0f, 1.0f, 1.0f};
  QVector3D rotate{0.0f, 0.0f, 0.0f}; // Euler XYZ [deg]
  float restitution = 0.3f;
  // Look only (procgen::PropStyle): tree trunk / crown, warehouse, container...
  // 0 = by shape. Not used by the physics.
  int style = 0;
};

struct WorldConfig {
  float gravity = 9.81f;       // m/s^2
  float ground_z = 0.0f;       // NED z of ground [m]
  float restitution = 0.0f;    // bounce factor [0,1]
  float linear_drag = 0.10f;   // N per (m/s)
  float angular_drag = 0.005f; // N*m per (rad/s)
  // Ground-contact righting: how hard/fast a tipped airframe topples to level.
  float ground_right_gain = 40.0f; // rad/s² per sin(tilt)
  float ground_right_damp = 6.0f;  // 1/s
  QVector<Obstacle> obstacles;     // static world shapes

  // Imported world mesh (rendered now; collidable in a later phase).
  QString worldMeshPath;   // empty = none
  float worldScale = 1.0f; // mesh units -> metres
  int worldUpAxis = 0;     // 0 = Z-up (Blender), 1 = Y-up (glTF)
  float worldMeshRestitution = 0.3f;
  bool worldMeshDoubleSided = true;
  QVector3D worldMeshOffset{0, 0, 0}; // NED world-space placement [m]

  // Procedural world (Phase 0). When proceduralBiome is non-empty it takes
  // precedence over worldMeshPath: the world is generated from these params
  // instead of imported, then fed to the same render + collision pipeline.
  QString proceduralBiome;        // empty = none; "meadow" supported now
  quint32 proceduralSeed = 1337u; // same seed+params -> same world
  float proceduralSizeM = 256.0f; // square extent [m]
  int proceduralResolution = 192; // grid samples per side

  // Live-tunable generator params (exposed in the World tab's Procedural
  // section). seed is driven by proceduralSeed; the rest are user knobs.
  QString worldPreset; // id of the preset that filled the fields ("" = custom)
  procgen::FieldParams field; // terrain shape + surface colour bands
  procgen::FloraParams flora; // grass density / slope / height
  procgen::GrassLook look;    // live grass/terrain shading knobs
};

// World-frame wind field (VSIM_CTL_SET_WIND): steady + gust + turbulence.
// enabled=false is a fast bypass in the physics (no wind, no RNG draw).
struct WindConfig {
  QVector3D steady{0, 0, 0}; // NED [m/s]  (windN, windE, windD)
  float gustAmp = 0.0f;      // peak gust [m/s]
  float gustPeriod = 0.0f;   // [s], <=0 disables the gust
  float turbSigma = 0.0f;    // turbulence RMS [m/s]
  float turbTau = 1.0f;      // correlation time [s]
  bool enabled = false;
};

// What a barometer does to the true pressure (VSIM_CTL_SET_BARO_ERR). All zero
// is off: the reading is the exact pressure. About 12 Pa is one metre of height
// near sea level.
struct BaroErrConfig {
  float noisePa = 0.0f;         // white noise per reading, 1 sigma [Pa], 0..500
  float walkPaPerSqrtS = 0.0f;  // random-walk offset [Pa/sqrt(s)], 0..50
  float lagMs = 0.0f;           // first-order response time constant, 0..2000
  float rateHz = 0.0f;          // output rate; 0 = every physics step, else 1..1000
};

// A modelled GNSS receiver (VSIM_CTL_SET_GPS). Off is the default: no receiver,
// no fix. Enabled, the engine makes a fix rateHz times a second from the true
// position and velocity, `lagMs` old, with white noise per axis. Rate and lag
// are what an RTL or failsafe test needs to vary, so both are exposed.
struct GpsConfig {
  bool enabled = false;
  float rateHz = 10.0f;      // fixes per second, 1..50
  float posNoiseM = 0.0f;    // position noise per axis, 1 sigma [m], 0..50
  float velNoiseMps = 0.0f;  // velocity noise per axis, 1 sigma [m/s], 0..5
  float lagMs = 0.0f;        // age of the reported data [ms], 0..1000
  int sats = 12;             // satellites reported, 4..32
};

// A fault in the modelled receiver (VSIM_CTL_SET_GPS_GLITCH). It acts only
// while the receiver is enabled, and sending a new GpsConfig clears it.
enum class GpsGlitchMode : int {
  None = 0,     // clears any glitch
  Offset = 1,   // the reported position jumps by the NED offset and stays
  Dropout = 2,  // loss of fix: no new reports, fix type 0, no satellites
  Freeze = 3,   // reports keep coming but repeat the last good position
};
struct GpsGlitchConfig {
  GpsGlitchMode mode = GpsGlitchMode::None;
  float offsetN = 0.0f;   // Offset mode: metres north
  float offsetE = 0.0f;   // east
  float offsetD = 0.0f;   // down; the vector length is limited to 100000 m
  float durationS = 0.0f; // 0..3600 sim seconds; 0 holds until cleared or reset
};

} // namespace vsim
