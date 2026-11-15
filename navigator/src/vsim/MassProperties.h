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

#include <QVector3D>
#include <array>
#include <vector>

// MassProperties — exact mass properties of a closed triangle mesh,
// assuming uniform density, via the divergence-theorem volume integrals
// (Eberly, "Polyhedral Mass Properties (Revisited)"). Used to derive the
// simulated drone's center of mass and full inertia tensor from its CAD
// mesh instead of hand-tuned diagonal constants.
namespace vsim {

struct MassProperties {
  float volume = 0.0f;    // m^3 (of the tessellated solid)
  float mass = 0.0f;      // kg (== requested target mass)
  QVector3D com{0, 0, 0}; // center of mass, body frame [m]

  // Inertia tensor about the CoM, body frame [kg*m^2]. Six unique
  // entries of the symmetric matrix; off-diagonals are the products of
  // inertia with the inertia-matrix sign convention (i.e. ixy = -∫xy dm),
  // so they drop straight into Mat3::symmetric(ixx,iyy,izz,ixy,ixz,iyz).
  float ixx = 0, iyy = 0, izz = 0;
  float ixy = 0, ixz = 0, iyz = 0;

  bool valid = false;
};

// `positions` is a triangle soup (3 vertices per triangle, in meters).
// The result is scaled so total mass == targetMass (uniform density).
// Winding is auto-corrected: an inside-out mesh still yields a positive
// volume and a correct tensor. Returns {valid=false} for empty/degenerate
// input (zero volume).
MassProperties computeMassProperties(const std::vector<QVector3D> &positions,
                                     float targetMass);

// Inertia of the box/arms airframe (no mesh loaded), from the mass and the
// motor layout. Row-major diagonal tensor about the CoM, body frame [kg*m^2].
//
// With a = mean(y_i^2) and b = mean(x_i^2) over the four motors (so a = b = p^2
// for the usual +-p square layout, arm length p*sqrt(2)):
//     Ixx = kBoxKx * m * a
//     Iyy = kBoxKy * m * b
//     Izz = kBoxKz * m * (a + b)
// The coefficients are fitted to the "5in" preset (m = 0.682 kg, p = 0.0866 m,
// I = 1.54e-3 / 1.80e-3 / 3.06e-3), which the GeometryConfig defaults pin, so
// the defaults come out unchanged to within 0.1 %: Kx = Ixx/(m p^2) = 0.301,
// Ky = 0.352, Kz = Izz/(m 2p^2) = 0.299. The engine has no inertia convention
// of its own (it takes the 3x3 tensor as sent, vsim_proto.h), so this is the
// GCS's. Inertia scales linearly with mass and quadratically with arm length.
constexpr float kBoxKx = 0.301f;
constexpr float kBoxKy = 0.352f;
constexpr float kBoxKz = 0.299f;
std::array<float, 9> boxInertia(float mass,
                                const std::array<QVector3D, 4> &motorPos);

} // namespace vsim
