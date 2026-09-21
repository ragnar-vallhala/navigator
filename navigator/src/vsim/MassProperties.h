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

} // namespace vsim
