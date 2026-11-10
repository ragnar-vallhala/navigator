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

// Procedural quadcopter model (Qt-free, so it is unit-testable headless).
//
// Given the airframe's motor layout and prop radius, builds lit-shader triangle
// meshes with per-vertex colour: a body (plates, stack, battery, arms, motors,
// camera) and one propeller mesh per spin direction (rotated about each motor
// axis by the renderer). Everything is generated in code; no asset is loaded.
//
// Frame: body NED (x forward, y right, z DOWN), like the physics. "Up" is -z.
// Vertex layout: 9 floats per vertex, triangle soup:
//   [px py pz  nx ny nz  r g b].

#include <array>
#include <string>
#include <vector>

namespace vsim {

// Proportions class: picks the frame style (arm width, battery, ducts...).
enum class DroneClass { Freestyle5, Light3, LongRange7, Heavy10, Whoop65 };

// "5in" "3in" "7in" "10in" "whoop65"; any other id (custom vehicle) is classed
// by prop radius so a custom build still gets sensible proportions.
DroneClass droneClassFor(const std::string &presetId, float propRadius);

struct DroneModelInput {
  float propRadius = 0.0622f; // [m]
  // Motor hub positions, body frame [m], and spin (+1 CCW / -1 CW from above).
  std::array<std::array<float, 3>, 4> motorPos = {{{0.0866f, 0.0866f, 0.0f},
                                                   {-0.0866f, 0.0866f, 0.0f},
                                                   {-0.0866f, -0.0866f, 0.0f},
                                                   {0.0866f, -0.0866f, 0.0f}}};
  std::array<int, 4> spin = {+1, -1, +1, -1};
  DroneClass cls = DroneClass::Freestyle5;
};

struct ModelMesh {
  std::vector<float> data; // 9 floats per vertex
  size_t vertexCount() const { return data.size() / 9; }
};

struct DroneModel {
  ModelMesh body;
  // Propeller per motor around its own origin (the hub), blades in the local
  // XY plane, already coloured (tip by spin, red band on the front pair).
  std::array<ModelMesh, 4> prop;
  // Hub centre per motor in the body frame (where the prop mesh is placed;
  // z is above the motor bell, i.e. negative).
  std::array<std::array<float, 3>, 4> hub{};
  float visualPropRadius = 0.0f; // blade radius actually modelled [m]
  int bladeCount = 2;
  // Axis-aligned bounds of body + props placed at their hubs.
  std::array<float, 3> bmin{}, bmax{};
};

// Never fails: odd input (zero radius, coincident / fewer than 4 distinct
// positions, NaNs) falls back to a default square layout of the same scale.
DroneModel buildDroneModel(const DroneModelInput &in);

// Colours shared with the renderer (blur disc tint).
std::array<float, 3> propTipColor(int spinSign);

} // namespace vsim
