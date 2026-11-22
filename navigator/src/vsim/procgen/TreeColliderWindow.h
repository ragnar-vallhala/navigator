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
// TreeColliderWindow.h — the vegetation colliders near the drone, in the
// engine's obstacle frame (Qt-free, engine-free).
//
// The engine tests every obstacle on every physics step, so only the colliders
// around the drone may be handed to it: the 3x3 chunks centred on the drone's
// chunk, widened by a margin of speed * kColliderLookaheadS so a fast drone
// finds trunks already in place at the window edge. The result is sorted, so
// two calls over the same set compare equal and a caller can send one update
// per batch only when the set changed.
//
// Frame: VegCollider.pos is the surface point, x/y in the same frame as the
// drone position and chunk grid, z the engine's NED down coordinate; the
// cylinder rises from pos by `height`, so its centre is pos.z - height / 2.
#pragma once

#include "Vegetation.h"

#include <vector>

namespace vsim::procgen {

// Contact restitution of a tree or rock collider (the engine's obstacle default).
inline constexpr float kColliderRestitution = 0.3f;
// Seconds of travel the window is widened by on each side.
inline constexpr float kColliderLookaheadS = 2.0f;

// One cylinder obstacle as the engine takes it: axis along the body-local z,
// `center` in NED.
struct EngineCylinder {
  PgVec3 center;
  float radius = 0.0f;
  float height = 0.0f;
  float restitution = kColliderRestitution;
  bool isRock = false;
};

bool operator==(const EngineCylinder &a, const EngineCylinder &b);
inline bool operator!=(const EngineCylinder &a, const EngineCylinder &b) {
  return !(a == b);
}

// Colliders inside the window around `drone` (3x3 chunks of `chunkM` metres
// around the drone's chunk, plus speed * kColliderLookaheadS on every side;
// lower edges inclusive, upper exclusive). `trees` keeps trunks (the Trees
// flag), `rocks` keeps rocks (the vegetation flag); a class that is off is
// omitted. Instances with a non-positive radius or height are dropped. Sorted
// by (x, y, z, radius, height, isRock). Empty when chunkM <= 0.
std::vector<EngineCylinder>
treeColliderWindow(const std::vector<VegCollider> &colliders, const PgVec3 &drone,
                   float speedMps, float chunkM, bool trees, bool rocks);

} // namespace vsim::procgen
