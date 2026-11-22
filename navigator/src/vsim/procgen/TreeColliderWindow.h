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
// Frames (traced, no conversion of x/y/z is needed beyond the centre shift):
//  - Input VegCollider.pos is the NED world frame the drone pose and the World
//    tab use: x north, y east, z DOWN, absolute metres (chunk (cx, cy) spans
//    [cx*chunkM, (cx+1)*chunkM) on x and y, no chunk-local offset).
//    scatterVegetation sets pos = (x, y, -surfaceHeight) (Vegetation.cpp:394,
//    458), so up is -z and pos is the base of the stem on the ground, not a
//    height above it. The renderer draws at pos.z + kSinkM, same frame
//    (VegetationRenderer.cpp:835-837).
//  - Output EngineCylinder is what a World-tab cylinder obstacle sends through
//    SimWorker::sendObstacles (SimWorker.cpp:211-231, copied one to one into
//    vsim_ctl_obstacle_t): centre in NED metres, size.x = radius, size.z = full
//    height, axis the body-local z (VsimTypes.h:111-115). A cylinder standing
//    on pos therefore has its centre at pos.z - height / 2 (half a height up).
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
