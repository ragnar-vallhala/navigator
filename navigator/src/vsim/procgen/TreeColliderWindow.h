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
// TreeColliderWindow.h -- the vegetation colliders near the drone, as the
// engine's environment-cylinder (ENV) set (Qt-free, engine-free).
//
// The engine tests the environment cylinders on every physics substep, so only
// the trunks around the drone may be handed to it: a disc of
// kColliderWindowRadiusM around the drone's (x, y), widened by
// speed * kColliderLookaheadS so a fast drone finds trunks already in place at
// the edge. The result is sorted, so two calls over the same set compare equal.
//
// Frames (traced, no conversion of x/y/z beyond the centre shift):
//  - Input VegCollider.pos is the NED world frame the drone pose and the World
//    tab use: x north, y east, z DOWN, absolute metres. scatterVegetation sets
//    pos = (x, y, -surfaceHeight) (Vegetation.cpp), so up is -z and pos is the
//    BASE of the stem on the ground, not a height above it.
//  - Output EngineCylinder is what vsim_env_cyl_t takes (vsim_proto.h, vayu
//    main): x, y the axis position, z the CENTRE height in NED ("a trunk
//    standing on the ground at z = g with height h has z = g - h / 2"), radius
//    and full height in metres. A trunk on pos therefore has centre z =
//    pos.z - height / 2, the same centre a World-tab cylinder obstacle uses.
//
// Wire (the byte layouts are defined here, locally, so navigator compiles
// against an SDK header that does not yet carry the ENV symbols; the layouts
// are vsim_proto.h's, pinned by static_asserts and golden tests):
//    ENV_BEGIN   empty body
//    ENV_ADD     uint32 count (1..12) then count x {x, y, z, radius, height}
//                as float32, 4 + 20 * count bytes
//    ENV_COMMIT  float32 restitution in [0, 1]
//    ENV_CLEAR   empty body
// Host byte order (the engine memcpy's the body). Messages are sent in that
// order; the engine swaps the whole set in at COMMIT.
#pragma once

#include "Vegetation.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace vsim::procgen {

// Contact restitution of the whole environment set (the COMMIT body).
inline constexpr float kColliderRestitution = 0.3f;
// Seconds of travel the window is widened by on each side.
inline constexpr float kColliderLookaheadS = 2.0f;
// Window radius around the drone when standing still.
inline constexpr float kColliderWindowRadiusM = 150.0f;
// Trunks the drone must always find in place, beyond its lookahead.
inline constexpr float kColliderNeedM = 50.0f;
// The window keeps (radius - lookahead - need) = this much slack; a rebuild is
// due after a quarter of it (25 m), independent of speed because the radius
// grows with the lookahead.
inline constexpr float kColliderMarginM = kColliderWindowRadiusM - kColliderNeedM;
inline constexpr float kColliderRebuildM = 0.25f * kColliderMarginM;

// vsim_proto.h: VSIM_ENV_ADD_MAX and VSIM_ENV_CAP.
inline constexpr std::size_t kEnvAddMax = 12;
inline constexpr std::size_t kEnvCap = 4096;
// vsim_proto.h control subtypes (VSIM_CTL_ENV_*).
inline constexpr std::uint32_t kEnvBegin = 16;
inline constexpr std::uint32_t kEnvAdd = 17;
inline constexpr std::uint32_t kEnvCommit = 18;
inline constexpr std::uint32_t kEnvClear = 19;

// One cylinder as the engine's ENV set takes it: vertical axis, `center` in NED.
struct EngineCylinder {
  PgVec3 center;
  float radius = 0.0f;
  float height = 0.0f;
  bool isRock = false;
};

bool operator==(const EngineCylinder &a, const EngineCylinder &b);
inline bool operator!=(const EngineCylinder &a, const EngineCylinder &b) {
  return !(a == b);
}

struct ColliderOptions {
  bool trees = true; // the Trees flag: trunks
  // Rocks are not vertical cylinders (a rock is a lump, a cylinder of its
  // radius and height overstates it), so they stay off until the engine has a
  // shape for them.
  bool rocks = false;
  std::size_t cap = kEnvCap; // most cylinders the engine accepts
};

struct ColliderWindow {
  std::vector<EngineCylinder> cylinders; // sorted by (x, y, z, radius, height, isRock)
  std::size_t dropped = 0;               // inside the window but over the cap (farthest ones)
};

// Colliders within kColliderWindowRadiusM + speed * kColliderLookaheadS of the
// drone's (x, y) (inclusive). Classes that are off are omitted, so Trees off
// (and rocks off) is an empty window: the caller sends ENV_CLEAR. Instances
// with a non-finite value or a non-positive radius or height are dropped. If
// more than opts.cap remain, the NEAREST opts.cap are kept and `dropped`
// counts the rest; the caller must log a non-zero count.
ColliderWindow treeColliderWindow(const std::vector<VegCollider> &colliders,
                                  const PgVec3 &drone, float speedMps,
                                  const ColliderOptions &opts = {});

// ENV_ADD bodies for `cyl`, at most kEnvAddMax per message, in order. Empty
// input gives no message. Each body is 4 + 20 * count bytes.
std::vector<std::vector<std::uint8_t>>
encodeEnvAdds(const std::vector<EngineCylinder> &cyl);

// ENV_COMMIT body: restitution clamped to [0, 1] (a non-finite value becomes
// kColliderRestitution).
std::vector<std::uint8_t> encodeEnvCommit(float restitution = kColliderRestitution);

// What a built window was built for.
struct ColliderBuildKey {
  bool valid = false; // false: nothing built yet
  float x = 0.0f, y = 0.0f; // drone (x, y) at the build
  bool trees = true;
  bool rocks = false;
  std::uint64_t world = 0; // caller's hash of seed / palette / world revision
};

// True when the window must be rebuilt: nothing built yet, the Trees/rocks flag
// or world changed, or the drone moved more than kColliderRebuildM (strictly)
// in x/y since the build.
bool colliderWindowNeedsRebuild(const ColliderBuildKey &last,
                                const ColliderBuildKey &now);

} // namespace vsim::procgen
