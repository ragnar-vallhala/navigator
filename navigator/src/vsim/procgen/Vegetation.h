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
// Vegetation.h — procedural trees, bushes, grass clumps and rocks (Qt-free).
//
// Places discrete vegetation instances for a terrain chunk. Each species has its
// own jittered global grid; a candidate point is owned by the chunk containing
// the point itself, and everything about it (position, height, slope, ground
// cover, hash) is a pure function of the point's global cell. So scattering two
// adjacent chunks yields exactly the instances of scattering both as one
// region: no gaps, no duplicates, and no neighbour lookups. Trees keep a minimum
// spacing because their jitter is confined to the inside of the cell.
//
// Instances only carry placement; the renderer picks a shared mesh per species/
// variant. vegetationColliders() lists the instances that should block the
// drone (tree trunks and large rocks).
#pragma once

#include "ProcgenTypes.h"
#include "TerrainField.h"

#include <cstdint>
#include <vector>

namespace vsim::procgen {

enum VegSpecies : uint8_t {
  kVegTree = 0,
  kVegBush = 1,
  kVegGrassClump = 2,
  kVegRock = 3,
};
inline constexpr int kVegSpeciesCount = 4;
inline constexpr int kVegVariants = 4; // distinct mesh variants per species

struct VegInstance {
  PgVec3 pos;       // base on the surface, NED world metres
  float yaw = 0.0f; // rotation about the vertical [rad]
  float scale = 1.0f; // size multiplier on the species' nominal mesh
  uint8_t species = kVegTree;
  uint8_t variant = 0; // 0 .. kVegVariants-1
  float tint = 0.0f;   // small colour variation, 0..1
};

// Sizes at instance scale 1 [m]: the single source for the renderer's species
// table and for the colliders, so what is drawn is what is solid. Every tree
// species is scaled to the same height, so a tree's collider height is exact
// whichever model is drawn; rocks are scaled by their longest footprint side
// (their height differs per model, kVegRockHeightM is the typical one).
inline constexpr float kVegTreeHeightM = 3.6f;
inline constexpr float kVegTreeTrunkRadiusM = 0.15f;
inline constexpr float kVegRockSpanM = 1.3f;
inline constexpr float kVegRockRadiusM = 0.5f * kVegRockSpanM;
inline constexpr float kVegRockHeightM = 0.5f;
// Rocks smaller than this scale are decoration only (no collider).
inline constexpr float kVegRockColliderMinScale = 1.4f;

// An area where nothing is placed (helipad, obstacle). A disc of radius rx, or
// an axis-aligned rectangle of half-extents (rx, ry) when `rect` is set.
struct VegExclusion {
  float x = 0.0f;
  float y = 0.0f;
  float rx = 0.0f;
  float ry = 0.0f;
  bool rect = false;

  static VegExclusion disc(float x, float y, float r) {
    return VegExclusion{x, y, r, r, false};
  }
  static VegExclusion box(float x, float y, float halfX, float halfY) {
    return VegExclusion{x, y, halfX, halfY, true};
  }
  bool contains(float px, float py) const;
};

struct VegSpeciesParams {
  float cellM = 10.0f;    // jittered grid cell [m]; <= 0 disables the species
  float density = 0.0f;   // chance a cell holds an instance, 0..1
  float flatMin = 0.85f;  // least flatness (1 = level) the species stands on
  float altLoFrac = 0.0f; // allowed elevation band, fraction of field heightM
  float altHiFrac = 1.0f;
  float scaleMin = 0.8f;
  float scaleMax = 1.3f;
  // Least TerrainField::grassFactor needed (keeps off tracks, ponds, paving;
  // rocks accept less since rock patches have no grass).
  float groundMin = 0.5f;
};

struct VegParams {
  uint32_t seed = 1337u;
  VegSpeciesParams species[kVegSpeciesCount];
  float treeMinSpacingM = 3.0f; // closest two trees may stand [m]
  std::vector<VegExclusion> exclusions;
  float clearAroundSpawnM = 8.0f; // nothing within this of the origin

  VegSpeciesParams &of(VegSpecies s) { return species[s]; }
  const VegSpeciesParams &of(VegSpecies s) const { return species[s]; }
};

// A sensible species mix for a GroundPalette id (unknown ids get a light mix).
VegParams vegParamsForPalette(int palette);

// Scatter over chunk (cx, cy) spanning [cx*chunkM,(cx+1)*chunkM]^2. Empty when
// chunkM <= 0.
std::vector<VegInstance> scatterVegetation(const TerrainField &f, int cx, int cy,
                                           float chunkM, const VegParams &p);

// Solid obstacle for a vegetation instance: a vertical cylinder from the surface
// (pos) up by `height`.
struct VegCollider {
  PgVec3 pos;
  float radius = 0.0f;
  float height = 0.0f;
  bool isRock = false;
};

// Trunks of trees and rocks at least kVegRockColliderMinScale; bushes and grass
// clumps have none.
std::vector<VegCollider>
vegetationColliders(const std::vector<VegInstance> &instances);

} // namespace vsim::procgen
