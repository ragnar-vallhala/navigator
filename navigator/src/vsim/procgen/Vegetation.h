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
// region: no gaps and no duplicates.
//
// Trees are full-size (8-20 m) and stand in stands: low-frequency noise (a pure
// function of the position) picks conifer or broadleaf and which species of the
// group, so one species clumps, with mixed zones where the fields cross, and a
// second field opens clearings. A tree candidate only survives if no
// higher-priority neighbouring candidate stands closer than the sum of their
// crown radii times a factor; the neighbour test reads only the candidates'
// global cells, never the chunk, so it is chunk independent too.
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

// The tree species. Sizes are at instance scale 1 [m]: the single source for the
// renderer's species table and for the colliders, so what is drawn is what is
// solid. The renderer scales every variant of a species to heightM, so the
// collider height is exact whichever variant is drawn. An instance's scale is
// drawn from [scaleMin, scaleMax], giving heights from 8 m to 20 m.
struct VegTreeKind {
  const char *id;      // asset species id (assets/vegetation/manifest.json)
  bool conifer;
  float heightM;       // drawn height at scale 1
  float scaleMin, scaleMax;
  float trunkRadiusM;  // collider radius at scale 1 (measured trunk, see assets)
  float crownRadiusM;  // canopy radius at scale 1, drives the spacing
};
inline constexpr int kVegTreeKindCount = 4;
// Trunk radii follow the measured ones (manifest "trunkRadius", median radial distance of
// the solid geometry 0.5-1.5 m above the base, at the model's own size) times
// the drawn-size / model-size ratio; tst_vegetation_scale checks them against
// the assets. Crown radii are the modelled canopy half-width at drawn size.
inline constexpr VegTreeKind kVegTreeKinds[kVegTreeKindCount] = {
    // id                   conifer height  scale range   trunk  crown
    {"fir_tree_01",         true,   16.0f,  0.70f, 1.25f, 0.15f, 2.8f},
    {"pine_sapling_medium", true,   12.0f,  0.75f, 1.30f, 0.15f, 2.8f},
    {"jacaranda_tree",      false,  13.0f,  0.62f, 1.45f, 0.45f, 8.2f},
    {"island_tree_01",      false,   9.5f,  0.85f, 1.30f, 0.50f, 4.6f},
};
inline const VegTreeKind &vegTreeKind(int kind) {
  return kVegTreeKinds[kind < 0 ? 0 : (kind >= kVegTreeKindCount ? kVegTreeKindCount - 1 : kind)];
}
// Index of the kind with this asset id, -1 when there is none.
int vegTreeKindIndex(const char *id);

struct VegInstance {
  PgVec3 pos;       // base on the surface, NED world metres
  float yaw = 0.0f; // rotation about the vertical [rad]
  float scale = 1.0f; // size multiplier on the species' nominal mesh
  uint8_t species = kVegTree;
  uint8_t variant = 0; // trees: any value, the renderer takes it modulo the
                       // kind's variant count; others 0 .. kVegVariants-1
  uint8_t kind = 0;    // trees: index into vegTreeKind()
  float tint = 0.0f;   // small colour variation, 0..1
  // Grass tufts: the palette's ground hue to tint toward, packed 0xRRGGBB with
  // the brightest channel at 255 (see vegPackHue); 0 = leave the colour alone.
  uint32_t paint = 0;
};

// Packs the hue of a colour (brightest channel scaled to 255) as 0xRRGGBB.
uint32_t vegPackHue(const PgVec3 &c);

// Rocks are scaled by their longest footprint side (their height differs per
// model, kVegRockHeightM is the typical one).
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
  // True when a disc of radius `pad` around the point reaches the zone (a crown
  // overhanging a pad).
  bool reaches(float px, float py, float pad) const;
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

// How trees gather into stands. All distances in metres.
struct VegTreeStands {
  float coniferShare = 0.5f; // 0 = broadleaf only .. 1 = conifer only (area share)
  float clumpM = 130.0f;     // size of a single-species stand
  float standM = 85.0f;      // size of a thick stand / clearing
  float openShare = 0.22f;   // roughly the part of the land left open
  float mixWidth = 0.10f;    // width of the mixed zone between two stands (0..0.5)
  float crownSpacing = 0.55f; // trunks stand >= this * (crown radii sum) apart
};

struct VegParams {
  uint32_t seed = 1337u;
  VegSpeciesParams species[kVegSpeciesCount];
  VegTreeStands stands;
  float treeMinSpacingM = 2.5f; // closest two trunks may stand [m]
  std::vector<VegExclusion> exclusions;
  float clearAroundSpawnM = 8.0f; // nothing within this of the origin
  // The palette's own ground colour (TerrainField::color on level low ground);
  // grass tufts are tinted toward it so they sit in the ground they stand on.
  PgVec3 groundColor{0.06f, 0.15f, 0.09f};

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

// Trunks of trees (radius and height of their species, times the instance scale)
// and rocks at least kVegRockColliderMinScale; bushes and grass clumps have none.
std::vector<VegCollider>
vegetationColliders(const std::vector<VegInstance> &instances);

} // namespace vsim::procgen
