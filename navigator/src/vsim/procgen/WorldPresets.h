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
// WorldPresets.h — named, seed-deterministic world presets (Qt-free).
//
// A preset is a pure function (id, seed) -> PresetData: terrain field + colour
// palette, arena size, and a procedurally laid-out set of static obstacles
// (racing gates/pylons, an obstacle yard, forest trees). Nothing is imported;
// the same (id, seed) always yields the same world. Obstacles are seated on the
// *meshed* surface (surfaceHeight), the same one the physics collides with, and
// kept clear of the spawn at the origin.
//
// Finite arenas: every preset except "meadow" (the original TerrainGen arena)
// is a square arena of PresetData::sizeM centred on the origin, meshed from the
// TerrainField as 2x2 chunks (meshArena) so the origin is a lattice node.
#pragma once

#include "Flora.h"
#include "ProcgenTypes.h"
#include "TerrainField.h"

#include <cstdint>
#include <string>
#include <vector>

namespace vsim::procgen {

// A static obstacle in NED metres. Mirrors vsim::Obstacle: type 0 box (full
// extents), 1 sphere (sx = radius), 2 cylinder (sx = radius, sz = height).
// (x, y) is the horizontal centre, zc the NED z of the shape centre.
struct PgObstacle {
  int type = 0;
  float x = 0.0f, y = 0.0f, zc = 0.0f;
  float sx = 1.0f, sy = 1.0f, sz = 1.0f;
  float yawDeg = 0.0f; // rotation about the vertical
};

// One racing gate (two posts + a top bar). Opening is widthM wide and at least
// clearanceM tall; (x, y) is the opening centre, yawDeg the travel heading.
struct RaceGate {
  float x = 0.0f, y = 0.0f;
  float yawDeg = 0.0f;
  float widthM = 0.0f;
  float clearanceM = 0.0f;
};

struct PresetData {
  std::string biome; // proceduralBiome value
  FieldParams field; // seed already applied
  FloraParams flora;
  float sizeM = 256.0f;
  int resolution = 192;
  std::vector<PgObstacle> obstacles;
  std::vector<RaceGate> gates; // racing only
  float propDiameterM = 0.0f;  // largest prop the layout was sized for
};

struct PresetInfo {
  const char *id;
  const char *label;
};

// Largest rotor diameter any airframe preset is expected to carry [m] (a 15-inch
// prop is 0.38 m; headroom left). Gate openings are >= 3x this.
constexpr float kMaxPropDiameterM = 0.5f;
// Radius around the spawn kept free of obstacles [m] (plus each shape's own
// footprint).
constexpr float kSpawnClearM = 12.0f;

const std::vector<PresetInfo> &worldPresets();
bool isFieldBiome(const std::string &biome); // hills/alpine/flat/desert/...
// Fill `out` for preset `id`; false if the id is unknown.
bool buildWorldPreset(const std::string &id, uint32_t seed, PresetData &out);

// Arena meshing: chunk geometry shared by the mesh, the spawn height and the
// minimap so they all use one lattice.
inline float arenaChunkM(float sizeM) { return sizeM * 0.5f; }
inline int arenaChunkRes(int resolution) {
  return resolution / 2 < 2 ? 2 : resolution / 2;
}
ProcMesh meshArena(const TerrainField &f, float sizeM, int resolution);
float arenaSurfaceHeight(const TerrainField &f, float sizeM, int resolution,
                         float wx, float wy);

// Wind presets: "off", "calm", "breezy", "gusty". The heading is derived from
// the seed (deterministic); magnitudes are fixed per preset.
struct WindPresetData {
  float steadyN = 0.0f, steadyE = 0.0f, steadyD = 0.0f; // NED [m/s]
  float gustAmp = 0.0f, gustPeriod = 0.0f;
  float turbSigma = 0.0f, turbTau = 1.0f;
  bool enabled = false;
};
const std::vector<PresetInfo> &windPresets();
bool buildWindPreset(const std::string &id, uint32_t seed, WindPresetData &out);

} // namespace vsim::procgen
