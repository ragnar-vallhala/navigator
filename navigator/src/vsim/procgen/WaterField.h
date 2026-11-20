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
// WaterField.h — ponds and lakes as real basins (Qt-free, deterministic).
//
// The world is cut into square cells (cellM). A cell holds at most one water
// body, placed from a hash of (seed, cell): a centre, a wobbly shore radius, and
// a flat surface level that is the lowest of the ground around the shore. The
// ground inside is lowered into a smooth bowl up to maxDepthM deep and blended
// back into the terrain over bankM, so the shore is where the ground crosses the
// level. The carve is applied INSIDE TerrainField::height(), the one height
// function the mesh, the collision, the minimap and the helipads all read, so
// they agree on every pond without any render-only offset.
//
// A body is dropped (the cell stays dry) where the ground around it is too
// uneven to hold a flat surface, too high, or too near the spawn pad. A body
// never leaves its own cell, so a point only ever needs its own cell.
#pragma once

#include <cstdint>
#include <vector>

namespace vsim::procgen {

// Pre-carve ground height, supplied by the terrain (TerrainField implements it).
struct HeightSource {
  virtual float baseHeight(float wx, float wy) const = 0;

protected:
  ~HeightSource() = default;
};

struct WaterParams {
  uint32_t seed = 1337u;
  float coverage = 0.0f;     // 0 = no water; palette pond amount (~0.25..0.45)
  float maxDepthM = 3.0f;    // deepest point of a basin; 0 disables water
  float spawnClearM = 60.0f; // no water within this distance of the origin
  float maxLevelM = 1e9f;    // no water on ground higher than this (alpine)
  float cellM = 170.0f;
};

struct WaterBody {
  float cx = 0.0f, cy = 0.0f;
  float radiusM = 0.0f; // mean shore radius
  float level = 0.0f;   // flat water surface elevation [m]
  float depthM = 0.0f;  // depth at the centre
  float bankM = 0.0f;   // blend width outside the shore
  float amp[2] = {0.0f, 0.0f};   // shore wobble (2 and 3 lobes)
  float phase[2] = {0.0f, 0.0f};
  // Shore radius toward `angle` [rad].
  float shoreRadius(float angle) const;
  // Distance from the centre to the shore toward (dx, dy), and the distance.
  float shoreRadiusToward(float dx, float dy) const;
};

class WaterField {
public:
  explicit WaterField(const WaterParams &p);

  bool enabled() const { return p_.coverage > 0.0f && p_.maxDepthM > 0.0f; }
  const WaterParams &params() const { return p_; }

  // The body whose cell contains (wx, wy), if there is one.
  bool bodyAt(const HeightSource &src, float wx, float wy, WaterBody &out) const;

  // Ground height with the basin cut in: `hBase` is src.baseHeight(wx, wy).
  float carve(const HeightSource &src, float wx, float wy, float hBase) const;

  // 1 inside the shore (smooth to 0 over ~1 m at the edge), else 0.
  float inside(const HeightSource &src, float wx, float wy) const;
  // 1 on the wet ring just outside the shore (mud / sand), else 0.
  float shoreRing(const HeightSource &src, float wx, float wy) const;
  // True when water lies within `marginM` of (wx, wy).
  bool nearWater(const HeightSource &src, float wx, float wy, float marginM) const;

  // Every body overlapping the world rectangle [x0,x1] x [y0,y1].
  std::vector<WaterBody> bodiesIn(const HeightSource &src, float x0, float y0,
                                  float x1, float y1) const;

private:
  bool makeBody(const HeightSource &src, int ix, int iy, WaterBody &out) const;
  WaterParams p_;
  uint32_t id_; // distinguishes instances for the per-thread body cache
};

} // namespace vsim::procgen
