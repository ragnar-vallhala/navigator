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
#include "Vegetation.h"

#include "GroundCover.h"

#include <algorithm>
#include <cmath>

namespace vsim::procgen {
namespace {

inline uint32_t h32(uint32_t x) {
  x ^= x >> 16;
  x *= 0x7feb352dU;
  x ^= x >> 15;
  x *= 0x846ca68bU;
  x ^= x >> 16;
  return x;
}
inline uint32_t hcell(int gx, int gy, uint32_t seed, uint32_t salt) {
  uint32_t h = static_cast<uint32_t>(gx) * 0x9e3779b1U;
  h ^= static_cast<uint32_t>(gy) * 0x85ebca77U;
  h ^= seed * 0xc2b2ae3dU;
  h ^= salt * 0x27d4eb2fU;
  return h32(h);
}
inline float u01(uint32_t h) { return (h & 0xffffffU) / 16777216.0f; }

// Height lattice step [m]. A power of two, anchored at the world origin, so the
// lattice (and the bilinear heights/slopes taken from it) is the same whatever
// chunk is being scattered.
constexpr float kLatticeM = 2.0f;

VegSpeciesParams sp(float cell, float density, float flatMin, float altLo,
                    float altHi, float sMin, float sMax, float groundMin) {
  VegSpeciesParams s;
  s.cellM = cell;
  s.density = density;
  s.flatMin = flatMin;
  s.altLoFrac = altLo;
  s.altHiFrac = altHi;
  s.scaleMin = sMin;
  s.scaleMax = sMax;
  s.groundMin = groundMin;
  return s;
}

// Per-species hash salt so species do not correlate with each other.
inline uint32_t speciesSalt(int s) { return 100u * static_cast<uint32_t>(s + 1); }

} // namespace

bool VegExclusion::contains(float px, float py) const {
  const float dx = px - x, dy = py - y;
  if (rect)
    return std::fabs(dx) <= rx && std::fabs(dy) <= ry;
  return dx * dx + dy * dy <= rx * rx;
}

VegParams vegParamsForPalette(int palette) {
  VegParams p;
  auto &T = p.species[kVegTree];
  auto &B = p.species[kVegBush];
  auto &G = p.species[kVegGrassClump];
  auto &R = p.species[kVegRock];
  // Light default mix (endless / unknown palettes).
  T = sp(16.0f, 0.20f, 0.85f, 0.0f, 0.55f, 0.8f, 1.4f, 0.5f);
  B = sp(8.0f, 0.20f, 0.80f, 0.0f, 0.65f, 0.7f, 1.3f, 0.5f);
  G = sp(5.0f, 0.25f, 0.85f, 0.0f, 0.55f, 0.7f, 1.4f, 0.5f);
  R = sp(20.0f, 0.10f, 0.60f, 0.0f, 1.0f, 0.5f, 2.2f, 0.1f);
  p.treeMinSpacingM = 3.0f;

  switch (palette) {
  case kPalForest:
    T = sp(6.0f, 0.85f, 0.85f, 0.0f, 0.70f, 0.8f, 1.5f, 0.5f);
    B = sp(6.0f, 0.35f, 0.80f, 0.0f, 0.70f, 0.7f, 1.3f, 0.5f);
    G = sp(5.0f, 0.30f, 0.85f, 0.0f, 0.60f, 0.7f, 1.4f, 0.5f);
    R = sp(20.0f, 0.10f, 0.60f, 0.0f, 1.0f, 0.5f, 2.0f, 0.1f);
    p.treeMinSpacingM = 3.5f;
    break;
  case kPalDesert:
    T.density = 0.0f;
    B = sp(12.0f, 0.30f, 0.80f, 0.0f, 0.80f, 0.5f, 1.0f, 0.2f);
    G = sp(8.0f, 0.15f, 0.85f, 0.0f, 0.60f, 0.6f, 1.1f, 0.2f);
    R = sp(10.0f, 0.40f, 0.55f, 0.0f, 1.0f, 0.5f, 2.5f, 0.05f);
    break;
  case kPalAlpine:
    T = sp(12.0f, 0.25f, 0.85f, 0.0f, 0.45f, 0.8f, 1.3f, 0.5f);
    B.density = 0.0f;
    G = sp(8.0f, 0.10f, 0.85f, 0.0f, 0.40f, 0.6f, 1.1f, 0.5f);
    R = sp(7.0f, 0.60f, 0.50f, 0.0f, 1.0f, 0.5f, 2.6f, 0.05f);
    break;
  case kPalYard:
    T.density = 0.02f;
    B.density = 0.02f;
    G.density = 0.03f;
    R.density = 0.0f;
    break;
  case kPalRacing:
    T.density = 0.0f;
    B.density = 0.0f;
    G.density = 0.02f;
    R.density = 0.0f;
    break;
  case kPalFlat:
    T.density = 0.0f;
    B.density = 0.02f;
    G.density = 0.05f;
    R.density = 0.02f;
    break;
  case kPalPasture:
    T = sp(22.0f, 0.20f, 0.85f, 0.0f, 0.50f, 0.8f, 1.4f, 0.5f);
    B = sp(10.0f, 0.20f, 0.80f, 0.0f, 0.60f, 0.7f, 1.3f, 0.5f);
    G = sp(5.0f, 0.30f, 0.85f, 0.0f, 0.55f, 0.7f, 1.4f, 0.5f);
    R.density = 0.05f;
    break;
  case kPalMeadow:
    T = sp(20.0f, 0.35f, 0.85f, 0.0f, 0.55f, 0.8f, 1.5f, 0.5f);
    B = sp(9.0f, 0.30f, 0.80f, 0.0f, 0.65f, 0.7f, 1.3f, 0.5f);
    G = sp(5.0f, 0.30f, 0.85f, 0.0f, 0.55f, 0.7f, 1.4f, 0.5f);
    break;
  case kPalHills:
    T = sp(18.0f, 0.30f, 0.82f, 0.0f, 0.55f, 0.8f, 1.5f, 0.5f);
    B = sp(9.0f, 0.30f, 0.78f, 0.0f, 0.65f, 0.7f, 1.3f, 0.5f);
    G = sp(5.0f, 0.25f, 0.85f, 0.0f, 0.55f, 0.7f, 1.4f, 0.5f);
    R = sp(14.0f, 0.25f, 0.55f, 0.0f, 1.0f, 0.5f, 2.2f, 0.1f);
    break;
  default:
    break;
  }
  return p;
}

std::vector<VegInstance> scatterVegetation(const TerrainField &f, int cx, int cy,
                                           float chunkM, const VegParams &p) {
  std::vector<VegInstance> out;
  if (!(chunkM > 0.0f))
    return out;

  const float x0 = static_cast<float>(cx) * chunkM;
  const float y0 = static_cast<float>(cy) * chunkM;
  const float x1 = x0 + chunkM, y1 = y0 + chunkM;

  // Heights on the global lattice, covering the chunk plus a margin for the
  // bilinear cell and the slope taps.
  const int li0 = static_cast<int>(std::floor(x0 / kLatticeM)) - 3;
  const int lj0 = static_cast<int>(std::floor(y0 / kLatticeM)) - 3;
  const int ln = static_cast<int>(std::ceil(chunkM / kLatticeM)) + 8;
  std::vector<float> H(static_cast<std::size_t>(ln) * ln);
  for (int j = 0; j < ln; ++j)
    for (int i = 0; i < ln; ++i)
      H[static_cast<std::size_t>(j) * ln + i] =
          f.height(static_cast<float>(li0 + i) * kLatticeM,
                   static_cast<float>(lj0 + j) * kLatticeM);
  // The cell index and fraction come from the global coordinate alone, so the
  // result is bit-identical whichever chunk's lattice window is in use.
  auto hAt = [&](float wx, float wy) -> float {
    const float ux = wx / kLatticeM, uy = wy / kLatticeM;
    const float flx = std::floor(ux), fly = std::floor(uy);
    const float fx = ux - flx, fy = uy - fly;
    const int i = std::min(std::max(static_cast<int>(flx) - li0, 0), ln - 2);
    const int j = std::min(std::max(static_cast<int>(fly) - lj0, 0), ln - 2);
    const float *r0 = &H[static_cast<std::size_t>(j) * ln + i];
    const float *r1 = r0 + ln;
    const float a = r0[0] * (1 - fx) + r0[1] * fx;
    const float b = r1[0] * (1 - fx) + r1[1] * fx;
    return a * (1 - fy) + b * fy;
  };

  const float heightM = f.params().heightM + 1e-3f;
  const float clear2 = p.clearAroundSpawnM * p.clearAroundSpawnM;

  for (int s = 0; s < kVegSpeciesCount; ++s) {
    const VegSpeciesParams &sv = p.species[s];
    if (!(sv.cellM > 0.0f) || !(sv.density > 0.0f))
      continue;
    const uint32_t salt = speciesSalt(s);
    // Jitter margin inside the cell. For trees it is sized so two points in
    // neighbouring cells are never closer than treeMinSpacingM.
    float margin = 0.05f;
    if (s == kVegTree)
      margin = std::min(0.5f, std::max(margin, p.treeMinSpacingM / (2.0f * sv.cellM)));

    const int gx0 = static_cast<int>(std::floor(x0 / sv.cellM));
    const int gx1 = static_cast<int>(std::floor(x1 / sv.cellM));
    const int gy0 = static_cast<int>(std::floor(y0 / sv.cellM));
    const int gy1 = static_cast<int>(std::floor(y1 / sv.cellM));
    for (int gx = gx0; gx <= gx1; ++gx) {
      for (int gy = gy0; gy <= gy1; ++gy) {
        if (u01(hcell(gx, gy, p.seed, salt)) >= sv.density)
          continue;
        const float span = 1.0f - 2.0f * margin;
        const float wx =
            (static_cast<float>(gx) + margin + span * u01(hcell(gx, gy, p.seed, salt + 1))) *
            sv.cellM;
        const float wy =
            (static_cast<float>(gy) + margin + span * u01(hcell(gx, gy, p.seed, salt + 2))) *
            sv.cellM;
        if (wx < x0 || wx >= x1 || wy < y0 || wy >= y1)
          continue; // owned by another chunk
        if (wx * wx + wy * wy < clear2)
          continue;
        bool excluded = false;
        for (const VegExclusion &e : p.exclusions)
          if (e.contains(wx, wy)) {
            excluded = true;
            break;
          }
        if (excluded)
          continue;

        const float h = hAt(wx, wy);
        const float frac = h / heightM;
        if (frac < sv.altLoFrac || frac > sv.altHiFrac)
          continue;
        const float dhdx = (hAt(wx + kLatticeM, wy) - hAt(wx - kLatticeM, wy)) /
                           (2.0f * kLatticeM);
        const float dhdy = (hAt(wx, wy + kLatticeM) - hAt(wx, wy - kLatticeM)) /
                           (2.0f * kLatticeM);
        const float flatness = 1.0f / std::sqrt(dhdx * dhdx + dhdy * dhdy + 1.0f);
        if (flatness < sv.flatMin)
          continue;
        if (f.grassFactor(wx, wy, h, flatness) < sv.groundMin)
          continue;

        VegInstance v;
        v.pos = PgVec3{wx, wy, -h};
        v.yaw = 6.2831853f * u01(hcell(gx, gy, p.seed, salt + 3));
        v.scale = sv.scaleMin + (sv.scaleMax - sv.scaleMin) *
                                    u01(hcell(gx, gy, p.seed, salt + 4));
        v.species = static_cast<uint8_t>(s);
        v.variant = static_cast<uint8_t>(hcell(gx, gy, p.seed, salt + 5) %
                                         static_cast<uint32_t>(kVegVariants));
        v.tint = u01(hcell(gx, gy, p.seed, salt + 6));
        out.push_back(v);
      }
    }
  }
  return out;
}

std::vector<VegCollider>
vegetationColliders(const std::vector<VegInstance> &instances) {
  std::vector<VegCollider> out;
  for (const VegInstance &v : instances) {
    if (v.species == kVegTree) {
      out.push_back(VegCollider{v.pos, kVegTreeTrunkRadiusM * v.scale,
                                kVegTreeHeightM * v.scale, false});
    } else if (v.species == kVegRock && v.scale >= kVegRockColliderMinScale) {
      out.push_back(VegCollider{v.pos, kVegRockRadiusM * v.scale,
                                kVegRockHeightM * v.scale, true});
    }
  }
  return out;
}

} // namespace vsim::procgen
