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
#include "TerrainField.h"

#include <algorithm>
#include <cmath>
#include <string>

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

inline float smoothstep01(float t) {
  t = std::min(std::max(t, 0.0f), 1.0f);
  return t * t * (3.0f - 2.0f * t);
}
inline float smoothstep(float a, float b, float x) {
  return smoothstep01((x - a) / (b - a));
}

// Smooth value noise in 0..1 at (x, y) lattice units; a pure function of the
// coordinates, so stands are the same whichever chunk asks.
float valueNoise(float x, float y, uint32_t seed, uint32_t salt) {
  const float fx = std::floor(x), fy = std::floor(y);
  const int ix = static_cast<int>(fx), iy = static_cast<int>(fy);
  const float tx = smoothstep01(x - fx), ty = smoothstep01(y - fy);
  auto g = [&](int i, int j) { return u01(hcell(i, j, seed, salt)); };
  const float a = g(ix, iy) * (1 - tx) + g(ix + 1, iy) * tx;
  const float b = g(ix, iy + 1) * (1 - tx) + g(ix + 1, iy + 1) * tx;
  return a * (1 - ty) + b * ty;
}
// Two octaves, stretched around 0.5 so that the stand fields cover 0..1.
float standField(float wx, float wy, float scaleM, uint32_t seed, uint32_t salt) {
  const float x = wx / scaleM, y = wy / scaleM;
  const float n = 0.68f * valueNoise(x, y, seed, salt) +
                  0.32f * valueNoise(2.07f * x + 13.0f, 2.07f * y + 29.0f, seed, salt + 1);
  return std::min(std::max(0.5f + (n - 0.5f) * 2.7f, 0.0f), 1.0f);
}

// Largest distance at which two trees may still compete for room [m]; the
// neighbour window of the tree scatter is sized from it.
constexpr float kMaxCompeteM = 12.0f;

struct TreeCand {
  bool on = false;
  float x = 0, y = 0;
  uint8_t kind = 0;
  float scale = 1;
  float crown = 0; // canopy radius at this scale
  uint32_t prio = 0;
  uint32_t variant = 0, tint = 0, yaw = 0; // raw hashes
};

} // namespace

int vegTreeKindIndex(const char *id) {
  for (int i = 0; i < kVegTreeKindCount; ++i)
    if (id && std::string(kVegTreeKinds[i].id) == id)
      return i;
  return -1;
}

bool VegExclusion::contains(float px, float py) const {
  const float dx = px - x, dy = py - y;
  if (rect)
    return std::fabs(dx) <= rx && std::fabs(dy) <= ry;
  return dx * dx + dy * dy <= rx * rx;
}

bool VegExclusion::reaches(float px, float py, float pad) const {
  const float dx = px - x, dy = py - y;
  if (rect)
    return std::fabs(dx) <= rx + pad && std::fabs(dy) <= ry + pad;
  const float r = rx + pad;
  return dx * dx + dy * dy <= r * r;
}

uint32_t vegPackHue(const PgVec3 &c) {
  const float m = std::max(std::max(c.x, c.y), std::max(c.z, 1e-4f));
  auto ch = [m](float v) {
    return static_cast<uint32_t>(std::lround(std::clamp(v / m, 0.0f, 1.0f) * 255.0f));
  };
  return (ch(c.x) << 16) | (ch(c.y) << 8) | ch(c.z);
}

VegParams vegParamsForPalette(int palette) {
  VegParams p;
  {
    FieldParams fp;
    fp.palette = palette;
    p.groundColor = TerrainField(fp).color(0.0f, 1.0f);
  }
  auto &T = p.species[kVegTree];
  auto &B = p.species[kVegBush];
  auto &G = p.species[kVegGrassClump];
  auto &R = p.species[kVegRock];
  // Trees: a 7 m candidate grid for every palette (density is the chance a
  // candidate stands, before the stand field and the crown spacing thin it);
  // the scale range is the species' own (VegTreeKind). Light default mix
  // (endless / unknown palettes).
  T = sp(7.0f, 0.045f, 0.85f, 0.0f, 0.55f, 1.0f, 1.0f, 0.5f);
  p.stands.coniferShare = 0.3f;
  B = sp(6.0f, 0.25f, 0.80f, 0.0f, 0.65f, 0.7f, 1.3f, 0.5f);
  G = sp(2.2f, 0.35f, 0.85f, 0.0f, 0.55f, 0.7f, 1.4f, 0.5f);
  R = sp(20.0f, 0.10f, 0.60f, 0.0f, 1.0f, 0.5f, 2.2f, 0.1f);
  switch (palette) {
  case kPalForest:
    T = sp(7.0f, 0.85f, 0.85f, 0.0f, 0.70f, 1.0f, 1.0f, 0.5f);
    p.stands.coniferShare = 0.55f; // crowns of the broadleaf are wide: fewer, bigger
    p.stands.clumpM = 80.0f;       // stands small enough that a view holds both groups
    p.stands.openShare = 0.12f;
    B = sp(4.0f, 0.50f, 0.80f, 0.0f, 0.70f, 0.7f, 1.3f, 0.5f);
    G = sp(1.6f, 0.50f, 0.85f, 0.0f, 0.60f, 0.7f, 1.4f, 0.5f);
    R = sp(20.0f, 0.10f, 0.60f, 0.0f, 1.0f, 0.5f, 2.0f, 0.1f);
    break;
  case kPalDesert:
    T.density = 0.0f;
    B = sp(12.0f, 0.30f, 0.80f, 0.0f, 0.80f, 0.5f, 1.0f, 0.2f);
    G = sp(8.0f, 0.15f, 0.85f, 0.0f, 0.60f, 0.6f, 1.1f, 0.2f);
    R = sp(10.0f, 0.40f, 0.55f, 0.0f, 1.0f, 0.5f, 2.5f, 0.05f);
    break;
  case kPalAlpine:
    T = sp(7.0f, 0.30f, 0.85f, 0.0f, 0.45f, 1.0f, 1.0f, 0.5f);
    p.stands.coniferShare = 0.9f;
    B.density = 0.0f;
    G = sp(8.0f, 0.10f, 0.85f, 0.0f, 0.40f, 0.6f, 1.1f, 0.5f);
    R = sp(7.0f, 0.60f, 0.50f, 0.0f, 1.0f, 0.5f, 2.6f, 0.05f);
    break;
  case kPalYard:
    T.density = 0.004f;
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
    T = sp(7.0f, 0.040f, 0.85f, 0.0f, 0.50f, 1.0f, 1.0f, 0.5f);
    p.stands.coniferShare = 0.1f;
    B = sp(7.0f, 0.30f, 0.80f, 0.0f, 0.60f, 0.7f, 1.3f, 0.5f);
    G = sp(1.6f, 0.50f, 0.85f, 0.0f, 0.55f, 0.7f, 1.4f, 0.5f);
    R.density = 0.05f;
    break;
  case kPalMeadow:
    T = sp(7.0f, 0.075f, 0.85f, 0.0f, 0.55f, 1.0f, 1.0f, 0.5f);
    p.stands.coniferShare = 0.2f;
    B = sp(5.0f, 0.45f, 0.80f, 0.0f, 0.65f, 0.7f, 1.3f, 0.5f);
    G = sp(1.5f, 0.55f, 0.85f, 0.0f, 0.55f, 0.7f, 1.4f, 0.5f);
    break;
  case kPalHills:
    T = sp(7.0f, 0.090f, 0.82f, 0.0f, 0.55f, 1.0f, 1.0f, 0.5f);
    p.stands.coniferShare = 0.4f;
    B = sp(6.0f, 0.40f, 0.78f, 0.0f, 0.65f, 0.7f, 1.3f, 0.5f);
    G = sp(1.7f, 0.50f, 0.85f, 0.0f, 0.55f, 0.7f, 1.4f, 0.5f);
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

  // Instances stand on the surface the terrain mesh draws, not on the raw
  // field: the mesh caps slopes (cappedGrid), which can leave the raw height
  // metres above it on steep ground. Placement tests keep using the field.
  const SurfacePatch surface(f, cx, cy, chunkM, p.meshRes, 3);

  const float heightM = f.params().heightM + 1e-3f;
  const float clear2 = p.clearAroundSpawnM * p.clearAroundSpawnM;

  // Position/terrain tests shared by every species. `pad` widens the exclusion
  // test (a crown overhanging a pad). Returns false when the point is rejected;
  // *h is the surface height.
  auto accept = [&](float wx, float wy, const VegSpeciesParams &sv, float pad,
                    float *h) -> bool {
    if (wx * wx + wy * wy < clear2)
      return false;
    for (const VegExclusion &e : p.exclusions)
      if (e.reaches(wx, wy, pad))
        return false;
    *h = hAt(wx, wy);
    const float frac = *h / heightM;
    if (frac < sv.altLoFrac || frac > sv.altHiFrac)
      return false;
    const float dhdx = (hAt(wx + kLatticeM, wy) - hAt(wx - kLatticeM, wy)) /
                       (2.0f * kLatticeM);
    const float dhdy = (hAt(wx, wy + kLatticeM) - hAt(wx, wy - kLatticeM)) /
                       (2.0f * kLatticeM);
    const float flatness = 1.0f / std::sqrt(dhdx * dhdx + dhdy * dhdy + 1.0f);
    if (flatness < sv.flatMin)
      return false;
    return f.grassFactor(wx, wy, *h, flatness) >= sv.groundMin;
  };

  for (int s = 0; s < kVegSpeciesCount; ++s) {
    const VegSpeciesParams &sv = p.species[s];
    if (!(sv.cellM > 0.0f) || !(sv.density > 0.0f))
      continue;
    const uint32_t salt = speciesSalt(s);

    if (s == kVegTree) {
      // ---- trees: stands, crowns, spacing ----
      const float cell = sv.cellM;
      const int R = static_cast<int>(std::ceil(kMaxCompeteM / cell)) + 1;
      const VegTreeStands &st = p.stands;
      const float w = std::max(st.mixWidth, 0.01f);
      const float th = 1.0f - st.coniferShare;
      auto candidate = [&](int gx, int gy) {
        TreeCand c;
        const uint32_t h0 = hcell(gx, gy, p.seed, salt);
        c.x = (static_cast<float>(gx) + u01(hcell(gx, gy, p.seed, salt + 1))) * cell;
        c.y = (static_cast<float>(gy) + u01(hcell(gx, gy, p.seed, salt + 2))) * cell;
        const float tD = standField(c.x, c.y, st.standM, p.seed, 9100u);
        const float gain = smoothstep(st.openShare, st.openShare + 0.30f, tD);
        if (u01(h0) >= sv.density * gain)
          return c;
        c.on = true;
        // species group (conifer / broadleaf), then which of the group
        float pc;
        if (st.coniferShare <= 0.0f)
          pc = 0.0f;
        else if (st.coniferShare >= 1.0f)
          pc = 1.0f;
        else
          pc = smoothstep(th - w, th + w,
                          standField(c.x, c.y, st.clumpM, p.seed, 9200u));
        const bool conifer = u01(hcell(gx, gy, p.seed, salt + 7)) < pc;
        const uint32_t grp = conifer ? 9300u : 9400u;
        // Which of the group: finer, softer clumps than the group itself, so
        // both conifers (and both broadleafs) turn up within one view.
        const float pk = smoothstep(0.5f - 2.5f * w, 0.5f + 2.5f * w,
                                    standField(c.x, c.y, 0.3f * st.clumpM, p.seed, grp));
        const bool first = u01(hcell(gx, gy, p.seed, salt + 8)) < pk;
        c.kind = static_cast<uint8_t>((conifer ? 0 : 2) + (first ? 0 : 1));
        const VegTreeKind &k = kVegTreeKinds[c.kind];
        c.scale = k.scaleMin + (k.scaleMax - k.scaleMin) *
                                   u01(hcell(gx, gy, p.seed, salt + 4));
        c.crown = k.crownRadiusM * c.scale;
        c.prio = hcell(gx, gy, p.seed, salt + 9);
        c.yaw = hcell(gx, gy, p.seed, salt + 3);
        c.variant = hcell(gx, gy, p.seed, salt + 5);
        c.tint = hcell(gx, gy, p.seed, salt + 6);
        return c;
      };

      const int gx0 = static_cast<int>(std::floor(x0 / cell));
      const int gx1 = static_cast<int>(std::floor(x1 / cell));
      const int gy0 = static_cast<int>(std::floor(y0 / cell));
      const int gy1 = static_cast<int>(std::floor(y1 / cell));
      const int nx = gx1 - gx0 + 1 + 2 * R, ny = gy1 - gy0 + 1 + 2 * R;
      std::vector<TreeCand> grid(static_cast<std::size_t>(nx) * ny);
      for (int j = 0; j < ny; ++j)
        for (int i = 0; i < nx; ++i)
          grid[static_cast<std::size_t>(j) * nx + i] = candidate(gx0 - R + i, gy0 - R + j);

      for (int gy = gy0; gy <= gy1; ++gy) {
        for (int gx = gx0; gx <= gx1; ++gx) {
          const int ci = gx - gx0 + R, cj = gy - gy0 + R;
          const TreeCand &c = grid[static_cast<std::size_t>(cj) * nx + ci];
          if (!c.on || c.x < x0 || c.x >= x1 || c.y < y0 || c.y >= y1)
            continue; // empty, or owned by another chunk
          bool loses = false;
          for (int dj = -R; dj <= R && !loses; ++dj)
            for (int di = -R; di <= R; ++di) {
              if (!di && !dj)
                continue;
              const TreeCand &o = grid[static_cast<std::size_t>(cj + dj) * nx + ci + di];
              if (!o.on)
                continue;
              const float need = std::min(
                  std::max(p.treeMinSpacingM, st.crownSpacing * (c.crown + o.crown)),
                  kMaxCompeteM);
              const float dx = o.x - c.x, dy = o.y - c.y;
              if (dx * dx + dy * dy >= need * need)
                continue;
              // the higher priority stands; ties (equal hashes) go to the
              // smaller global cell so exactly one of the pair survives
              const bool oWins =
                  o.prio > c.prio ||
                  (o.prio == c.prio && std::make_pair(gy + dj, gx + di) <
                                           std::make_pair(gy, gx));
              if (oWins) {
                loses = true;
                break;
              }
            }
          if (loses)
            continue;
          float h = 0.0f;
          if (!accept(c.x, c.y, sv, 0.5f * c.crown, &h))
            continue;
          VegInstance v;
          v.pos = PgVec3{c.x, c.y, -surface.at(c.x, c.y)};
          v.yaw = 6.2831853f * u01(c.yaw);
          v.scale = c.scale;
          v.species = static_cast<uint8_t>(kVegTree);
          v.kind = c.kind;
          v.variant = static_cast<uint8_t>(c.variant % 12u);
          v.tint = u01(c.tint);
          out.push_back(v);
        }
      }
      continue;
    }

    // ---- everything else: one jittered cell grid per species ----
    const float margin = 0.05f;
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
        float h = 0.0f;
        if (!accept(wx, wy, sv, 0.0f, &h))
          continue;

        VegInstance v;
        v.yaw = 6.2831853f * u01(hcell(gx, gy, p.seed, salt + 3));
        v.scale = sv.scaleMin + (sv.scaleMax - sv.scaleMin) *
                                    u01(hcell(gx, gy, p.seed, salt + 4));
        // A rock stands on its footprint's lowest point (a slope would
        // otherwise leave the downhill side in the air); the rest on the spot.
        float hz = surface.at(wx, wy);
        if (s == kVegRock) {
          // A rock has one yaw and no tilt, so on a slope it can only rest on
          // the lowest point under its footprint (the downhill toe); that
          // buries the uphill side. Bury it by at most kVegRockMaxSinkM per
          // unit scale (so it never vanishes into the slope) and leave a toe
          // gap of at most kVegRockMaxToeGapM per unit scale: steeper ground
          // takes no rock rather than one hovering over it.
          const float lo =
              surface.lowestUnder(wx, wy, kVegRockRadiusM * v.scale);
          const float buriedMax = hz - kVegRockMaxSinkM * v.scale;
          if (buriedMax - lo > kVegRockMaxToeGapM * v.scale)
            continue;
          hz = std::max(lo, buriedMax) - kVegRockSinkM;
        }
        v.pos = PgVec3{wx, wy, -hz};
        v.species = static_cast<uint8_t>(s);
        v.variant = static_cast<uint8_t>(hcell(gx, gy, p.seed, salt + 5) %
                                         static_cast<uint32_t>(kVegVariants));
        v.tint = u01(hcell(gx, gy, p.seed, salt + 6));
        if (s == kVegGrassClump)
          v.paint = vegPackHue(p.groundColor);
        out.push_back(v);
      }
    }
  }
  return out;
}

// Tree colliders, when wired to the sim, must follow the Trees flag (rocks are not trees).
std::vector<VegCollider>
vegetationColliders(const std::vector<VegInstance> &instances) {
  std::vector<VegCollider> out;
  for (const VegInstance &v : instances) {
    if (v.species == kVegTree) {
      const VegTreeKind &k = vegTreeKind(v.kind);
      out.push_back(
          VegCollider{v.pos, k.trunkRadiusM * v.scale, k.heightM * v.scale, false});
    } else if (v.species == kVegRock && v.scale >= kVegRockColliderMinScale) {
      out.push_back(VegCollider{v.pos, kVegRockRadiusM * v.scale,
                                kVegRockHeightM * v.scale, true});
    }
  }
  return out;
}

} // namespace vsim::procgen
