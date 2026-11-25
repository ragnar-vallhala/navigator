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
#include "Helipads.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace vsim::procgen {
namespace {
uint32_t hash(int gx, int gy) {
  uint32_t h = uint32_t(gx) * 0x9e3779b1u ^ uint32_t(gy) * 0x85ebca77u;
  h ^= h >> 16;
  h *= 0x7feb352du;
  h ^= h >> 15;
  h *= 0x846ca68bu;
  h ^= h >> 16;
  return h;
}
} // namespace

float helipadRelief(const HeightFn &surface, float x, float y, float radiusM,
                    int ringPoints) {
  const float c = surface(x, y);
  float worst = 0.0f;
  for (int a = 0; a < ringPoints; ++a) {
    const float t = 6.2831853f * float(a) / float(ringPoints);
    worst = std::max(worst, std::fabs(surface(x + radiusM * std::cos(t),
                                              y + radiusM * std::sin(t)) - c));
  }
  return worst;
}

std::vector<PgVec3> helipadsAround(const HeightFn &surface, const HeightFn &raw,
                                   float cx, float cy, float range,
                                   const HelipadParams &p) {
  std::vector<PgVec3> pads;
  if (!surface || !raw)
    return pads;
  pads.push_back(PgVec3{0.0f, 0.0f, surface(0.0f, 0.0f)});

  const float kGrid = p.cellM, kRange = range, kEps = 2.0f;
  const int kCandidates = 4; // probe several spots/cell, keep the flattest
  auto slopeAt = [&](float px, float py) {
    const float dhdx =
        (raw(px + kEps, py) - raw(px - kEps, py)) / (2.0f * kEps);
    const float dhdy =
        (raw(px, py + kEps) - raw(px, py - kEps)) / (2.0f * kEps);
    return std::sqrt(dhdx * dhdx + dhdy * dhdy);
  };
  const int gx0 = int(std::floor((cx - kRange) / kGrid));
  const int gx1 = int(std::floor((cx + kRange) / kGrid));
  const int gy0 = int(std::floor((cy - kRange) / kGrid));
  const int gy1 = int(std::floor((cy + kRange) / kGrid));
  for (int gx = gx0; gx <= gx1; ++gx)
    for (int gy = gy0; gy <= gy1; ++gy) {
      if (gx == 0 && gy == 0)
        continue; // origin reserved for the home pad
      if ((hash(gx, gy) & 0xffu) > 180u)
        continue; // ~70% of cells get a pad
      float bestSlope = 1e9f, bx = 0, by = 0;
      for (int k = 0; k < kCandidates; ++k) {
        const uint32_t h2 =
            hash(gx * 73856093 + k * 19349663, gy * 83492791 - k);
        const float jx = float(h2 & 0xffu) / 255.0f - 0.5f;
        const float jy = float((h2 >> 8) & 0xffu) / 255.0f - 0.5f;
        const float px = (float(gx) + 0.5f + jx * 0.8f) * kGrid;
        const float py = (float(gy) + 0.5f + jy * 0.8f) * kGrid;
        if (p.blocked && p.blocked(px, py, p.radiusM + p.blockedClearM))
          continue;
        const float s = slopeAt(px, py);
        if (s < bestSlope) {
          bestSlope = s;
          bx = px;
          by = py;
        }
      }
      if (bestSlope > p.maxSlope)
        continue; // no flat-enough spot in this cell
      if ((bx - cx) * (bx - cx) + (by - cy) * (by - cy) > kRange * kRange)
        continue;
      if (p.maxReliefM > 0.0f &&
          helipadRelief(surface, bx, by, p.radiusM, p.ringPoints) >
              p.maxReliefM)
        continue; // footprint would float or bury the pad
      pads.push_back(PgVec3{bx, by, surface(bx, by)});
    }
  return pads;
}

std::vector<PgVec3> helipadCollisionSoup(const std::vector<PgVec3> &pads,
                                         float x0, float y0, float x1, float y1,
                                         float radiusM, float deckM,
                                         int segments) {
  std::vector<PgVec3> out;
  const float R = radiusM;
  const int N = segments;
  auto tri2 = [&](const PgVec3 &a, const PgVec3 &b, const PgVec3 &c) {
    out.push_back(a);
    out.push_back(b);
    out.push_back(c);
    out.push_back(a);
    out.push_back(c);
    out.push_back(b);
  };
  for (const PgVec3 &pad : pads) {
    if (pad.x < x0 - R || pad.x > x1 + R || pad.y < y0 - R || pad.y > y1 + R)
      continue;
    const float px = pad.x, py = pad.y;
    const float zDeck = -pad.z - deckM; // top
    const float zBase = -pad.z;         // terrain
    const PgVec3 ctr{px, py, zDeck};
    for (int j = 0; j < N; ++j) {
      const double a0 = 2 * M_PI * j / N, a1 = 2 * M_PI * (j + 1) / N;
      const PgVec3 r0{static_cast<float>(px + R * std::cos(a0)),
                      static_cast<float>(py + R * std::sin(a0)), zDeck};
      const PgVec3 r1{static_cast<float>(px + R * std::cos(a1)),
                      static_cast<float>(py + R * std::sin(a1)), zDeck};
      const PgVec3 b0{r0.x, r0.y, zBase};
      const PgVec3 b1{r1.x, r1.y, zBase};
      tri2(ctr, r0, r1); // deck cap (landing surface)
      tri2(r0, b0, b1);  // side wall down to terrain
      tri2(r0, b1, r1);
    }
  }
  return out;
}

} // namespace vsim::procgen
