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
// FarTerrain.cpp — see FarTerrain.h.
#include "FarTerrain.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace vsim {

namespace {

constexpr int kRings = 4;
constexpr float kCell[kRings] = {48.0f, 96.0f, 192.0f, 384.0f};
// How far each ring reaches beyond the previous one's outer edge [m].
constexpr float kWidth[kRings] = {480.0f, 768.0f, 1536.0f, 3072.0f};

struct Rect {
  float x0, x1, y0, y1;
  bool contains(float x, float y) const {
    return x > x0 && x < x1 && y > y0 && y < y1;
  }
};

float snapDown(float v, float c) { return std::floor(v / c) * c; }
float snapUp(float v, float c) { return std::ceil(v / c) * c; }

struct Vert {
  float f[9]; // pos(3), normal(3), colour(3)
};

} // namespace

void FarTerrain::setField(std::shared_ptr<const procgen::TerrainField> f) {
  field_ = std::move(f);
  built_ = Window{};
  verts_.clear();
}

float FarTerrain::groundZ(float x, float y, float fallback) const {
  return field_ ? -field_->height(x, y) : fallback;
}

bool FarTerrain::update(const Window &w) {
  if (!field_ || !w.valid() || w == built_)
    return false;
  build(w);
  built_ = w;
  return true;
}

void FarTerrain::build(const Window &w) {
  const procgen::TerrainField &f = *field_;
  verts_.clear();

  // R_0: the window shrunk by the overlap, on the finest lattice.
  Rect inner{
      snapUp(static_cast<float>(w.minCx) * w.chunkM + kOverlapM, kCell[0]),
      snapDown(static_cast<float>(w.maxCx + 1) * w.chunkM - kOverlapM, kCell[0]),
      snapUp(static_cast<float>(w.minCy) * w.chunkM + kOverlapM, kCell[0]),
      snapDown(static_cast<float>(w.maxCy + 1) * w.chunkM - kOverlapM, kCell[0])};
  const Rect r0 = inner;

  for (int ring = 0; ring < kRings; ++ring) {
    const float c = kCell[ring];
    const bool last = ring == kRings - 1;
    // Outer edge on the NEXT ring's lattice (so cell sizes meet in a clean 2:1).
    const float oc = last ? c : kCell[ring + 1];
    const Rect outer{snapDown(inner.x0 - kWidth[ring], oc),
                     snapUp(inner.x1 + kWidth[ring], oc),
                     snapDown(inner.y0 - kWidth[ring], oc),
                     snapUp(inner.y1 + kWidth[ring], oc)};

    std::unordered_map<std::int64_t, Vert> cache;
    auto key = [](int i, int j) {
      return (static_cast<std::int64_t>(i) << 32) ^
             static_cast<std::int64_t>(static_cast<std::uint32_t>(j));
    };
    // Heights are cached per lattice node: each node's normal reuses its
    // neighbours' heights instead of four more field evaluations.
    std::unordered_map<std::int64_t, float> hcache;
    auto heightAt = [&](float x, float y) {
      const auto k = key(static_cast<int>(std::lround(x / c)),
                         static_cast<int>(std::lround(y / c)));
      auto it = hcache.find(k);
      if (it == hcache.end())
        it = hcache.emplace(k, f.height(x, y)).first;
      return it->second;
    };
    auto sample = [&](float x, float y) {
      const float h = heightAt(x, y);
      float nx = (heightAt(x + c, y) - heightAt(x - c, y)) / (2.0f * c);
      float ny = (heightAt(x, y + c) - heightAt(x, y - c)) / (2.0f * c);
      float nz = -1.0f;
      const float len = std::sqrt(nx * nx + ny * ny + 1.0f);
      nx /= len;
      ny /= len;
      nz /= len;
      const procgen::PgVec3 n{nx, ny, nz};
      const float flat = n.z < 0.0f ? -n.z : 0.0f;
      const procgen::PgVec3 col = f.colorAt(x, y, h, flat).color;
      // Sink below the near surface around the overlap, fading with distance
      // beyond the window (Chebyshev distance outside R_0).
      const float d = std::max({r0.x0 - x, x - r0.x1, r0.y0 - y, y - r0.y1, 0.0f});
      const float sink = kSinkM * std::max(0.0f, 1.0f - d / kSinkFadeM);
      return Vert{{x, y, -h + sink, n.x, n.y, n.z, col.x, col.y, col.z}};
    };
    auto vertAt = [&](int i, int j) -> const Vert & {
      const auto k = key(i, j);
      auto it = cache.find(k);
      if (it != cache.end())
        return it->second;
      const float x = static_cast<float>(i) * c, y = static_cast<float>(j) * c;
      Vert v = sample(x, y);
      // On the outer edge, odd nodes lie on the midpoint of the coarser ring's
      // edge segment: take its value so the two rings meet without a crack.
      if (!last) {
        const bool onX = x == outer.x0 || x == outer.x1;
        const bool onY = y == outer.y0 || y == outer.y1;
        if ((onX != onY) && (((onX ? j : i) & 1) != 0)) {
          const Vert a = sample(onX ? x : x - c, onX ? y - c : y);
          const Vert b = sample(onX ? x : x + c, onX ? y + c : y);
          for (int q = 2; q < 9; ++q)
            v.f[q] = 0.5f * (a.f[q] + b.f[q]);
        }
      }
      return cache.emplace(k, v).first->second;
    };

    const int i0 = static_cast<int>(std::lround(outer.x0 / c));
    const int i1 = static_cast<int>(std::lround(outer.x1 / c));
    const int j0 = static_cast<int>(std::lround(outer.y0 / c));
    const int j1 = static_cast<int>(std::lround(outer.y1 / c));
    for (int j = j0; j < j1; ++j) {
      for (int i = i0; i < i1; ++i) {
        const float cx = (static_cast<float>(i) + 0.5f) * c;
        const float cy = (static_cast<float>(j) + 0.5f) * c;
        if (inner.contains(cx, cy))
          continue; // covered by the finer ring / the chunks
        const Vert &v00 = vertAt(i, j), &v10 = vertAt(i + 1, j),
                   &v11 = vertAt(i + 1, j + 1), &v01 = vertAt(i, j + 1);
        // Same split as the chunk meshes: (00,10,11) and (00,11,01).
        for (const Vert *v : {&v00, &v10, &v11, &v00, &v11, &v01})
          verts_.insert(verts_.end(), v->f, v->f + 9);
      }
    }
    inner = outer;
  }
}

} // namespace vsim
