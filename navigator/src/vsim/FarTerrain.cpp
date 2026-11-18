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
#include <chrono>
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
  if (pending_.valid())
    pending_.wait(); // a build in flight reads the old field
  pending_ = {};
  pendingWin_ = Window{};
  field_ = std::move(f);
  clip_[0] = clip_[1] = clip_[2] = clip_[3] = 0.0f;
  built_ = Window{};
  verts_.clear();
}

float FarTerrain::groundZ(float x, float y, float fallback) const {
  return field_ ? -field_->height(x, y) : fallback;
}

bool FarTerrain::update(const Window &w) {
  if (!field_)
    return false;
  bool changed = false;
  // Collect a finished build (the heavy part runs on a worker thread: sampling
  // the meshed surface around the window costs ~100 ms).
  if (pending_.valid() &&
      pending_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    Built b = pending_.get();
    verts_ = std::move(b.verts);
    std::copy(b.clip, b.clip + 4, clip_);
    built_ = pendingWin_;
    changed = true;
  }
  if (!pending_.valid() && w.valid() && !(w == built_)) {
    pendingWin_ = w;
    auto field = field_;
    pending_ = std::async(std::launch::async,
                          [field, w] { return build(*field, w); });
  }
  return changed;
}

FarTerrain::Built FarTerrain::build(const procgen::TerrainField &f,
                                    const Window &w) {
  Built out;
  std::vector<float> &verts_ = out.verts;
  float(&clip_)[4] = out.clip;

  // R_0: the window shrunk by the overlap, on the finest lattice.
  Rect inner{
      snapUp(static_cast<float>(w.minCx) * w.chunkM + kOverlapM, kCell[0]),
      snapDown(static_cast<float>(w.maxCx + 1) * w.chunkM - kOverlapM, kCell[0]),
      snapUp(static_cast<float>(w.minCy) * w.chunkM + kOverlapM, kCell[0]),
      snapDown(static_cast<float>(w.maxCy + 1) * w.chunkM - kOverlapM, kCell[0])};
  const Rect win{static_cast<float>(w.minCx) * w.chunkM,
                 static_cast<float>(w.maxCx + 1) * w.chunkM,
                 static_cast<float>(w.minCy) * w.chunkM,
                 static_cast<float>(w.maxCy + 1) * w.chunkM};
  const float chunkM = w.chunkM;
  clip_[0] = win.x0;
  clip_[1] = win.y0;
  clip_[2] = win.x1;
  clip_[3] = win.y1;

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
    std::unordered_map<std::int64_t, float> hcache, scache;
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
      // Near the window the ring follows the surface the chunks actually mesh
      // (slope-capped, so it can sit well below the raw field on steep ground)
      // and blends to the raw field kBlendM beyond the edge. The renderer clips
      // the ring inside the window, so there is nothing to poke through.
      const float d = std::max({win.x0 - x, x - win.x1, win.y0 - y, y - win.y1});
      float hz = h;
      if (d < kBlendM) {
        const float wgt = std::clamp(d / kBlendM, 0.0f, 1.0f);
        const auto k = key(static_cast<int>(std::lround(x / c)),
                           static_cast<int>(std::lround(y / c)));
        auto it = scache.find(k);
        if (it == scache.end())
          it = scache
                   .emplace(k, procgen::surfaceHeight(f, x, y, chunkM,
                                                      procgen::kStreamChunkRes))
                   .first;
        hz = it->second * (1.0f - wgt) + h * wgt;
      }
      return Vert{{x, y, -hz, n.x, n.y, n.z, col.x, col.y, col.z}};
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
  return out;
}

} // namespace vsim
