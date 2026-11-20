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
#include "WaterField.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>

namespace vsim::procgen {

namespace {

constexpr float kTwoPi = 6.2831853f;
constexpr float kBankM = 12.0f;
constexpr float kRimMargin = 0.15f; // water sits this far below the lowest rim
constexpr float kMaxRimRelief = 6.0f; // ground may vary this much around a body
constexpr int kRimAngles = 16;

uint32_t mix32(uint32_t x) {
  x ^= x >> 16;
  x *= 0x7feb352du;
  x ^= x >> 15;
  x *= 0x846ca68bu;
  x ^= x >> 16;
  return x;
}

uint32_t cellHash(uint32_t seed, int ix, int iy, uint32_t salt) {
  return mix32(seed ^ mix32(static_cast<uint32_t>(ix) * 73856093u ^
                            static_cast<uint32_t>(iy) * 19349663u ^ salt));
}

float unit(uint32_t h) { return static_cast<float>(h & 0xffffffu) / 16777215.0f; }

float smoothstep(float a, float b, float x) {
  const float t = std::clamp((x - a) / (b - a), 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

std::atomic<uint32_t> gNextId{1};

// Last body looked up on this thread: a chunk mesh asks for the same cell
// thousands of times in a row, and building a body costs ~50 height samples.
struct CacheEntry {
  uint32_t id = 0;
  int ix = 0, iy = 0;
  bool valid = false, present = false;
  WaterBody body;
};
thread_local CacheEntry tCache[2];

int cellIndex(float v, float cell) { return static_cast<int>(std::floor(v / cell)); }

} // namespace

float WaterBody::shoreRadius(float angle) const {
  return radiusM * (1.0f + amp[0] * std::sin(2.0f * angle + phase[0]) +
                    amp[1] * std::sin(3.0f * angle + phase[1]));
}

float WaterBody::shoreRadiusToward(float dx, float dy) const {
  return shoreRadius(std::atan2(dy, dx));
}

// Per-field cache of every cell's body (or its absence). The thread-local
// entries above are a lock-free front for the common "same cell again" case;
// this catches sweeps that cross many 170 m cells (minimap, wide chunks).
struct WaterField::BodyCache {
  struct Slot {
    bool present = false;
    WaterBody body;
  };
  std::shared_mutex mu;
  std::unordered_map<uint64_t, Slot> map;
  static uint64_t key(int ix, int iy) {
    return (static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 32) |
           static_cast<uint32_t>(iy);
  }
};

WaterField::WaterField(const WaterParams &p)
    : p_(p), id_(gNextId.fetch_add(1, std::memory_order_relaxed)),
      cache_(std::make_shared<BodyCache>()) {}

bool WaterField::makeBody(const HeightSource &src, int ix, int iy,
                          WaterBody &b) const {
  if (!enabled())
    return false;
  const uint32_t h0 = cellHash(p_.seed, ix, iy, 0x77a7e4u);
  // Probability of a body in a cell grows with the palette's pond amount.
  if (unit(h0) > std::min(0.6f, p_.coverage * 0.8f))
    return false;

  b.radiusM = 14.0f + 18.0f * unit(cellHash(p_.seed, ix, iy, 0x1u));
  b.bankM = kBankM;
  b.amp[0] = 0.10f + 0.12f * unit(cellHash(p_.seed, ix, iy, 0x2u));
  b.amp[1] = 0.05f + 0.07f * unit(cellHash(p_.seed, ix, iy, 0x3u));
  b.phase[0] = kTwoPi * unit(cellHash(p_.seed, ix, iy, 0x4u));
  b.phase[1] = kTwoPi * unit(cellHash(p_.seed, ix, iy, 0x5u));
  // Whole body (shore + bank) stays inside its own cell.
  const float reach = b.radiusM * (1.0f + b.amp[0] + b.amp[1]) + b.bankM;
  const float span = p_.cellM - 2.0f * reach;
  if (span <= 0.0f)
    return false;
  b.cx = (static_cast<float>(ix) * p_.cellM) + reach +
         span * unit(cellHash(p_.seed, ix, iy, 0x6u));
  b.cy = (static_cast<float>(iy) * p_.cellM) + reach +
         span * unit(cellHash(p_.seed, ix, iy, 0x7u));

  // Keep the spawn and its pad dry.
  if (std::hypot(b.cx, b.cy) < p_.spawnClearM + reach)
    return false;

  // Level = lowest ground around the shore; reject uneven or high ground.
  float lo = 1e9f, hi = -1e9f, rimLo = 1e9f;
  for (int i = 0; i < kRimAngles; ++i) {
    const float a = kTwoPi * static_cast<float>(i) / kRimAngles;
    const float R = b.shoreRadius(a);
    const float ca = std::cos(a), sa = std::sin(a);
    const float ds[3] = {R, R + 0.5f * b.bankM, R + b.bankM};
    for (int k = 0; k < 3; ++k) {
      const float h = src.baseHeight(b.cx + ca * ds[k], b.cy + sa * ds[k]);
      lo = std::min(lo, h);
      hi = std::max(hi, h);
      rimLo = std::min(rimLo, h);
    }
  }
  hi = std::max(hi, src.baseHeight(b.cx, b.cy));
  if (hi - lo > kMaxRimRelief)
    return false;
  b.level = rimLo - std::min(kRimMargin, 0.25f * std::max(rimLo, 0.0f));
  if (b.level > p_.maxLevelM)
    return false;
  // Never cut below the ground plane (heights stay >= 0): shallow on low ground.
  b.depthM = std::min(p_.maxDepthM, 0.9f * b.level);
  return b.depthM >= 0.12f; // flat worlds get shallow ponds, not none
}

bool WaterField::bodyAt(const HeightSource &src, float wx, float wy,
                        WaterBody &out) const {
  if (!enabled())
    return false;
  const int ix = cellIndex(wx, p_.cellM), iy = cellIndex(wy, p_.cellM);
  for (CacheEntry &c : tCache)
    if (c.valid && c.id == id_ && c.ix == ix && c.iy == iy) {
      if (c.present)
        out = c.body;
      return c.present;
    }
  static thread_local int next = 0;
  CacheEntry &c = tCache[next];
  next ^= 1;
  c.id = id_;
  c.ix = ix;
  c.iy = iy;
  c.valid = true;
  const uint64_t key = BodyCache::key(ix, iy);
  {
    std::shared_lock lock(cache_->mu);
    const auto it = cache_->map.find(key);
    if (it != cache_->map.end()) {
      c.present = it->second.present;
      c.body = it->second.body;
      if (c.present)
        out = c.body;
      return c.present;
    }
  }
  c.present = makeBody(src, ix, iy, c.body);
  {
    std::unique_lock lock(cache_->mu);
    if (cache_->map.size() > 8192) // bounded: a long flight sweeps many cells
      cache_->map.clear();
    cache_->map[key] = {c.present, c.body};
  }
  if (c.present)
    out = c.body;
  return c.present;
}

float WaterField::carve(const HeightSource &src, float wx, float wy,
                        float hBase) const {
  WaterBody b;
  if (!bodyAt(src, wx, wy, b))
    return hBase;
  const float dx = wx - b.cx, dy = wy - b.cy;
  const float d = std::sqrt(dx * dx + dy * dy);
  const float R = b.shoreRadiusToward(dx, dy);
  if (d >= R + b.bankM)
    return hBase;
  const float q = d / R;
  const float bowl = q < 1.0f ? 1.0f - q * q : 0.0f;
  const float target = b.level - b.depthM * bowl;
  const float s = 1.0f - smoothstep(R, R + b.bankM, d);
  const float h = hBase + (target - hBase) * s;
  // Outside the shore the ground is never below the water (a dip between the
  // rim samples that set the level is filled), so the pond cannot leak.
  return d >= R ? std::max(h, b.level) : h; // (continuous at the shore: h = level there)
}

float WaterField::inside(const HeightSource &src, float wx, float wy) const {
  WaterBody b;
  if (!bodyAt(src, wx, wy, b))
    return 0.0f;
  const float dx = wx - b.cx, dy = wy - b.cy;
  const float R = b.shoreRadiusToward(dx, dy);
  return 1.0f - smoothstep(R - 1.0f, R, std::sqrt(dx * dx + dy * dy));
}

float WaterField::shoreRing(const HeightSource &src, float wx, float wy) const {
  WaterBody b;
  if (!bodyAt(src, wx, wy, b))
    return 0.0f;
  const float dx = wx - b.cx, dy = wy - b.cy;
  const float d = std::sqrt(dx * dx + dy * dy);
  const float R = b.shoreRadiusToward(dx, dy);
  return smoothstep(R - 1.0f, R, d) * (1.0f - smoothstep(R + 1.5f, R + 5.0f, d));
}

bool WaterField::nearWater(const HeightSource &src, float wx, float wy,
                           float marginM) const {
  if (!enabled())
    return false;
  const float r = marginM;
  const int ix0 = cellIndex(wx - r, p_.cellM), ix1 = cellIndex(wx + r, p_.cellM);
  const int iy0 = cellIndex(wy - r, p_.cellM), iy1 = cellIndex(wy + r, p_.cellM);
  for (int iy = iy0; iy <= iy1; ++iy)
    for (int ix = ix0; ix <= ix1; ++ix) {
      WaterBody b;
      if (!makeBody(src, ix, iy, b))
        continue;
      const float dx = wx - b.cx, dy = wy - b.cy;
      if (std::sqrt(dx * dx + dy * dy) < b.shoreRadiusToward(dx, dy) + marginM)
        return true;
    }
  return false;
}

std::vector<WaterBody> WaterField::bodiesIn(const HeightSource &src, float x0,
                                            float y0, float x1, float y1) const {
  std::vector<WaterBody> out;
  if (!enabled())
    return out;
  for (int iy = cellIndex(y0, p_.cellM); iy <= cellIndex(y1, p_.cellM); ++iy)
    for (int ix = cellIndex(x0, p_.cellM); ix <= cellIndex(x1, p_.cellM); ++ix) {
      WaterBody b;
      if (makeBody(src, ix, iy, b))
        out.push_back(b);
    }
  return out;
}

} // namespace vsim::procgen
