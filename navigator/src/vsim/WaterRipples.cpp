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
#include "WaterRipples.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace vsim {

namespace {

constexpr float kBaseDamp = 0.35f;   // 1/s, open water
constexpr float kShoreDamp = 14.0f;  // 1/s outside the shore: absorb
constexpr float kSpongeDamp = 12.0f; // 1/s at the window edge
constexpr int kSpongeTexels = 14;

float smoothstep(float a, float b, float x) {
  const float t = std::clamp((x - a) / (b - a), 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

// Deterministic per-texel, per-step noise in [-1, 1].
float hashNoise(int ix, int iy, uint64_t step) {
  uint32_t h = uint32_t(ix) * 73856093u ^ uint32_t(iy) * 19349663u ^
               uint32_t(step) * 83492791u;
  h ^= h >> 13;
  h *= 0x5bd1e995u;
  h ^= h >> 15;
  return float(h & 0xffffu) / 32767.5f - 1.0f;
}

// Smooth stirring: value noise on a lattice 3 texels apart (so the forcing has
// no texel-scale content the grid would show as squares), re-seeded every 4
// steps. Deterministic in (texel, step).
float stirNoise(int ix, int iy, uint64_t step) {
  constexpr float kLat = 3.0f;
  const float fx = float(ix) / kLat, fy = float(iy) / kLat;
  const int x0 = int(std::floor(fx)), y0 = int(std::floor(fy));
  const float tx = smoothstep(0.0f, 1.0f, fx - float(x0));
  const float ty = smoothstep(0.0f, 1.0f, fy - float(y0));
  const uint64_t s = step >> 2;
  const float a = hashNoise(x0, y0, s), b = hashNoise(x0 + 1, y0, s);
  const float c = hashNoise(x0, y0 + 1, s), d = hashNoise(x0 + 1, y0 + 1, s);
  return (a * (1 - tx) + b * tx) * (1 - ty) + (c * (1 - tx) + d * tx) * ty;
}

} // namespace

WaterRipples::WaterRipples()
    : h_(size_t(kN) * kN, 0.0f), v_(size_t(kN) * kN, 0.0f),
      damp_(size_t(kN) * kN, kShoreDamp) {}

void WaterRipples::bindBody(const procgen::WaterBody &b) {
  if (bound_ && b.cx == bx_ && b.cy == by_)
    return;
  bound_ = true;
  bx_ = b.cx;
  by_ = b.cy;
  level_ = b.level;
  // Snap the window origin to whole texels so a texel never straddles a shift.
  const float half = 0.5f * sizeM();
  ox_ = std::floor((b.cx - half) / kCellM) * kCellM;
  oy_ = std::floor((b.cy - half) / kCellM) * kCellM;
  std::fill(h_.begin(), h_.end(), 0.0f);
  std::fill(v_.begin(), v_.end(), 0.0f);
  energy_ = maxAbs_ = 0.0f;
  clockSet_ = false;
  acc_ = 0.0;
  for (int j = 0; j < kN; ++j)
    for (int i = 0; i < kN; ++i) {
      const float wx = ox_ + (float(i) + 0.5f) * kCellM;
      const float wy = oy_ + (float(j) + 0.5f) * kCellM;
      const float dx = wx - b.cx, dy = wy - b.cy;
      const float d = std::sqrt(dx * dx + dy * dy);
      const float R = b.shoreRadiusToward(dx, dy);
      // Open water inside the shore, absorbing past it (a short ramp so the
      // shoreline itself eats waves instead of reflecting them).
      float g = kBaseDamp + (kShoreDamp - kBaseDamp) * smoothstep(R - 1.0f, R + 0.5f, d);
      const int edge = std::min(std::min(i, kN - 1 - i), std::min(j, kN - 1 - j));
      if (edge < kSpongeTexels)
        g = std::max(g, kBaseDamp + (kSpongeDamp - kBaseDamp) *
                                        (1.0f - float(edge) / float(kSpongeTexels)));
      damp_[size_t(j) * kN + size_t(i)] = g;
    }
}

void WaterRipples::setRotors(const RippleRotor *r, int n) {
  rotors_.assign(r, r + std::max(n, 0));
  forced_ = false;
  for (const RippleRotor &q : rotors_)
    if (q.strength > 0.02f && q.heightM < kMaxHeightM)
      forced_ = true;
}

void WaterRipples::advanceTo(double tSec) {
  if (!bound_)
    return;
  if (!clockSet_) {
    clock_ = tSec;
    clockSet_ = true;
    return;
  }
  const double dt = tSec - clock_;
  if (dt <= 0.0)
    return; // a second view asking for the same moment: nothing to do
  clock_ = tSec;
  acc_ += std::min(dt, 0.25);
  int n = 0;
  while (acc_ >= double(kStepS) && n < kMaxSubsteps) {
    step();
    acc_ -= double(kStepS);
    ++n;
  }
  if (n == kMaxSubsteps) // fell behind: drop the backlog instead of spiralling
    acc_ = 0.0;
}

void WaterRipples::stepFixed(int n) {
  for (int k = 0; k < n; ++k)
    step();
}

void WaterRipples::applyForcing() {
  for (const RippleRotor &r : rotors_) {
    if (r.strength <= 0.02f || r.heightM >= kMaxHeightM || r.heightM < -0.2f)
      continue;
    const float hh = std::max(r.heightM, 0.0f);
    // Downwash weakens roughly with the square of height (and is cut off
    // smoothly at the range limit).
    const float fall = (1.0f - smoothstep(0.6f * kMaxHeightM, kMaxHeightM, hh)) /
                       (1.0f + (hh / 1.5f) * (hh / 1.5f));
    const float s = std::clamp(r.strength, 0.0f, 1.0f) * fall;
    // The wash spreads with height; never narrower than ~1.5 texels, which is
    // all the grid can resolve (a prop is far smaller than a texel).
    const float reff = std::max(r.radiusM + 0.25f * hh, 1.5f * kCellM);
    const float reach = 2.4f * reff + 0.5f;
    const int i0 = std::max(0, int(std::floor((r.x - reach - ox_) / kCellM)));
    const int i1 = std::min(kN - 1, int(std::floor((r.x + reach - ox_) / kCellM)));
    const int j0 = std::max(0, int(std::floor((r.y - reach - oy_) / kCellM)));
    const int j1 = std::min(kN - 1, int(std::floor((r.y + reach - oy_) / kCellM)));
    const float dimple = 0.025f * s; // metres the surface is pressed down
    for (int j = j0; j <= j1; ++j)
      for (int i = i0; i <= i1; ++i) {
        const float wx = ox_ + (float(i) + 0.5f) * kCellM;
        const float wy = oy_ + (float(j) + 0.5f) * kCellM;
        const float dx = wx - r.x, dy = wy - r.y, d = std::sqrt(dx * dx + dy * dy);
        const size_t k = size_t(j) * kN + size_t(i);
        const float disc = 1.0f - smoothstep(0.6f * reff, 1.5f * reff + 0.3f, d);
        // Spring toward the pressed-down surface under the wash...
        v_[k] += (-dimple * disc - h_[k]) * disc * 30.0f * kStepS;
        // ...and stir the ring around it with deterministic turbulence.
        const float ring = smoothstep(0.3f * reff, 1.0f * reff + 0.2f, d) *
                           (1.0f - smoothstep(1.2f * reff, reach, d));
        v_[k] += stirNoise(i, j, steps_) * 2.0f * s * ring * kStepS;
      }
  }
}

void WaterRipples::step() {
  if (!bound_)
    return;
  if (forced_)
    applyForcing();
  const float c2 = kSpeedMps * kSpeedMps / (kCellM * kCellM);
  float maxAbs = 0.0f, e = 0.0f;
  for (int j = 1; j < kN - 1; ++j)
    for (int i = 1; i < kN - 1; ++i) {
      const size_t k = size_t(j) * kN + size_t(i);
      const float lap = h_[k - 1] + h_[k + 1] + h_[k - kN] + h_[k + kN] - 4.0f * h_[k];
      v_[k] += (c2 * lap - damp_[k] * v_[k]) * kStepS;
    }
  for (int j = 1; j < kN - 1; ++j)
    for (int i = 1; i < kN - 1; ++i) {
      const size_t k = size_t(j) * kN + size_t(i);
      h_[k] += v_[k] * kStepS;
      // Surface relaxation toward the level too, so a dimple does not drift.
      h_[k] -= h_[k] * 0.05f * kStepS;
      maxAbs = std::max(maxAbs, std::fabs(h_[k]));
      e += std::fabs(h_[k]) + 0.1f * std::fabs(v_[k]);
    }
  maxAbs_ = maxAbs;
  energy_ = e / float(kN * kN);
  ++steps_;
}

float WaterRipples::heightAt(float wx, float wy) const {
  const float fx = (wx - ox_) / kCellM - 0.5f, fy = (wy - oy_) / kCellM - 0.5f;
  const int i = int(std::floor(fx)), j = int(std::floor(fy));
  if (i < 0 || j < 0 || i >= kN - 1 || j >= kN - 1)
    return 0.0f;
  const float tx = fx - float(i), ty = fy - float(j);
  const auto at = [&](int a, int b) { return h_[size_t(b) * kN + size_t(a)]; };
  return (at(i, j) * (1 - tx) + at(i + 1, j) * tx) * (1 - ty) +
         (at(i, j + 1) * (1 - tx) + at(i + 1, j + 1) * tx) * ty;
}

uint16_t WaterRipples::floatToHalf(float f) {
  uint32_t x;
  std::memcpy(&x, &f, 4);
  const uint32_t sign = (x >> 16) & 0x8000u;
  int32_t e = int32_t((x >> 23) & 0xffu) - 127 + 15;
  uint32_t m = x & 0x7fffffu;
  if (e <= 0) { // subnormal half or zero
    if (e < -10)
      return uint16_t(sign);
    m |= 0x800000u;
    const uint32_t shift = uint32_t(14 - e);
    uint32_t hm = m >> shift;
    if ((m >> (shift - 1)) & 1u)
      ++hm;
    return uint16_t(sign | hm);
  }
  if (e >= 31)
    return uint16_t(sign | 0x7c00u); // overflow to inf (heights are centimetres)
  uint32_t hm = m >> 13;
  if (m & 0x1000u)
    ++hm; // round to nearest (a carry rolls into the exponent correctly)
  return uint16_t(sign | (uint32_t(e) << 10) + hm);
}

void WaterRipples::toHalf(std::vector<uint16_t> &out) const {
  out.resize(h_.size());
  for (size_t i = 0; i < h_.size(); ++i)
    out[i] = floatToHalf(h_[i]);
}

} // namespace vsim
