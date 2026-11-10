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
#include "GroundCover.h"

#include <algorithm>
#include <cmath>

namespace vsim::procgen {
namespace {

inline float clamp01(float v) {
  return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}
inline float smoothstep(float e0, float e1, float x) {
  const float t = clamp01((x - e0) / (e1 - e0));
  return t * t * (3.0f - 2.0f * t);
}
inline PgVec3 mix(const PgVec3 &a, const PgVec3 &b, float t) {
  return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
}
inline PgVec3 mul(const PgVec3 &a, const PgVec3 &b) {
  return {a.x * b.x, a.y * b.y, a.z * b.z};
}
inline PgVec3 scale(const PgVec3 &a, float s) { return {a.x * s, a.y * s, a.z * s}; }

inline uint32_t hash2(int x, int y, uint32_t salt) {
  uint32_t h = static_cast<uint32_t>(x) * 0x9e3779b1u ^
               static_cast<uint32_t>(y) * 0x85ebca77u ^ salt * 0xc2b2ae3du;
  h ^= h >> 16;
  h *= 0x7feb352du;
  h ^= h >> 15;
  h *= 0x846ca68bu;
  h ^= h >> 16;
  return h;
}
inline float u01(uint32_t h) { return (h & 0xffffffu) / 16777216.0f; }

constexpr int kMaxCrops = 6;

// One row per palette id. Amounts are rough coverage fractions (0 = none).
struct CoverStyle {
  PgVec3 tintCool, tintWarm; // multipliers blended by the macro tint noise
  float patchAmt[2];         // coverage of patch A / B
  PgVec3 patchCol[2];        // absolute colours
  float patchBare[2];        // how much each patch suppresses grass (0..1)
  float pathAmt;             // 0 = no tracks, else number of tracks (0..1 weight)
  float pathHalfW;           // track half width [m]
  PgVec3 pathCol;
  float rockAmt;
  PgVec3 rockCol;
  float waterAmt;            // pond coverage (0 = none)
  PgVec3 waterCol, shoreCol;
  float fieldsAmt;           // 0 = none; else blend of parcel colours into ground
  float fieldW, fieldH;      // parcel size [m]
  int nCrops;
  PgVec3 crops[kMaxCrops];
  float stripeAmt;           // mown stripes (+-fraction of brightness)
  float fadeH;               // cover fades out above this h01 (>1 = never)
};

const CoverStyle &styleFor(int pal) {
  static const CoverStyle kStyles[] = {
      // 0 endless: the dark canopy-floor green, with lighter dry / loam patches
      {{0.86f, 0.98f, 1.05f}, {1.16f, 1.06f, 0.84f},
       {0.40f, 0.24f}, {{0.20f, 0.30f, 0.11f}, {0.30f, 0.26f, 0.14f}}, {0.2f, 0.7f},
       0.7f, 1.3f, {0.34f, 0.28f, 0.19f},
       0.35f, {0.36f, 0.35f, 0.33f},
       0.40f, {0.10f, 0.19f, 0.22f}, {0.14f, 0.15f, 0.10f},
       0.0f, 0, 0, 0, {},
       0.0f, 9.0f},
      // 1 desert: pale salt-flat patches, dark gravel, a faint track
      {{0.94f, 0.92f, 0.98f}, {1.10f, 1.02f, 0.88f},
       {0.40f, 0.30f}, {{0.80f, 0.75f, 0.62f}, {0.55f, 0.42f, 0.30f}}, {0.5f, 0.5f},
       0.5f, 1.4f, {0.52f, 0.40f, 0.28f},
       0.45f, {0.34f, 0.27f, 0.22f},
       0.0f, {}, {},
       0.0f, 0, 0, 0, {},
       0.0f, 9.0f},
      // 2 forest: moss, leaf litter, a trail, a dark pond, mossy rocks
      {{0.84f, 1.0f, 0.96f}, {1.16f, 1.02f, 0.82f},
       {0.46f, 0.36f}, {{0.20f, 0.32f, 0.10f}, {0.28f, 0.21f, 0.11f}}, {0.2f, 0.5f},
       0.8f, 1.2f, {0.36f, 0.28f, 0.18f},
       0.30f, {0.34f, 0.35f, 0.30f},
       0.38f, {0.09f, 0.16f, 0.15f}, {0.15f, 0.14f, 0.08f},
       0.0f, 0, 0, 0, {},
       0.0f, 9.0f},
      // 3 pasture (generic brighter grass)
      {{0.88f, 1.0f, 1.04f}, {1.14f, 1.05f, 0.86f},
       {0.40f, 0.24f}, {{0.38f, 0.52f, 0.18f}, {0.50f, 0.48f, 0.26f}}, {0.2f, 0.6f},
       0.7f, 1.3f, {0.52f, 0.44f, 0.30f},
       0.30f, {0.46f, 0.43f, 0.39f},
       0.40f, {0.14f, 0.27f, 0.30f}, {0.30f, 0.28f, 0.17f},
       0.0f, 0, 0, 0, {},
       0.0f, 9.0f},
      // 4 yard: asphalt / concrete / gravel zones, weedy and oil-stained patches
      {{0.94f, 0.97f, 1.02f}, {1.05f, 1.02f, 0.95f},
       {0.12f, 0.24f}, {{0.28f, 0.36f, 0.20f}, {0.24f, 0.24f, 0.25f}}, {0.7f, 0.2f},
       0.7f, 1.5f, {0.50f, 0.46f, 0.38f},
       0.0f, {},
       0.0f, {}, {},
       0.85f, 46.0f, 34.0f, 5,
       {{0.30f, 0.30f, 0.31f}, {0.58f, 0.57f, 0.54f}, {0.46f, 0.43f, 0.38f},
        {0.34f, 0.35f, 0.34f}, {0.52f, 0.50f, 0.45f}},
       0.0f, 9.0f},
      // 5 meadow (TerrainGen): wildflower yellow / clover, dry patches, a pond
      {{0.88f, 1.0f, 1.05f}, {1.14f, 1.05f, 0.84f},
       {0.52f, 0.42f}, {{0.56f, 0.54f, 0.22f}, {0.20f, 0.36f, 0.12f}}, {0.4f, 0.1f},
       0.7f, 1.4f, {0.50f, 0.42f, 0.28f},
       0.30f, {0.46f, 0.43f, 0.38f},
       0.42f, {0.13f, 0.26f, 0.30f}, {0.28f, 0.28f, 0.16f},
       0.0f, 0, 0, 0, {},
       0.0f, 0.6f},
      // 6 alpine: grassy pockets low down, scree and tarns, a trail
      {{0.90f, 0.98f, 1.06f}, {1.10f, 1.03f, 0.90f},
       {0.34f, 0.55f}, {{0.30f, 0.44f, 0.16f}, {0.42f, 0.40f, 0.36f}}, {0.2f, 0.9f},
       0.5f, 1.5f, {0.50f, 0.45f, 0.36f},
       0.50f, {0.36f, 0.35f, 0.34f},
       0.36f, {0.10f, 0.22f, 0.28f}, {0.34f, 0.33f, 0.30f},
       0.0f, 0, 0, 0, {},
       0.0f, 0.9f},
      // 7 hills: patchwork of fields, hedge tracks, ponds
      {{0.90f, 1.0f, 1.04f}, {1.12f, 1.04f, 0.88f},
       {0.30f, 0.20f}, {{0.24f, 0.42f, 0.14f}, {0.50f, 0.48f, 0.24f}}, {0.2f, 0.5f},
       0.8f, 1.4f, {0.50f, 0.42f, 0.28f},
       0.18f, {0.46f, 0.43f, 0.38f},
       0.34f, {0.14f, 0.27f, 0.31f}, {0.30f, 0.28f, 0.17f},
       0.70f, 70.0f, 95.0f, 6,
       {{0.28f, 0.48f, 0.14f}, {0.54f, 0.50f, 0.22f}, {0.40f, 0.54f, 0.20f},
        {0.34f, 0.26f, 0.17f}, {0.62f, 0.56f, 0.26f}, {0.22f, 0.38f, 0.14f}},
       0.0f, 9.0f},
      // 8 flat: crop fields with tracks between them, a few ponds
      {{0.92f, 1.0f, 1.03f}, {1.10f, 1.04f, 0.90f},
       {0.26f, 0.16f}, {{0.30f, 0.46f, 0.15f}, {0.50f, 0.48f, 0.24f}}, {0.2f, 0.4f},
       0.8f, 1.5f, {0.52f, 0.44f, 0.30f},
       0.10f, {0.46f, 0.43f, 0.39f},
       0.30f, {0.14f, 0.27f, 0.31f}, {0.30f, 0.28f, 0.17f},
       0.80f, 52.0f, 80.0f, 6,
       {{0.28f, 0.48f, 0.14f}, {0.58f, 0.52f, 0.22f}, {0.40f, 0.55f, 0.19f},
        {0.36f, 0.27f, 0.17f}, {0.66f, 0.58f, 0.28f}, {0.20f, 0.36f, 0.13f}},
       0.0f, 9.0f},
      // 9 racing: mown lawn stripes, a cinder track, damp hollows
      {{0.92f, 1.0f, 1.03f}, {1.08f, 1.04f, 0.92f},
       {0.42f, 0.26f}, {{0.30f, 0.46f, 0.15f}, {0.50f, 0.50f, 0.24f}}, {0.2f, 0.5f},
       0.8f, 1.6f, {0.46f, 0.38f, 0.26f},
       0.12f, {0.46f, 0.43f, 0.39f},
       0.26f, {0.14f, 0.27f, 0.31f}, {0.28f, 0.27f, 0.16f},
       0.0f, 0, 0, 0, {},
       0.11f, 9.0f},
  };
  constexpr int n = static_cast<int>(sizeof(kStyles) / sizeof(kStyles[0]));
  return kStyles[pal < 0 || pal >= n ? 0 : pal];
}

} // namespace

GroundCover::GroundCover(uint32_t seed, int palette)
    : pal_(palette), rot_(static_cast<float>((seed >> 3) % 628u) * 0.005f),
      tint_(seed ^ 0x1f123bb5u), patch_(seed ^ 0x2545f491u),
      fine_(seed ^ 0x6c8e9cf5u), path_(seed ^ 0x3c6ef372u),
      water_(seed ^ 0x510e527fu), rock_(seed ^ 0x9b05688cu) {}

GroundCover::Sample GroundCover::sample(float wx, float wy, const PgVec3 &base,
                                        float h01, float flatness) const {
  const CoverStyle &s = styleFor(pal_);
  // Cover fades out with altitude (alpine), and keeps off the spawn pad's
  // immediate surroundings for water/rock so the drone does not start in a pond.
  const float alt = 1.0f - smoothstep(s.fadeH * 0.75f, s.fadeH, h01);
  const float r = std::sqrt(wx * wx + wy * wy);
  const float spawn = smoothstep(16.0f, 28.0f, r);
  const float flat = smoothstep(0.90f, 0.985f, flatness);
  float grass = 1.0f;

  // Macro tint: a hundreds-of-metres warm/cool drift over the whole world.
  const float nt = 0.5f + 0.5f * tint_.fbm(wx / 260.0f, wy / 260.0f, 3, 2.0f, 0.5f) * 1.4f;
  PgVec3 col = mul(base, mix(s.tintCool, s.tintWarm, clamp01(nt)));
  col = scale(col, 0.93f + 0.14f * (0.5f + 0.5f * tint_.gradient(wx / 31.0f + 7.1f, wy / 31.0f)));

  // Fine breakup used to rag the edges of every patch.
  const float rag = fine_.fbm(wx / 7.0f, wy / 7.0f, 2, 2.0f, 0.5f);

  // Parcels of fields / paving zones on a rotated grid with wobbly edges.
  float edgeTrack = 0.0f;
  if (s.fieldsAmt > 0.0f && s.nCrops > 0) {
    const float c = std::cos(rot_), sn = std::sin(rot_);
    const float wob = 14.0f * patch_.fbm(wx / 90.0f + 3.0f, wy / 90.0f, 2, 2.0f, 0.5f);
    const float u = ((wx * c + wy * sn) + wob) / s.fieldW;
    const float v = ((-wx * sn + wy * c) + wob * 0.7f) / s.fieldH;
    const int iu = static_cast<int>(std::floor(u)), iv = static_cast<int>(std::floor(v));
    const float fu = u - static_cast<float>(iu), fv = v - static_cast<float>(iv);
    const uint32_t hc = hash2(iu, iv, 77u);
    PgVec3 crop = s.crops[hc % static_cast<uint32_t>(s.nCrops)];
    // Per-parcel brightness so neighbours of one crop still differ.
    crop = scale(crop, 0.9f + 0.2f * u01(hc >> 8));
    col = mix(col, crop, s.fieldsAmt * alt);
    // A track runs along some parcel borders (the left/bottom border of a parcel).
    const float du = std::min(fu, 1.0f - fu) * s.fieldW;
    const float dv = std::min(fv, 1.0f - fv) * s.fieldH;
    const uint32_t hl = hash2(iu, iv, 91u), hr = hash2(iu + 1, iv, 91u);
    const uint32_t hb = hash2(iu, iv, 92u), ht = hash2(iu, iv + 1, 92u);
    const float dl = (fu < 0.5f ? (hl % 3u == 0u) : (hr % 3u == 0u)) ? du : 1e3f;
    const float db = (fv < 0.5f ? (hb % 3u == 0u) : (ht % 3u == 0u)) ? dv : 1e3f;
    edgeTrack = 1.0f - smoothstep(0.9f, 2.0f, std::min(dl, db));
  }

  // Patches A and B (dry / lush / loam ...), ragged edges, fading with height.
  for (int i = 0; i < 2; ++i) {
    if (s.patchAmt[i] <= 0.0f)
      continue;
    const float nn = patch_.fbm(wx / (i ? 38.0f : 62.0f) + 11.0f * i, wy / (i ? 38.0f : 62.0f), 3, 2.0f, 0.5f);
    const float thr = 0.50f - 0.55f * s.patchAmt[i];
    const float m = smoothstep(thr - 0.05f, thr + 0.07f, nn + 0.22f * rag) * alt;
    const PgVec3 pc = scale(s.patchCol[i], 0.92f + 0.16f * (0.5f + 0.5f * rag));
    col = mix(col, pc, m * 0.75f);
    grass *= 1.0f - s.patchBare[i] * m;
  }

  // Scattered rock: clusters (region mask) of small blobs (high-frequency mask).
  if (s.rockAmt > 0.0f) {
    const float region = smoothstep(0.05f, 0.30f, rock_.fbm(wx / 120.0f, wy / 120.0f, 2, 2.0f, 0.5f) + (s.rockAmt - 0.3f));
    const float blob = smoothstep(0.30f, 0.46f, rock_.fbm(wx / 6.5f + 40.0f, wy / 6.5f, 2, 2.0f, 0.5f));
    const float m = region * blob * spawn * alt;
    const float tone = 0.8f + 0.4f * (0.5f + 0.5f * fine_.gradient(wx / 2.3f, wy / 2.3f));
    col = mix(col, scale(s.rockCol, tone), m);
    grass *= 1.0f - m;
  }

  // Tracks: zero-crossings of a meandering noise field, width from the field's
  // gradient so they stay a constant ~2 m wide.
  if (s.pathAmt > 0.0f || edgeTrack > 0.0f) {
    float pm = edgeTrack;
    if (s.pathAmt > 0.0f) {
      const float k = 1.0f / 170.0f;
      const float e = 2.0f;
      auto pn = [&](float x, float y) { return path_.fbm(x * k, y * k, 2, 2.0f, 0.5f); };
      const float n0 = pn(wx, wy);
      const float gx = (pn(wx + e, wy) - n0) / e, gy = (pn(wx, wy + e) - n0) / e;
      const float d = std::fabs(n0) / (std::sqrt(gx * gx + gy * gy) + 1e-5f);
      pm = std::max(pm, (1.0f - smoothstep(s.pathHalfW * 0.7f, s.pathHalfW * 1.6f, d)) * s.pathAmt);
    }
    pm *= smoothstep(0.82f, 0.93f, flatness) * alt;
    const PgVec3 pc = scale(s.pathCol, 0.9f + 0.2f * (0.5f + 0.5f * rag));
    col = mix(col, pc, pm);
    grass *= 1.0f - pm;
  }

  // Ponds: flat low-lying colour only (height is untouched, so no physics).
  if (s.waterAmt > 0.0f) {
    const float nn = water_.fbm(wx / 105.0f, wy / 105.0f, 3, 2.0f, 0.5f);
    const float thr = 0.52f - 0.95f * s.waterAmt;
    const float inside = smoothstep(thr, thr + 0.05f, nn + 0.10f * rag);
    const float shore = smoothstep(thr - 0.06f, thr, nn + 0.10f * rag);
    const float gate = flat * spawn * alt;
    col = mix(col, s.shoreCol, shore * gate * 0.85f);
    // Deeper toward the middle, with a faint sky sheen.
    const float deep = smoothstep(thr, thr + 0.22f, nn);
    const PgVec3 wc = mix(scale(s.waterCol, 1.35f), s.waterCol, deep);
    col = mix(col, scale(wc, 0.94f + 0.12f * (0.5f + 0.5f * fine_.gradient(wx / 5.0f, wy / 9.0f))), inside * gate);
    grass *= 1.0f - std::max(inside, shore * 0.8f) * gate;
  }

  // Mown stripes (racing lawn).
  if (s.stripeAmt > 0.0f) {
    const float c = std::cos(rot_), sn = std::sin(rot_);
    const float u = (wx * c + wy * sn) / 7.0f;
    const float band = smoothstep(0.42f, 0.58f, u - std::floor(u));
    col = scale(col, 1.0f + s.stripeAmt * (band * 2.0f - 1.0f));
  }

  col.x = clamp01(col.x);
  col.y = clamp01(col.y);
  col.z = clamp01(col.z);
  return {col, clamp01(grass)};
}

} // namespace vsim::procgen
