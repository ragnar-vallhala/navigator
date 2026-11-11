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
// PropStyle.h — what a static obstacle is meant to look like (Qt-free).
//
// An obstacle is just a box / sphere / cylinder to the physics; the preset
// layouts tag each one with a PropStyle so the renderer can dress it as a tree
// trunk, a leafy or conifer crown, a warehouse, a shipping container, a silo or
// a course marker instead of one flat colour per shape. propColor() is a pure
// function of (style, shape, x, y): the per-prop variation (leaf green, container
// colour) is hashed from the position, so a world looks the same every run.
// All albedos are mid-tone (0.15 .. 0.9): nothing is near-black, so lighting
// never has to rescue it.
#pragma once

#include "ProcgenTypes.h"

#include <cmath>
#include <cstdint>

namespace vsim::procgen {

enum PropStyle {
  kPropDefault = 0, // by shape: concrete box, terracotta ball, steel post
  kPropTrunk = 1,
  kPropCanopy = 2, // sphere resting on a trunk (leafy or conifer crown)
  kPropBuilding = 3,
  kPropContainer = 4,
  kPropSilo = 5,
  kPropGate = 6, // race gate bar / turn pylon
};

// Stable [0,1) hash of a position + salt.
inline float propRand(float x, float y, uint32_t salt) {
  uint32_t h = static_cast<uint32_t>(static_cast<int32_t>(std::lround(x * 8.0f))) * 0x9e3779b1u;
  h ^= static_cast<uint32_t>(static_cast<int32_t>(std::lround(y * 8.0f))) * 0x85ebca77u;
  h ^= salt * 0xc2b2ae3du;
  h ^= h >> 16;
  h *= 0x7feb352du;
  h ^= h >> 15;
  h *= 0x846ca68bu;
  h ^= h >> 16;
  return static_cast<float>(h & 0xffffffu) / 16777216.0f;
}

// A tree at (x, y) is a conifer ~1 in 3 times.
inline bool propIsConifer(float x, float y) { return propRand(x, y, 5u) < 0.34f; }

// `shape`: 0 box, 1 sphere, 2 cylinder. `part` lets a crown ask for lighter /
// darker variants of its green (0 = base).
inline PgVec3 propColor(int style, int shape, float x, float y) {
  const float a = propRand(x, y, 1u), b = propRand(x, y, 2u);
  auto lerp = [](float p, float q, float t) { return p + (q - p) * t; };
  switch (style) {
  case kPropTrunk:
    return {lerp(0.30f, 0.42f, a), lerp(0.21f, 0.29f, a), lerp(0.13f, 0.18f, b)};
  case kPropCanopy:
    if (propIsConifer(x, y)) // blue-green firs
      return {lerp(0.10f, 0.17f, a), lerp(0.27f, 0.36f, b), lerp(0.17f, 0.23f, a)};
    if (b > 0.9f) // the odd autumn tree
      return {lerp(0.62f, 0.78f, a), lerp(0.40f, 0.50f, a), 0.12f};
    return {lerp(0.16f, 0.32f, a), lerp(0.34f, 0.50f, b), lerp(0.10f, 0.17f, a)};
  case kPropBuilding: { // corrugated warehouse sheeting
    static const float t[4][3] = {{0.72f, 0.74f, 0.76f}, {0.58f, 0.68f, 0.78f},
                                  {0.80f, 0.76f, 0.64f}, {0.66f, 0.30f, 0.26f}};
    const int i = static_cast<int>(a * 4.0f) & 3;
    return {t[i][0], t[i][1], t[i][2]};
  }
  case kPropContainer: { // shipping-container colours
    static const float t[6][3] = {{0.64f, 0.24f, 0.17f}, {0.18f, 0.36f, 0.58f},
                                  {0.20f, 0.46f, 0.32f}, {0.86f, 0.66f, 0.16f},
                                  {0.82f, 0.42f, 0.14f}, {0.80f, 0.81f, 0.80f}};
    const int i = static_cast<int>(a * 6.0f) % 6;
    return {t[i][0], t[i][1], t[i][2]};
  }
  case kPropSilo:
    return {lerp(0.76f, 0.88f, a), lerp(0.78f, 0.88f, a), lerp(0.78f, 0.86f, a)};
  case kPropGate:
    return {0.95f, 0.42f, 0.07f};
  default:
    break;
  }
  if (shape == 1) // plain ball: terracotta
    return {0.72f, 0.46f, 0.30f};
  if (shape == 2) // plain post: steel
    return {0.58f, 0.62f, 0.65f};
  return {0.64f, 0.62f, 0.58f}; // plain box: concrete
}

} // namespace vsim::procgen
