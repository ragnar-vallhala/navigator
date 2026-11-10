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
#include "WorldPresets.h"

#include <algorithm>
#include <cmath>

namespace vsim::procgen {
namespace {

constexpr float kPi = 3.14159265358979f;

// Small deterministic PRNG (splitmix32-style); no global state.
struct Rng {
  uint32_t s;
  Rng(uint32_t seed, uint32_t salt) : s(seed * 2654435761u ^ salt) {
    next();
    next();
  }
  uint32_t next() {
    s += 0x9e3779b9u;
    uint32_t z = s;
    z = (z ^ (z >> 16)) * 0x85ebca6bu;
    z = (z ^ (z >> 13)) * 0xc2b2ae35u;
    return z ^ (z >> 16);
  }
  float uni() { return static_cast<float>(next() >> 8) / 16777216.0f; }
  float range(float a, float b) { return a + (b - a) * uni(); }
};

// Terrain context handed to the layout generators.
struct Ground {
  const TerrainField &f;
  float sizeM;
  int res;
  float at(float x, float y) const {
    return arenaSurfaceHeight(f, sizeM, res, x, y);
  }
  // Lowest and highest surface under a round footprint.
  void span(float x, float y, float r, float &lo, float &hi) const {
    lo = hi = at(x, y);
    if (r < 0.6f) { // slim posts/trunks: one mesh sample, sunk a little
      lo -= 0.15f;
      return;
    }
    const float d[4][2] = {{r, 0}, {-r, 0}, {0, r}, {0, -r}};
    for (const auto &o : d) {
      const float g = at(x + o[0], y + o[1]);
      lo = std::min(lo, g);
      hi = std::max(hi, g);
    }
  }
};

// Seat an upright box/cylinder of nominal height h on the surface under its
// footprint: the base sits on the lowest point (no floating gap on a slope) and
// the height grows by the relief so the top stays ~h above the high side.
void seatUpright(const Ground &g, PgObstacle &o, float footprintR, float h) {
  float lo, hi;
  g.span(o.x, o.y, footprintR, lo, hi);
  const float hh = h + (hi - lo);
  o.zc = -(lo + hh * 0.5f);
  o.sz = hh;
}

bool clearOfSpawn(float x, float y, float footprintR) {
  return std::sqrt(x * x + y * y) >= kSpawnClearM + footprintR;
}

void layoutRacing(const Ground &g, uint32_t seed, PresetData &d) {
  Rng rng(seed, 0x52414345u);
  const int n = 8 + static_cast<int>(rng.next() % 3u); // 8..10 gates
  const float R = rng.range(48.0f, 62.0f);
  const float p1 = rng.range(0, 2 * kPi), p2 = rng.range(0, 2 * kPi);
  const float a2 = rng.range(0.04f, 0.10f), a3 = rng.range(0.02f, 0.06f);
  auto path = [&](float th, float &x, float &y) {
    const float r =
        R * (1.0f + a2 * std::sin(2 * th + p1) + a3 * std::sin(3 * th + p2));
    x = r * std::cos(th);
    y = r * std::sin(th);
  };
  // Equal arc-length resampling of the closed path.
  const int M = 4096;
  std::vector<float> cum(M + 1, 0.0f);
  float px, py;
  path(0, px, py);
  for (int i = 1; i <= M; ++i) {
    float x, y;
    path(2 * kPi * static_cast<float>(i) / M, x, y);
    cum[i] = cum[i - 1] + std::hypot(x - px, y - py);
    px = x;
    py = y;
  }
  const float total = cum[M];
  const float th0 = rng.range(0, 2 * kPi);
  auto thetaAt = [&](float s) { // arc length -> angle
    const int i = static_cast<int>(
                      std::upper_bound(cum.begin(), cum.end(), s) -
                      cum.begin()) -
                  1;
    const int j = std::min(std::max(i, 0), M - 1);
    const float t = (s - cum[j]) / std::max(1e-6f, cum[j + 1] - cum[j]);
    return 2 * kPi * (static_cast<float>(j) + t) / M;
  };

  const float width = 3.0f, postR = 0.1f, clearance = 3.0f;
  d.propDiameterM = kMaxPropDiameterM;
  for (int i = 0; i < n; ++i) {
    const float th =
        th0 + thetaAt(std::fmod(total * static_cast<float>(i) / n, total));
    float x, y, xa, ya;
    path(th, x, y);
    path(th + 0.01f, xa, ya);
    const float yaw = std::atan2(ya - y, xa - x); // travel heading
    const float tx = std::cos(yaw), ty = std::sin(yaw);
    const float nx = -ty, ny = tx;
    const float off = width * 0.5f + postR;
    const float px2[2] = {x + nx * off, x - nx * off};
    const float py2[2] = {y + ny * off, y - ny * off};
    const float gp[2] = {g.at(px2[0], py2[0]), g.at(px2[1], py2[1])};
    const float gc = g.at(x, y);
    const float top = std::max(std::max(gp[0], gp[1]), gc) + clearance;
    for (int side = 0; side < 2; ++side) {
      PgObstacle post;
      post.type = 2;
      post.x = px2[side];
      post.y = py2[side];
      post.sx = postR;
      post.sz = top - gp[side];
      post.zc = -(gp[side] + post.sz * 0.5f);
      d.obstacles.push_back(post);
    }
    PgObstacle bar;
    bar.type = 0;
    bar.x = x;
    bar.y = y;
    bar.sx = 0.12f;
    bar.sy = width + 2 * postR + 0.1f;
    bar.sz = 0.2f;
    bar.zc = -(top + 0.1f);
    bar.style = kPropGate;
    bar.yawDeg = yaw * 180.0f / kPi;
    d.obstacles.push_back(bar);
    RaceGate rg;
    rg.x = x;
    rg.y = y;
    rg.yawDeg = bar.yawDeg;
    rg.widthM = width;
    rg.clearanceM = top - gc;
    d.gates.push_back(rg);

    // Turn-marker pylon between this gate and the next, on the outside.
    const float ths =
        th0 + thetaAt(std::fmod(total * (static_cast<float>(i) + 0.5f) / n,
                                total));
    float mx, my;
    path(ths, mx, my);
    const float len = std::hypot(mx, my);
    PgObstacle pyl;
    pyl.type = 2;
    pyl.x = mx + mx / len * 8.0f;
    pyl.y = my + my / len * 8.0f;
    pyl.sx = 0.35f;
    pyl.style = kPropGate;
    seatUpright(g, pyl, 0.35f, 4.0f);
    d.obstacles.push_back(pyl);
  }
}

struct Placed {
  float x, y, r;
};

void layoutYard(const Ground &g, uint32_t seed, PresetData &d) {
  Rng rng(seed, 0x59415244u);
  std::vector<Placed> placed;
  const float lim = d.sizeM * 0.5f - 12.0f;
  auto tryPlace = [&](PgObstacle o, float fr, float h, float gap) {
    for (int attempt = 0; attempt < 60; ++attempt) {
      o.x = rng.range(-lim, lim);
      o.y = rng.range(-lim, lim);
      if (!clearOfSpawn(o.x, o.y, fr))
        continue;
      bool ok = true;
      for (const Placed &p : placed)
        if (std::hypot(o.x - p.x, o.y - p.y) < p.r + fr + gap) {
          ok = false;
          break;
        }
      if (!ok)
        continue;
      seatUpright(g, o, fr, h);
      d.obstacles.push_back(o);
      placed.push_back({o.x, o.y, fr});
      return;
    }
  };
  const int warehouses = 2 + static_cast<int>(rng.next() % 2u);
  for (int i = 0; i < warehouses; ++i) {
    PgObstacle o;
    o.type = 0;
    o.sx = rng.range(10.0f, 16.0f);
    o.sy = rng.range(6.0f, 9.0f);
    o.yawDeg = 90.0f * static_cast<float>(rng.next() % 2u);
    o.style = kPropBuilding;
    tryPlace(o, 0.5f * std::hypot(o.sx, o.sy), rng.range(4.5f, 6.5f), 6.0f);
  }
  const int crates = 45 + static_cast<int>(rng.next() % 12u);
  for (int i = 0; i < crates; ++i) {
    PgObstacle o;
    if (rng.uni() < 0.35f) { // tank / silo
      o.type = 2;
      o.sx = rng.range(0.8f, 2.2f);
      o.style = kPropSilo;
      tryPlace(o, o.sx, rng.range(3.0f, 7.0f), 3.0f);
    } else { // shipping-container-ish box
      o.type = 0;
      o.sx = rng.range(2.0f, 6.0f);
      o.sy = rng.range(2.0f, 3.2f);
      o.yawDeg = 15.0f * static_cast<float>(rng.next() % 12u);
      o.style = kPropContainer;
      tryPlace(o, 0.5f * std::hypot(o.sx, o.sy), rng.range(1.6f, 3.2f), 3.0f);
    }
  }
}

void layoutForest(const Ground &g, uint32_t seed, PresetData &d) {
  Rng rng(seed, 0x464f5253u);
  std::vector<Placed> placed;
  const float lim = d.sizeM * 0.5f - 6.0f;
  const int target = 240;
  for (int attempt = 0;
       attempt < 20000 && static_cast<int>(placed.size()) < target; ++attempt) {
    const float x = rng.range(-lim, lim), y = rng.range(-lim, lim);
    const float crown = rng.range(1.6f, 3.0f);
    const float trunkR = rng.range(0.18f, 0.32f);
    const float trunkH = rng.range(4.0f, 7.0f);
    if (!clearOfSpawn(x, y, crown + 2.0f))
      continue;
    bool ok = true;
    for (const Placed &p : placed)
      if (std::hypot(x - p.x, y - p.y) < p.r + crown + 1.5f) {
        ok = false;
        break;
      }
    if (!ok)
      continue;
    PgObstacle trunk;
    trunk.type = 2;
    trunk.x = x;
    trunk.y = y;
    trunk.sx = trunkR;
    trunk.style = kPropTrunk;
    seatUpright(g, trunk, trunkR, trunkH);
    d.obstacles.push_back(trunk);
    // Crown: a sphere resting on the trunk top.
    PgObstacle top;
    top.type = 1;
    top.x = x;
    top.y = y;
    top.sx = crown;
    top.style = kPropCanopy;
    top.zc = trunk.zc - trunk.sz * 0.5f - crown * 0.55f;
    d.obstacles.push_back(top);
    placed.push_back({x, y, crown});
  }
}

// Open terrain (hills/alpine/flat/desert) has nothing of known size in it, so a
// pilot cannot judge height or speed. Plant slim survey poles on rings around
// the spawn (collidable obstacles like any other). The renderer paints thin
// poles in 1 m red/white bands, so counting bands reads height directly.
void layoutMarkers(const Ground &g, uint32_t seed, PresetData &d) {
  Rng rng(seed, 0x4d41524bu);
  const float rings[3] = {24.0f, 48.0f, 90.0f};
  const int counts[3] = {4, 6, 6};
  const float lim = d.sizeM * 0.5f - 6.0f;
  for (int r = 0; r < 3; ++r) {
    const float a0 = rng.range(0.0f, 2.0f * kPi);
    for (int i = 0; i < counts[r]; ++i) {
      const float a = a0 + 2.0f * kPi * static_cast<float>(i) /
                               static_cast<float>(counts[r]) +
                      rng.range(-0.2f, 0.2f);
      const float rad = rings[r] * rng.range(0.9f, 1.1f);
      PgObstacle o;
      o.type = 2;
      o.x = rad * std::cos(a);
      o.y = rad * std::sin(a);
      o.sx = kMarkerPoleR;
      if (std::fabs(o.x) > lim || std::fabs(o.y) > lim ||
          !clearOfSpawn(o.x, o.y, o.sx))
        continue;
      seatUpright(g, o, o.sx, kMarkerPoleH);
      d.obstacles.push_back(o);
    }
  }
}

PresetData baseArena(const std::string &biome, uint32_t seed, float sizeM) {
  PresetData d;
  d.biome = biome;
  d.field.seed = seed;
  d.flora.seed = seed;
  d.sizeM = sizeM;
  // ~1.5 m grid cells; even so the 2x2 chunk split is exact.
  d.resolution = static_cast<int>(sizeM / 1.5f) / 2 * 2;
  d.field.octaves = 5;
  d.field.flatRadiusM = 9.0f;
  return d;
}

void noSnowNoRock(FieldParams &f) {
  f.colBrownT = 1.5f;
  f.colRockT = 1.8f;
  f.colSnowT = 2.0f;
}

} // namespace

const std::vector<PresetInfo> &worldPresets() {
  static const std::vector<PresetInfo> v = {
      {"meadow", "Meadow"},
      {"hills", "Rolling hills"},
      {"alpine", "Alpine / rocky"},
      {"flat", "Flat field"},
      {"desert", "Desert dunes"},
      {"forest", "Forest"},
      {"racing", "Racing course"},
      {"yard", "Obstacle yard"},
  };
  return v;
}

bool isFieldBiome(const std::string &b) {
  return b == "hills" || b == "alpine" || b == "flat" || b == "desert" ||
         b == "forest" || b == "racing" || b == "yard";
}

bool buildWorldPreset(const std::string &id, uint32_t seed, PresetData &out) {
  if (id == "meadow") {
    out = PresetData{};
    out.biome = "meadow";
    out.field.seed = seed;
    out.flora.seed = seed;
    return true;
  }
  if (!isFieldBiome(id))
    return false;
  PresetData d = baseArena(id, seed, 400.0f);
  FieldParams &f = d.field;
  if (id == "hills") {
    f.heightM = 55.0f;
    f.featureM = 130.0f;
    f.mountainMix = 0.35f;
    f.mountainBias = 0.6f;
    f.maxSlope = 0.5f;
    f.palette = kPalHills;
    noSnowNoRock(f);
    f.colSlopeT = 0.9f;
  } else if (id == "alpine") {
    f.heightM = 110.0f;
    f.featureM = 200.0f;
    f.mountainMix = 0.85f;
    f.mountainBias = 0.15f;
    f.maxSlope = 0.9f;
    f.palette = kPalAlpine;
    f.colBrownT = 0.28f;
    f.colRockT = 0.55f;
    f.colSnowT = 0.58f;
    f.colSlopeT = 0.45f;
    f.flatRadiusM = 12.0f;
  } else if (id == "flat") {
    d.sizeM = 300.0f;
    d.resolution = 200;
    f.heightM = 1.2f;
    f.mountainMix = 0.0f;
    f.maxSlope = 0.3f;
    f.palette = kPalFlat;
    noSnowNoRock(f);
  } else if (id == "desert") {
    f.shape = 1;
    f.heightM = 16.0f;
    f.featureM = 75.0f; // dune wavelength
    f.maxSlope = 0.5f;
    f.palette = kPalDesert;
    noSnowNoRock(f);
    f.colBrownT = 0.35f;
    f.colSlopeT = 0.9f;
  } else if (id == "forest") {
    d.sizeM = 300.0f;
    d.resolution = 200;
    f.heightM = 16.0f;
    f.featureM = 150.0f;
    f.mountainMix = 0.1f;
    f.mountainBias = 0.5f;
    f.maxSlope = 0.5f;
    f.palette = kPalForest;
    noSnowNoRock(f);
    f.colSlopeT = 0.8f;
  } else if (id == "racing") {
    d.sizeM = 300.0f;
    d.resolution = 200;
    f.heightM = 5.0f;
    f.mountainMix = 0.0f;
    f.maxSlope = 0.25f;
    f.palette = kPalRacing;
    noSnowNoRock(f);
  } else { // yard
    d.sizeM = 240.0f;
    d.resolution = 160;
    f.heightM = 0.6f;
    f.mountainMix = 0.0f;
    f.maxSlope = 0.2f;
    f.palette = kPalYard;
    noSnowNoRock(f);
  }
  const TerrainField field(f);
  const Ground g{field, d.sizeM, d.resolution};
  if (id == "racing")
    layoutRacing(g, seed, d);
  else if (id == "yard")
    layoutYard(g, seed, d);
  else if (id == "forest")
    layoutForest(g, seed, d);
  else
    layoutMarkers(g, seed, d); // hills / alpine / flat / desert
  out = std::move(d);
  return true;
}

ProcMesh meshArena(const TerrainField &f, float sizeM, int resolution) {
  ProcMesh all;
  const float cm = arenaChunkM(sizeM);
  const int cr = arenaChunkRes(resolution);
  for (int cy = -1; cy <= 0; ++cy)
    for (int cx = -1; cx <= 0; ++cx) {
      ProcMesh m = meshFieldChunk(f, cx, cy, cm, cr);
      all.positions.insert(all.positions.end(), m.positions.begin(),
                           m.positions.end());
      all.normals.insert(all.normals.end(), m.normals.begin(), m.normals.end());
      all.colors.insert(all.colors.end(), m.colors.begin(), m.colors.end());
    }
  return all;
}

float arenaSurfaceHeight(const TerrainField &f, float sizeM, int resolution,
                         float wx, float wy) {
  return surfaceHeight(f, wx, wy, arenaChunkM(sizeM),
                       arenaChunkRes(resolution));
}

const std::vector<PresetInfo> &windPresets() {
  static const std::vector<PresetInfo> v = {
      {"off", "No wind"},
      {"calm", "Calm"},
      {"breezy", "Breezy"},
      {"gusty", "Gusty"},
  };
  return v;
}

bool buildWindPreset(const std::string &id, uint32_t seed,
                     WindPresetData &out) {
  WindPresetData w;
  float speed = 0.0f;
  if (id == "off") {
    out = w;
    return true;
  } else if (id == "calm") {
    speed = 1.0f;
    w.turbSigma = 0.2f;
    w.turbTau = 2.0f;
  } else if (id == "breezy") {
    speed = 4.0f;
    w.gustAmp = 1.5f;
    w.gustPeriod = 8.0f;
    w.turbSigma = 0.6f;
    w.turbTau = 1.5f;
  } else if (id == "gusty") {
    speed = 6.0f;
    w.gustAmp = 5.0f;
    w.gustPeriod = 5.0f;
    w.turbSigma = 1.5f;
    w.turbTau = 1.0f;
  } else {
    return false;
  }
  Rng rng(seed, 0x57494e44u);
  const float h = rng.range(0.0f, 2.0f * kPi); // heading wind blows toward
  w.steadyN = speed * std::cos(h);
  w.steadyE = speed * std::sin(h);
  w.enabled = true;
  out = w;
  return true;
}

} // namespace vsim::procgen
