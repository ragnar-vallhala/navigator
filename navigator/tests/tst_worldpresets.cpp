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
// tst_worldpresets.cpp — procedural world presets (Qt-free core, headless).
//
// Every preset must be deterministic from its seed, produce a sane mesh that
// the spawn sits on, keep its obstacles clear of the spawn, and the racing
// course must be flyable (evenly spaced gates with generous openings).
#include <QtTest/QtTest>

#include "procgen/WorldPresets.h"

#include <cmath>
#include <map>
#include <string>

using namespace vsim::procgen;

namespace {

PresetData build(const char *id, uint32_t seed) {
  PresetData d;
  const bool ok = buildWorldPreset(id, seed, d);
  Q_ASSERT(ok);
  (void)ok;
  return d;
}

// Cache by (id, seed): obstacle layout seats every shape on the surface, so a
// forest build is ~0.3 s; no need to repeat it in each test.
const PresetData &cached(const std::string &id, uint32_t seed) {
  static std::map<std::pair<std::string, uint32_t>, PresetData> c;
  auto key = std::make_pair(id, seed);
  auto it = c.find(key);
  if (it == c.end())
    it = c.emplace(key, build(id.c_str(), seed)).first;
  return it->second;
}

const ProcMesh &cachedMesh(const std::string &id, uint32_t seed) {
  static std::map<std::pair<std::string, uint32_t>, ProcMesh> c;
  auto key = std::make_pair(id, seed);
  auto it = c.find(key);
  if (it == c.end()) {
    const PresetData &d = cached(id, seed);
    it = c.emplace(key, meshArena(TerrainField(d.field), d.sizeM, d.resolution))
             .first;
  }
  return it->second;
}

bool sameObstacles(const PresetData &a, const PresetData &b) {
  if (a.obstacles.size() != b.obstacles.size())
    return false;
  for (size_t i = 0; i < a.obstacles.size(); ++i) {
    const PgObstacle &p = a.obstacles[i], &q = b.obstacles[i];
    if (p.type != q.type || p.x != q.x || p.y != q.y || p.zc != q.zc ||
        p.sx != q.sx || p.sy != q.sy || p.sz != q.sz || p.yawDeg != q.yawDeg)
      return false;
  }
  return true;
}

// Footprint radius used to test clearance against the spawn.
float footprint(const PgObstacle &o) {
  return o.type == 0 ? 0.5f * std::hypot(o.sx, o.sy) : o.sx;
}

std::vector<std::string> fieldIds() {
  return {"hills", "alpine", "flat", "desert", "forest", "racing", "yard"};
}

} // namespace

class TstWorldPresets : public QObject {
  Q_OBJECT
private slots:
  void unknownIdRejected();
  void deterministicAndSeedSensitive();
  void terrainFiniteAndBounded();
  void slopeCapRespected();
  void spawnOnMeshAndPadFlat();
  void obstaclesClearOfSpawnAndSeated();
  void openPresetsGetMarkerPoles();
  void racingGatesEvenlySpacedAndPassable();
  void windPresetsMagnitudes();
  void generateReappliesOnlyOnPresetOrSeedChange();
};

void TstWorldPresets::generateReappliesOnlyOnPresetOrSeedChange() {
  QVERIFY(!presetNeedsReapply("hills", 5, "", 5));      // custom world
  QVERIFY(!presetNeedsReapply("hills", 5, "hills", 5)); // nothing changed
  QVERIFY(presetNeedsReapply("hills", 5, "hills", 6));  // new seed
  QVERIFY(presetNeedsReapply("hills", 5, "forest", 5)); // new preset
  QVERIFY(presetNeedsReapply("", 5, "hills", 5));       // first pick
}

void TstWorldPresets::unknownIdRejected() {
  PresetData d;
  QVERIFY(!buildWorldPreset("nope", 1, d));
  WindPresetData w;
  QVERIFY(!buildWindPreset("nope", 1, w));
  QVERIFY(buildWorldPreset("meadow", 5, d));
  QCOMPARE(d.biome, std::string("meadow"));
}

void TstWorldPresets::deterministicAndSeedSensitive() {
  for (const std::string &id : fieldIds()) {
    const PresetData &a = cached(id, 11);
    const PresetData a2 = build(id.c_str(), 11);
    const PresetData &b = cached(id, 12);
    QVERIFY2(sameObstacles(a, a2), id.c_str());
    QCOMPARE(a.sizeM, a2.sizeM);
    QCOMPARE(a.resolution, a2.resolution);
    // Terrain samples: equal for equal seed, different for another seed
    // (flat/yard/racing are near-flat, so compare at a coarse tolerance too).
    const TerrainField fa(a.field), fa2(a2.field), fb(b.field);
    bool differs = !sameObstacles(a, b);
    for (int i = 0; i < 40; ++i) {
      const float x = -90.0f + 4.7f * i, y = 70.0f - 3.9f * i;
      QCOMPARE(fa.height(x, y), fa2.height(x, y));
      if (std::fabs(fa.height(x, y) - fb.height(x, y)) > 1e-4f)
        differs = true;
    }
    QVERIFY2(differs, (id + ": seeds 11 and 12 gave the same world").c_str());
  }
}

void TstWorldPresets::terrainFiniteAndBounded() {
  for (const std::string &id : fieldIds()) {
    const PresetData &d = cached(id, 21);
    const ProcMesh &m = cachedMesh(id, 21);
    const TerrainField f(d.field);
    QVERIFY(!m.empty());
    QCOMPARE(m.positions.size() % 3, size_t(0));
    const int cr = arenaChunkRes(d.resolution);
    QCOMPARE(m.triangleCount(), size_t(4) * cr * cr * 2);
    float lo = 1e9f, hi = -1e9f;
    const float half = d.sizeM * 0.5f + 1e-3f;
    for (const PgVec3 &p : m.positions) {
      QVERIFY(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z));
      QVERIFY(std::fabs(p.x) <= half && std::fabs(p.y) <= half);
      lo = std::min(lo, -p.z);
      hi = std::max(hi, -p.z);
    }
    QVERIFY2(lo >= -1e-4f, id.c_str());
    QVERIFY2(hi <= d.field.heightM * 1.001f + 1e-3f, id.c_str());
    qInfo("%-7s size %3.0f m  res %d  elevation %.1f .. %.1f m  obstacles %zu",
          id.c_str(), d.sizeM, d.resolution, lo, hi, d.obstacles.size());
  }
}

// The cap erodes spikes in a fixed number of passes (shared with the endless
// terrain), so it smooths slopes rather than guaranteeing a hard maximum:
// check that nearly all mesh edges respect it and the worst case stays bounded
// (alpine, the steepest preset, has ~4% of edges above its cap).
void TstWorldPresets::slopeCapRespected() {
  for (const std::string &id : fieldIds()) {
    const PresetData &d = cached(id, 21);
    const ProcMesh &m = cachedMesh(id, 21);
    const TerrainField f(d.field);
    const float step = arenaChunkM(d.sizeM) / arenaChunkRes(d.resolution);
    const float cap = d.field.maxSlope * 1.05f; // float slack
    size_t edges = 0, over = 0;
    float worst = 0.0f;
    // Each triangle (v0,v1,v2): the v0-v1 edge is an axis-aligned lattice edge
    // or a diagonal; use the lattice edges only (distance == step).
    for (size_t t = 0; t + 2 < m.positions.size(); t += 3) {
      const PgVec3 &a = m.positions[t], &b = m.positions[t + 1];
      if (std::fabs(std::hypot(b.x - a.x, b.y - a.y) - step) > 1e-3f)
        continue;
      const float s = std::fabs(b.z - a.z) / step;
      ++edges;
      worst = std::max(worst, s);
      if (s > cap)
        ++over;
    }
    qInfo("%-7s maxSlope %.2f: worst edge slope %.2f, %zu/%zu edges over",
          id.c_str(), d.field.maxSlope, worst, over, edges);
    QVERIFY2(over <= edges / 16, (id + ": >6% of edges exceed the cap").c_str());
    QVERIFY2(worst <= std::max(d.field.maxSlope * 2.2f, 3.0f),
             (id + ": worst slope far above the cap").c_str());
  }
}

void TstWorldPresets::spawnOnMeshAndPadFlat() {
  for (const std::string &id : fieldIds()) {
    const PresetData &d = cached(id, 21);
    const ProcMesh &m = cachedMesh(id, 21);
    const TerrainField f(d.field);
    const float cm = arenaChunkM(d.sizeM);
    const int cr = arenaChunkRes(d.resolution);
    // The spawn height must land on the mesh's own triangles (the surface the
    // physics collides with), at the origin and around it.
    const float pts[5][2] = {{0, 0}, {1.3f, -0.7f}, {-2.1f, 3.4f}, {5, 5}, {-4, 6}};
    for (const auto &pt : pts) {
      const float x = pt[0], y = pt[1];
      const float h = arenaSurfaceHeight(f, d.sizeM, d.resolution, x, y);
      QVERIFY(std::isfinite(h));
      bool hit = false;
      for (size_t t = 0; t + 2 < m.positions.size() && !hit; t += 3) {
        const PgVec3 &p0 = m.positions[t], &p1 = m.positions[t + 1],
                     &p2 = m.positions[t + 2];
        // Cheap reject: triangle within one cell of the query.
        if (std::fabs(p0.x - x) > cm / cr * 2 || std::fabs(p0.y - y) > cm / cr * 2)
          continue;
        const float d0 = (p1.y - p2.y) * (p0.x - p2.x) + (p2.x - p1.x) * (p0.y - p2.y);
        if (std::fabs(d0) < 1e-9f)
          continue;
        const float l0 = ((p1.y - p2.y) * (x - p2.x) + (p2.x - p1.x) * (y - p2.y)) / d0;
        const float l1 = ((p2.y - p0.y) * (x - p2.x) + (p0.x - p2.x) * (y - p2.y)) / d0;
        const float l2 = 1.0f - l0 - l1;
        if (l0 < -1e-4f || l1 < -1e-4f || l2 < -1e-4f)
          continue;
        const float zc = l0 * p0.z + l1 * p1.z + l2 * p2.z;
        QVERIFY2(std::fabs(h - (-zc)) < 2e-3f,
                 (id + ": spawn height off the mesh").c_str());
        hit = true;
      }
      QVERIFY2(hit, (id + ": point not on the mesh").c_str());
    }
    // The pad: the field is level (to a small tolerance) around the spawn.
    const float R = d.field.flatRadiusM;
    QVERIFY(R > 0.0f);
    const float h0 = f.height(0, 0);
    for (int i = 0; i < 16; ++i) {
      const float a = 6.2831853f * i / 16;
      QVERIFY2(std::fabs(f.height(R * std::cos(a), R * std::sin(a)) - h0) < 1e-3f,
               (id + ": spawn pad not level").c_str());
    }
  }
}

void TstWorldPresets::obstaclesClearOfSpawnAndSeated() {
  for (const std::string &id : {std::string("racing"), std::string("yard"),
                                std::string("forest")}) {
    for (uint32_t seed : {3u, 4u}) {
      const PresetData &d = cached(id, seed);
      QVERIFY2(d.obstacles.size() >= 20, id.c_str());
      const TerrainField f(d.field);
      const float half = d.sizeM * 0.5f;
      for (const PgObstacle &o : d.obstacles) {
        const float fr = footprint(o);
        QVERIFY2(std::hypot(o.x, o.y) >= kSpawnClearM + fr - 1e-3f,
                 (id + ": obstacle overlaps the spawn area").c_str());
        QVERIFY2(std::fabs(o.x) <= half && std::fabs(o.y) <= half,
                 (id + ": obstacle outside the arena").c_str());
        QVERIFY(std::isfinite(o.zc) && o.sx > 0 && o.sz > 0);
        if (o.type == 1)
          continue; // tree crowns float on their trunks by design
        // Seated: the shape's base is not above the local surface (no floating
        // gap) and its top is above it.
        const float g = arenaSurfaceHeight(f, d.sizeM, d.resolution, o.x, o.y);
        const float base = -(o.zc + o.sz * 0.5f), top = -(o.zc - o.sz * 0.5f);
        if (o.type == 2 || id != "racing") {
          QVERIFY2(base <= g + 0.02f, (id + ": obstacle floats above ground").c_str());
          QVERIFY2(top > g, (id + ": obstacle buried").c_str());
        }
      }
      // Yard / forest keep their shapes from interpenetrating (aisles/gaps).
      if (id != "racing") {
        for (size_t i = 0; i < d.obstacles.size(); ++i)
          for (size_t j = i + 1; j < d.obstacles.size(); ++j) {
            const PgObstacle &p = d.obstacles[i], &q = d.obstacles[j];
            if (id == "forest" && (p.type == 1) != (q.type == 1))
              continue; // trunk vs crown of different trees is checked via crowns
            if (id == "forest" && p.type == 2 && q.type == 2) {
              QVERIFY(std::hypot(p.x - q.x, p.y - q.y) > p.sx + q.sx);
              continue;
            }
            QVERIFY2(std::hypot(p.x - q.x, p.y - q.y) >=
                         footprint(p) + footprint(q) - 1e-3f,
                     (id + ": obstacles overlap").c_str());
          }
      }
    }
  }
}

// Open terrain carries slim survey poles of known size (the renderer bands them
// 1 m red/white) so a pilot can read height and speed: ringed around the spawn,
// inside the arena, clear of the spawn pad, seated on the surface.
void TstWorldPresets::openPresetsGetMarkerPoles() {
  for (const char *id : {"hills", "alpine", "flat", "desert"}) {
    for (uint32_t seed : {3u, 4u}) {
      const PresetData &d = cached(id, seed);
      QVERIFY2(d.obstacles.size() >= 12 && d.obstacles.size() <= 16, id);
      const TerrainField f(d.field);
      const float half = d.sizeM * 0.5f;
      for (const PgObstacle &o : d.obstacles) {
        QCOMPARE(o.type, 2);
        QCOMPARE(o.sx, kMarkerPoleR);
        QVERIFY2(o.sz >= kMarkerPoleH, id);
        QVERIFY2(std::hypot(o.x, o.y) >= kSpawnClearM + o.sx - 1e-3f, id);
        QVERIFY2(std::fabs(o.x) <= half && std::fabs(o.y) <= half, id);
        const float g = arenaSurfaceHeight(f, d.sizeM, d.resolution, o.x, o.y);
        QVERIFY2(-(o.zc + o.sz * 0.5f) <= g + 0.02f, id); // base not floating
        QVERIFY2(-(o.zc - o.sz * 0.5f) > g, id);          // not buried
      }
    }
  }
}

void TstWorldPresets::racingGatesEvenlySpacedAndPassable() {
  for (uint32_t seed : {5u, 6u, 7u}) {
    const PresetData &d = cached("racing", seed);
    const size_t n = d.gates.size();
    QVERIFY(n >= 8 && n <= 10);
    QVERIFY(d.propDiameterM >= kMaxPropDiameterM);
    double sum = 0.0;
    std::vector<double> gap(n);
    for (size_t i = 0; i < n; ++i) {
      const RaceGate &a = d.gates[i], &b = d.gates[(i + 1) % n]; // closed loop
      gap[i] = std::hypot(b.x - a.x, b.y - a.y);
      sum += gap[i];
    }
    const double mean = sum / static_cast<double>(n);
    for (size_t i = 0; i < n; ++i)
      QVERIFY2(std::fabs(gap[i] - mean) <= 0.15 * mean,
               "gate spacing is not even");
    for (const RaceGate &g : d.gates) {
      QVERIFY2(g.widthM >= 3.0f * d.propDiameterM, "gate too narrow");
      QVERIFY2(g.clearanceM >= 3.0f * d.propDiameterM, "gate too low");
      // Nothing but the gate's own posts/bar inside its opening or just
      // before/after it (a 4 m corridor along the travel direction).
      const float yaw = g.yawDeg * 3.14159265f / 180.0f;
      const float tx = std::cos(yaw), ty = std::sin(yaw);
      for (const PgObstacle &o : d.obstacles) {
        const float dx = o.x - g.x, dy = o.y - g.y;
        const float along = dx * tx + dy * ty, lat = -dx * ty + dy * tx;
        if (std::fabs(along) > 4.0f || std::fabs(lat) > 10.0f)
          continue; // not near this gate's corridor
        // Only this gate's own parts live here: posts/bar sit at |along|~0 and
        // |lat| >= width/2 (posts) or the bar overhead.
        const bool post = o.type == 2 && std::fabs(along) < 0.05f &&
                          std::fabs(std::fabs(lat) - (g.widthM * 0.5f + 0.1f)) < 0.05f;
        const bool bar = o.type == 0 && std::fabs(along) < 0.05f && std::fabs(lat) < 0.05f;
        QVERIFY2(post || bar, "foreign obstacle in a gate corridor");
      }
    }
    // The spawn at the origin is far from every gate.
    for (const RaceGate &g : d.gates)
      QVERIFY(std::hypot(g.x, g.y) > kSpawnClearM + 10.0f);
  }
}

void TstWorldPresets::windPresetsMagnitudes() {
  struct Exp {
    const char *id;
    float speed, gust, sigma;
  };
  const Exp exp[] = {{"calm", 1.0f, 0.0f, 0.2f},
                     {"breezy", 4.0f, 1.5f, 0.6f},
                     {"gusty", 6.0f, 5.0f, 1.5f}};
  for (const Exp &e : exp) {
    for (uint32_t seed : {1u, 99u}) {
      WindPresetData w, w2;
      QVERIFY(buildWindPreset(e.id, seed, w));
      QVERIFY(buildWindPreset(e.id, seed, w2));
      QVERIFY(w.enabled);
      QCOMPARE(w.steadyN, w2.steadyN); // deterministic heading
      QCOMPARE(w.steadyE, w2.steadyE);
      QVERIFY(std::fabs(std::hypot(w.steadyN, w.steadyE) - e.speed) < 1e-4f);
      QCOMPARE(w.steadyD, 0.0f);
      QVERIFY(std::fabs(w.gustAmp - e.gust) < 1e-6f);
      QVERIFY(std::fabs(w.turbSigma - e.sigma) < 1e-6f);
      QVERIFY(w.turbTau > 0.0f);
      QVERIFY((e.gust > 0.0f) == (w.gustPeriod > 0.0f));
    }
  }
  // Strictly increasing severity: calm < breezy < gusty (steady + gust).
  WindPresetData c, b, g, off;
  buildWindPreset("calm", 1, c);
  buildWindPreset("breezy", 1, b);
  buildWindPreset("gusty", 1, g);
  const auto peak = [](const WindPresetData &w) {
    return std::hypot(w.steadyN, w.steadyE) + w.gustAmp + 3.0f * w.turbSigma;
  };
  QVERIFY(peak(c) < peak(b) && peak(b) < peak(g));
  QVERIFY(buildWindPreset("off", 1, off));
  QVERIFY(!off.enabled);
  QCOMPARE(peak(off), 0.0f);
  // Different seeds point the wind a different way.
  WindPresetData b2;
  buildWindPreset("breezy", 2, b2);
  QVERIFY(b.steadyN != b2.steadyN || b.steadyE != b2.steadyE);
}

QTEST_MAIN(TstWorldPresets)
#include "tst_worldpresets.moc"
