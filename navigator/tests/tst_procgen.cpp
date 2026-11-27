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
// tst_procgen.cpp — Phase 0 procedural terrain core.
//
// The procgen module is Qt-free; this QtTest binary just drives it and checks
// the invariants the render + collision pipelines rely on: determinism,
// triangle-soup integrity, in-bounds geometry, unit upward normals, and that
// the bilinear sampler agrees with the lattice it was built from.
#include <QtTest/QtTest>

#include "vsim/procgen/GroundCover.h"
#include "ChunkStreamer.h"
#include "ChunkWorkQueue.h"
#include "procgen/Flora.h"
#include "procgen/EdgeTrack.h"
#include "procgen/GroundCover.h"
#include "procgen/Heightfield.h"
#include "procgen/Helipads.h"
#include "procgen/WorldPresets.h"
#include "procgen/Noise.h"
#include "procgen/TerrainField.h"
#include "procgen/TerrainGen.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

using namespace vsim::procgen;

namespace {
TerrainParams smallParams(uint32_t seed = 1337u) {
  TerrainParams p;
  p.seed = seed;
  p.sizeM = 128.0f;
  p.resolution = 48; // keep the test fast; invariants are resolution-agnostic
  return p;
}
} // namespace

// Sample coordinates lo..hi in steps of `step`, built from an integer index
// (a float loop counter drifts and trips cert-flp30-c).
static std::vector<float> axis(float lo, float hi, float step) {
  std::vector<float> v;
  const int n = int((hi - lo) / step);
  for (int i = 0; i <= n; ++i) v.push_back(lo + float(i) * step);
  return v;
}

class TstProcgen : public QObject {
  Q_OBJECT
private slots:
  void noiseIsDeterministicAndBounded();
  void fbmStaysInRange();
  void heightfieldHeightsBounded();
  void sampleWorldMatchesLatticeNodes();
  void meshSoupIntegrity();
  void meshGeometryInBounds();
  void normalsAreUnitAndUpward();
  void sameSeedSameMesh();
  void differentSeedDiffersMesh();
  void fieldIsDeterministicAndBounded();
  void slopeCapIsTheSameInEveryDirection();
  void parcelBordersAreBlendedNotHardSteps();
  void groundCoverDeterministicAndBounded();
  void groundCoverGivesVarietyInEveryPalette();
  void groundCoverPondsAndTracksExist();
  void edgeTrackRuleAndSplitAgree();
  void edgeTrackNodesInterpolateWithoutFalseLines();
  void fieldMeshColoursVary();
  void fieldChunksTileSeamlessly();
  void fieldChunkSoupIntegrity();
  void streamerLoadsNeighborhood();
  void planHandsOutTheWholeWindowAtOnce();
  void workQueueServesTheNearestChunkFirst();
  void workQueueDropsChunksThatLeftTheWindow();
  void workQueueIgnoresAStaleGeneration();
  void workQueueHandsEachChunkToOneWorker();
  void workQueueCapsTheRunners();
  void streamerStableWithinCell();
  void streamerPagesOnCrossing();
  void collisionRecentreHysteresis();
  void collisionRecentreLeadsFastVehicle();
  void collisionWindowKeepsUpWithShipLatency();
  void floraSitsOnSurfaceAndDeterministic();
  void floraTilesWithoutDuplicates();
  void renderAndCollisionSurfacesAgree();
  void spawnAndPadHeightsMatchCollisionSurface();
  void helipadFootprintsAreLevel();
  void fineReliefIsBoundedGentleAndOptIn();
  void padCollisionSoupHasBothWindingsAndOnlyTheWindowsPads();
  void padSearchIsTheSameOnAnyThread();
  void surfaceWindowIsTheSurfaceHeight();
  void footprintWindowChangesNoPad();
};

void TstProcgen::noiseIsDeterministicAndBounded() {
  Noise a(42u), b(42u), c(43u);
  bool differs = false;
  for (int i = 0; i < 200; ++i) {
    const float x = i * 0.37f, y = i * 0.19f;
    const float va = a.gradient(x, y);
    QCOMPARE(va, b.gradient(x, y));     // same seed -> identical
    QVERIFY(va >= -1.5f && va <= 1.5f); // ~[-1,1] with margin
    if (std::fabs(va - c.gradient(x, y)) > 1e-6f)
      differs = true;
  }
  QVERIFY(differs); // a different seed decorrelates
}

void TstProcgen::fbmStaysInRange() {
  Noise n(7u);
  for (int i = 0; i < 500; ++i) {
    const float x = i * 0.123f, y = i * 0.071f;
    const float f = n.fbm(x, y, 6, 2.0f, 0.5f);
    QVERIFY(f >= -1.01f && f <= 1.01f);
    const float r = n.ridged(x, y, 6, 2.0f, 0.5f);
    QVERIFY(r >= -0.01f && r <= 1.01f);
  }
}

void TstProcgen::heightfieldHeightsBounded() {
  const TerrainParams p = smallParams();
  const Heightfield hf = generateHeightfield(p);
  QVERIFY(hf.valid());
  QCOMPARE(hf.n, p.resolution);
  float lo = 1e9f, hi = -1e9f;
  for (float v : hf.h) {
    lo = std::min(lo, v);
    hi = std::max(hi, v);
  }
  QVERIFY(lo >= -1e-4f);            // never below the ground plane
  QVERIFY(hi <= p.heightM + 1e-3f); // never above the peak cap
  QVERIFY(hi > p.heightM * 0.2f);   // and it actually rises (not flat)
}

void TstProcgen::sampleWorldMatchesLatticeNodes() {
  const Heightfield hf = generateHeightfield(smallParams());
  // Bilinear sampling exactly at a node must return that node's height.
  for (int iy = 0; iy < hf.n; iy += 7) {
    for (int ix = 0; ix < hf.n; ix += 7) {
      const PgVec3 w = hf.worldAt(ix, iy);
      QVERIFY(std::fabs(hf.sampleWorld(w.x, w.y) - hf.at(ix, iy)) < 1e-2f);
    }
  }
}

void TstProcgen::meshSoupIntegrity() {
  const TerrainParams p = smallParams();
  const ProcMesh m = generateMeadow(p);
  QCOMPARE(m.positions.size() % 3, std::size_t(0));
  QCOMPARE(m.normals.size(), m.positions.size());
  QCOMPARE(m.colors.size(), m.positions.size());
  // Two triangles per grid cell, (n-1)^2 cells.
  const std::size_t cells =
      static_cast<std::size_t>(p.resolution - 1) * (p.resolution - 1);
  QCOMPARE(m.triangleCount(), cells * 2);
}

void TstProcgen::meshGeometryInBounds() {
  const TerrainParams p = smallParams();
  const ProcMesh m = generateMeadow(p);
  const float half = p.sizeM * 0.5f + 1e-3f;
  for (const PgVec3 &v : m.positions) {
    QVERIFY(v.x >= -half && v.x <= half);
    QVERIFY(v.y >= -half && v.y <= half);
    // Terrain is above the ground plane: z = -height <= 0.
    QVERIFY(v.z <= 1e-3f && v.z >= -(p.heightM + 1e-2f));
  }
  // Colors are valid RGB in [0,1].
  for (const PgVec3 &c : m.colors) {
    QVERIFY(c.x >= 0.0f && c.x <= 1.0f);
    QVERIFY(c.y >= 0.0f && c.y <= 1.0f);
    QVERIFY(c.z >= 0.0f && c.z <= 1.0f);
  }
}

void TstProcgen::normalsAreUnitAndUpward() {
  const ProcMesh m = generateMeadow(smallParams());
  for (const PgVec3 &nrm : m.normals) {
    const float len = std::sqrt(nrm.x * nrm.x + nrm.y * nrm.y + nrm.z * nrm.z);
    QVERIFY(std::fabs(len - 1.0f) < 1e-3f);
    // In NED, up is -Z; every terrain normal must point at least somewhat up.
    QVERIFY(nrm.z < 0.0f);
  }
}

void TstProcgen::sameSeedSameMesh() {
  const ProcMesh a = generateMeadow(smallParams(2024u));
  const ProcMesh b = generateMeadow(smallParams(2024u));
  QCOMPARE(a.positions.size(), b.positions.size());
  for (std::size_t i = 0; i < a.positions.size(); ++i) {
    QCOMPARE(a.positions[i].x, b.positions[i].x);
    QCOMPARE(a.positions[i].y, b.positions[i].y);
    QCOMPARE(a.positions[i].z, b.positions[i].z);
  }
}

void TstProcgen::differentSeedDiffersMesh() {
  const ProcMesh a = generateMeadow(smallParams(1u));
  const ProcMesh b = generateMeadow(smallParams(2u));
  QCOMPARE(a.positions.size(), b.positions.size());
  std::size_t differing = 0;
  for (std::size_t i = 0; i < a.positions.size(); ++i)
    if (a.positions[i].z != b.positions[i].z)
      ++differing;
  QVERIFY(differing > a.positions.size() / 10); // fields are genuinely distinct
}

void TstProcgen::fieldIsDeterministicAndBounded() {
  FieldParams fp;
  fp.seed = 99u;
  TerrainField a(fp), b(fp);
  for (int i = 0; i < 300; ++i) {
    const float wx = i * 13.7f - 800.0f, wy = i * 7.1f - 400.0f;
    const float h = a.height(wx, wy);
    QCOMPARE(h, b.height(wx, wy));                   // deterministic
    QVERIFY(h >= -1e-4f && h <= fp.heightM + 1e-3f); // bounded, never negative
  }
}

void TstProcgen::slopeCapIsTheSameInEveryDirection() {
  // A ramp rising k per node along one grid axis, and one rising k per node
  // along both (2k over a diagonal step of length sqrt(2)). With k just under
  // the cap the axis ramp is allowed; the diagonal ramp, 2k/sqrt(2) per metre
  // toward its lowest diagonal neighbour, exceeds the cap and must be cut. The
  // old four-neighbour cap left it standing and carved stair teeth into steep
  // diagonal crests.
  constexpr int gn = 21;
  constexpr float maxStep = 1.0f, k = 0.9f;
  constexpr float kSqrt2 = 1.41421356f;
  const auto grid = [&](bool diagonal) {
    std::vector<float> H(static_cast<std::size_t>(gn) * gn);
    for (int j = 0; j < gn; ++j)
      for (int i = 0; i < gn; ++i)
        H[j * gn + i] = diagonal ? k * float(i + j) : k * float(i);
    return H;
  };
  const std::vector<float> axisIn = grid(false), diagIn = grid(true);
  std::vector<float> axis = axisIn, diag = diagIn;
  erodeSlopeCap(axis, gn, maxStep, 1);
  erodeSlopeCap(diag, gn, maxStep, 1);
  int axisCut = 0, diagCut = 0;
  for (int j = 1; j < gn - 1; ++j)
    for (int i = 1; i < gn - 1; ++i) {
      const int c = j * gn + i;
      axisCut += axis[c] < axisIn[c] - 1e-4f;
      diagCut += diag[c] < diagIn[c] - 1e-4f;
      // Whatever the direction, no node ends above its lowest neighbour by more
      // than the cap allows for that neighbour's distance.
      QVERIFY(diag[c] <= diagIn[c - gn - 1] + kSqrt2 * maxStep + 1e-4f);
    }
  QCOMPARE(axisCut, 0);                   // k < maxStep: a gentle axis ramp stays
  QCOMPARE(diagCut, (gn - 2) * (gn - 2)); // the steeper diagonal ramp is cut
}

void TstProcgen::parcelBordersAreBlendedNotHardSteps() {
  // Walk lines across many parcels at the chunk mesh's vertex spacing. The field
  // parcels used to switch colour on a hard border (a step of 0.5 in one vertex);
  // warped borders blended over several metres must stay under kMaxStep per
  // vertex. The mean step guards against a test that sees one flat colour.
  constexpr float kMaxStep = 0.12f;
  const float vertexM = kStreamChunkM / float(kStreamChunkRes);
  for (int palette : {7, 8})
    for (uint32_t seed : {3u, 11u, 99u}) {
      GroundCover gc(seed, palette);
      float mx = 0.0f;
      double sum = 0.0;
      long n = 0;
      for (int row = 0; row < 12; ++row) {
        PgVec3 prev{};
        for (int i = 0; i < 3000; ++i) {
          const PgVec3 c = gc.parcelColour(-3000.0f + vertexM * float(i), -900.0f + 163.0f * float(row));
          if (i > 0) {
            const float d = std::max({std::fabs(c.x - prev.x), std::fabs(c.y - prev.y), std::fabs(c.z - prev.z)});
            mx = std::max(mx, d);
            sum += double(d);
            ++n;
          }
          prev = c;
        }
      }
      QVERIFY2(mx <= kMaxStep, qPrintable(QStringLiteral("palette %1 seed %2: step %3 between two vertices").arg(palette).arg(seed).arg(double(mx))));
      QVERIFY2(sum / double(n) > 0.003, "the parcels must actually differ in colour");
    }
}

void TstProcgen::fieldChunksTileSeamlessly() {
  // Two horizontally-adjacent chunks must agree exactly along their shared
  // edge: same positions, normals, AND colors. This is what makes streaming
  // seamless.
  FieldParams fp;
  fp.seed = 5u;
  TerrainField f(fp);
  const float chunkM = 128.0f;
  const int res = 32;
  const ProcMesh left = meshFieldChunk(f, 0, 0, chunkM, res);
  const ProcMesh right = meshFieldChunk(f, 1, 0, chunkM, res);
  QVERIFY(!left.empty() && !right.empty());

  // Collect the vertices on the boundary x == chunkM from both chunks, keyed by
  // their y coordinate; every left-edge sample of `right` must coincide with a
  // right-edge sample of `left`.
  auto matchAt = [](const ProcMesh &m, float xWanted, float y) -> int {
    int found = -1;
    for (std::size_t i = 0; i < m.positions.size(); ++i) {
      if (std::fabs(m.positions[i].x - xWanted) < 1e-3f &&
          std::fabs(m.positions[i].y - y) < 1e-3f) {
        found = (int)i;
        break;
      }
    }
    return found;
  };
  int checked = 0;
  for (std::size_t i = 0; i < right.positions.size(); ++i) {
    if (std::fabs(right.positions[i].x - chunkM) > 1e-3f)
      continue; // left edge
    const int li = matchAt(left, chunkM, right.positions[i].y);
    QVERIFY(li >= 0);
    QCOMPARE(left.positions[li].z, right.positions[i].z);
    QCOMPARE(left.normals[li].x, right.normals[i].x);
    QCOMPARE(left.normals[li].y, right.normals[i].y);
    QCOMPARE(left.normals[li].z, right.normals[i].z);
    QCOMPARE(left.colors[li].x, right.colors[i].x);
    if (++checked > 40)
      break;
  }
  QVERIFY(checked > 5);
}

void TstProcgen::fieldChunkSoupIntegrity() {
  FieldParams fp;
  TerrainField f(fp);
  const int res = 24;
  const ProcMesh m = meshFieldChunk(f, -3, 2, 160.0f, res);
  QCOMPARE(m.positions.size() % 3, std::size_t(0));
  QCOMPARE(m.normals.size(), m.positions.size());
  QCOMPARE(m.colors.size(), m.positions.size());
  QCOMPARE(m.triangleCount(), static_cast<std::size_t>(res) * res * 2);
  // Chunk (-3,2) occupies world x in [-480,-320], y in [320,480].
  for (const PgVec3 &v : m.positions) {
    QVERIFY(v.x >= -480.1f && v.x <= -319.9f);
    QVERIFY(v.y >= 319.9f && v.y <= 480.1f);
    QVERIFY(v.z <= 1e-3f); // terrain above ground plane
  }
}

static vsim::ChunkStreamer::Config streamCfg(int renderRadius) {
  vsim::ChunkStreamer::Config c;
  c.field.seed = 11u;
  c.chunkM = 100.0f;
  c.resolution = 8; // tiny chunks -> fast test
  c.renderRadius = renderRadius;
  c.collisionRadius = 1;
  c.maxQueued = 4; // cap per plan(); drain over several calls
  return c;
}

// Pump plan() at a fixed point, "building" each requested chunk (markBuilt),
// until no more work is produced. Returns {totalBuilds, totalRemoves,
// collisionEverDue}.
struct DrainResult {
  std::size_t builds, removes;
  bool collision;
};
static DrainResult drain(vsim::ChunkStreamer &s, float wx, float wy) {
  DrainResult r{0, 0, false};
  for (int i = 0; i < 200; ++i) {
    const vsim::StreamPlan p = s.plan(wx, wy);
    r.builds += p.toBuild.size();
    r.removes += p.toRemove.size();
    r.collision = r.collision || p.collisionDue;
    for (const vsim::ChunkReq &req : p.toBuild)
      s.markBuilt(req.key);
    if (p.toBuild.empty() && p.toRemove.empty())
      break;
  }
  return r;
}

// The default hand-out is the whole window in ONE plan(): the workers pull
// from a queue, so how many chunks the GUI hands over per tick no longer
// decides how long the mesh pool has work.
void TstProcgen::planHandsOutTheWholeWindowAtOnce() {
  vsim::ChunkStreamer s;
  vsim::ChunkStreamer::Config c = streamCfg(3);
  c.maxQueued = vsim::ChunkStreamer::Config().maxQueued; // the default
  s.configure(c);
  const vsim::StreamPlan p = s.plan(0.0f, 0.0f);
  QCOMPARE(p.toBuild.size(), std::size_t(49)); // 7x7
  QCOMPARE(s.centreCx(), 0);
  QCOMPARE(s.centreCy(), 0);
}

static std::vector<vsim::ChunkReq> cells(
    std::initializer_list<std::pair<int, int>> l) {
  std::vector<vsim::ChunkReq> v;
  for (const auto &c : l)
    v.push_back({vsim::ChunkStreamer::keyOf(c.first, c.second), c.first,
                 c.second});
  return v;
}

void TstProcgen::workQueueServesTheNearestChunkFirst() {
  vsim::ChunkWorkQueue q;
  q.reset(1);
  q.setWindow(0, 0, 5);
  q.push(cells({{3, 3}, {0, 1}, {-2, 0}, {0, 0}, {1, 1}}), 1);
  vsim::ChunkReq r;
  std::vector<std::pair<int, int>> order;
  while (q.pop(1, r))
    order.push_back({r.cx, r.cy});
  const std::vector<std::pair<int, int>> want = {
      {0, 0}, {0, 1}, {1, 1}, {-2, 0}, {3, 3}};
  QCOMPARE(order, want);
  // The view moves: what is nearest is decided at pop time, not push time.
  q.push(cells({{0, 0}, {4, 4}}), 1);
  q.setWindow(4, 4, 5);
  QVERIFY(q.pop(1, r));
  QCOMPARE(std::make_pair(r.cx, r.cy), std::make_pair(4, 4));
}

void TstProcgen::workQueueDropsChunksThatLeftTheWindow() {
  vsim::ChunkWorkQueue q;
  q.reset(1);
  q.setWindow(0, 0, 2);
  q.push(cells({{0, 0}, {2, 2}, {9, 9}}), 1);
  q.setWindow(0, 0, 2);
  QCOMPARE(q.pending(), std::size_t(2)); // {9,9} dropped by the window
  q.setWindow(20, 20, 2);                // everything left
  QCOMPARE(q.pending(), std::size_t(0));
  const std::vector<qint64> dropped = q.takeDropped();
  QCOMPARE(dropped.size(), std::size_t(3));
  QVERIFY(q.takeDropped().empty()); // taken once
  vsim::ChunkReq r;
  QVERIFY(!q.pop(1, r));
}

void TstProcgen::workQueueIgnoresAStaleGeneration() {
  vsim::ChunkWorkQueue q;
  q.reset(1);
  q.push(cells({{0, 0}, {1, 0}}), 1);
  q.reset(2); // world changed: pending work is gone
  QCOMPARE(q.pending(), std::size_t(0));
  q.push(cells({{5, 5}}), 1); // a late push from the old generation
  QCOMPARE(q.pending(), std::size_t(0));
  vsim::ChunkReq r;
  QVERIFY(!q.pop(1, r)); // an old worker gets nothing and ends
  q.push(cells({{5, 5}}), 2);
  QVERIFY(!q.pop(1, r));
  QVERIFY(q.pop(2, r));
}

// Many workers popping at once: every chunk is served exactly once.
void TstProcgen::workQueueHandsEachChunkToOneWorker() {
  vsim::ChunkWorkQueue q;
  q.reset(7);
  q.setWindow(0, 0, 100);
  std::vector<vsim::ChunkReq> all;
  for (int i = 0; i < 40; ++i)
    for (int j = 0; j < 40; ++j)
      all.push_back({vsim::ChunkStreamer::keyOf(i, j), i, j});
  q.push(all, 7);
  std::mutex m;
  std::vector<qint64> got;
  std::vector<std::thread> ts;
  for (int t = 0; t < 4; ++t)
    ts.emplace_back([&] {
      vsim::ChunkReq r;
      while (q.pop(7, r)) {
        std::lock_guard<std::mutex> g(m);
        got.push_back(r.key);
      }
    });
  for (auto &t : ts)
    t.join();
  QCOMPARE(got.size(), all.size());
  std::sort(got.begin(), got.end());
  QVERIFY(std::adjacent_find(got.begin(), got.end()) == got.end());
}

void TstProcgen::workQueueCapsTheRunners() {
  vsim::ChunkWorkQueue q;
  QVERIFY(q.acquireRunner(2));
  QVERIFY(q.acquireRunner(2));
  QVERIFY(!q.acquireRunner(2)); // the global pool keeps room for other jobs
  q.releaseRunner();
  QVERIFY(q.acquireRunner(2));
  q.releaseRunner();
  q.releaseRunner();
  q.releaseRunner(); // an extra release does not go negative
  QCOMPARE(q.runners(), 0);
}

void TstProcgen::streamerLoadsNeighborhood() {
  vsim::ChunkStreamer s;
  s.configure(streamCfg(2));
  QVERIFY(s.active());
  const DrainResult r = drain(s, 0.0f, 0.0f);
  // Chebyshev radius 2 -> a 5x5 = 25 window, streamed in over several plans.
  QCOMPARE(r.builds, std::size_t(25));
  QCOMPARE(r.removes, std::size_t(0));
  QVERIFY(r.collision);
}

void TstProcgen::streamerStableWithinCell() {
  vsim::ChunkStreamer s;
  s.configure(streamCfg(2));
  drain(s, 0.0f, 0.0f);
  // Move within the same chunk cell (cell is 100 m): fully loaded already, so
  // no new builds and no collision rebuild.
  const vsim::StreamPlan p = s.plan(40.0f, 25.0f);
  QVERIFY(p.toBuild.empty());
  QVERIFY(p.toRemove.empty());
  QVERIFY(!p.collisionDue);
}

void TstProcgen::streamerPagesOnCrossing() {
  const int r = 2;
  vsim::ChunkStreamer s;
  s.configure(streamCfg(r));
  drain(s, 0.0f, 0.0f);
  // Cross one cell east (x: 0 -> cell 1). One column leaves, one enters; the
  // loaded-set size is conserved, so total builds == total removes == (2r+1).
  const DrainResult d = drain(s, 150.0f, 0.0f);
  QCOMPARE(d.builds, static_cast<std::size_t>(2 * r + 1));
  QCOMPARE(d.removes, static_cast<std::size_t>(2 * r + 1));
  QVERIFY(d.collision); // centre changed cell -> collision reshipped
}

// ---- collision window recentre (pure decision) -----------------------------
using CR = vsim::ChunkStreamer;

void TstProcgen::collisionRecentreHysteresis() {
  const float C = 100.0f;
  // Hovering just over the border of the centre chunk (cell 0): no flapping.
  QVERIFY(!CR::collisionRecentre(101.0f, 50.0f, 0, 0, 0, 0, C).recentre);
  QVERIFY(!CR::collisionRecentre(120.0f, 50.0f, 0, 0, 0, 0, C).recentre);
  QVERIFY(!CR::collisionRecentre(-20.0f, 50.0f, 0, 0, 0, 0, C).recentre);
  // Clearly committed to the neighbour (> 0.75 chunk from the centre middle).
  const auto r = CR::collisionRecentre(130.0f, 50.0f, 0, 0, 0, 0, C);
  QVERIFY(r.recentre);
  QCOMPARE(r.cx, 1);
  QCOMPARE(r.cy, 0);
  // Same spot, but the window is already centred on cell 1: stay put (the
  // decision is stable, it does not flip back).
  QVERIFY(!CR::collisionRecentre(130.0f, 50.0f, 0, 0, 1, 0, C).recentre);
  // Unset centre always recentres, on the vehicle's chunk.
  const auto f = CR::collisionRecentre(-30.0f, 250.0f, 0, 0, INT_MIN, INT_MIN, C);
  QVERIFY(f.recentre);
  QCOMPARE(f.cx, -1);
  QCOMPARE(f.cy, 2);
}

void TstProcgen::collisionRecentreLeadsFastVehicle() {
  const float C = 192.0f;
  // Mid-chunk, doing 150 m/s east: after 1 s it is 150 m on, i.e. in cell 1.
  // The window must already be moving, long before the chunk boundary.
  const auto r = CR::collisionRecentre(96.0f, 96.0f, 150.0f, 0, 0, 0, C);
  QVERIFY(r.recentre);
  QCOMPARE(r.cx, 1);
  QCOMPARE(r.cy, 0);
  // A slow vehicle at the same spot does not trigger it.
  QVERIFY(!CR::collisionRecentre(96.0f, 96.0f, 5.0f, 0, 0, 0, C).recentre);
  // Absurd speed: the look-ahead is capped, so the target is at most one chunk
  // from the vehicle and its own chunk stays inside a radius-1 window.
  const auto fast = CR::collisionRecentre(96.0f, 96.0f, 5000.0f, 0, 0, 0, C);
  QVERIFY(fast.recentre);
  QVERIFY(std::abs(fast.cx - 0) <= 1);
}

// Fly straight lines at several speeds, polling at 10 Hz like the widget does,
// with the new window only taking effect `shipS` after a recentre (BVH build +
// queue). The vehicle's chunk must always be inside the SHIPPED radius-1 window.
void TstProcgen::collisionWindowKeepsUpWithShipLatency() {
  const float C = 192.0f, dt = 0.1f, shipS = 0.30f; // measured ~0.1-0.2 s + margin
  for (float speed : {10.0f, 40.0f, 80.0f, 120.0f}) {
    for (float ang : {0.0f, 0.6f, 1.5708f, 2.4f}) {
      const float vx = speed * std::cos(ang), vy = speed * std::sin(ang);
      int polledCx = INT_MIN, polledCy = INT_MIN; // what the streamer decided
      int shipCx = INT_MIN, shipCy = INT_MIN;     // what the engine has
      int pendCx = 0, pendCy = 0;
      float pendAt = -1.0f;
      for (int step = 0; step < 600; ++step) {
        const float t = static_cast<float>(step) * dt;
        const float x = 7.0f + vx * t, y = 11.0f + vy * t;
        if (pendAt >= 0.0f && t >= pendAt) {
          shipCx = pendCx;
          shipCy = pendCy;
          pendAt = -1.0f;
        }
        const auto r =
            CR::collisionRecentre(x, y, vx, vy, polledCx, polledCy, C);
        if (r.recentre && (r.cx != polledCx || r.cy != polledCy)) {
          polledCx = pendCx = r.cx;
          polledCy = pendCy = r.cy;
          pendAt = t + shipS;
        }
        if (t < 1.0f)
          continue; // initial load
        const int dx = static_cast<int>(std::floor(x / C)) - shipCx;
        const int dy = static_cast<int>(std::floor(y / C)) - shipCy;
        QVERIFY2(std::abs(dx) <= 1 && std::abs(dy) <= 1,
                 qPrintable(QString("outran window at %1 m/s ang %2 t=%3")
                                .arg(speed).arg(ang).arg(t)));
      }
    }
  }
}

void TstProcgen::floraSitsOnSurfaceAndDeterministic() {
  FieldParams fp;
  fp.seed = 4u;
  TerrainField f(fp);
  FloraParams gp;
  gp.seed = 4u;
  gp.spacing = 2.0f;
  const std::vector<FloraInstance> a = scatterFlora(f, 0, 0, 160.0f, gp);
  const std::vector<FloraInstance> b = scatterFlora(f, 0, 0, 160.0f, gp);
  QVERIFY(!a.empty());
  QCOMPARE(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    // Deterministic.
    QCOMPARE(a[i].pos.x, b[i].pos.x);
    QCOMPARE(a[i].pos.z, b[i].pos.z);
    // Base sits on the surface. Grass samples a coarse height grid (bilinear),
    // not the exact field, so allow the small interpolation error.
    const float surf = -f.height(a[i].pos.x, a[i].pos.y);
    QVERIFY(std::fabs(a[i].pos.z - surf) < 0.25f);
    // Height is positive and within a few std deviations of the mean.
    QVERIFY(a[i].height > 0.0f &&
            a[i].height <= gp.heightMean + 6.0f * gp.heightStdDev + 0.1f);
    QVERIFY(a[i].tint.x >= 0.0f && a[i].tint.x <= 1.0f);
  }
}

void TstProcgen::floraTilesWithoutDuplicates() {
  // A blade is owned by exactly one chunk: the same world cell must not appear
  // in two adjacent chunks. Check no blade in chunk (1,0) lies in chunk (0,0)'s
  // x-range and vice versa — ownership is by the unjittered cell centre, so
  // positions stay within (roughly) their chunk.
  FieldParams fp;
  TerrainField f(fp);
  FloraParams gp;
  gp.spacing = 2.0f;
  const float chunkM = 160.0f;
  const std::vector<FloraInstance> c0 = scatterFlora(f, 0, 0, chunkM, gp);
  const std::vector<FloraInstance> c1 = scatterFlora(f, 1, 0, chunkM, gp);
  QVERIFY(!c0.empty() && !c1.empty());
  // Jitter can push a base slightly over the seam; allow a small margin.
  for (const FloraInstance &b : c0)
    QVERIFY(b.pos.x < chunkM + gp.spacing);
  for (const FloraInstance &b : c1)
    QVERIFY(b.pos.x >= chunkM - gp.spacing);
}


// Elevation [m] of a chunk mesh's own triangles at world (x, y): walks the same
// two-triangle split meshFieldChunk emits, so it is exactly the surface a
// physics BVH built from that mesh presents.
static float meshElevationAt(const ProcMesh &m, int cx, int cy, float chunkM,
                             int res, float x, float y) {
  const float step = chunkM / static_cast<float>(res);
  const float u = (x - cx * chunkM) / step, v = (y - cy * chunkM) / step;
  const int i = std::min(res - 1, std::max(0, static_cast<int>(std::floor(u))));
  const int j = std::min(res - 1, std::max(0, static_cast<int>(std::floor(v))));
  const float fx = u - i, fy = v - j;
  const std::size_t b = (static_cast<std::size_t>(j) * res + i) * 6;
  auto z = [&](std::size_t k) { return -m.positions[b + k].z; };
  // Tri A = (0,0),(1,0),(1,1) where fx >= fy; tri B = (0,0),(1,1),(0,1).
  if (fx >= fy)
    return z(0) + (z(1) - z(0)) * fx + (z(2) - z(1)) * fy;
  return z(3) + (z(4) - z(5)) * fx + (z(5) - z(3)) * fy;
}

// Max |a-b| of the two chunk tessellations over random in-chunk points.
static float maxMeshDeviation(uint32_t seed, int cx, int cy, float chunkM,
                              int resA, int resB, int samples, uint32_t &rng) {
  FieldParams fp;
  fp.seed = seed;
  TerrainField f(fp);
  const ProcMesh a = meshFieldChunk(f, cx, cy, chunkM, resA);
  const ProcMesh b = meshFieldChunk(f, cx, cy, chunkM, resB);
  auto rnd = [&]() {
    rng = rng * 1664525u + 1013904223u;
    return static_cast<float>(rng >> 8) / 16777216.0f;
  };
  float mx = 0;
  for (int n = 0; n < samples; ++n) {
    const float x = (cx + rnd()) * chunkM, y = (cy + rnd()) * chunkM;
    mx = std::max(mx, std::fabs(meshElevationAt(a, cx, cy, chunkM, resA, x, y) -
                                meshElevationAt(b, cx, cy, chunkM, resB, x, y)));
  }
  return mx;
}

void TstProcgen::renderAndCollisionSurfacesAgree() {
  // The streamer's defaults ARE what the sim runs: physics must collide with the
  // surface that is drawn, to well under a centimetre.
  const vsim::ChunkStreamer::Config cfg;
  const float tol = 1e-3f; // m
  uint32_t rng = 12345u;
  float worst = 0;
  for (uint32_t seed : {1u, 7u, 42u, 1337u, 90210u})
    for (int c = 0; c < 3; ++c)
      worst = std::max(worst,
                       maxMeshDeviation(seed, c - 1, (c * 2) % 3 - 1, cfg.chunkM,
                                        cfg.resolution, cfg.collisionResolution,
                                        150, rng));
  qInfo("default render vs collision max deviation: %.5f m", worst);
  QVERIFY2(worst <= tol, "collision surface differs from the rendered surface");

  // A coarser collision mesh is allowed only inside a stated bound; this pins
  // the documented figure (measured ~1.1 m for 48 vs 100 cells on 192 m chunks).
  float coarse = 0;
  for (uint32_t seed : {1u, 7u, 42u, 1337u, 90210u})
    coarse = std::max(coarse, maxMeshDeviation(seed, 0, 0, cfg.chunkM,
                                               cfg.resolution, 48, 300, rng));
  qInfo("render(100) vs collision(48) max deviation: %.3f m", coarse);
  QVERIFY(coarse <= 2.0f);
}

void TstProcgen::spawnAndPadHeightsMatchCollisionSurface() {
  const vsim::ChunkStreamer::Config cfg;
  const float tol = 1e-3f;
  uint32_t rng = 777u;
  auto rnd = [&]() {
    rng = rng * 1664525u + 1013904223u;
    return static_cast<float>(rng >> 8) / 16777216.0f;
  };
  float worst = 0, worstRaw = 0;
  for (uint32_t seed : {3u, 11u, 5150u}) {
    FieldParams fp;
    fp.seed = seed;
    TerrainField f(fp);
    // The 3x3 collision neighbourhood around the origin, exactly as shipped.
    std::vector<ProcMesh> chunks;
    for (int j = -1; j <= 1; ++j)
      for (int i = -1; i <= 1; ++i)
        chunks.push_back(meshFieldChunk(f, i, j, cfg.chunkM,
                                        cfg.collisionResolution));
    std::vector<const ProcMesh *> ptrs;
    for (const ProcMesh &m : chunks)
      ptrs.push_back(&m);
    const vsim::LoadedMesh coll = vsim::collisionMeshFromChunks(ptrs);
    QVERIFY(coll.valid);

    // Points: the origin (home pad / spawn), chunk seams, and random spots.
    std::vector<std::pair<float, float>> pts = {
        {0.0f, 0.0f}, {cfg.chunkM, 5.0f}, {-cfg.chunkM, -cfg.chunkM}, {3.3f, -77.7f}};
    for (int n = 0; n < 40; ++n)
      pts.emplace_back((rnd() * 2 - 1) * cfg.chunkM * 0.99f,
                       (rnd() * 2 - 1) * cfg.chunkM * 0.99f);
    for (const auto &[x, y] : pts) {
      const float deck = surfaceHeight(f, x, y, cfg.chunkM, cfg.collisionResolution);
      // Brute-force the actual collision triangles for the one covering (x, y).
      bool hit = false;
      for (std::size_t t = 0; t + 2 < coll.positions.size() && !hit; t += 3) {
        const QVector3D &p0 = coll.positions[t], &p1 = coll.positions[t + 1],
                        &p2 = coll.positions[t + 2];
        const float d = (p1.y() - p2.y()) * (p0.x() - p2.x()) +
                        (p2.x() - p1.x()) * (p0.y() - p2.y());
        if (std::fabs(d) < 1e-9f)
          continue;
        const float l0 = ((p1.y() - p2.y()) * (x - p2.x()) +
                          (p2.x() - p1.x()) * (y - p2.y())) / d;
        const float l1 = ((p2.y() - p0.y()) * (x - p2.x()) +
                          (p0.x() - p2.x()) * (y - p2.y())) / d;
        const float l2 = 1.0f - l0 - l1;
        const float e = -1e-4f;
        if (l0 < e || l1 < e || l2 < e)
          continue;
        const float zc = l0 * p0.z() + l1 * p1.z() + l2 * p2.z(); // NED z
        worst = std::max(worst, std::fabs(deck - (-zc)));
        worstRaw = std::max(worstRaw, std::fabs(f.height(x, y) - (-zc)));
        hit = true;
      }
      QVERIFY2(hit, "point not covered by the collision mesh");
    }
  }
  qInfo("deck/spawn vs collision triangles: max %.5f m (raw f.height(): %.3f m)",
        worst, worstRaw);
  QVERIFY2(worst <= tol, "pad/spawn height is not on the collision surface");
}


// ---- ground cover ----------------------------------------------------------

namespace {
const PgVec3 kBase{0.30f, 0.40f, 0.20f};
float luma(const PgVec3 &c) { return 0.3f * c.x + 0.59f * c.y + 0.11f * c.z; }
} // namespace

void TstProcgen::groundCoverDeterministicAndBounded() {
  for (int pal = 0; pal <= 9; ++pal) {
    const GroundCover a(1337u, pal), b(1337u, pal), c(4242u, pal);
    bool seedDiffers = false;
    for (float y : axis(-200.0f, 200.0f, 9.7f))
      for (float x : axis(-200.0f, 200.0f, 9.7f)) {
        const GroundCover::Sample sa = a.sample(x, y, kBase, 0.2f, 0.99f);
        const GroundCover::Sample sb = b.sample(x, y, kBase, 0.2f, 0.99f);
        const GroundCover::Sample sc = c.sample(x, y, kBase, 0.2f, 0.99f);
        QCOMPARE(sa.color.x, sb.color.x);
        QCOMPARE(sa.color.y, sb.color.y);
        QCOMPARE(sa.color.z, sb.color.z);
        QCOMPARE(sa.grass, sb.grass);
        for (float v : {sa.color.x, sa.color.y, sa.color.z, sa.grass}) {
          QVERIFY(std::isfinite(v));
          QVERIFY(v >= 0.0f && v <= 1.0f);
        }
        seedDiffers = seedDiffers || sa.color.x != sc.color.x;
      }
    QVERIFY2(seedDiffers, qPrintable(QString("palette %1 ignores the seed").arg(pal)));
  }
  // Steep / high inputs and the origin must not produce NaN either.
  const GroundCover g(7u, kPalAlpine);
  const GroundCover::Sample s = g.sample(0.0f, 0.0f, kBase, 1.0f, 0.0f);
  QVERIFY(std::isfinite(s.color.x + s.color.y + s.color.z + s.grass));
}

void TstProcgen::groundCoverGivesVarietyInEveryPalette() {
  for (int pal = 0; pal <= 9; ++pal) {
    const GroundCover g(1337u, pal);
    float lo = 9.0f, hi = -9.0f;
    double sum = 0.0, sum2 = 0.0;
    int n = 0, nonGrass = 0;
    for (float y : axis(-200.0f, 200.0f, 4.0f))
      for (float x : axis(-200.0f, 200.0f, 4.0f)) {
        const GroundCover::Sample s = g.sample(x, y, kBase, 0.05f, 1.0f);
        const float l = luma(s.color);
        lo = std::min(lo, l);
        hi = std::max(hi, l);
        sum += l;
        sum2 += static_cast<double>(l) * l;
        ++n;
        nonGrass += s.grass < 0.5f ? 1 : 0;
      }
    const double mean = sum / n, sd = std::sqrt(std::max(0.0, sum2 / n - mean * mean));
    const QString tag = QString("palette %1").arg(pal);
    // The cover layer alone (ponds are WaterField basins now, not cover).
    QVERIFY2(sd > 0.015, qPrintable(tag + " luma stddev too small"));
    QVERIFY2(hi - lo > 0.12f, qPrintable(tag + " luma range too small"));
    // Some, but not most, of the ground is track/water/rock/bare.
    const double frac = static_cast<double>(nonGrass) / n;
    QVERIFY2(frac > 0.002 && frac < 0.7, qPrintable(tag + QString(" non-grass %1").arg(frac)));
  }
}

void TstProcgen::groundCoverPondsAndTracksExist() {
  // Ponds are WaterField basins now: every pond palette gets bodies somewhere
  // over a few kilometres, and the meshed colour at a pond's centre reads as
  // water (blue-dominant, grass-free).
  for (int pal : {0, 2, 3, 5, 6, 7, 8, 9}) {
    int bodies = 0;
    for (uint32_t seed : {1337u, 7u, 21u}) {
      FieldParams p;
      p.seed = seed;
      p.palette = pal;
      p.heightM = 70.0f;
      const TerrainField f(p);
      for (const WaterBody &b :
           f.water().bodiesIn(f.heightSource(), -2000, -2000, 2000, 2000)) {
        ++bodies;
        const GroundCover::Sample s =
            f.colorAt(b.cx, b.cy, f.height(b.cx, b.cy), 1.0f);
        QVERIFY2(s.grass < 0.2f && s.color.z > s.color.x + 0.03f,
                 qPrintable(QString("palette %1: pond centre is not water").arg(pal)));
      }
    }
    QVERIFY2(bodies >= 2, qPrintable(QString("palette %1: only %2 ponds").arg(pal).arg(bodies)));
  }
  // The desert has no ponds.
  FieldParams dp;
  dp.palette = kPalDesert;
  const TerrainField desert(dp);
  QVERIFY(desert.water().bodiesIn(desert.heightSource(), -2000, -2000, 2000, 2000).empty());
}

void TstProcgen::edgeTrackRuleAndSplitAgree() {
  // The shared rule: full track on the border, none at 2 m, none where the
  // unsigned distance says no border is near (the sign-jump guard).
  QCOMPARE(edgeTrackWeight(0.0f, 0.0f), kEdgeTrackMax);
  QCOMPARE(edgeTrackWeight(-0.3f, 0.3f), edgeTrackWeight(0.3f, 0.3f));
  QCOMPARE(edgeTrackWeight(kEdgeTrackD1, 0.0f), 0.0f);
  QCOMPARE(edgeTrackWeight(0.0f, kEdgeValidMax), 0.0f);
  QVERIFY(edgeTrackWeight(1.4f, 1.4f) > 0.0f && edgeTrackWeight(1.4f, 1.4f) < kEdgeTrackMax);

  // sampleSplit() keeps sample()'s colour exactly (the baked track, what Low
  // draws) and adds the track-free colour the High shader draws the track over:
  // equal to the baked one wherever the track has no weight.
  FieldParams p;
  p.seed = 11u;
  p.palette = 7; // hills: parcels with tracked borders
  p.heightM = 30.0f;
  const GroundCover gc(p.seed, p.palette);
  int onTrack = 0, off = 0;
  for (int i = 0; i < 4000; ++i) {
    const float x = -300.0f + 0.37f * float(i % 1600), y = -250.0f + 0.0763f * float(i);
    const PgVec3 base{0.3f, 0.5f, 0.2f};
    const GroundCover::Sample a = gc.sample(x, y, base, 0.3f, 0.99f);
    const GroundCover::Sample b = gc.sampleSplit(x, y, base, 0.3f, 0.99f);
    QCOMPARE(a.color.x, b.color.x);
    QCOMPARE(a.color.y, b.color.y);
    QCOMPARE(a.color.z, b.color.z);
    QCOMPARE(a.grass, b.grass);
    if (edgeTrackWeight(b.edgeS, b.edgeA) > 0.0f) {
      ++onTrack;
      QVERIFY(b.edgeK > 0.0f);
    } else {
      ++off;
      QCOMPARE(b.split.x, b.color.x);
      QCOMPARE(b.split.y, b.color.y);
      QCOMPARE(b.split.z, b.color.z);
    }
  }
  QVERIFY2(onTrack > 20 && off > 20, "the probe grid must cover track and ground");
}

void TstProcgen::edgeTrackNodesInterpolateWithoutFalseLines() {
  FieldParams p;
  p.seed = 11u;
  p.palette = 7;
  p.heightM = 30.0f;
  const TerrainField f(p);
  const float step = kStreamChunkM / float(kStreamChunkRes);
  auto node = [&](int ix, int iy) {
    const float x = float(ix) * step, y = float(iy) * step;
    const float h = f.height(x, y);
    const PgVec3 n = f.normal(x, y, step);
    return f.colorAtSplit(x, y, h, n.z < 0.0f ? -n.z : 0.0f);
  };
  int probes = 0, bad = 0, falseLine = 0;
  double worst = 0.0;
  for (int iy = -60; iy < 60; ++iy)
    for (int ix = -60; ix < 60; ++ix) {
      const auto c00 = node(ix, iy), c10 = node(ix + 1, iy), c01 = node(ix, iy + 1), c11 = node(ix + 1, iy + 1);
      for (float fy : {0.25f, 0.75f})
        for (float fx : {0.25f, 0.75f}) {
          const float x = (float(ix) + fx) * step, y = (float(iy) + fy) * step;
          auto lerp2 = [&](float a, float b, float c, float d) {
            return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy;
          };
          const float s = lerp2(c00.edgeS, c10.edgeS, c01.edgeS, c11.edgeS);
          const float a = lerp2(c00.edgeA, c10.edgeA, c01.edgeA, c11.edgeA);
          const float h = f.height(x, y);
          const PgVec3 n = f.normal(x, y, step);
          const auto fine = f.colorAtSplit(x, y, h, n.z < 0.0f ? -n.z : 0.0f);
          const float wi = edgeTrackWeight(s, a), wf = edgeTrackWeight(fine.edgeS, fine.edgeA);
          ++probes;
          worst = std::max(worst, double(std::fabs(wi - wf)));
          if (std::fabs(wi - wf) > 0.05f)
            ++bad;
          if (fine.edgeA >= kEdgeFar && wi > 0.0f)
            ++falseLine; // a track where no border is within 8 m
        }
    }
  qInfo("edge track nodes: %d probes, %d off by >0.05 (worst %.3f), %d false lines", probes, bad, worst, falseLine);
  // A parcel's tracked-border choice can switch inside one cell, so a handful of
  // probes next to such a cusp may differ; a sign-jump line would be thousands.
  QVERIFY(falseLine * 10000 <= probes);
  QVERIFY(bad * 100 <= probes);
}

void TstProcgen::fieldMeshColoursVary() {
  for (int pal = 0; pal <= 9; ++pal) {
    FieldParams p;
    p.seed = 99u;
    p.palette = pal;
    p.heightM = 20.0f;
    const TerrainField f(p);
    const ProcMesh m = meshFieldChunk(f, 0, 0, 200.0f, 64);
    float lo = 9.0f, hi = -9.0f;
    for (const PgVec3 &c : m.colors) {
      QVERIFY(std::isfinite(c.x + c.y + c.z));
      lo = std::min(lo, luma(c));
      hi = std::max(hi, luma(c));
    }
    QVERIFY2(hi - lo > 0.08f, qPrintable(QString("palette %1 mesh is one colour").arg(pal)));
  }
}

// A pad is a rigid disc (deck 0.9 m above the centre surface, skirt 1.3 m below):
// the meshed surface under its whole footprint must stay within 0.6 m of the
// centre, or the pad floats / is buried. Counts the violations the old
// single-point screen let through (footprint screen off) and requires none now.
void TstProcgen::helipadFootprintsAreLevel() {
  const float kMaxRelief = 0.6f;
  int oldViolations = 0, total = 0;
  for (const char *id : {"alpine", "hills", "flat", "desert"})
    for (uint32_t seed : {1337u, 42u, 9001u}) {
      PresetData d;
      QVERIFY(buildWorldPreset(id, seed, d));
      const TerrainField f(d.field);
      const HeightFn surf = [&](float x, float y) {
        return streamSurfaceHeight(f, x, y);
      };
      const HeightFn raw = [&](float x, float y) { return f.height(x, y); };

      HelipadParams oldP; // old behaviour: no footprint screen
      oldP.maxReliefM = 0.0f;
      for (const PgVec3 &p : helipadsAround(surf, raw, 0.0f, 0.0f, 350.0f, oldP))
        if (helipadRelief(surf, p.x, p.y, oldP.radiusM) > kMaxRelief)
          ++oldViolations;

      const std::vector<PgVec3> pads =
          helipadsAround(surf, raw, 0.0f, 0.0f, 350.0f);
      QVERIFY(!pads.empty());
      QCOMPARE(pads[0].x, 0.0f); // home pad first, at the origin
      QCOMPARE(pads[0].y, 0.0f);
      for (const PgVec3 &p : pads) {
        ++total;
        const float rel = helipadRelief(surf, p.x, p.y, 3.0f);
        QVERIFY2(rel <= kMaxRelief,
                 qPrintable(QString("%1 seed %2 pad (%3,%4) relief %5 m")
                                .arg(id).arg(seed).arg(p.x).arg(p.y).arg(rel)));
        QVERIFY(std::fabs(p.z - surf(p.x, p.y)) < 1e-4f);
      }
    }
  qInfo("helipads checked: %d; footprint violations without the screen: %d",
        total, oldViolations);
  QVERIFY2(oldViolations > 0, "fixture no longer exercises the screen");
}


// Fine relief adds hummocks of at most reliefM on top of the field, with a
// gentle slope, leaves the spawn pad level, and does nothing unless asked for
// (flat, racing, yard and dune worlds keep their surface exactly).
void TstProcgen::fineReliefIsBoundedGentleAndOptIn() {
  FieldParams base;
  base.seed = 77u;
  base.heightM = 12.0f;
  base.mountainMix = 0.0f;
  base.flatRadiusM = 10.0f;
  FieldParams rel = base;
  rel.reliefM = 0.3f;
  const TerrainField fb(base), fr(rel), fr2(rel);
  float maxDiff = 0.0f, maxSlope = 0.0f, minDiff = 1.0f;
  for (int ix = 0; ix <= 162; ++ix)
    for (int iy = 0; iy <= 162; ++iy) {
      const float x = -300.0f + 3.7f * static_cast<float>(ix);
      const float y = -300.0f + 3.7f * static_cast<float>(iy);
      const float d = fr.height(x, y) - fb.height(x, y);
      maxDiff = std::max(maxDiff, d);
      minDiff = std::min(minDiff, d);
      QCOMPARE(fr.height(x, y), fr2.height(x, y)); // deterministic
      const float s = std::fabs(fr.height(x + 1.0f, y) - fr.height(x, y)) -
                      std::fabs(fb.height(x + 1.0f, y) - fb.height(x, y));
      maxSlope = std::max(maxSlope, s);
    }
  QVERIFY(minDiff >= -1e-5f);                 // never digs below the field
  QVERIFY(maxDiff <= 0.3f + 1e-4f);           // within reliefM
  QVERIFY(maxDiff > 0.1f);                    // and it actually shows
  QVERIFY(maxSlope < 0.1f);                   // adds less than 0.1 rise/run
  QCOMPARE(fr.height(0.0f, 0.0f), fb.height(0.0f, 0.0f)); // pad stays level
  QCOMPARE(fr.height(6.0f, 3.0f), fb.height(6.0f, 3.0f));
  // opt-in: default params and a dune world are untouched
  FieldParams dune = rel;
  dune.shape = 1;
  FieldParams duneOff = dune;
  duneOff.reliefM = 0.0f;
  const TerrainField fd(dune), fdo(duneOff);
  for (int i = 0; i <= 17; ++i) {
    const float x = -80.0f + 9.1f * static_cast<float>(i);
    QCOMPARE(fd.height(x, 5.0f), fdo.height(x, 5.0f));
  }
  // the presets: only the hilly ones ask for relief
  for (const char *id : {"flat", "racing", "yard", "desert"}) {
    PresetData d;
    QVERIFY(buildWorldPreset(id, 5u, d));
    QCOMPARE(d.field.reliefM, 0.0f);
  }
  for (const char *id : {"hills", "alpine", "forest", "meadow"}) {
    PresetData d;
    QVERIFY(buildWorldPreset(id, 5u, d));
    QVERIFY(d.field.reliefM > 0.0f && d.field.reliefM <= 0.3f);
  }
}

// The solid the physics lands on: deck cap + wall per pad, each triangle in both
// windings, and only the pads of the window (plus the footprint's margin).
void TstProcgen::padCollisionSoupHasBothWindingsAndOnlyTheWindowsPads() {
  const float R = 3.0f, deck = 0.9f;
  // z is the surface elevation (up); the soup is in NED, so the deck is at
  // -z - deck and the terrain under it at -z.
  const std::vector<PgVec3> pads = {{10.0f, 20.0f, 50.0f},   // inside
                                    {-2.0f, 20.0f, 40.0f},   // 2 m left of the window: inside by the margin
                                    {-10.0f, 20.0f, 40.0f}}; // 10 m left: outside
  const std::vector<PgVec3> soup =
      helipadCollisionSoup(pads, 0.0f, 0.0f, 100.0f, 100.0f, R, deck, 20);
  // 20 segments x 3 triangles x 2 windings x 3 vertices, for two pads.
  QCOMPARE(soup.size(), std::size_t(2 * 20 * 3 * 2 * 3));
  bool sawDeck = false, sawBase = false;
  for (std::size_t i = 0; i + 5 < soup.size(); i += 6) {
    // (a, b, c) then (a, c, b): the reverse winding of the same triangle.
    QCOMPARE(soup[i + 3].x, soup[i].x);
    QCOMPARE(soup[i + 4].x, soup[i + 2].x);
    QCOMPARE(soup[i + 5].x, soup[i + 1].x);
    for (std::size_t k = 0; k < 6; ++k) {
      const float z = soup[i + k].z;
      sawDeck = sawDeck || std::fabs(z - (-50.0f - deck)) < 1e-4f;
      sawBase = sawBase || std::fabs(z - (-50.0f)) < 1e-4f;
    }
  }
  QVERIFY(sawDeck && sawBase);
  // Nothing from the pad outside the margin: every vertex is within R of a
  // pad centre that is inside.
  for (const PgVec3 &v : soup)
    QVERIFY(v.x > -2.0f - R - 1e-3f);
  QVERIFY(helipadCollisionSoup({}, 0, 0, 1, 1, R, deck).empty());
}

// The pad search runs on the collision worker, not the GUI thread, so it must
// give the same pads wherever and however often it runs at once.
void TstProcgen::padSearchIsTheSameOnAnyThread() {
  FieldParams fp;
  fp.seed = 7u;
  const TerrainField f(fp);
  const HeightFn surface = [&f](float x, float y) {
    return surfaceHeight(f, x, y, 128.0f, 16);
  };
  const HeightFn raw = [&f](float x, float y) { return f.height(x, y); };
  auto search = [&] { return helipadsAround(surface, raw, 40.0f, -30.0f, 300.0f); };
  const std::vector<PgVec3> here = search();
  QVERIFY2(here.size() > 1, "the test world has no pads to compare");
  std::vector<PgVec3> a, b;
  std::thread ta([&] { a = search(); });
  std::thread tb([&] { b = search(); });
  std::vector<PgVec3> c = search(); // all three at once
  ta.join();
  tb.join();
  for (const std::vector<PgVec3> *other : {&a, &b, &c}) {
    QCOMPARE(other->size(), here.size());
    for (std::size_t i = 0; i < here.size(); ++i) {
      QCOMPARE((*other)[i].x, here[i].x);
      QCOMPARE((*other)[i].y, here[i].y);
      QCOMPARE((*other)[i].z, here[i].z);
    }
  }
}

// One window for a pad's whole ring must answer what a window per sample does.
void TstProcgen::surfaceWindowIsTheSurfaceHeight() {
  const float chunkM = 192.0f;
  const int res = 100;
  float worst = 0.0f;
  int n = 0;
  for (uint32_t seed : {3u, 7u, 21u}) {
    FieldParams fp;
    fp.seed = seed;
    const TerrainField f(fp);
    // Centres everywhere, including on and beside chunk edges (where the ring
    // crosses into the next chunk's lattice) and far from the origin.
    const float cs[][2] = {{0.0f, 0.0f},      {95.3f, -40.1f},  {191.0f, 5.0f},
                           {192.0f, 192.0f},  {-1.0f, -190.5f}, {383.5f, 100.0f},
                           {-2004.7f, 1388.2f}, {5.4f, 575.9f}};
    for (const auto &c : cs) {
      const float R = 3.0f;
      const SurfaceWindow w(f, c[0], c[1], R, chunkM, res);
      for (int a = 0; a <= 16; ++a) {
        const float t = 6.2831853f * float(a) / 16.0f;
        const float x = a == 16 ? c[0] : c[0] + R * std::cos(t);
        const float y = a == 16 ? c[1] : c[1] + R * std::sin(t);
        const float want = surfaceHeight(f, x, y, chunkM, res);
        worst = std::max(worst, std::fabs(w.at(x, y) - want));
        ++n;
      }
      // Outside its window it still answers (by falling back), correctly.
      QCOMPARE(w.at(c[0] + 60.0f, c[1]),
               surfaceHeight(f, c[0] + 60.0f, c[1], chunkM, res));
    }
  }
  qInfo("surface window vs surfaceHeight: %d samples, worst diff %.3g m", n, worst);
  QVERIFY2(worst < 1e-3f, "the window is not the surface surfaceHeight gives");
}

// The pad list with the window hook is the list without it.
void TstProcgen::footprintWindowChangesNoPad() {
  const float chunkM = 192.0f;
  const int res = 100;
  int total = 0;
  for (uint32_t seed : {3u, 7u, 21u}) {
    FieldParams fp;
    fp.seed = seed;
    const TerrainField f(fp);
    const HeightFn surface = [&](float x, float y) {
      return surfaceHeight(f, x, y, chunkM, res);
    };
    const HeightFn raw = [&](float x, float y) { return f.height(x, y); };
    HelipadParams plain;
    HelipadParams windowed;
    windowed.footprintSurface = [&](float x, float y, float r) {
      const auto w = std::make_shared<const SurfaceWindow>(f, x, y, r, chunkM, res);
      return HeightFn([w](float a, float b) { return w->at(a, b); });
    };
    for (const auto &c : {std::pair<float, float>{0.0f, 0.0f},
                          std::pair<float, float>{480.0f, -300.0f}}) {
      const auto a = helipadsAround(surface, raw, c.first, c.second, 260.0f, plain);
      const auto b = helipadsAround(surface, raw, c.first, c.second, 260.0f, windowed);
      QCOMPARE(b.size(), a.size());
      for (std::size_t i = 0; i < a.size(); ++i) {
        QCOMPARE(b[i].x, a[i].x);
        QCOMPARE(b[i].y, a[i].y);
        QVERIFY(std::fabs(b[i].z - a[i].z) < 1e-4f);
      }
      total += int(a.size());
    }
  }
  QVERIFY2(total > 6, "the test worlds have no pads to compare");
}

QTEST_MAIN(TstProcgen)
#include "tst_procgen.moc"
