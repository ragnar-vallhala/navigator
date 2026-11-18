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
// tst_vegetation — vegetation scatter: determinism, chunk seams, exclusion
// zones, slope limits, palette density, colliders (Qt-free core, headless).
#include <QtTest/QtTest>

#include "procgen/GroundCover.h"
#include "procgen/TerrainField.h"
#include "procgen/Vegetation.h"

#include <algorithm>
#include <cmath>
#include <tuple>
#include <vector>

using namespace vsim::procgen;

namespace {

TerrainField makeField(int palette, uint32_t seed = 11u) {
  FieldParams fp;
  fp.seed = seed;
  fp.palette = palette;
  return TerrainField(fp);
}

using Key = std::tuple<float, float, int, int>;
std::vector<Key> keysOf(const std::vector<VegInstance> &v) {
  std::vector<Key> k;
  for (const VegInstance &i : v)
    k.emplace_back(i.pos.x, i.pos.y, i.species, i.variant);
  std::sort(k.begin(), k.end());
  return k;
}

} // namespace

class TstVegetation : public QObject {
  Q_OBJECT
private slots:
  void deterministic();
  void seamlessAcrossChunks();
  void respectsExclusions();
  void respectsSlopeLimit();
  void treesKeepMinSpacing();
  void forestDenserThanYard();
  void collidersOnlyTreesAndLargeRocks();
  void emptyForNoChunk();
};

void TstVegetation::deterministic() {
  const TerrainField f = makeField(kPalForest);
  const VegParams p = vegParamsForPalette(kPalForest);
  const auto a = scatterVegetation(f, 1, -2, 128.0f, p);
  const auto b = scatterVegetation(f, 1, -2, 128.0f, p);
  QVERIFY(!a.empty());
  QCOMPARE(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    QCOMPARE(a[i].pos.x, b[i].pos.x);
    QCOMPARE(a[i].pos.y, b[i].pos.y);
    QCOMPARE(a[i].pos.z, b[i].pos.z);
    QCOMPARE(a[i].scale, b[i].scale);
    QCOMPARE(a[i].yaw, b[i].yaw);
    QCOMPARE(int(a[i].variant), int(b[i].variant));
  }
  VegParams q = p;
  q.seed = p.seed + 1;
  QVERIFY(keysOf(scatterVegetation(f, 1, -2, 128.0f, q)) != keysOf(a));
}

void TstVegetation::seamlessAcrossChunks() {
  for (int pal : {int(kPalForest), int(kPalHills), int(kPalDesert)}) {
    const TerrainField f = makeField(pal);
    const VegParams p = vegParamsForPalette(pal);
    const float c = 128.0f;
    // One 256 m chunk vs its four 128 m quadrants.
    std::vector<VegInstance> parts;
    for (int j = 0; j <= 1; ++j)
      for (int i = 0; i <= 1; ++i) {
        const auto s = scatterVegetation(f, i, j, c, p);
        parts.insert(parts.end(), s.begin(), s.end());
      }
    const auto whole = scatterVegetation(f, 0, 0, 2.0f * c, p);
    QVERIFY(!whole.empty());
    QCOMPARE(parts.size(), whole.size());
    QVERIFY(keysOf(parts) == keysOf(whole));
    // Heights are chunk-independent too.
    auto zs = [](std::vector<VegInstance> v) {
      std::sort(v.begin(), v.end(), [](const VegInstance &a, const VegInstance &b) {
        return std::tie(a.pos.x, a.pos.y, a.species) <
               std::tie(b.pos.x, b.pos.y, b.species);
      });
      std::vector<float> z;
      for (auto &i : v)
        z.push_back(i.pos.z);
      return z;
    };
    QVERIFY(zs(parts) == zs(whole));
  }
}

void TstVegetation::respectsExclusions() {
  const TerrainField f = makeField(kPalForest);
  VegParams p = vegParamsForPalette(kPalForest);
  const auto base = scatterVegetation(f, 0, 0, 256.0f, p);
  QVERIFY(base.size() > 50);
  // Exclude a disc and a box around real instances so something is removed.
  p.exclusions.push_back(VegExclusion::disc(base[3].pos.x, base[3].pos.y, 25.0f));
  p.exclusions.push_back(VegExclusion::box(200.0f, 200.0f, 30.0f, 20.0f));
  p.clearAroundSpawnM = 12.0f;
  const auto v = scatterVegetation(f, 0, 0, 256.0f, p);
  QVERIFY(v.size() < base.size());
  for (const VegInstance &i : v) {
    for (const VegExclusion &e : p.exclusions)
      QVERIFY(!e.contains(i.pos.x, i.pos.y));
    QVERIFY(std::hypot(i.pos.x, i.pos.y) >= 12.0f);
  }
}

void TstVegetation::respectsSlopeLimit() {
  const TerrainField f = makeField(kPalHills);
  const VegParams p = vegParamsForPalette(kPalHills);
  const auto v = scatterVegetation(f, 0, 0, 256.0f, p);
  QVERIFY(v.size() > 20);
  for (const VegInstance &i : v) {
    const float e = 2.0f;
    const float dx = (f.height(i.pos.x + e, i.pos.y) - f.height(i.pos.x - e, i.pos.y)) / (2 * e);
    const float dy = (f.height(i.pos.x, i.pos.y + e) - f.height(i.pos.x, i.pos.y - e)) / (2 * e);
    const float flat = 1.0f / std::sqrt(dx * dx + dy * dy + 1.0f);
    // The scatter reads a 2 m height lattice, so allow its interpolation error.
    QVERIFY2(flat >= p.species[i.species].flatMin - 0.04f,
             qPrintable(QString("species %1 flatness %2").arg(i.species).arg(flat)));
    // Base sits on the surface (lattice interpolation error only).
    QVERIFY(std::fabs(i.pos.z + f.height(i.pos.x, i.pos.y)) < 0.5f);
  }
}

void TstVegetation::treesKeepMinSpacing() {
  const TerrainField f = makeField(kPalForest);
  const VegParams p = vegParamsForPalette(kPalForest);
  std::vector<VegInstance> trees;
  for (const VegInstance &i : scatterVegetation(f, 0, 0, 256.0f, p))
    if (i.species == kVegTree)
      trees.push_back(i);
  QVERIFY(trees.size() > 100);
  for (std::size_t a = 0; a < trees.size(); ++a)
    for (std::size_t b = a + 1; b < trees.size(); ++b)
      QVERIFY(std::hypot(trees[a].pos.x - trees[b].pos.x,
                         trees[a].pos.y - trees[b].pos.y) >= p.treeMinSpacingM - 1e-3f);
}

void TstVegetation::forestDenserThanYard() {
  const float c = 256.0f;
  const auto forest = scatterVegetation(makeField(kPalForest), 0, 0, c,
                                        vegParamsForPalette(kPalForest));
  const auto yard = scatterVegetation(makeField(kPalYard), 0, 0, c,
                                      vegParamsForPalette(kPalYard));
  const auto desert = scatterVegetation(makeField(kPalDesert), 0, 0, c,
                                        vegParamsForPalette(kPalDesert));
  auto count = [](const std::vector<VegInstance> &v, int sp) {
    return std::count_if(v.begin(), v.end(),
                         [&](const VegInstance &i) { return i.species == sp; });
  };
  QVERIFY(forest.size() > 5 * std::max<std::size_t>(yard.size(), 1));
  QVERIFY(count(forest, kVegTree) > count(yard, kVegTree));
  QVERIFY(count(forest, kVegTree) > count(desert, kVegTree));
  QVERIFY(count(desert, kVegRock) > 0);
  QCOMPARE(count(desert, kVegTree), 0);
}

void TstVegetation::collidersOnlyTreesAndLargeRocks() {
  std::vector<VegInstance> in;
  auto add = [&](VegSpecies s, float scale) {
    VegInstance v;
    v.pos = PgVec3{1.0f, 2.0f, -3.0f};
    v.species = s;
    v.scale = scale;
    in.push_back(v);
  };
  add(kVegTree, 1.5f);
  add(kVegBush, 2.0f);
  add(kVegGrassClump, 2.0f);
  add(kVegRock, 0.5f);
  add(kVegRock, 2.0f);
  const auto c = vegetationColliders(in);
  QCOMPARE(c.size(), std::size_t(2));
  QVERIFY(!c[0].isRock);
  QCOMPARE(c[0].radius, kVegTreeTrunkRadiusM * 1.5f);
  QCOMPARE(c[0].height, kVegTreeHeightM * 1.5f);
  QVERIFY(c[1].isRock);
  QCOMPARE(c[1].radius, kVegRockRadiusM * 2.0f);

  // On real scatter output nothing but trees and big rocks gets a collider.
  const auto v = scatterVegetation(makeField(kPalHills), 0, 0, 256.0f,
                                   vegParamsForPalette(kPalHills));
  std::size_t expect = 0;
  for (const VegInstance &i : v)
    expect += (i.species == kVegTree ||
               (i.species == kVegRock && i.scale >= kVegRockColliderMinScale));
  QVERIFY(expect > 0);
  QCOMPARE(vegetationColliders(v).size(), expect);
}

void TstVegetation::emptyForNoChunk() {
  const TerrainField f = makeField(kPalForest);
  const VegParams p = vegParamsForPalette(kPalForest);
  QVERIFY(scatterVegetation(f, 0, 0, 0.0f, p).empty());
  QVERIFY(scatterVegetation(f, 0, 0, -5.0f, p).empty());
  QVERIFY(vegetationColliders({}).empty());
}

QTEST_MAIN(TstVegetation)
#include "tst_vegetation.moc"
