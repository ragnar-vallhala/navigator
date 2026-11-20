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

using Key = std::tuple<float, float, int, int, int, float>;
std::vector<Key> keysOf(const std::vector<VegInstance> &v) {
  std::vector<Key> k;
  for (const VegInstance &i : v)
    k.emplace_back(i.pos.x, i.pos.y, i.species, i.variant, i.kind, i.scale);
  std::sort(k.begin(), k.end());
  return k;
}

// Trees of a scatter over [0, side)^2 in 256 m chunks.
std::vector<VegInstance> treesOver(int palette, float side, uint32_t seed = 11u) {
  const TerrainField f = makeField(palette, seed);
  const VegParams p = vegParamsForPalette(palette);
  std::vector<VegInstance> trees;
  const int n = static_cast<int>(side / 256.0f);
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i)
      for (const VegInstance &v : scatterVegetation(f, i, j, 256.0f, p))
        if (v.species == kVegTree)
          trees.push_back(v);
  return trees;
}

// Mean over trees of the share of the other trees within `r` metres that are of
// the tree's own kind (or conifer/broadleaf group when `byGroup`); labels are
// taken from `lab`, positions from `t`. Trees without neighbours are skipped.
double sameLabelShare(const std::vector<VegInstance> &t, const std::vector<int> &lab,
                      float r) {
  double sum = 0;
  int cnt = 0;
  for (std::size_t a = 0; a < t.size(); ++a) {
    int same = 0, all = 0;
    for (std::size_t b = 0; b < t.size(); ++b) {
      if (a == b)
        continue;
      const float dx = t[a].pos.x - t[b].pos.x, dy = t[a].pos.y - t[b].pos.y;
      if (dx * dx + dy * dy > r * r)
        continue;
      ++all;
      same += lab[a] == lab[b];
    }
    if (all >= 3) {
      sum += double(same) / all;
      ++cnt;
    }
  }
  return cnt ? sum / cnt : 0.0;
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
  void treeSpeciesAreFullSize();
  void treesAreMixedAndClumped();
  void treesHaveGapsAndMixedZones();
  void crownsSetTheSpacing();
  void collidersFollowTheDrawnSize();
  void paletteSpeciesShares();
  void forestHasTwoConifersAndTwoBroadleafs();
  void grassIsTintedPerPalette();
  void coniferShareFollowsPalette();
  void treeTintSpreads();
  void rocksRestOnTheRenderedMesh();
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
    // The mesh lattice is the same 2.56 m grid either way (50 cells per 128 m
    // quadrant, 100 per 256 m chunk).
    VegParams pq = p;
    pq.meshRes = p.meshRes / 2;
    std::vector<VegInstance> parts;
    for (int j = 0; j <= 1; ++j)
      for (int i = 0; i <= 1; ++i) {
        const auto s = scatterVegetation(f, i, j, c, pq);
        parts.insert(parts.end(), s.begin(), s.end());
      }
    const auto whole = scatterVegetation(f, 0, 0, 2.0f * c, p);
    QVERIFY(!whole.empty());
    QCOMPARE(parts.size(), whole.size());
    QVERIFY(keysOf(parts) == keysOf(whole));
    // Heights are chunk-independent too (to float noise: each chunk samples
    // the mesh lattice from its own origin).
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
    const std::vector<float> zp = zs(parts), zw = zs(whole);
    QCOMPARE(zp.size(), zw.size());
    for (std::size_t k = 0; k < zp.size(); ++k)
      QVERIFY2(std::fabs(zp[k] - zw[k]) < 2e-3f,
               qPrintable(QString("instance %1: %2 vs %3").arg(k).arg(zp[k]).arg(zw[k])));
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
  in.back().kind = 2;
  add(kVegBush, 2.0f);
  add(kVegGrassClump, 2.0f);
  add(kVegRock, 0.5f);
  add(kVegRock, 2.0f);
  const auto c = vegetationColliders(in);
  QCOMPARE(c.size(), std::size_t(2));
  QVERIFY(!c[0].isRock);
  QCOMPARE(c[0].radius, vegTreeKind(2).trunkRadiusM * 1.5f);
  QCOMPARE(c[0].height, vegTreeKind(2).heightM * 1.5f);
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

void TstVegetation::treeSpeciesAreFullSize() {
  int conifers = 0, broadleaf = 0;
  for (int k = 0; k < kVegTreeKindCount; ++k) {
    const VegTreeKind &t = vegTreeKind(k);
    (t.conifer ? conifers : broadleaf)++;
    QVERIFY2(t.heightM * t.scaleMin >= 8.0f, t.id);
    QVERIFY2(t.heightM * t.scaleMax <= 20.0f, t.id);
    QVERIFY2(t.scaleMax > 1.2f * t.scaleMin, t.id); // real size variation
    QVERIFY2(t.trunkRadiusM > 0.1f && t.trunkRadiusM < t.crownRadiusM, t.id);
    QCOMPARE(vegTreeKindIndex(t.id), k);
  }
  QVERIFY(conifers >= 2);
  QVERIFY(broadleaf >= 2);
  QCOMPARE(vegTreeKindIndex("no_such_tree"), -1);

  // Every placed tree is 8-20 m tall and each kind actually occurs.
  const auto trees = treesOver(kPalForest, 512.0f);
  QVERIFY(trees.size() > 500);
  std::vector<int> perKind(kVegTreeKindCount, 0);
  float lo = 1e9f, hi = 0.0f;
  for (const VegInstance &v : trees) {
    QVERIFY(v.kind < kVegTreeKindCount);
    perKind[v.kind]++;
    const float h = vegTreeKind(v.kind).heightM * v.scale;
    lo = std::min(lo, h);
    hi = std::max(hi, h);
    QVERIFY2(h >= 8.0f && h <= 20.0f, qPrintable(QString::number(h)));
  }
  qInfo("forest tree heights %.1f .. %.1f m; per kind %d %d %d %d", lo, hi, perKind[0],
        perKind[1], perKind[2], perKind[3]);
  for (int n : perKind)
    QVERIFY(n > int(trees.size()) / 20);
  QVERIFY(lo < 11.0f && hi > 17.0f); // the range is used, not one size
}

void TstVegetation::treesAreMixedAndClumped() {
  // Stands of one species cluster: a tree's neighbours share its kind (and its
  // conifer/broadleaf group) clearly more often than if the same kinds were
  // scattered at random over the same positions.
  const auto trees = treesOver(kPalForest, 768.0f);
  QVERIFY(trees.size() > 1000);
  std::vector<int> kinds, groups;
  for (const VegInstance &v : trees) {
    kinds.push_back(v.kind);
    groups.push_back(vegTreeKind(v.kind).conifer ? 1 : 0);
  }
  auto shuffled = [](std::vector<int> l) {
    uint32_t s = 12345u;
    for (std::size_t i = l.size(); i > 1; --i) {
      s = s * 1664525u + 1013904223u;
      std::swap(l[i - 1], l[(s >> 8) % i]);
    }
    return l;
  };
  const float r = 30.0f;
  const double kindReal = sameLabelShare(trees, kinds, r);
  const double kindRand = sameLabelShare(trees, shuffled(kinds), r);
  const double grpReal = sameLabelShare(trees, groups, r);
  const double grpRand = sameLabelShare(trees, shuffled(groups), r);
  qInfo("same-kind share within %.0f m: stands %.3f vs uniform %.3f (index %.2f); "
        "same-group %.3f vs %.3f (index %.2f)",
        r, kindReal, kindRand, kindReal / kindRand, grpReal, grpRand, grpReal / grpRand);
  QVERIFY2(kindReal > 1.4 * kindRand, "species do not clump");
  QVERIFY2(grpReal > 1.25 * grpRand, "conifer/broadleaf stands do not clump");
  // ... yet the land is mixed, not two big blocks.
  QVERIFY(kindReal < 0.97);
}

void TstVegetation::treesHaveGapsAndMixedZones() {
  const auto trees = treesOver(kPalForest, 768.0f);
  // Clearings: share of 24 m cells without a single tree.
  const float cell = 24.0f;
  const int n = static_cast<int>(768.0f / cell);
  std::vector<int> occ(static_cast<size_t>(n) * static_cast<size_t>(n), 0);
  for (const VegInstance &v : trees) {
    const int i = std::min(n - 1, std::max(0, int(v.pos.x / cell)));
    const int j = std::min(n - 1, std::max(0, int(v.pos.y / cell)));
    occ[j * n + i]++;
  }
  const int empty = int(std::count(occ.begin(), occ.end(), 0));
  const double emptyShare = double(empty) / (n * n);
  qInfo("empty 24 m cells: %.1f %%", 100.0 * emptyShare);
  QVERIFY2(emptyShare > 0.04, "no clearings");
  QVERIFY2(emptyShare < 0.45, "too open for a forest");
  // Mixed zones: trees that have both a conifer and a broadleaf within 25 m.
  int mixed = 0;
  for (std::size_t a = 0; a < trees.size(); a += 3) {
    bool c = false, b = false;
    for (std::size_t o = 0; o < trees.size(); ++o) {
      const float dx = trees[a].pos.x - trees[o].pos.x, dy = trees[a].pos.y - trees[o].pos.y;
      if (dx * dx + dy * dy > 25.0f * 25.0f)
        continue;
      (vegTreeKind(trees[o].kind).conifer ? c : b) = true;
    }
    mixed += c && b;
  }
  const double mixedShare = double(mixed) / std::ceil(double(trees.size()) / 3.0);
  qInfo("trees with both groups within 25 m: %.1f %%", 100.0 * mixedShare);
  QVERIFY2(mixedShare > 0.05, "no mixed zones");
  QVERIFY2(mixedShare < 0.85, "no pure stands");
}

void TstVegetation::crownsSetTheSpacing() {
  const VegParams p = vegParamsForPalette(kPalForest);
  const auto trees = treesOver(kPalForest, 512.0f);
  QVERIFY(trees.size() > 500);
  float minRatio = 1e9f;
  for (std::size_t a = 0; a < trees.size(); ++a)
    for (std::size_t b = a + 1; b < trees.size(); ++b) {
      const float d = std::hypot(trees[a].pos.x - trees[b].pos.x,
                                 trees[a].pos.y - trees[b].pos.y);
      if (d > 30.0f)
        continue;
      const float ca = vegTreeKind(trees[a].kind).crownRadiusM * trees[a].scale;
      const float cb = vegTreeKind(trees[b].kind).crownRadiusM * trees[b].scale;
      const float need = std::min(
          std::max(p.treeMinSpacingM, p.stands.crownSpacing * (ca + cb)), 12.0f);
      QVERIFY2(d >= need - 1e-3f, qPrintable(QString("d %1 < need %2").arg(d).arg(need)));
      minRatio = std::min(minRatio, d / (ca + cb));
    }
  qInfo("closest trunk spacing / crown radii sum: %.2f", minRatio);
}

void TstVegetation::collidersFollowTheDrawnSize() {
  const auto trees = treesOver(kPalForest, 256.0f);
  QVERIFY(trees.size() > 100);
  const auto cs = vegetationColliders(trees);
  QCOMPARE(cs.size(), trees.size());
  for (std::size_t i = 0; i < trees.size(); ++i) {
    const VegTreeKind &k = vegTreeKind(trees[i].kind);
    QCOMPARE(cs[i].height, k.heightM * trees[i].scale);
    QCOMPARE(cs[i].radius, k.trunkRadiusM * trees[i].scale);
    QVERIFY(cs[i].height >= 8.0f && cs[i].height <= 20.0f);
    QVERIFY(cs[i].radius > 0.1f && cs[i].radius < 1.5f); // a trunk, not a sapling
    QVERIFY(!cs[i].isRock);
  }
}

void TstVegetation::paletteSpeciesShares() {
  auto conifShare = [](int pal) {
    const auto t = treesOver(pal, 1024.0f, 5u);
    int c = 0;
    for (const VegInstance &v : t)
      c += vegTreeKind(v.kind).conifer;
    return std::make_pair(t.size(), t.empty() ? 0.0 : double(c) / t.size());
  };
  const auto alpine = conifShare(kPalAlpine);
  const auto meadow = conifShare(kPalMeadow);
  const auto forest = conifShare(kPalForest);
  qInfo("conifer share: alpine %.2f (%zu), forest %.2f (%zu), meadow %.2f (%zu)",
        alpine.second, alpine.first, forest.second, forest.first, meadow.second,
        meadow.first);
  QVERIFY(alpine.first > 50 && meadow.first > 20);
  QVERIFY(alpine.second > forest.second);
  QVERIFY(forest.second > meadow.second);
  QVERIFY(alpine.second > 0.7);
  QVERIFY(meadow.second < 0.4);
}

// The forest world must put real conifer stands in front of the player, not only
// allow them: every kind appears, conifers are a meaningful share.
void TstVegetation::forestHasTwoConifersAndTwoBroadleafs() {
  const TerrainField f = makeField(kPalForest);
  const VegParams p = vegParamsForPalette(kPalForest);
  int n[kVegTreeKindCount] = {0}, total = 0;
  for (int j = -2; j <= 1; ++j)
    for (int i = -2; i <= 1; ++i)
      for (const VegInstance &v : scatterVegetation(f, i, j, 256.0f, p))
        if (v.species == kVegTree) { ++n[v.kind]; ++total; }
  QVERIFY(total > 200);
  int conifers = 0;
  for (int k = 0; k < kVegTreeKindCount; ++k) {
    QVERIFY2(n[k] > total / 40, qPrintable(QString("kind %1: %2 of %3").arg(k).arg(n[k]).arg(total)));
    if (vegTreeKind(k).conifer)
      conifers += n[k];
  }
  const double share = double(conifers) / total;
  QVERIFY2(share > 0.15 && share < 0.85, qPrintable(QString("conifer share %1").arg(share)));
}

// The conifer share of the trees that stand follows the palette's
// stands.coniferShare (crown spacing and clumping move it, not far), and where
// it is high both conifer kinds are common, so a chase view is not mostly
// broadleaf.
void TstVegetation::coniferShareFollowsPalette() {
  std::vector<std::pair<double, double>> shares; // set share, measured share
  for (int pal : {int(kPalForest), int(kPalAlpine), int(kPalHills), int(kPalMeadow), int(kPalPasture)}) {
    const TerrainField f = makeField(pal);
    const VegParams p = vegParamsForPalette(pal);
    int n[kVegTreeKindCount] = {0}, total = 0;
    for (int j = -2; j <= 1; ++j)
      for (int i = -2; i <= 1; ++i)
        for (const VegInstance &v : scatterVegetation(f, i, j, 256.0f, p))
          if (v.species == kVegTree) { ++n[v.kind]; ++total; }
    QVERIFY2(total > 40, qPrintable(QString("palette %1: %2 trees").arg(pal).arg(total)));
    const int conifers = n[0] + n[1];
    const double share = double(conifers) / total;
    qInfo("palette %d: coniferShare %.2f measured %.2f (fir %d pine %d jac %d isl %d)", pal,
          double(p.stands.coniferShare), share, n[0], n[1], n[2], n[3]);
    // Conifers pack tighter (smaller crowns), so the count runs above the area share.
    QVERIFY2(share > double(p.stands.coniferShare) - 0.15 &&
                 share < double(p.stands.coniferShare) + 0.40,
             qPrintable(QString("palette %1: share %2 vs %3").arg(pal).arg(share).arg(double(p.stands.coniferShare))));
    shares.emplace_back(double(p.stands.coniferShare), share);
    if (p.stands.coniferShare >= 0.35f && conifers > 40) {
      QVERIFY2(n[0] > conifers / 4 && n[1] > conifers / 4,
               qPrintable(QString("palette %1: fir %2 pine %3").arg(pal).arg(n[0]).arg(n[1])));
    }
  }
  // A palette with a clearly higher setting measures clearly higher.
  for (const auto &x : shares)
    for (const auto &y : shares)
      if (x.first >= y.first + 0.3)
        QVERIFY2(x.second > y.second,
                 qPrintable(QString("set %1/%2 measured %3/%4").arg(x.first).arg(y.first).arg(x.second).arg(y.second)));
}

// Each tree's colour factor is a hash of its cell: it spreads over 0..1, so the
// crowns of a stand do not share one green.
void TstVegetation::treeTintSpreads() {
  const TerrainField f = makeField(kPalForest);
  const VegParams p = vegParamsForPalette(kPalForest);
  double sum = 0, sum2 = 0;
  int n = 0, lo = 0, hi = 0;
  for (int j = -1; j <= 0; ++j)
    for (int i = -1; i <= 0; ++i)
      for (const VegInstance &v : scatterVegetation(f, i, j, 256.0f, p))
        if (v.species == kVegTree) {
          sum += v.tint; sum2 += double(v.tint) * v.tint; ++n;
          lo += v.tint < 0.25f; hi += v.tint > 0.75f;
        }
  QVERIFY(n > 100);
  const double mean = sum / n, sd = std::sqrt(sum2 / n - mean * mean);
  QVERIFY2(sd > 0.22, qPrintable(QString("tint sd %1").arg(sd)));
  QVERIFY(lo > n / 8 && hi > n / 8);
}

// Grass tufts carry the palette's own ground hue to tint toward; it differs
// between palettes and matches TerrainField's ground colour; other species
// carry none.
void TstVegetation::grassIsTintedPerPalette() {
  uint32_t seen[3] = {0, 0, 0};
  int idx = 0;
  for (int pal : {int(kPalForest), int(kPalDesert), int(kPalPasture)}) {
    const TerrainField f = makeField(pal);
    const VegParams p = vegParamsForPalette(pal);
    FieldParams fp;
    fp.palette = pal;
    const PgVec3 ground = TerrainField(fp).color(0.0f, 1.0f);
    QCOMPARE(p.groundColor.x, ground.x);
    QCOMPARE(p.groundColor.z, ground.z);
    int grass = 0;
    for (int j = -1; j <= 0; ++j)
      for (int i = -1; i <= 0; ++i)
        for (const VegInstance &v : scatterVegetation(f, i, j, 256.0f, p)) {
          if (v.species == kVegGrassClump) {
            QCOMPARE(v.paint, vegPackHue(ground));
            ++grass;
          } else {
            QCOMPARE(v.paint, 0u);
          }
        }
    QVERIFY2(grass > 20, qPrintable(QString("palette %1: %2 grass").arg(pal).arg(grass)));
    seen[idx++] = vegPackHue(ground);
  }
  QVERIFY(seen[0] != seen[1] && seen[1] != seen[2] && seen[0] != seen[2]);
}

// Rocks stand on the terrain mesh the pilot sees (meshFieldChunk: slope cap
// included, which the raw field can sit metres above on steep ground), on the
// lowest point under their tilted base (they lean toward the slope). Checked with surfaceHeight(), the mesh's
// own height function, over the rock's nominal footprint circle (half the span
// times the instance scale; tolerance 1 cm for float noise).
void TstVegetation::rocksRestOnTheRenderedMesh() {
  struct World {
    const char *name;
    FieldParams fp;
    int minRocks;
  };
  std::vector<World> worlds;
  { // flat field, meadow palette (the flat palette scatters no rocks)
    FieldParams f;
    f.heightM = 1.2f;
    f.mountainMix = 0.0f;
    f.maxSlope = 0.3f;
    f.palette = kPalMeadow;
    worlds.push_back({"flat", f, 3});
  }
  { // the hills preset's relief
    FieldParams f;
    f.heightM = 55.0f;
    f.featureM = 130.0f;
    f.mountainMix = 0.35f;
    f.mountainBias = 0.6f;
    f.maxSlope = 0.5f;
    f.palette = kPalHills;
    f.reliefM = 0.25f;
    worlds.push_back({"hills", f, 10});
  }
  { // the alpine preset's relief: steep, capped at 0.9
    FieldParams f;
    f.heightM = 110.0f;
    f.featureM = 200.0f;
    f.mountainMix = 0.85f;
    f.mountainBias = 0.15f;
    f.maxSlope = 0.9f;
    f.palette = kPalAlpine;
    f.reliefM = 0.3f;
    f.flatRadiusM = 12.0f;
    worlds.push_back({"alpine", f, 30});
  }
  const float K = kStreamChunkM;
  for (World &w : worlds) {
    w.fp.seed = 21u;
    const TerrainField f(w.fp);
    const VegParams p = vegParamsForPalette(w.fp.palette);
    int rocks = 0, steep = 0, tilted = 0;
    for (int cy = -1; cy <= 0; ++cy)
      for (int cx = -1; cx <= 0; ++cx)
        for (const VegInstance &v : scatterVegetation(f, cx, cy, K, p)) {
          if (v.species != kVegRock)
            continue;
          ++rocks;
          const float base = -v.pos.z, r = kVegRockRadiusM * v.scale;
          const float tc = surfaceHeight(f, v.pos.x, v.pos.y, K, kStreamChunkRes);
          // The rock's base is a plane through (pos, base) tilted by its
          // gradient. plane - surface over the footprint: its maximum is the
          // hover at the lowest-touching point, its minimum the burial.
          float hover = -1e9f, burial = 1e9f;
          for (int ring = 0; ring <= 2; ++ring)
            for (int k = 0; k < (ring ? 8 : 1); ++k) {
              const float a = 0.785398f * static_cast<float>(k);
              const float rr = r * 0.5f * static_cast<float>(ring);
              const float dx = rr * std::cos(a), dy = rr * std::sin(a);
              const float plane = base + v.tiltX * dx + v.tiltY * dy;
              const float d =
                  plane - surfaceHeight(f, v.pos.x + dx, v.pos.y + dy, K, kStreamChunkRes);
              hover = std::max(hover, d);
              burial = std::min(burial, d);
            }
          const QString tag = QString("%1 rock at %2,%3 scale %4: base %5, centre %6, tilt %7,%8, hover %9, burial %10")
                                  .arg(w.name).arg(v.pos.x).arg(v.pos.y).arg(v.scale)
                                  .arg(base).arg(tc).arg(v.tiltX).arg(v.tiltY)
                                  .arg(hover).arg(burial);
          // Rests in the ground (the tilted base is under the mesh somewhere)...
          QVERIFY2(burial <= 0.01f, qPrintable(tag));
          // ...is not buried past the cap at its centre (it must stay visible)...
          QVERIFY2(tc - base <= kVegRockMaxSinkM * v.scale + kVegRockSinkM + 0.01f,
                   qPrintable(tag));
          // ...and hovers over its lowest-touching point by no more than the allowance.
          QVERIFY2(hover <= kVegRockMaxToeGapM * v.scale + 0.01f, qPrintable(tag));
          QVERIFY2(std::fabs(v.tiltX) <= kVegTiltMax && std::fabs(v.tiltY) <= kVegTiltMax,
                   qPrintable(tag));
          tilted += (v.tiltX != 0.0f || v.tiltY != 0.0f);
          steep += (std::hypot(v.tiltX, v.tiltY) > 0.2f);
        }
    QVERIFY2(rocks >= w.minRocks, qPrintable(QString("%1: %2 rocks").arg(w.name).arg(rocks)));
    if (QByteArray(w.name) == "alpine") {
      QVERIFY2(steep > 5, "the alpine sample must include tilted rocks");
      QVERIFY2(tilted > rocks / 2, "most alpine rocks must be tilted to the slope");
    }
  }
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
