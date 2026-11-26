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
// tst_treecolliderwindow -- the collider window around the drone and its ENV
// wire frames: NED cylinder conversion, 150 m disc, Trees/rocks flags, the
// 4096 cap keeping the nearest, golden ENV_ADD / ENV_COMMIT bytes, the rebuild
// policy (Qt-free core, headless).
#include <QtTest/QtTest>

#include "VsimTypes.h"
#include "procgen/GroundCover.h"
#include "procgen/TerrainField.h"
#include "procgen/TreeColliderWindow.h"

#include <algorithm>
#include <cmath>
#include <cstring>

using namespace vsim::procgen;

namespace {
VegCollider tree(float x, float y, float z = 0.0f, float r = 0.3f, float h = 10.0f) {
  return VegCollider{PgVec3{x, y, z}, r, h, false};
}
VegCollider rock(float x, float y, float z = 0.0f) {
  return VegCollider{PgVec3{x, y, z}, 1.0f, 1.5f, true};
}
ColliderOptions both() {
  ColliderOptions o;
  o.rocks = true;
  return o;
}
std::vector<EngineCylinder> win(const std::vector<VegCollider> &in, PgVec3 d = {},
                                float speed = 0.0f, ColliderOptions o = {}) {
  return treeColliderWindow(in, d, speed, o).cylinders;
}
float f32(const std::vector<std::uint8_t> &m, size_t byteOff) {
  float v;
  std::memcpy(&v, m.data() + byteOff, sizeof v);
  return v;
}

// What SimWorker::sendObstacles (vsim/SimWorker.cpp:211-231) hands the engine
// for a World-tab obstacle: pos, size and restitution copied one to one into
// vsim_ctl_obstacle_t. For a cylinder pos is the CENTRE in NED metres (z down),
// size.x the radius and size.z the full height (vsim/VsimTypes.h:111-115). The
// ENV cylinder (vsim_env_cyl_t) uses the same centre, so a trunk's ENV frame
// must carry the numbers the World-tab obstacle for the same base would.
struct WireCylinder {
  float pos[3];
  float size[3];
  float restitution;
};
WireCylinder sendObstaclesWire(const vsim::Obstacle &o) {
  return WireCylinder{{o.pos.x(), o.pos.y(), o.pos.z()},
                      {o.size.x(), o.size.y(), o.size.z()},
                      o.restitution};
}
// The World-tab way to say "a cylinder standing on the surface point `base`
// (NED, z down) that is `h` tall": the centre is half the height above it.
vsim::Obstacle worldTabCylinder(PgVec3 base, float r, float h) {
  vsim::Obstacle o;
  o.type = vsim::Obstacle::Cylinder;
  o.pos = QVector3D(base.x, base.y, base.z - 0.5f * h);
  o.size = QVector3D(r, r, h);
  return o;
}
} // namespace

class TstTreeColliderWindow : public QObject {
  Q_OBJECT
private slots:
  void nedConversion() {
    // Surface at z = -3 (3 m above the NED origin), 10 m tall: centre 5 m higher.
    const auto w = win({tree(5, 6, -3.0f, 0.4f, 10.0f)});
    QCOMPARE(int(w.size()), 1);
    QCOMPARE(w[0].center.x, 5.0f);
    QCOMPARE(w[0].center.y, 6.0f);
    QCOMPARE(w[0].center.z, -8.0f);
    QCOMPARE(w[0].radius, 0.4f);
    QCOMPARE(w[0].height, 10.0f);
    QVERIFY(!w[0].isRock);
    QCOMPARE(w[0].center.z + w[0].height * 0.5f, -3.0f); // the base is the ground
  }

  void treesOnlyByDefault() {
    const std::vector<VegCollider> in{tree(1, 1), rock(2, 2)};
    const auto t = win(in);
    QCOMPARE(int(t.size()), 1);
    QVERIFY(!t[0].isRock);
    QCOMPARE(int(win(in, {}, 0, both()).size()), 2);
    ColliderOptions rocksOnly;
    rocksOnly.trees = false;
    rocksOnly.rocks = true;
    const auto r = win(in, {}, 0, rocksOnly);
    QCOMPARE(int(r.size()), 1);
    QVERIFY(r[0].isRock);
  }

  void treesOffIsEmptyWindow() {
    ColliderOptions off;
    off.trees = false;
    const auto w = treeColliderWindow({tree(1, 1), tree(20, 5), rock(3, 3)}, {}, 0, off);
    QVERIFY(w.cylinders.empty()); // rocks default off too: the caller sends ENV_CLEAR
    QCOMPARE(int(w.dropped), 0);
  }

  void windowIsADiscAroundTheDrone() {
    const PgVec3 d{50, -20, 0};
    const float R = kColliderWindowRadiusM;
    const auto in = [&](float dx, float dy) {
      return !win({tree(d.x + dx, d.y + dy)}, d).empty();
    };
    QVERIFY(in(R, 0));             // on the radius: inclusive
    QVERIFY(!in(R + 0.1f, 0));
    QVERIFY(in(-R, 0));
    QVERIFY(in(0, R));
    QVERIFY(!in(0, -R - 0.1f));
    const float c = R * 0.7f;      // 0.7 R on both axes is 0.99 R away: inside
    QVERIFY(in(c, c));
    QVERIFY(!in(R * 0.75f, R * 0.75f)); // 1.06 R: outside a disc, inside a square
  }

  void speedWidensWindow() {
    const float m = 10.0f * kColliderLookaheadS, R = kColliderWindowRadiusM;
    QCOMPARE(int(win({tree(R + m - 0.1f, 0)}, {}, 10).size()), 1);
    QCOMPARE(int(win({tree(R + m + 0.1f, 0)}, {}, 10).size()), 0);
    QCOMPARE(int(win({tree(R + 5, 0)}, {}, 0).size()), 0);
    QCOMPARE(int(win({tree(R - 5, 0)}, {}, -50).size()), 1); // negative does not shrink
  }

  void orderingIsStableAndDiffable() {
    std::vector<VegCollider> in{tree(30, 5), tree(10, 9), rock(10, 2), tree(10, 2),
                                tree(20, 1)};
    const auto a = win(in, {}, 0, both());
    std::reverse(in.begin(), in.end());
    QVERIFY(a == win(in, {}, 0, both()));
    QCOMPARE(a.front().center.x, 10.0f);
    QCOMPARE(a.front().center.y, 2.0f);
    QCOMPARE(a.back().center.x, 30.0f);
    in.push_back(tree(40, 40));
    QVERIFY(win(in, {}, 0, both()) != a);
  }

  void degenerateInputs() {
    QVERIFY(win({}).empty());
    QVERIFY(win({tree(1, 1, 0, 0.0f, 5)}).empty());
    QVERIFY(win({tree(1, 1, 0, 0.3f, 0.0f)}).empty());
    const float nan = std::nanf("");
    QVERIFY(win({tree(nan, 1)}).empty()); // the engine would drop the whole message
    QVERIFY(win({tree(1, 1, 0, nan, 5)}).empty());
    QVERIFY(win({tree(1, 1, nan)}).empty());
    QCOMPARE(int(win({tree(kColliderWindowRadiusM, 0)}, {}, nan).size()), 1); // NaN speed: no widening
  }

  // Over the cap, the nearest kept, the farthest counted as dropped.
  void capKeepsNearestAndCountsDrops() {
    std::vector<VegCollider> in;
    for (int i = 0; i < 10; ++i)
      in.push_back(tree(float(i + 1), 0)); // distance 1..10 from the origin
    in.push_back(tree(0, -0.5f));          // nearest of all
    ColliderOptions o;
    o.cap = 4;
    const ColliderWindow w = treeColliderWindow(in, {}, 0, o);
    QCOMPARE(int(w.cylinders.size()), 4);
    QCOMPARE(int(w.dropped), 7);
    std::vector<float> xs;
    for (const EngineCylinder &c : w.cylinders)
      xs.push_back(c.center.x);
    QCOMPARE(xs, (std::vector<float>{0.0f, 1.0f, 2.0f, 3.0f})); // sorted by x
    // The drone's position, not the origin, decides what is near.
    const ColliderWindow v = treeColliderWindow(in, PgVec3{10, 0, 0}, 0, o);
    QCOMPARE(v.cylinders.front().center.x, 7.0f);
    QCOMPARE(v.cylinders.back().center.x, 10.0f);
    // At or under the cap nothing is dropped.
    o.cap = 11;
    QCOMPARE(int(treeColliderWindow(in, {}, 0, o).dropped), 0);
  }

  void capDefaultIsTheEngineCap() {
    QCOMPARE(kEnvCap, size_t(4096));
    std::vector<VegCollider> in;
    for (int i = 0; i < 5000; ++i)
      in.push_back(tree(float(i % 100) - 50.0f, float(i) / 100.0f - 25.0f)); // all within 150 m
    const ColliderWindow w = treeColliderWindow(in, {}, 0);
    QCOMPARE(int(w.cylinders.size()), 4096);
    QCOMPARE(int(w.dropped), 5000 - 4096);
  }

  // Golden bytes of ENV_ADD: count then {x, y, z, radius, height} float32 LE.
  void encodeEnvAddGolden() {
    EngineCylinder a;
    a.center = PgVec3{1.0f, -2.0f, -8.0f};
    a.radius = 0.5f;
    a.height = 10.0f;
    EngineCylinder b;
    b.center = PgVec3{0.0f, 0.25f, -0.75f};
    b.radius = 2.0f;
    b.height = 1.5f;
    const auto m = encodeEnvAdds({a, b});
    QCOMPARE(int(m.size()), 1);
    const std::vector<std::uint8_t> golden{
        2,    0,    0,    0,                      // count = 2
        0x00, 0x00, 0x80, 0x3f,                   // x 1.0
        0x00, 0x00, 0x00, 0xc0,                   // y -2.0
        0x00, 0x00, 0x00, 0xc1,                   // z -8.0
        0x00, 0x00, 0x00, 0x3f,                   // radius 0.5
        0x00, 0x00, 0x20, 0x41,                   // height 10.0
        0x00, 0x00, 0x00, 0x00,                   // x 0.0
        0x00, 0x00, 0x80, 0x3e,                   // y 0.25
        0x00, 0x00, 0x40, 0xbf,                   // z -0.75
        0x00, 0x00, 0x00, 0x40,                   // radius 2.0
        0x00, 0x00, 0xc0, 0x3f};                  // height 1.5
    QCOMPARE(m[0], golden);
    QCOMPARE(int(m[0].size()), 4 + 20 * 2);
  }

  void encodeEnvAddBatchesOfTwelve() {
    QVERIFY(encodeEnvAdds({}).empty());
    std::vector<EngineCylinder> v(25);
    for (size_t i = 0; i < v.size(); ++i) {
      v[i].center = PgVec3{float(i), 0, -1};
      v[i].radius = 0.3f;
      v[i].height = 2.0f;
    }
    const auto m = encodeEnvAdds(v);
    QCOMPARE(int(m.size()), 3);
    QCOMPARE(int(m[0].size()), 4 + 20 * 12);
    QCOMPARE(int(m[1].size()), 4 + 20 * 12);
    QCOMPARE(int(m[2].size()), 4 + 20 * 1);
    QVERIFY(m[0].size() <= 256);
    std::uint32_t c;
    std::memcpy(&c, m[2].data(), 4);
    QCOMPARE(c, 1u);
    QCOMPARE(f32(m[2], 4), 24.0f);  // the last cylinder is #24, order kept
    QCOMPARE(f32(m[1], 4), 12.0f);  // second message starts at #12
    // 4096 cylinders: ceil(4096 / 12) messages, all within the ctl body.
    QCOMPARE(int(encodeEnvAdds(std::vector<EngineCylinder>(4096)).size()), 342);
  }

  void encodeEnvCommitGolden() {
    const std::vector<std::uint8_t> g{0x9a, 0x99, 0x99, 0x3e}; // 0.3f
    QCOMPARE(encodeEnvCommit(), g);
    QCOMPARE(encodeEnvCommit(0.3f), g);
    QCOMPARE(f32(encodeEnvCommit(2.0f), 0), 1.0f);   // clamped
    QCOMPARE(f32(encodeEnvCommit(-1.0f), 0), 0.0f);
    QCOMPARE(f32(encodeEnvCommit(std::nanf("")), 0), kColliderRestitution);
    QCOMPARE(int(encodeEnvCommit().size()), 4);
  }

  void subtypesMatchTheProtocol() {
    // vsim_proto.h: VSIM_CTL_ENV_BEGIN..CLEAR and VSIM_ENV_ADD_MAX / VSIM_ENV_CAP.
    QCOMPARE(kEnvBegin, 16u);
    QCOMPARE(kEnvAdd, 17u);
    QCOMPARE(kEnvCommit, 18u);
    QCOMPARE(kEnvClear, 19u);
    QCOMPARE(kEnvAddMax, size_t(12));
  }

  // A tree placed the way scatterVegetation places it (pos.z = -surface height,
  // NED, up is -z) encodes to the same centre, radius and height as the
  // equivalent World-tab obstacle after SimWorker's conversion.
  void encodedTrunkMatchesWorldTabObstacle() {
    const PgVec3 base{12.0f, -7.0f, -42.5f}; // surface 42.5 m above NED zero
    const auto w = win({tree(base.x, base.y, base.z, 0.35f, 9.0f)});
    QCOMPARE(int(w.size()), 1);
    const auto m = encodeEnvAdds(w);
    const WireCylinder o = sendObstaclesWire(worldTabCylinder(base, 0.35f, 9.0f));
    QCOMPARE(f32(m[0], 4), o.pos[0]);
    QCOMPARE(f32(m[0], 8), o.pos[1]);
    QCOMPARE(f32(m[0], 12), o.pos[2]);
    QCOMPARE(f32(m[0], 16), o.size[0]);
    QCOMPARE(f32(m[0], 20), o.size[2]);
    QCOMPARE(f32(encodeEnvCommit(), 0), o.restitution);
    QVERIFY(w[0].center.z - 0.5f * w[0].height < base.z); // above the ground
  }

  // The real chain: scatterVegetation -> vegetationColliders -> window. Every
  // trunk in the disc stands on the terrain and encodes to the World-tab values.
  void realScatterPathMatchesWorldTab() {
    FieldParams fp;
    fp.seed = 11u;
    fp.palette = kPalForest;
    const TerrainField f(fp);
    const VegParams p = vegParamsForPalette(kPalForest);
    const float chunkM = 256.0f;
    std::vector<VegInstance> inst;
    for (int j = 0; j < 2; ++j)
      for (int i = 0; i < 2; ++i) {
        const auto v = scatterVegetation(f, i, j, chunkM, p);
        inst.insert(inst.end(), v.begin(), v.end());
      }
    const std::vector<VegCollider> cols = vegetationColliders(inst);
    QVERIFY(cols.size() > 20);
    const PgVec3 drone{128, 128, 0};
    ColliderOptions o = both();
    const ColliderWindow w = treeColliderWindow(cols, drone, 0, o);
    size_t expect = 0;
    for (const VegCollider &c : cols) {
      QVERIFY2(std::fabs(c.pos.z + f.height(c.pos.x, c.pos.y)) < 3.0f,
               "collider base is not at -terrain height");
      const float dx = c.pos.x - drone.x, dy = c.pos.y - drone.y;
      if (dx * dx + dy * dy > kColliderWindowRadiusM * kColliderWindowRadiusM)
        continue;
      ++expect;
      const WireCylinder wt = sendObstaclesWire(worldTabCylinder(c.pos, c.radius, c.height));
      bool found = false;
      for (const EngineCylinder &e : w.cylinders)
        if (e.center.x == wt.pos[0] && e.center.y == wt.pos[1] && e.center.z == wt.pos[2] &&
            e.radius == wt.size[0] && e.height == wt.size[2])
          found = true;
      QVERIFY(found);
    }
    QVERIFY(expect > 5);
    QCOMPARE(w.cylinders.size(), expect);
    QCOMPARE(int(w.dropped), 0);
    // Trees only: the rocks in that set are gone, the trunks stay.
    const auto t = win(cols, drone);
    size_t trunks = 0;
    for (const EngineCylinder &e : w.cylinders)
      trunks += e.isRock ? 0 : 1;
    QCOMPARE(t.size(), trunks);
  }

  // The key once packed these into one XOR word, where (streamGen 1, rev 0) and
  // (streamGen 0, rev 0x10000) were the same word and a changed world kept its
  // stale collider set. Exact fields tell them apart.
  void worldKeyFieldsDoNotAlias() {
    ColliderBuildKey a;
    a.valid = true;
    a.streamGen = 1;
    a.rev = 0;
    ColliderBuildKey b = a;
    b.streamGen = 0;
    b.rev = 0x10000u;
    QVERIFY(colliderWindowNeedsRebuild(a, b));
    QVERIFY(colliderWindowNeedsRebuild(b, a));
    // seed 1 / palette 0 against seed 0 / palette 1<<..: also distinct.
    ColliderBuildKey c = a;
    c.seed = 5;
    c.palette = 0;
    ColliderBuildKey d = c;
    d.seed = 0;
    d.palette = 5;
    QVERIFY(colliderWindowNeedsRebuild(c, d));
  }

  void rebuildPolicy() {
    ColliderBuildKey last;
    ColliderBuildKey now;
    now.x = 5;
    QVERIFY(colliderWindowNeedsRebuild(last, now)); // nothing built yet
    last = now;
    last.valid = true;
    QVERIFY(!colliderWindowNeedsRebuild(last, now));
    const float q = kColliderRebuildM;
    QCOMPARE(q, 25.0f);
    ColliderBuildKey m = now;
    m.x += q; // exactly a quarter of the margin: not yet
    QVERIFY(!colliderWindowNeedsRebuild(last, m));
    m.x = now.x + q + 0.1f;
    QVERIFY(colliderWindowNeedsRebuild(last, m));
    m = now; // diagonal: distance, not per-axis
    m.x += 0.8f * q;
    m.y -= 0.8f * q;
    QVERIFY(colliderWindowNeedsRebuild(last, m)); // 1.13 q
    m = now;
    m.x += 0.5f * q;
    m.y += 0.5f * q;
    QVERIFY(!colliderWindowNeedsRebuild(last, m)); // 0.71 q
    // A flag or world change rebuilds with the drone standing still.
    m = now;
    m.trees = !m.trees;
    QVERIFY(colliderWindowNeedsRebuild(last, m));
    m = now;
    m.rocks = !m.rocks;
    QVERIFY(colliderWindowNeedsRebuild(last, m));
    // Every world field alone is a reason to rebuild.
    m = now;
    m.seed += 1;
    QVERIFY(colliderWindowNeedsRebuild(last, m));
    m = now;
    m.palette += 1;
    QVERIFY(colliderWindowNeedsRebuild(last, m));
    m = now;
    m.streamGen += 1;
    QVERIFY(colliderWindowNeedsRebuild(last, m));
    m = now;
    m.rev += 1;
    QVERIFY(colliderWindowNeedsRebuild(last, m));
    // A non-finite position is not a reason to keep a stale window.
    m = now;
    m.x = std::nanf("");
    QVERIFY(colliderWindowNeedsRebuild(last, m));
    // The hysteresis is a quarter of the margin by construction.
    QCOMPARE(kColliderMarginM, kColliderWindowRadiusM - kColliderNeedM);
    QCOMPARE(kColliderRebuildM, 0.25f * kColliderMarginM);
  }
};

QTEST_MAIN(TstTreeColliderWindow)
#include "tst_treecolliderwindow.moc"
