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
// tst_treecolliderwindow — the collider window around the drone: NED cylinder
// conversion, Trees/vegetation flags, window edges, stable ordering (Qt-free
// core, headless).
#include <QtTest/QtTest>

#include "VsimTypes.h"
#include "procgen/GroundCover.h"
#include "procgen/TerrainField.h"
#include "procgen/TreeColliderWindow.h"

#include <algorithm>
#include <cmath>

using namespace vsim::procgen;

namespace {
VegCollider tree(float x, float y, float z = 0.0f, float r = 0.3f, float h = 10.0f) {
  return VegCollider{PgVec3{x, y, z}, r, h, false};
}
VegCollider rock(float x, float y, float z = 0.0f) {
  return VegCollider{PgVec3{x, y, z}, 1.0f, 1.5f, true};
}

// What SimWorker::sendObstacles (vsim/SimWorker.cpp:211-231) hands the engine
// for a World-tab obstacle: pos, size and restitution copied one to one into
// vsim_ctl_obstacle_t (b.pos = o.pos, b.size = o.size, b.restitution). That
// code is inline in a member that needs the engine thread, so the arithmetic
// is repeated here; for a cylinder pos is the CENTRE in NED metres (z down),
// size.x the radius and size.z the full height, axis the body-local z
// (vsim/VsimTypes.h:111-115).
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
    const auto w = treeColliderWindow({tree(5, 6, -3.0f, 0.4f, 10.0f)}, {}, 0, 100, true, true);
    QCOMPARE(int(w.size()), 1);
    QCOMPARE(w[0].center.x, 5.0f);
    QCOMPARE(w[0].center.y, 6.0f);
    QCOMPARE(w[0].center.z, -8.0f);
    QCOMPARE(w[0].radius, 0.4f);
    QCOMPARE(w[0].height, 10.0f);
    QCOMPARE(w[0].restitution, 0.3f);
    QVERIFY(!w[0].isRock);
    // Top of the cylinder is the surface point's opposite end: centre - h/2 is
    // above the surface (more negative), centre + h/2 is the ground.
    QCOMPARE(w[0].center.z + w[0].height * 0.5f, -3.0f);
  }

  void flagsSelectClasses() {
    const std::vector<VegCollider> in{tree(1, 1), rock(2, 2)};
    QCOMPARE(int(treeColliderWindow(in, {}, 0, 100, true, true).size()), 2);
    const auto t = treeColliderWindow(in, {}, 0, 100, true, false);
    QCOMPARE(int(t.size()), 1);
    QVERIFY(!t[0].isRock);
    const auto r = treeColliderWindow(in, {}, 0, 100, false, true);
    QCOMPARE(int(r.size()), 1);
    QVERIFY(r[0].isRock);
    QVERIFY(treeColliderWindow(in, {}, 0, 100, false, false).empty());
  }

  void windowIsThreeByThreeChunks() {
    // Drone in chunk (0,0) of 100 m chunks: window is [-100, 200) on both axes.
    const PgVec3 d{50, 50, 0};
    const auto in = [&](float x, float y) {
      return !treeColliderWindow({tree(x, y)}, d, 0, 100, true, true).empty();
    };
    QVERIFY(in(-100.0f, 0));       // lower edge inclusive
    QVERIFY(!in(-100.01f, 0));
    QVERIFY(in(199.99f, 0));
    QVERIFY(!in(200.0f, 0));       // upper edge exclusive
    QVERIFY(in(0, -100.0f));
    QVERIFY(!in(0, -100.01f));
    QVERIFY(in(0, 199.99f));
    QVERIFY(!in(0, 200.0f));
    QVERIFY(!in(1000, 1000));
  }

  void windowFollowsDroneChunk() {
    // Drone in chunk (-1, 2): window x in [-200, 100), y in [100, 400).
    const PgVec3 d{-10, 250, 0};
    QVERIFY(!treeColliderWindow({tree(-150, 150)}, d, 0, 100, true, true).empty());
    QVERIFY(treeColliderWindow({tree(150, 150)}, d, 0, 100, true, true).empty());
    QVERIFY(treeColliderWindow({tree(-250, 150)}, d, 0, 100, true, true).empty());
  }

  void speedWidensWindow() {
    const PgVec3 d{50, 50, 0};
    const float m = 10.0f * kColliderLookaheadS;
    QVERIFY(treeColliderWindow({tree(200.0f + m - 0.01f, 0)}, d, 10, 100, true, true).size() == 1);
    QVERIFY(treeColliderWindow({tree(200.0f + m, 0)}, d, 10, 100, true, true).empty());
    QVERIFY(treeColliderWindow({tree(-100.0f - m, 0)}, d, 10, 100, true, true).size() == 1);
    QVERIFY(treeColliderWindow({tree(-100.0f - m - 0.01f, 0)}, d, 10, 100, true, true).empty());
    // Standing still, the same tree is outside; negative speed does not shrink.
    QVERIFY(treeColliderWindow({tree(205, 0)}, d, 0, 100, true, true).empty());
    QVERIFY(treeColliderWindow({tree(150, 0)}, d, -50, 100, true, true).size() == 1);
  }

  void orderingIsStableAndDiffable() {
    std::vector<VegCollider> in{tree(30, 5), tree(10, 9), rock(10, 2), tree(10, 2),
                                tree(20, 1)};
    const auto a = treeColliderWindow(in, {}, 0, 100, true, true);
    std::reverse(in.begin(), in.end());
    const auto b = treeColliderWindow(in, {}, 0, 100, true, true);
    QVERIFY(a == b);
    QVERIFY(std::is_sorted(a.begin(), a.end(), [](const EngineCylinder &p, const EngineCylinder &q) {
      return p.center.x != q.center.x ? p.center.x < q.center.x
                                      : p.center.y < q.center.y;
    }));
    QCOMPARE(a.front().center.x, 10.0f);
    QCOMPARE(a.front().center.y, 2.0f);
    QCOMPARE(a.back().center.x, 30.0f);
    // A changed set compares unequal.
    in.push_back(tree(40, 40));
    QVERIFY(treeColliderWindow(in, {}, 0, 100, true, true) != a);
  }

  // A tree placed the way scatterVegetation places it (pos.z = -surface height,
  // NED, up is -z) lands on the same engine cylinder as the equivalent World-tab
  // obstacle after SimWorker's conversion.
  void matchesWorldTabObstacle() {
    const PgVec3 base{12.0f, -7.0f, -42.5f}; // surface 42.5 m above NED zero
    const auto w = treeColliderWindow({tree(base.x, base.y, base.z, 0.35f, 9.0f)},
                                      {}, 0, 100, true, true);
    QCOMPARE(int(w.size()), 1);
    const WireCylinder o = sendObstaclesWire(worldTabCylinder(base, 0.35f, 9.0f));
    QCOMPARE(w[0].center.x, o.pos[0]);
    QCOMPARE(w[0].center.y, o.pos[1]);
    QCOMPARE(w[0].center.z, o.pos[2]);
    QCOMPARE(w[0].radius, o.size[0]);
    QCOMPARE(w[0].height, o.size[2]);
    QCOMPARE(w[0].restitution, o.restitution);
    // And it is above the ground: the top is at a smaller (more up) NED z.
    QVERIFY(w[0].center.z - 0.5f * w[0].height < base.z);
  }

  // The real chain: scatterVegetation -> vegetationColliders -> window. Every
  // trunk stands on the terrain (pos.z = -height, so NED), and its cylinder
  // equals the World-tab obstacle for the same base.
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
    // Drone in chunk (0,0): chunks -1..1 are in the window, so (0..1, 0..1) all are.
    const auto w = treeColliderWindow(cols, PgVec3{10, 10, 0}, 0, chunkM, true, true);
    QCOMPARE(w.size(), cols.size());
    for (const VegCollider &c : cols) {
      // NED: the surface is at z = -height (within the mesh-vs-field difference).
      QVERIFY2(std::fabs(c.pos.z + f.height(c.pos.x, c.pos.y)) < 3.0f,
               "collider base is not at -terrain height");
      const WireCylinder o = sendObstaclesWire(worldTabCylinder(c.pos, c.radius, c.height));
      const EngineCylinder e = [&] {
        for (const EngineCylinder &x : w)
          if (x.center.x == o.pos[0] && x.center.y == o.pos[1] && x.radius == o.size[0] &&
              x.height == o.size[2])
            return x;
        return EngineCylinder{};
      }();
      QVERIFY(e.radius > 0.0f);
      QCOMPARE(e.center.z, o.pos[2]);
      QCOMPARE(e.restitution, o.restitution);
    }
  }

  void degenerateInputs() {
    QVERIFY(treeColliderWindow({}, {}, 0, 100, true, true).empty());
    QVERIFY(treeColliderWindow({tree(1, 1)}, {}, 0, 0, true, true).empty());
    QVERIFY(treeColliderWindow({tree(1, 1)}, {}, 0, -5, true, true).empty());
    QVERIFY(treeColliderWindow({tree(1, 1, 0, 0.0f, 5)}, {}, 0, 100, true, true).empty());
    QVERIFY(treeColliderWindow({tree(1, 1, 0, 0.3f, 0.0f)}, {}, 0, 100, true, true).empty());
  }
};

QTEST_MAIN(TstTreeColliderWindow)
#include "tst_treecolliderwindow.moc"
