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

#include "procgen/TreeColliderWindow.h"

#include <algorithm>

using namespace vsim::procgen;

namespace {
VegCollider tree(float x, float y, float z = 0.0f, float r = 0.3f, float h = 10.0f) {
  return VegCollider{PgVec3{x, y, z}, r, h, false};
}
VegCollider rock(float x, float y, float z = 0.0f) {
  return VegCollider{PgVec3{x, y, z}, 1.0f, 1.5f, true};
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
