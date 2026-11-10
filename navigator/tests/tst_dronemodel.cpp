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
// tst_dronemodel.cpp — procedural drone model generator (Qt-free core).

#include <QtTest/QtTest>

#include "DroneModel.h"

#include <cmath>
#include <string>

using namespace vsim;

namespace {

struct Row {
  const char *id;
  float r, p;
  int blades;
};
// Preset prop radius / motor offset (AirframePresets.cpp rows).
const Row kRows[] = {{"5in", 0.0622f, 0.0866f, 3},
                     {"3in", 0.0381f, 0.0502f, 3},
                     {"7in", 0.0889f, 0.1156f, 2},
                     {"10in", 0.1270f, 0.1485f, 2},
                     {"whoop65", 0.0155f, 0.0230f, 4}};

DroneModelInput inputFor(const Row &r, float scale = 1.0f) {
  DroneModelInput in;
  in.propRadius = r.r * scale;
  const float p = r.p * scale;
  in.motorPos = {{{p, p, 0}, {-p, p, 0}, {-p, -p, 0}, {p, -p, 0}}};
  in.spin = {+1, -1, +1, -1};
  in.cls = droneClassFor(r.id, in.propRadius);
  return in;
}

bool allFinite(const ModelMesh &m) {
  for (float v : m.data)
    if (!std::isfinite(v))
      return false;
  return true;
}

// x of vertices whose colour matches c (exact).
bool meanX(const ModelMesh &m, std::array<float, 3> c, float *out) {
  double s = 0;
  int n = 0;
  for (size_t v = 0; v < m.vertexCount(); ++v) {
    const float *d = &m.data[v * 9];
    if (std::fabs(d[6] - c[0]) < 1e-4f && std::fabs(d[7] - c[1]) < 1e-4f &&
        std::fabs(d[8] - c[2]) < 1e-4f) {
      s += d[0];
      ++n;
    }
  }
  if (!n)
    return false;
  *out = float(s / n);
  return true;
}

} // namespace

class TstDroneModel : public QObject {
  Q_OBJECT
private slots:
  void classMapping() {
    QCOMPARE(int(droneClassFor("5in", 0.0f)), int(DroneClass::Freestyle5));
    QCOMPARE(int(droneClassFor("whoop65", 1.0f)), int(DroneClass::Whoop65));
    // custom: by radius
    QCOMPARE(int(droneClassFor("", 0.015f)), int(DroneClass::Whoop65));
    QCOMPARE(int(droneClassFor("", 0.038f)), int(DroneClass::Light3));
    QCOMPARE(int(droneClassFor("", 0.062f)), int(DroneClass::Freestyle5));
    QCOMPARE(int(droneClassFor("", 0.089f)), int(DroneClass::LongRange7));
    QCOMPARE(int(droneClassFor("", 0.127f)), int(DroneClass::Heavy10));
    QCOMPARE(int(droneClassFor("", -1.0f)), int(DroneClass::Freestyle5));
  }

  void trianglesAndNormals() {
    for (const Row &r : kRows) {
      const DroneModel m = buildDroneModel(inputFor(r));
      QVERIFY2(m.body.vertexCount() > 100, r.id);
      QCOMPARE(m.body.vertexCount() % 3, size_t(0));
      QVERIFY(allFinite(m.body));
      for (int i = 0; i < 4; ++i) {
        QVERIFY2(m.prop[i].vertexCount() > 20, r.id);
        QVERIFY(allFinite(m.prop[i]));
      }
      for (size_t v = 0; v < m.body.vertexCount(); ++v) {
        const float *d = &m.body.data[v * 9];
        const float l = std::sqrt(d[3] * d[3] + d[4] * d[4] + d[5] * d[5]);
        QVERIFY2(std::fabs(l - 1.0f) < 1e-3f, r.id);
      }
      // a handful of draw calls: body + 4 props, a few thousand triangles at most
      QVERIFY(m.body.vertexCount() < 60000);
    }
  }

  void boundingBoxScalesWithPropRadius() {
    for (const Row &r : kRows) {
      const DroneModel a = buildDroneModel(inputFor(r, 1.0f));
      const DroneModel b = buildDroneModel(inputFor(r, 2.0f));
      for (int ax = 0; ax < 2; ++ax) { // x and y spans
        const float sa = a.bmax[ax] - a.bmin[ax];
        const float sb = b.bmax[ax] - b.bmin[ax];
        QVERIFY2(std::fabs(sb / sa - 2.0f) < 0.15f, r.id);
      }
      // the model reaches out past the motors by about a prop radius (matches
      // the physics footprint; 2-blade props lie along x, so measure radially)
      float far = 0;
      for (size_t v = 0; v < a.body.vertexCount(); ++v)
        far = std::max(far, std::hypot(a.body.data[v * 9], a.body.data[v * 9 + 1]));
      for (int i = 0; i < 4; ++i)
        for (size_t v = 0; v < a.prop[i].vertexCount(); ++v)
          far = std::max(far, std::hypot(a.hub[i][0] + a.prop[i].data[v * 9],
                                         a.hub[i][1] + a.prop[i].data[v * 9 + 1]));
      QVERIFY2(far >= std::hypot(r.p, r.p) + 0.5f * a.visualPropRadius, r.id);
      QVERIFY2(far <= std::hypot(r.p, r.p) + 1.3f * a.visualPropRadius, r.id);
    }
  }

  void propsDoNotOverlap() {
    for (const Row &r : kRows) {
      const DroneModel m = buildDroneModel(inputFor(r));
      float ext[4];
      for (int i = 0; i < 4; ++i) {
        float e = 0;
        for (size_t v = 0; v < m.prop[i].vertexCount(); ++v)
          e = std::max(e, std::hypot(m.prop[i].data[v * 9],
                                     m.prop[i].data[v * 9 + 1]));
        ext[i] = e;
        QVERIFY2(e <= r.r * 1.01f, r.id);
        QVERIFY2(e > 0.9f * m.visualPropRadius, r.id); // blades reach the radius
      }
      for (int i = 0; i < 4; ++i)
        for (int j = i + 1; j < 4; ++j) {
          const float d = std::hypot(m.hub[i][0] - m.hub[j][0],
                                     m.hub[i][1] - m.hub[j][1]);
          QVERIFY2(d > ext[i] + ext[j], r.id);
        }
    }
  }

  void crowdedMotorsShrinkProps() {
    // Props larger than the motor spacing allows must not intersect.
    DroneModelInput in = inputFor(kRows[0]);
    in.propRadius = 0.2f; // spacing is only 0.173
    const DroneModel m = buildDroneModel(in);
    QVERIFY(m.visualPropRadius < 0.0866f);
    float e = 0;
    for (size_t v = 0; v < m.prop[0].vertexCount(); ++v)
      e = std::max(e, std::hypot(m.prop[0].data[v * 9], m.prop[0].data[v * 9 + 1]));
    QVERIFY(2 * e < 0.1732f);
  }

  void frontIsPlusX() {
    for (const Row &r : kRows) {
      const DroneModel m = buildDroneModel(inputFor(r));
      float fx = 0, rx = 0, lx = 0;
      QVERIFY2(meanX(m.body, {1.0f, 0.25f, 0.06f}, &fx), r.id); // front accent
      QVERIFY2(fx > 0.0f, r.id);
      if (std::string(r.id) != "whoop65") {
        QVERIFY2(meanX(m.body, {0.90f, 0.92f, 0.96f}, &rx), r.id); // rear accent
        QVERIFY2(rx < 0.0f, r.id);
      }
      QVERIFY2(meanX(m.body, {0.08f, 0.16f, 0.45f}, &lx), r.id); // lens
      QVERIFY2(lx > 0.0f, r.id);
      // front props carry the red band, rear ones do not
      float bx = 0;
      QVERIFY2(meanX(m.prop[0], {1.0f, 0.18f, 0.08f}, &bx), r.id); // FR
      QVERIFY2(!meanX(m.prop[1], {1.0f, 0.18f, 0.08f}, &bx), r.id); // RR
    }
  }

  void spinColoursDiffer() {
    const DroneModel m = buildDroneModel(inputFor(kRows[0]));
    float x = 0;
    QVERIFY(meanX(m.prop[0], propTipColor(+1), &x)); // FR is CCW
    QVERIFY(!meanX(m.prop[0], propTipColor(-1), &x));
    QVERIFY(meanX(m.prop[1], propTipColor(-1), &x)); // RR is CW
    QVERIFY(propTipColor(+1) != propTipColor(-1));
  }

  void perClassVariation() {
    size_t counts[5];
    float widths[5];
    for (int i = 0; i < 5; ++i) {
      const DroneModel m = buildDroneModel(inputFor(kRows[i]));
      QCOMPARE(m.bladeCount, kRows[i].blades);
      counts[i] = m.body.vertexCount();
      widths[i] = (m.bmax[0] - m.bmin[0]) + (m.bmax[1] - m.bmin[1]);
    }
    // sizes order by class: whoop < 3in < 5in < 7in < 10in
    QVERIFY(widths[4] < widths[1] && widths[1] < widths[0] &&
            widths[0] < widths[2] && widths[2] < widths[3]);
    QVERIFY(counts[4] != counts[0]); // whoop has ducts, no stack
    QVERIFY(counts[2] != counts[0]); // 7in adds the GPS mast
  }

  void degenerateGeometry() {
    auto check = [](DroneModelInput in) {
      const DroneModel m = buildDroneModel(in);
      QVERIFY(m.body.vertexCount() > 0);
      QVERIFY(allFinite(m.body));
      for (int i = 0; i < 4; ++i) {
        QVERIFY(m.prop[i].vertexCount() > 0);
        QVERIFY(allFinite(m.prop[i]));
      }
      for (int a = 0; a < 3; ++a) {
        QVERIFY(std::isfinite(m.bmin[a]) && std::isfinite(m.bmax[a]));
        QVERIFY(m.bmax[a] >= m.bmin[a]);
      }
      QVERIFY(m.visualPropRadius > 0.0f);
    };
    DroneModelInput in;
    in.propRadius = 0.0f; // zero radius
    check(in);
    in.propRadius = -3.0f;
    check(in);
    in.propRadius = std::nanf("");
    check(in);
    in = DroneModelInput();
    for (auto &p : in.motorPos) // all motors on one point
      p = {0, 0, 0};
    check(in);
    in = DroneModelInput();
    in.motorPos[1] = in.motorPos[0]; // fewer than 4 distinct positions
    check(in);
    in = DroneModelInput();
    in.motorPos[2][0] = std::nanf(""); // NaN position
    check(in);
    in = DroneModelInput();
    for (auto &p : in.motorPos) // absurd positions
      p = {p[0] * 1e6f, p[1] * 1e6f, 0};
    check(in);
    in = DroneModelInput();
    in.propRadius = 1e9f; // huge radius
    check(in);
    in = DroneModelInput();
    in.cls = DroneClass::Whoop65;
    in.propRadius = 0.0f;
    check(in);
  }
};

QTEST_APPLESS_MAIN(TstDroneModel)
#include "tst_dronemodel.moc"
