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
// tst_shadow_cascades.cpp — cascaded shadow-map fitting (pure maths, no GL).
//
// Shimmer-free: for ANY camera pose a fixed world point lands on the same
// fractional shadow texel in every cascade (the window only ever moves by whole
// texels). Coverage: every corner of the camera's slice lies inside its cascade
// with the PCF margin to spare. The unsnapped variant must fail the first
// check, so the test can tell the two apart.
#include <QtTest/QtTest>

#include "ShadowCascades.h"

#include <cmath>
#include <cstdint>

using namespace vsim;

namespace {
QMatrix4x4 camToWorld(const QVector3D &pos, float yaw, float pitch) {
  QMatrix4x4 m;
  m.translate(pos);
  m.rotate(yaw, 0, 0, 1);
  m.rotate(pitch, 0, 1, 0);
  m.rotate(-90.0f, 0, 1, 0); // view space looks down -Z: map it onto world +X
  return m;
}
// Texel coordinate (in the cascade's own tile) of world point p.
QVector2D texelOf(const ShadowCascadeSet &s, int ci, const QVector3D &p) {
  const QVector4D c = s.vp[ci] * QVector4D(p, 1.0f);
  return QVector2D((c.x() * 0.5f + 0.5f) * float(s.tileSize),
                   (c.y() * 0.5f + 0.5f) * float(s.tileSize));
}
float fracDist(float a, float b) { // distance between a and b modulo one texel
  float d = std::fmod(std::fabs(a - b), 1.0f);
  return std::min(d, 1.0f - d);
}
} // namespace

class TstShadowCascades : public QObject {
  Q_OBJECT
private:
  float maxFracShift(bool snap) {
    // Fixed pose sequence (xorshift) so the test is deterministic.
    uint32_t state = 7u;
    auto uni = [&state](float lo, float hi) {
      state ^= state << 13;
      state ^= state >> 17;
      state ^= state << 5;
      return lo + (hi - lo) * float(state & 0xFFFFFFu) / float(0x1000000u);
    };
    auto pos = [&] { return uni(-900.0f, 900.0f); };
    auto ang = [&] { return uni(-180.0f, 180.0f); };
    auto pit = [&] { return uni(-60.0f, 30.0f); };
    auto alt = [&] { return uni(-300.0f, -2.0f); };
    const QVector3D sun = QVector3D(0.5f, 0.4f, -0.35f).normalized();
    const QVector3D probe(123.4f, -56.7f, -8.9f);
    const ShadowCascadeSet ref = fitShadowCascades(
        sun, camToWorld({0, 0, -10}, 0, 0), 60.0f, 16.0f / 9.0f, kMaxCascades,
        kShadowSizeHigh, snap);
    float worst = 0.0f;
    for (int k = 0; k < 200; ++k) {
      const ShadowCascadeSet s = fitShadowCascades(
          sun, camToWorld({pos(), pos(), alt()}, ang(), pit()),
          60.0f, 16.0f / 9.0f, kMaxCascades, kShadowSizeHigh, snap);
      for (int ci = 0; ci < kMaxCascades; ++ci) {
        const QVector2D a = texelOf(ref, ci, probe), b = texelOf(s, ci, probe);
        worst = std::max({worst, fracDist(a.x(), b.x()), fracDist(a.y(), b.y())});
      }
    }
    return worst;
  }

private slots:
  void snappedWindowMovesInWholeTexels() {
    // The 0.02 allowance is float error in a 1e3 m coordinate (texel ~ 5 cm).
    const float worst = maxFracShift(true);
    qInfo("snapped: worst fractional texel shift %.4f", worst);
    QVERIFY2(worst < 0.02f, "a snapped cascade moved by a sub-texel amount");
  }
  void unsnappedWouldShimmer() {
    const float worst = maxFracShift(false);
    qInfo("unsnapped: worst fractional texel shift %.4f", worst);
    QVERIFY2(worst > 0.2f, "the test cannot tell snapped from unsnapped");
  }
  void cascadesCoverTheirSlice() {
    const QVector3D sun = QVector3D(0.6f, 0.2f, -0.2f).normalized();
    const QMatrix4x4 c2w = camToWorld({50, -20, -30}, 33.0f, -12.0f);
    const ShadowCascadeSet s = fitShadowCascades(
        sun, c2w, 60.0f, 16.0f / 9.0f, kMaxCascades, kShadowSizeHigh);
    QCOMPARE(s.count, kMaxCascades);
    const float th = std::tan(30.0f * 0.0174532925f);
    for (int ci = 0; ci < kMaxCascades; ++ci) {
      const float f = kCascadeSplitsM[ci];
      const float n = ci ? kCascadeSplitsM[ci - 1] : 0.0f;
      for (float d : {n, f})
        for (int sx : {-1, 1})
          for (int sy : {-1, 1}) {
            const QVector3D p = c2w.map(QVector3D(
                sx * d * th * 16.0f / 9.0f, sy * d * th, -d));
            const QVector4D c = s.vp[ci] * QVector4D(p, 1.0f);
            // inside the window with the PCF margin (8 texels of 2048) to spare
            const float lim = 1.0f - 2.0f * kShadowMarginTexels / float(kShadowSizeHigh) + 1e-3f;
            QVERIFY2(std::fabs(c.x()) <= lim && std::fabs(c.y()) <= lim,
                     qPrintable(QString("cascade %1 corner outside").arg(ci)));
            QVERIFY(c.z() >= -1.0f && c.z() <= 1.0f);
          }
    }
    // Each cascade is coarser than the one before.
    QVERIFY(s.info[0].x() < s.info[1].x() && s.info[1].x() < s.info[2].x());
  }
  void lowPathIsOneCascade() {
    const ShadowCascadeSet s =
        fitShadowCascades(QVector3D(0.5f, 0.4f, -0.5f), QMatrix4x4(), 60.0f,
                          1.6f, 1, kShadowSizeLow);
    QCOMPARE(s.count, 1);
    QCOMPARE(s.atlasW(), kShadowSizeLow);
    QCOMPARE(s.splits.w(), 1.0f);
    QCOMPARE(s.splits.x(), kLowShadowRangeM);
  }
};

QTEST_MAIN(TstShadowCascades)
#include "tst_shadow_cascades.moc"
