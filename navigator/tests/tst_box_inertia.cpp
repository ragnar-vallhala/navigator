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
// vsim::boxInertia: the box/arms model's inertia from mass + motor layout.
// Pure, no widget, no engine.

#include "AirframePresets.h"
#include "MassProperties.h"
#include "VsimTypes.h"

#include <QtTest>

#include <cmath>

namespace {
std::array<QVector3D, 4> posOf(const vsim::GeometryConfig &c) {
  return {c.motors[0].pos, c.motors[1].pos, c.motors[2].pos, c.motors[3].pos};
}
} // namespace

class TstBoxInertia : public QObject {
  Q_OBJECT
private slots:
  // The defaults (5-inch preset) keep their inertia to within rounding.
  void defaultsUnchanged() {
    const vsim::GeometryConfig d;
    const auto I = vsim::boxInertia(d.mass, posOf(d));
    for (int i : {0, 4, 8})
      QVERIFY2(std::fabs(I[i] - d.inertia[i]) < 1e-3f * d.inertia[i],
               qPrintable(QString("I[%1]=%2 default %3")
                              .arg(i)
                              .arg(I[i])
                              .arg(d.inertia[i])));
    for (int i : {1, 2, 3, 5, 6, 7})
      QCOMPARE(I[i], 0.0f);
  }

  void linearInMass() {
    const vsim::GeometryConfig d;
    const auto a = vsim::boxInertia(0.5f, posOf(d));
    const auto b = vsim::boxInertia(1.5f, posOf(d));
    for (int i : {0, 4, 8})
      QVERIFY(std::fabs(b[i] - 3.0f * a[i]) < 1e-5f * b[i]);
  }

  void quadraticInArmLength() {
    vsim::GeometryConfig d;
    const auto a = vsim::boxInertia(d.mass, posOf(d));
    for (auto &m : d.motors)
      m.pos *= 2.0f;
    const auto b = vsim::boxInertia(d.mass, posOf(d));
    for (int i : {0, 4, 8})
      QVERIFY(std::fabs(b[i] - 4.0f * a[i]) < 1e-5f * b[i]);
  }

  // The formula is fitted to the 5in build; the other presets are different
  // builds, but the same order of magnitude (within a factor 2).
  void presetsSameOrder() {
    for (const vsim::AirframePreset &p : vsim::airframePresets()) {
      const auto I = vsim::boxInertia(p.geometry.mass, posOf(p.geometry));
      for (int i : {0, 4, 8}) {
        const float r = I[i] / p.geometry.inertia[i];
        QVERIFY2(r > 0.5f && r < 2.0f, qPrintable(p.id));
      }
    }
  }
};

QTEST_APPLESS_MAIN(TstBoxInertia)
#include "tst_box_inertia.moc"
