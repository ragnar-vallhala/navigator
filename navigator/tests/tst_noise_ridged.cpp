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
// tst_noise_ridged.cpp - Noise::ridged(): the `soft` argument that rounds the
// ridge crests. soft = 0 is the sharp original, bit for bit.
#include "procgen/Noise.h"

#include <QtTest/QtTest>

#include <cmath>

using vsim::procgen::Noise;

namespace {

// The unsoftened one-octave reference: (1 - |gradient|)^2 with norm 1.
float sharpOne(const Noise &n, float x, float y) {
  const float r = 1.0f - std::fabs(n.gradient(x, y));
  return r * r;
}

float softOne(const Noise &n, float x, float y, float s) {
  const float v = n.gradient(x, y);
  const float r = 1.0f - (std::sqrt(v * v + s * s) - s);
  return r * r;
}

} // namespace

class TstNoiseRidged : public QObject {
  Q_OBJECT
private slots:
  void softZeroIsTheOriginalExactly() {
    const Noise n(7u);
    for (int j = 0; j < 40; ++j)
      for (int i = 0; i < 40; ++i) {
        const float x = 0.37f * float(i), y = 0.41f * float(j);
        QCOMPARE(n.ridged(x, y, 4, 2.0f, 0.55f, 0.0f),
                 n.ridged(x, y, 4, 2.0f, 0.55f)); // default arg is 0
        QCOMPARE(n.ridged(x, y, 1, 2.0f, 0.55f, 0.0f), sharpOne(n, x, y));
      }
  }

  void softMatchesItsFormulaAndStaysNearSharp() {
    const Noise n(7u);
    for (float s : {0.05f, 0.1f, 0.2f}) {
      for (int j = 0; j < 30; ++j)
        for (int i = 0; i < 30; ++i) {
          const float x = 0.43f * float(i), y = 0.29f * float(j);
          const float a = n.ridged(x, y, 1, 2.0f, 0.55f, s);
          QVERIFY(std::fabs(a - softOne(n, x, y, s)) < 1e-6f);
          // sqrt(v^2+s^2)-s lies in [|v|-s, |v|], so the softened crest is never
          // below the sharp one and at most 2s above it (it rounds the peak).
          const float sharp = sharpOne(n, x, y);
          QVERIFY2(a >= sharp - 1e-6f, "soft ridge below the sharp one");
          QVERIFY2(a - sharp <= 2.0f * s + 1e-6f, "soft ridge too far above");
        }
    }
  }

  // The kink: |v| has slope jump 2 across v = 0; the softened one is smooth, so
  // the second difference of the ridge across a zero crossing is small.
  void softRemovesTheCrestKink() {
    const Noise n(7u);
    // Walk along x at fixed y and find sign changes of the gradient noise.
    const float y = 3.3f;
    float prevV = n.gradient(0.0f, y);
    double sharpKink = 0.0, softKink = 0.0;
    int crossings = 0;
    const float h = 0.002f;
    for (int k = 1; k < 30000; ++k) {
      const float x = h * float(k);
      const float v = n.gradient(x, y);
      if ((v < 0.0f) != (prevV < 0.0f)) {
        ++crossings;
        const auto d2 = [&](float s) {
          const float a = n.ridged(x - h, y, 1, 2.0f, 0.55f, s);
          const float b = n.ridged(x, y, 1, 2.0f, 0.55f, s);
          const float c = n.ridged(x + h, y, 1, 2.0f, 0.55f, s);
          return std::fabs(double(a) - 2.0 * double(b) + double(c));
        };
        sharpKink += d2(0.0f);
        softKink += d2(0.2f);
      }
      prevV = v;
    }
    QVERIFY2(crossings > 10, "no zero crossings sampled");
    qInfo("%d crossings: mean |second difference| sharp %.2e, soft %.2e",
          crossings, sharpKink / crossings, softKink / crossings);
    QVERIFY2(softKink < 0.2 * sharpKink, "crest still has a kink");
  }
};

QTEST_APPLESS_MAIN(TstNoiseRidged)
#include "tst_noise_ridged.moc"
