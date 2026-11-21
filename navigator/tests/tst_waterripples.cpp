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
// tst_waterripples.cpp -- the prop-wash ripple sim (vsim/WaterRipples, Qt-free).
#include <QtTest/QtTest>

#include "WaterRipples.h"

#include <cmath>

using vsim::RippleRotor;
using vsim::WaterRipples;

namespace {

vsim::procgen::WaterBody pond(float cx = 500.0f, float cy = -300.0f) {
  vsim::procgen::WaterBody b;
  b.cx = cx;
  b.cy = cy;
  b.radiusM = 30.0f;
  b.bankM = 12.0f;
  b.amp[0] = b.amp[1] = 0.0f;
  b.level = 40.0f;
  b.depthM = 2.0f;
  return b;
}

RippleRotor rotorAt(const vsim::procgen::WaterBody &b, float height, float strength) {
  RippleRotor r;
  r.x = b.cx;
  r.y = b.cy;
  r.heightM = height;
  r.radiusM = 0.0622f;
  r.strength = strength;
  return r;
}

// Largest distance from (cx, cy) of a texel whose |h| exceeds `thresh`.
float outerRadius(const WaterRipples &w, float cx, float cy, float thresh) {
  float best = 0.0f;
  const auto &h = w.heights();
  for (int j = 0; j < WaterRipples::kN; ++j)
    for (int i = 0; i < WaterRipples::kN; ++i) {
      if (std::fabs(h[size_t(j) * WaterRipples::kN + size_t(i)]) < thresh)
        continue;
      const float x = w.originX() + (float(i) + 0.5f) * WaterRipples::kCellM - cx;
      const float y = w.originY() + (float(j) + 0.5f) * WaterRipples::kCellM - cy;
      best = std::max(best, std::sqrt(x * x + y * y));
    }
  return best;
}

} // namespace

class TstWaterRipples : public QObject {
  Q_OBJECT
private slots:
  void unboundSimDoesNothing() {
    WaterRipples w;
    QVERIFY(!w.bound());
    w.advanceTo(0.0);
    w.advanceTo(1.0);
    QCOMPARE(w.stepCount(), uint64_t(0));
    QVERIFY(!w.active());
  }

  void rotorStirsTheSurfaceAndItSettlesAgain() {
    const auto b = pond();
    WaterRipples w;
    w.bindBody(b);
    const RippleRotor r = rotorAt(b, 0.4f, 1.0f);
    w.setRotors(&r, 1);
    QVERIFY(w.active());
    w.stepFixed(120);
    const float forced = w.maxAbsHeight();
    QVERIFY2(forced > 0.003f, qPrintable(QString("max=%1").arg(forced)));
    QVERIFY2(forced < 0.5f, "wash must not throw metre-high waves");
    w.setRotors(nullptr, 0);
    float prev = forced;
    for (int s = 0; s < 8; ++s) { // 8 s of calm: no growth anywhere, ends flat
      w.stepFixed(60);
      QVERIFY2(std::isfinite(w.maxAbsHeight()), "sim blew up");
      QVERIFY(w.maxAbsHeight() <= prev * 1.05f + 1e-6f);
      prev = w.maxAbsHeight();
    }
    QVERIFY2(w.maxAbsHeight() < forced * 0.25f,
             qPrintable(QString("%1 -> %2").arg(forced).arg(w.maxAbsHeight())));
  }

  void ringsTravelAtTheWaveSpeed() {
    const auto b = pond();
    WaterRipples w;
    w.bindBody(b);
    const RippleRotor r = rotorAt(b, 0.3f, 1.0f);
    w.setRotors(&r, 1);
    w.stepFixed(30);
    w.setRotors(nullptr, 0);
    const float thr = 0.1f * w.maxAbsHeight();
    const float r0 = outerRadius(w, b.cx, b.cy, thr);
    w.stepFixed(60); // one second
    const float r1 = outerRadius(w, b.cx, b.cy, thr * 0.5f);
    const float travelled = r1 - r0;
    QVERIFY2(travelled > 0.5f * WaterRipples::kSpeedMps &&
                 travelled < 1.6f * WaterRipples::kSpeedMps,
             qPrintable(QString("r0=%1 r1=%2").arg(r0).arg(r1)));
  }

  void noForcingAboveRangeOrWhenIdle() {
    const auto b = pond();
    WaterRipples w;
    w.bindBody(b);
    RippleRotor hi = rotorAt(b, WaterRipples::kMaxHeightM + 1.0f, 1.0f);
    w.setRotors(&hi, 1);
    QVERIFY(!w.active());
    w.stepFixed(120);
    QCOMPARE(w.maxAbsHeight(), 0.0f);
    RippleRotor idle = rotorAt(b, 0.3f, 0.0f);
    w.setRotors(&idle, 1);
    w.stepFixed(120);
    QCOMPARE(w.maxAbsHeight(), 0.0f);
  }

  void closerMeansStronger() {
    const auto b = pond();
    float peak[2];
    int k = 0;
    for (float h : {0.3f, 4.0f}) {
      WaterRipples w;
      w.bindBody(b);
      const RippleRotor r = rotorAt(b, h, 1.0f);
      w.setRotors(&r, 1);
      w.stepFixed(90);
      peak[k++] = w.maxAbsHeight();
    }
    QVERIFY2(peak[0] > 2.0f * peak[1], qPrintable(QString("%1 vs %2").arg(peak[0]).arg(peak[1])));
  }

  void shoreAndWindowEdgeAbsorb() {
    // A rotor right at the shore: the energy left a few seconds later is small,
    // i.e. nothing bounces around the window.
    const auto b = pond();
    WaterRipples w;
    w.bindBody(b);
    RippleRotor r = rotorAt(b, 0.3f, 1.0f);
    r.x = b.cx + 0.8f * b.radiusM;
    w.setRotors(&r, 1);
    w.stepFixed(60);
    const float forced = w.maxAbsHeight();
    w.setRotors(nullptr, 0);
    w.stepFixed(60 * 10);
    QVERIFY2(w.maxAbsHeight() < 0.15f * forced,
             qPrintable(QString("%1 -> %2").arg(forced).arg(w.maxAbsHeight())));
    // Outside the shore the surface stays (nearly) flat.
    QVERIFY(std::fabs(w.heightAt(b.cx + b.radiusM + 8.0f, b.cy)) < 0.02f * forced + 1e-6f);
  }

  // Two views / any render rate: the same clock gives the same steps and result.
  void resultDoesNotDependOnRenderRate() {
    const auto b = pond();
    const RippleRotor r = rotorAt(b, 0.5f, 1.0f);
    auto run = [&](double frame, WaterRipples &w) {
      w.bindBody(b);
      w.setRotors(&r, 1);
      const int frames = int(std::lround(3.0 / frame));
      for (int f = 0; f <= frames; ++f)
        w.advanceTo(double(f) * frame);
    };
    WaterRipples a, c, d;
    run(1.0 / 30.0, a);
    run(1.0 / 144.0, c);
    const long diff = long(a.stepCount()) - long(c.stepCount());
    QVERIFY2(std::labs(diff) <= 2, qPrintable(QString("%1 vs %2").arg(a.stepCount()).arg(c.stepCount())));
    QVERIFY(a.stepCount() >= 170 && a.stepCount() <= 181);
    // A second caller asking for the same moment does not step again.
    const uint64_t n = c.stepCount();
    c.advanceTo(3.0);
    c.advanceTo(2.0);
    QCOMPARE(c.stepCount(), n);
    (void)d;
  }

  // A stopped sim reports the same tick over and over: the water must freeze.
  void anUnchangedClockMeansNoSteps() {
    const auto b = pond();
    const RippleRotor r = rotorAt(b, 0.3f, 1.0f);
    WaterRipples w;
    w.bindBody(b);
    w.setRotors(&r, 1);
    w.advanceTo(10.0);
    w.advanceTo(10.5);
    const uint64_t n = w.stepCount();
    QVERIFY(n > 0);
    const std::vector<float> before = w.heights();
    for (int i = 0; i < 1000; ++i)
      w.advanceTo(10.5);
    QCOMPARE(w.stepCount(), n);
    QVERIFY(w.heights() == before);
  }

  // Re-Start resets the tick counter: no jump, no stall until it catches up.
  void aClockThatGoesBackwardsReAnchors() {
    const auto b = pond();
    const RippleRotor r = rotorAt(b, 0.3f, 1.0f);
    WaterRipples w;
    w.bindBody(b);
    w.setRotors(&r, 1);
    w.advanceTo(500.0);
    w.advanceTo(500.5);
    const uint64_t n = w.stepCount();
    w.advanceTo(0.2); // back to near zero: re-anchor only
    QCOMPARE(w.stepCount(), n);
    for (int f = 1; f <= 30; ++f) // and half a second of 60 Hz frames runs again
      w.advanceTo(0.2 + double(f) / 60.0);
    QVERIFY(w.stepCount() >= n + 29 && w.stepCount() <= n + 31);
  }

  void aStallDropsTheBacklogInsteadOfSpiralling() {
    const auto b = pond();
    WaterRipples w;
    w.bindBody(b);
    w.advanceTo(0.0);
    w.advanceTo(30.0); // a long hitch
    QVERIFY(w.stepCount() <= uint64_t(WaterRipples::kMaxSubsteps));
    w.advanceTo(30.02); // and the next frame is back to normal
    QVERIFY(w.stepCount() <= uint64_t(WaterRipples::kMaxSubsteps) + 2);
  }

  void bindingTheSamePondKeepsStateAndANewOneResets() {
    const auto b = pond();
    WaterRipples w;
    w.bindBody(b);
    const RippleRotor r = rotorAt(b, 0.3f, 1.0f);
    w.setRotors(&r, 1);
    w.stepFixed(30);
    const float m = w.maxAbsHeight();
    QVERIFY(m > 0.0f);
    w.bindBody(b);
    QCOMPARE(w.maxAbsHeight(), m);
    w.bindBody(pond(900.0f, 100.0f));
    w.stepFixed(1);
    QCOMPARE(w.maxAbsHeight(), 0.0f);
    // The window origin is a whole number of texels.
    const float t = w.originX() / WaterRipples::kCellM;
    QVERIFY(std::fabs(t - std::round(t)) < 1e-3f);
  }

  void halfFloatConversion() {
    QCOMPARE(WaterRipples::floatToHalf(0.0f), uint16_t(0x0000));
    QCOMPARE(WaterRipples::floatToHalf(1.0f), uint16_t(0x3c00));
    QCOMPARE(WaterRipples::floatToHalf(-2.0f), uint16_t(0xc000));
    QCOMPARE(WaterRipples::floatToHalf(0.5f), uint16_t(0x3800));
    QCOMPARE(WaterRipples::floatToHalf(1e9f), uint16_t(0x7c00)); // overflow -> inf
    QCOMPARE(WaterRipples::floatToHalf(1e-12f), uint16_t(0x0000)); // underflow -> 0
    // A centimetre-scale height survives to ~0.1 %.
    const uint16_t hbits = WaterRipples::floatToHalf(0.0123f);
    const int e = (hbits >> 10) & 31, m = hbits & 1023;
    const float back = std::ldexp(1.0f + float(m) / 1024.0f, e - 15);
    QVERIFY(std::fabs(back - 0.0123f) < 0.0123f * 0.002f);
  }
};

QTEST_MAIN(TstWaterRipples)
#include "tst_waterripples.moc"
