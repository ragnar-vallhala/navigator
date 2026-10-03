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
// battery::Estimator: when a BATTERY reading is a pack, its cell count, and
// its percent.

#include "BatteryEstimator.h"

#include <QtTest>

namespace {
constexpr uint8_t kGood =
    NAVLINK_BATTERY_FLAGS_PRESENT | NAVLINK_BATTERY_FLAGS_CONVERTED;
}

class TstBatteryEstimator : public QObject {
  Q_OBJECT
private slots:
  void onlyPresentAndConvertedIsAPack();
  void infersCellsFromVoltage();
  void cellCountHoldsThroughSag();
  void overrideWinsOverInference();
  void percentIsLinearPerCellAndClamped();
};

void TstBatteryEstimator::onlyPresentAndConvertedIsAPack() {
  battery::Estimator e;
  // The disconnected-pack case the flags exist for: the divider reads the ESC
  // rail's residual charge and converts it fine, but nothing is PRESENT.
  battery::Reading r = e.update(2.65f, NAVLINK_BATTERY_FLAGS_CONVERTED);
  QVERIFY(!r.pack);
  QCOMPARE(r.status, QString("no pack"));

  r = e.update(16.0f, NAVLINK_BATTERY_FLAGS_PRESENT); // not converted
  QVERIFY(!r.pack);
  r = e.update(16.0f, NAVLINK_BATTERY_FLAGS_TIMEOUT);
  QCOMPARE(r.status, QString("ADC timeout"));
  r = e.update(0.0f, NAVLINK_BATTERY_FLAGS_INIT_FAIL);
  QCOMPARE(r.status, QString("ADC init failed"));
  // Railed passes the presence floor and converts: still not a pack.
  r = e.update(47.0f, kGood | NAVLINK_BATTERY_FLAGS_RAILED);
  QVERIFY(!r.pack);
  QCOMPARE(r.status, QString("ADC railed"));

  r = e.update(16.0f, kGood);
  QVERIFY(r.pack);
  QVERIFY(r.status.isEmpty());
}

void TstBatteryEstimator::infersCellsFromVoltage() {
  // Full and storage-to-empty voltages of common packs all infer their count.
  const struct {
    float v;
    int cells;
  } cases[] = {{12.6f, 3}, {11.1f, 3}, {10.6f, 3}, {16.8f, 4}, {14.8f, 4},
               {14.2f, 4}, {25.2f, 6}, {22.2f, 6}, {4.2f, 1},  {3.7f, 1}};
  for (const auto &c : cases) {
    battery::Estimator e;
    QCOMPARE(e.update(c.v, kGood).cells, c.cells);
    QVERIFY(e.update(c.v, kGood).cellsInferred);
  }
}

void TstBatteryEstimator::cellCountHoldsThroughSag() {
  battery::Estimator e;
  QCOMPARE(e.update(16.4f, kGood).cells, 4);
  // Under load a 4S pack at 3.1 V/cell reads 12.4 V, which alone is "3S".
  QCOMPARE(e.update(12.4f, kGood).cells, 4);
  // Unplugged, then a 3S pack: inferred afresh, not stuck at 4.
  QVERIFY(!e.update(2.6f, NAVLINK_BATTERY_FLAGS_CONVERTED).pack);
  QCOMPARE(e.update(12.4f, kGood).cells, 3);
  // A stale link forgets the count too.
  e.update(16.4f, kGood);
  e.reset();
  QCOMPARE(e.update(12.4f, kGood).cells, 3);
}

void TstBatteryEstimator::overrideWinsOverInference() {
  battery::Estimator e;
  // A 6S pack first seen at 3.3 V/cell (19.8 V) infers as 5S...
  QCOMPARE(e.update(19.8f, kGood).cells, 5);
  // ...which is what the Settings override is for.
  e.setCellsOverride(6);
  const battery::Reading r = e.update(19.8f, kGood);
  QCOMPARE(r.cells, 6);
  QVERIFY(!r.cellsInferred);
  e.setCellsOverride(0); // back to auto
  QVERIFY(e.update(19.8f, kGood).cellsInferred);
}

void TstBatteryEstimator::percentIsLinearPerCellAndClamped() {
  battery::Estimator e;
  e.setCellsOverride(4);
  // The wire voltage is a float (16.8f is 16.7999992...), so compare to within
  // a hundredth of a percent rather than exactly.
  auto pct = [&](float v) { return e.update(v, kGood).percent; };
  QVERIFY(qAbs(pct(16.8f) - 100.0) < 0.01); // 4.20 V/cell
  QVERIFY(qAbs(pct(14.0f) - 0.0) < 0.01);   // 3.50 V/cell
  QVERIFY(qAbs(pct(15.4f) - 50.0) < 0.01);  // 3.85 V/cell
  QCOMPARE(pct(17.2f), 100.0);              // overcharged: clamped
  QCOMPARE(pct(12.8f), 0.0);                // flat (3.2 V/cell): clamped
}

QTEST_MAIN(TstBatteryEstimator)
#include "tst_battery_estimator.moc"
