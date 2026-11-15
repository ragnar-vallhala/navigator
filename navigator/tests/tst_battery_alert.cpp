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
// battery::AlertTracker: thresholds, hysteresis, debounce, announce-once.

#include "BatteryAlert.h"

#include <QtTest>

#include <cmath>

namespace {
// A 4S pack reading at `perCell` V/cell, as a valid pack.
battery::Reading pack(double perCell, int cells = 4) {
  battery::Reading r;
  r.pack = true;
  r.cells = cells;
  r.volts = perCell * cells;
  return r;
}
battery::Reading nopack() {
  battery::Reading r;
  r.status = "no pack";
  r.volts = 2.6; // residual charge on the rail: not evidence of a pack
  return r;
}
using battery::AlertLevel;
using battery::AlertTracker;

// Feed the same reading every 100 ms for `ms`; return how many escalations.
int feed(AlertTracker &t, const battery::Reading &r, qint64 &now, qint64 ms,
         AlertTracker::Result *last = nullptr) {
  int esc = 0;
  for (qint64 end = now + ms; now < end; now += 100) {
    const AlertTracker::Result res = t.update(r, now);
    if (res.escalated)
      ++esc;
    if (last)
      *last = res;
  }
  return esc;
}
} // namespace

class TstBatteryAlert : public QObject {
  Q_OBJECT
private slots:
  void okAboveWarn();
  void warnNeedsDwell();
  void briefSagDoesNotAlert();
  void announcedOncePerStep();
  void hysteresisHoldsAndClears();
  void thresholdHoverDoesNotFlap();
  void criticalFromWarnAndBack();
  void usesPerCellVoltage();
  void notAPackNeverAlerts();
  void flickeringPackHoldsLevelSilently();
  void lostPackClearsAfterGrace();
  void crossedThresholdsCollapse();
};

void TstBatteryAlert::okAboveWarn() {
  AlertTracker t;
  qint64 now = 0;
  QCOMPARE(feed(t, pack(3.9), now, 10000), 0);
  QCOMPARE(t.level(), AlertLevel::Ok);
}

void TstBatteryAlert::warnNeedsDwell() {
  AlertTracker t;
  qint64 now = 0;
  feed(t, pack(3.9), now, 500);
  AlertTracker::Result r;
  QCOMPARE(feed(t, pack(3.45), now, AlertTracker::kEnterMs - 100, &r), 0);
  QCOMPARE(t.level(), AlertLevel::Ok);
  QCOMPARE(feed(t, pack(3.45), now, 300, &r), 1);
  QCOMPARE(t.level(), AlertLevel::Warn);
}

void TstBatteryAlert::briefSagDoesNotAlert() {
  AlertTracker t;
  qint64 now = 0;
  for (int i = 0; i < 5; ++i) { // 1 s sags under throttle, 3 s recovered
    QCOMPARE(feed(t, pack(3.2), now, 1000), 0);
    QCOMPARE(feed(t, pack(3.8), now, 3000), 0);
  }
  QCOMPARE(t.level(), AlertLevel::Ok);
}

void TstBatteryAlert::announcedOncePerStep() {
  AlertTracker t;
  qint64 now = 0;
  // 60 s at warn: exactly one announcement, no repeats.
  QCOMPARE(feed(t, pack(3.4), now, 60000), 1);
  QCOMPARE(t.level(), AlertLevel::Warn);
  // then critical: one more.
  QCOMPARE(feed(t, pack(3.2), now, 60000), 1);
  QCOMPARE(t.level(), AlertLevel::Critical);
}

void TstBatteryAlert::hysteresisHoldsAndClears() {
  AlertTracker t;
  qint64 now = 0;
  feed(t, pack(3.4), now, 3000);
  QCOMPARE(t.level(), AlertLevel::Warn);
  // Just above the warn line but inside the hysteresis band: stays warn.
  feed(t, pack(3.52), now, 20000);
  QCOMPARE(t.level(), AlertLevel::Warn);
  // Clear of the band: needs kExitMs, then Ok, and recovery is silent.
  AlertTracker::Result r;
  QCOMPARE(feed(t, pack(3.6), now, AlertTracker::kExitMs - 100, &r), 0);
  QCOMPARE(t.level(), AlertLevel::Warn);
  feed(t, pack(3.6), now, 500, &r);
  QCOMPARE(t.level(), AlertLevel::Ok);
}

void TstBatteryAlert::thresholdHoverDoesNotFlap() {
  AlertTracker t;
  qint64 now = 0;
  int esc = 0;
  int changes = 0;
  // 60 s alternating 1 s either side of the warn line (3.50).
  for (int i = 0; i < 30; ++i) {
    for (double v : {3.48, 3.53}) {
      for (qint64 end = now + 1000; now < end; now += 100) {
        const auto r = t.update(pack(v), now);
        esc += r.escalated;
        changes += r.changed;
      }
    }
  }
  QVERIFY(esc <= 1);
  QVERIFY(changes <= 1);
}

void TstBatteryAlert::criticalFromWarnAndBack() {
  AlertTracker t;
  qint64 now = 0;
  feed(t, pack(3.4), now, 3000);
  QCOMPARE(feed(t, pack(3.25), now, 3000), 1);
  QCOMPARE(t.level(), AlertLevel::Critical);
  // Back above crit+hyst but under warn: drops to Warn, silently.
  QCOMPARE(feed(t, pack(3.4), now, 6000), 0);
  QCOMPARE(t.level(), AlertLevel::Warn);
  // 3.32 is inside crit's hysteresis band; from Critical it stays critical.
  AlertTracker u;
  now = 0;
  feed(u, pack(3.25), now, 3000);
  QCOMPARE(u.level(), AlertLevel::Critical);
  feed(u, pack(3.32), now, 20000);
  QCOMPARE(u.level(), AlertLevel::Critical);
}

void TstBatteryAlert::usesPerCellVoltage() {
  AlertTracker t;
  qint64 now = 0;
  // 12.4 V is fine on 3S (4.13/cell) and a warning on 4S (3.1/cell)... both
  // carried by the Reading's own cell count, which already honours Settings.
  feed(t, pack(4.13, 3), now, 5000);
  QCOMPARE(t.level(), AlertLevel::Ok);
  battery::Reading four;
  four.pack = true;
  four.volts = 12.4;
  four.cells = 4;
  AlertTracker::Result r;
  feed(t, four, now, 3000, &r);
  QCOMPARE(t.level(), AlertLevel::Critical); // 3.1 V/cell < 3.30
  QVERIFY(std::abs(r.perCellV - 3.1) < 1e-9);
}

void TstBatteryAlert::notAPackNeverAlerts() {
  AlertTracker t;
  qint64 now = 0;
  QCOMPARE(feed(t, nopack(), now, 30000), 0);
  QCOMPARE(t.level(), AlertLevel::Ok);
  battery::Reading bad = pack(2.0); // low, but flags say it is not a pack
  bad.pack = false;
  QCOMPARE(feed(t, bad, now, 30000), 0);
  QCOMPARE(t.level(), AlertLevel::Ok);
}

void TstBatteryAlert::flickeringPackHoldsLevelSilently() {
  AlertTracker t;
  qint64 now = 0;
  feed(t, pack(3.2), now, 3000);
  QCOMPARE(t.level(), AlertLevel::Critical);
  int esc = 0;
  for (int i = 0; i < 20; ++i) { // PRESENT blinking: 1 s gone, 1 s back
    esc += feed(t, nopack(), now, 1000);
    esc += feed(t, pack(3.2), now, 1000);
  }
  QCOMPARE(esc, 0);
  QCOMPARE(t.level(), AlertLevel::Critical);
}

void TstBatteryAlert::lostPackClearsAfterGrace() {
  AlertTracker t;
  qint64 now = 0;
  feed(t, pack(3.2), now, 3000);
  feed(t, nopack(), now, AlertTracker::kLostMs - 100);
  QCOMPARE(t.level(), AlertLevel::Critical);
  feed(t, nopack(), now, 300);
  QCOMPARE(t.level(), AlertLevel::Ok);
  // The next low pack announces afresh.
  QCOMPARE(feed(t, pack(3.2), now, 3000), 1);
}

void TstBatteryAlert::crossedThresholdsCollapse() {
  AlertTracker t;
  t.setThresholds(3.40, 3.60);
  QCOMPARE(t.critV(), 3.40);
  qint64 now = 0;
  feed(t, pack(3.45), now, 5000);
  QCOMPARE(t.level(), AlertLevel::Ok);
}

QTEST_MAIN(TstBatteryAlert)
#include "tst_battery_alert.moc"
