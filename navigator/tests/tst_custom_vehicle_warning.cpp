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
// CustomVehicleWarning: warn on an untuned Custom vehicle, never on a preset,
// and stop nagging once the autotuner result was applied for this vehicle.

#include "CustomVehicleWarning.h"

#include <QtTest>

class TstCustomVehicleWarning : public QObject {
  Q_OBJECT
private slots:
  void customUntunedWarns() { QVERIFY(shouldWarnCustomUntuned(true, false)); }
  void presetNeverWarns() {
    QVERIFY(!shouldWarnCustomUntuned(false, false));
    QVERIFY(!shouldWarnCustomUntuned(false, true));
  }
  void customAfterAutotuneDoesNotWarn() {
    QVERIFY(!shouldWarnCustomUntuned(true, true));
  }

  void stateLifecycle() {
    CustomTuneState s;
    QVERIFY(s.shouldWarn(true));
    s.onGainsApplied();
    QVERIFY(!s.shouldWarn(true));
    s.onVehicleChanged(); // geometry edited / preset chosen: result is stale
    QVERIFY(s.shouldWarn(true));
    QVERIFY(!s.shouldWarn(false)); // a tuned preset clears the warning
  }
};

QTEST_MAIN(TstCustomVehicleWarning)
#include "tst_custom_vehicle_warning.moc"
