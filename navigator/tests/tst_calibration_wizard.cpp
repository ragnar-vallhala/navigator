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
#include <QtTest>

#include "CalibrationWizard.h"

// Gated calibration wizard step machine: per-mode step lists, firmware-driven
// advancement (current + done tracking), progress, and completion.
class TstCalibrationWizard : public QObject {
  Q_OBJECT

private slots:
  void modesHaveExpectedSteps();
  void sixAxisAdvancesAndCompletes();
  void advancesByArrivalOrderNotCatalog();
  void nonStepInstructionIgnored();
  void magIsSingleFigure8();
};

void TstCalibrationWizard::modesHaveExpectedSteps() {
  CalibrationWizard w;
  w.begin(CalibMode::Gyro);
  QCOMPARE(w.stepCount(), 1);
  w.begin(CalibMode::AccelBias);
  QCOMPARE(w.stepCount(), 1);
  w.begin(CalibMode::Accel6Axis);
  QCOMPARE(w.stepCount(), 6); // six faces — the firmware's active SIXPOINT fit
  w.begin(CalibMode::Mag);
  QCOMPARE(w.stepCount(), 1);
  QCOMPARE(w.steps().first().orient, CalibUpdateType::FreeRot);
  w.begin(CalibMode::BoardLevel);
  QCOMPARE(w.stepCount(), 1);
  QCOMPARE(w.steps().first().orient, CalibUpdateType::BoardLevel);
}

void TstCalibrationWizard::sixAxisAdvancesAndCompletes() {
  CalibrationWizard w;
  w.begin(CalibMode::Accel6Axis);
  QVERIFY(w.isActive());
  QVERIFY(!w.isComplete());
  QCOMPARE(w.currentIndex(), -1);
  QCOMPARE(w.progress(), 0.0);

  w.onInstruction(CalibUpdateType::NoseUp);
  QCOMPARE(w.currentIndex(), 0);
  QCOMPARE(w.doneCount(), 0);

  w.markComplete();
  QVERIFY(w.isComplete());
  QCOMPARE(w.doneCount(), 6);
  QCOMPARE(w.progress(), 1.0);
  QVERIFY(!w.isActive());
}

void TstCalibrationWizard::advancesByArrivalOrderNotCatalog() {
  CalibrationWizard w;
  w.begin(CalibMode::Accel6Axis);

  // The firmware owns the pose sequence; whatever it prompts first is step 0 of
  // the run with nothing yet done — regardless of where that pose sits in the
  // GCS catalog. Progress must track the firmware's run order, not the catalog
  // index, so the bar never moves backwards.
  w.onInstruction(CalibUpdateType::RightDown);
  QCOMPARE(w.currentIndex(), 0);
  QCOMPARE(w.doneCount(), 0);
  QCOMPARE(w.progress(), 0.0);
  QCOMPARE(w.steps().at(0).orient, CalibUpdateType::RightDown);

  // Each new prompt advances monotonically: one pose done, now on the second,
  // and the checklist reflects the real arrival order.
  w.onInstruction(CalibUpdateType::NoseUp);
  QCOMPARE(w.currentIndex(), 1);
  QCOMPARE(w.doneCount(), 1);
  QVERIFY(qFuzzyCompare(w.progress(), 1.0 / 6.0));
  QCOMPARE(w.steps().at(1).orient, CalibUpdateType::NoseUp);

  // A re-prompt of an already-completed pose must never move backwards.
  w.onInstruction(CalibUpdateType::RightDown);
  QCOMPARE(w.currentIndex(), 1);
  QCOMPARE(w.doneCount(), 1);
}

void TstCalibrationWizard::nonStepInstructionIgnored() {
  CalibrationWizard w;
  w.begin(CalibMode::Accel6Axis);
  w.onInstruction(CalibUpdateType::LeftDown); // first arrival -> slot 0
  QCOMPARE(w.currentIndex(), 0);
  // A Progress "still calibrating" status must not move the wizard.
  w.onInstruction(CalibUpdateType::Progress);
  QCOMPARE(w.currentIndex(), 0);
  QCOMPARE(w.doneCount(), 0);
}

void TstCalibrationWizard::magIsSingleFigure8() {
  CalibrationWizard w;
  w.begin(CalibMode::Mag);
  w.onInstruction(CalibUpdateType::FreeRot);
  QCOMPARE(w.currentIndex(), 0);
  w.markComplete();
  QVERIFY(w.isComplete());
  QCOMPARE(w.progress(), 1.0);
}

QTEST_APPLESS_MAIN(TstCalibrationWizard)
#include "tst_calibration_wizard.moc"
