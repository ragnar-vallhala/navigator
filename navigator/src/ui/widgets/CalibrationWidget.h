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
#pragma once

#include "../../protocol/DroneProtocol.h"
#include "../core/CalibrationWizard.h"
#include "../core/Types.h"
#include <QButtonGroup>
#include <QGroupBox>
#include <QLabel>
#include <QListWidget>
#include <QMap>
#include <QProgressBar>
#include <QPushButton>
#include <QRadioButton>
#include <QStackedWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

class CalibrationWidget : public QWidget {
  Q_OBJECT

public:
  explicit CalibrationWidget(QWidget *parent = nullptr);
  void setProtocol(DroneProtocol *protocol);

public slots:
  // Enable / disable the calibration action surface based on the
  // current serial connection. When disconnected, sensor cards +
  // Start / Cancel are disabled and the footer reads "NOT CONNECTED"
  // so the user doesn't trigger a no-op (FR-UX-18 / Phase-0 0o).
  void setConnected(bool connected);

  // The vehicle's state name (STANDBY, ARMED, ...). Start is enabled only in
  // STANDBY/PREARM, Cancel never while armed or flying (core/ArmGate.h).
  void setVehicleState(const QString &state);

  // COMMAND_ACK from the FC. The widget shows "started" / "canceled" only on
  // the ACK of its own CMD_CALIBRATE_IMU, and the refusal if it is not accepted.
  void onCommandAck(quint32 command, quint8 reqSeq, quint8 result);

signals:
  void backToHomeRequested();
  void commandRequested(const QByteArray &data);

private slots:
  void onStartClicked();
  void onCancelClicked();
  void onStatusReceived(const QString &msg);
  void onSensorSelected(int id);
  void onProgressReceived(float pct);
  void onInstructionReceived(int type);

private:
  enum class Awaiting { None, Start, Cancel };
  void updateGate();
  void resetToIdle(const QString &status, const QString &hint);
  bool m_connected = false;
  bool m_running = false; // a calibration was started and has not ended
  QString m_vehicleState;
  Awaiting m_awaiting = Awaiting::None;
  QTimer m_ackTimer;

private:
  DroneProtocol *m_protocol = nullptr;

  // Sensor Selection Cards
  QWidget *m_sensorSelectArea;
  QPushButton *m_accBtn;
  QPushButton *m_gyrBtn;
  QPushButton *m_magBtn;
  QPushButton *m_lvlBtn;   // board-level / mounting-tilt trim (imu_id 4)
  int m_selectedImuId = 2; // Default to Gyro

  // Configuration Area
  QGroupBox *m_configGroup;
  QButtonGroup *m_typeGroup;
  QRadioButton *m_biasOnlyRadio;
  QRadioButton *m_fullCalibRadio;

  // Interactive Instruction Panel
  QGroupBox *m_instructionGroup;
  QLabel *m_instructionText;
  QProgressBar *m_progressBar;

  // Axis-status pills (the 6 face poses of the accel calibration)
  QWidget *m_axisStatusArea;
  QLabel *m_axisLabelX;
  QLabel *m_axisLabelY;
  QLabel *m_axisLabelZ;
  QLabel *m_axisLabelNX;
  QLabel *m_axisLabelNY;
  QLabel *m_axisLabelNZ;

  QMap<CalibUpdateType, QLabel *> m_axisMap;
  CalibUpdateType m_currentAxis = CalibUpdateType::Progress;

  // Gated step wizard: the guided checklist driven by firmware instructions.
  CalibrationWizard m_wizard;
  QListWidget *m_stepList = nullptr;
  CalibMode currentMode() const;
  void refreshSteps();

  // Terminal handler for a running calibration. Driven by the firmware's
  // explicit CALIBRATION_STATUS COMPLETE/FAILED steps (not by inferring a
  // STANDBY heartbeat — ending in FAILSAFE is a valid, non-failure outcome).
  void finishCalibration(bool success);

  // Main Controls
  QPushButton *m_startBtn;
  QPushButton *m_cancelBtn;
  QLabel *m_statusLabel;

  void sendCalibrationCommand(int imu_id, int type);
};
