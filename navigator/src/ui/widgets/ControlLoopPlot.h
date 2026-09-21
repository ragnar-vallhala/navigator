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

#include "RealTimeGraph.h"
#include "protocol/DroneProtocol.h"
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QQueue>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QWidget>

class ControlLoopPlot : public QWidget {
  Q_OBJECT

public:
  explicit ControlLoopPlot(QWidget *parent = nullptr);
  void setProtocol(DroneProtocol *protocol);

signals:
  void backToHomeRequested();

private slots:
  void onControlLoopDataReceived(const ControlLoopData &data);
  void onImuReceived(const ImuData &data);
  void onEstPerfReceived(const EstPerfData &data);

private:
  DroneProtocol *m_protocol = nullptr;
  // Pause freezes the plots (mockup header Pause); telemetry keeps arriving but
  // the slots stop appending while paused.
  bool m_paused = false;

  RealTimeGraph *m_angleGraph;
  RealTimeGraph *m_rateGraph;
  RealTimeGraph *m_outputGraph;
  RealTimeGraph *m_dtGraph;
  RealTimeGraph *m_estLatGraph;

  // Angle labels
  QLabel *m_rollAngleSpVal;
  QLabel *m_pitchAngleSpVal;
  QLabel *m_yawAngleSpVal;
  QLabel *m_rollAngleCurrVal;
  QLabel *m_pitchAngleCurrVal;
  QLabel *m_yawAngleCurrVal;

  // Rate labels
  QLabel *m_rollRateSpVal;
  QLabel *m_pitchRateSpVal;
  QLabel *m_yawRateSpVal;
  QLabel *m_rollRateCurrVal;
  QLabel *m_pitchRateCurrVal;
  QLabel *m_yawRateCurrVal;

  // Output labels
  QLabel *m_rollOutVal;
  QLabel *m_pitchOutVal;
  QLabel *m_yawOutVal;
  QLabel *m_throttleOutVal;
  QLabel *m_dtOuterVal;
  QLabel *m_dtInnerVal;
  QLabel *m_dtOuterStdVal;
  QLabel *m_dtInnerStdVal;

  // Estimator-cost labels
  QLabel *m_estPeakVal;
  QLabel *m_estMeanVal;
  QLabel *m_estCadenceVal;

  QQueue<float> m_outerDtHistory;
  QQueue<float> m_innerDtHistory;
  const int m_stdWindowSize = 100;

  float m_gyroScale = 1.0f;
};
