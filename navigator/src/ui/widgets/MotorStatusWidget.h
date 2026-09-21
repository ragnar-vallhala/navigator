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
#include "RollingStats.h"
#include <QLabel>
#include <QVector>
#include <QWidget>

class DroneViewWidget;

class MotorStatusWidget : public QWidget {
  Q_OBJECT

public:
  explicit MotorStatusWidget(QWidget *parent = nullptr);
  ~MotorStatusWidget() override = default;

  void setMotorSpeeds(const QVector<float> &speeds);

signals:
  void backToHomeRequested();

private:
  DroneViewWidget *m_droneView;
  RealTimeGraph *m_graph;
  QLabel *m_valLabels[4];
  QLabel *m_stdLabels[4];
  // Power group (mockup .gframe "Power"). Avg-throttle / max-spread are derived
  // from the live motor speeds; battery / current / ESC-temp / mAh are
  // placeholders until power telemetry exists (keep-the-widget policy).
  QLabel *m_avgThrottle = nullptr;
  QLabel *m_maxSpread = nullptr;
  QLabel *m_battery = nullptr;
  QLabel *m_current = nullptr;
  QLabel *m_escTemp = nullptr;
  QLabel *m_mahUsed = nullptr;
  RollingStats m_stats[4];
  QVector<float> m_speeds;
};
