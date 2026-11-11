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

#include "AttitudeWidget.h"
#include "Drone3DWidget.h"
#include <QGroupBox>
#include <QLabel>
#include <QStackedWidget>

class QVBoxLayout;

/**
 * Home "view3d" panel (homepanel::kView3d): the Attitude group with the 2D / 3D
 * segmented toggle, the firmware state pill, the 2D HUD / 3D airframe stack and
 * the flight-mode pill. The roll/pitch/yaw numbers (NumbersPanel) are added by
 * the owner through bodyLayout() so they still render inside this group.
 */
class AttitudePanel : public QGroupBox {
  Q_OBJECT

public:
  explicit AttitudePanel(QWidget *parent = nullptr);

  AttitudeWidget *attitude() const { return m_attitude; }
  Drone3DWidget *drone3d() const { return m_drone3d; }
  QStackedWidget *stack() const { return m_stack; }
  QLabel *statusLabel() const { return m_statusLabel; }
  QLabel *flightModeLabel() const { return m_flightModeLabel; }
  QVBoxLayout *bodyLayout() const { return m_body; }

signals:
  /// The 2D / 3D toggle was clicked (true = 3D). The panel does not switch the
  /// stack itself; the owner decides (MainWindow::onToggle3d).
  void view3dToggled(bool is3d);

private:
  QVBoxLayout *m_body = nullptr;
  QStackedWidget *m_stack = nullptr;
  AttitudeWidget *m_attitude = nullptr;
  Drone3DWidget *m_drone3d = nullptr;
  QLabel *m_statusLabel = nullptr;
  QLabel *m_flightModeLabel = nullptr;
};
