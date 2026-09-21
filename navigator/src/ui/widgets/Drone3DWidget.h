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

#include "../core/Types.h"
#include <QMatrix4x4>
#include <QOpenGLWidget>
#include <QVector3D>

class QTimer;

// Stylised quad-copter attitude view, mirroring the UI mockup's #adi3d: an
// X-frame seen from a tilted top-down angle — two crossed arms, four motor pods
// (front red, rear green), an accent "nose" heading marker and spinning props,
// over a soft ground shadow. Banks/pitches/yaws with the live attitude.
class Drone3DWidget : public QOpenGLWidget {
  Q_OBJECT

public:
  explicit Drone3DWidget(QWidget *parent = nullptr);
  void setAttitude(const AttitudeData &att);
  void setAttitude(float roll, float pitch, float yaw);

protected:
  void paintEvent(QPaintEvent *event) override;
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

private:
  // Displayed angles are eased toward the latest target each frame (spin timer)
  // so bursty / low-rate telemetry renders as smooth motion instead of snapping.
  float m_roll = 0.0f;
  float m_pitch = 0.0f;
  float m_yaw = 0.0f;
  float m_targetRoll = 0.0f;
  float m_targetPitch = 0.0f;
  float m_targetYaw = 0.0f;
  bool m_haveTarget = false; // snap to the first sample, ease after that
  float m_propPhase = 0.0f;  // degrees, advanced by the spin timer
  QTimer *m_spinTimer = nullptr;
};
