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

#include "Types.h"
#include "AttitudeSink.h"
#include "GlPaintHost.h"
#include <QOpenGLFunctions>
#include <QOpenGLWidget>

class QTimer;

/**
 * An artificial horizon widget drawn with QPainter.
 *
 * Displays a sky/ground split rotated by roll with a pitch offset,
 * compass heading (yaw) in a bottom strip, and wing level markers.
 */
// The drawing and the eased angles of the artificial horizon, shared by both
// views below (one implementation of the picture).
class AttitudeScene {
public:
  // Set the target; true when this was the first sample (snapped, repaint now).
  bool setTarget(float roll, float pitch, float yaw);
  void tick(); // ease the displayed angles toward the target (one frame)
  void paint(QPainter &p, int W, int H);

private:
  // Banked-horizon ADI (mockup style): the sky/ground horizon tilts with roll
  // and shifts with pitch inside `adi`; a roll-arc + bank pointer sit on top, a
  // scrolling heading tape runs along the bottom strip, fixed wings in the
  // centre. `adi` is the viewport rect above the tape.
  void drawHorizon(QPainter &p, const QRectF &adi);
  void drawPitchLadder(QPainter &p, const QRectF &adi);
  void drawRollArc(QPainter &p, const QRectF &adi);
  void drawAircraftRef(QPainter &p, const QRectF &adi);
  void drawHeadingTape(QPainter &p, const QRectF &tape);

  // Displayed angles are eased toward the latest target each frame
  // so bursty / low-rate telemetry renders as smooth motion instead of snapping.
  float m_roll = 0.0f;
  float m_pitch = 0.0f;
  float m_yaw = 0.0f;
  float m_targetRoll = 0.0f;
  float m_targetPitch = 0.0f;
  float m_targetYaw = 0.0f;
  bool m_haveTarget = false; // snap to the first sample, ease after that
};

class AttitudeWidget : public QOpenGLWidget,
                       public AttitudeSink,
                       protected QOpenGLFunctions {
  Q_OBJECT

public:
  explicit AttitudeWidget(QWidget *parent = nullptr);

  QSize minimumSizeHint() const override { return {260, 260}; }
  QSize sizeHint() const override { return {320, 320}; }

public slots:
  void setAttitude(const AttitudeData &att) override;
  void setAttitude(float roll, float pitch, float yaw) override;

protected:
  void initializeGL() override;
  void resizeGL(int w, int h) override;
  void paintGL() override;
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

private:
  AttitudeScene m_scene;
  QTimer *m_smoothTimer = nullptr;
};

// The same view for the dock host: a QOpenGLWindow in a window container
// (GlPaintHost), which keeps one GL context while ADS re-parents the panel.
class AttitudeDockedWidget : public GlPaintHost, public AttitudeSink {
  Q_OBJECT

public:
  explicit AttitudeDockedWidget(QWidget *parent = nullptr);

  QSize minimumSizeHint() const override { return {260, 260}; }
  QSize sizeHint() const override { return {320, 320}; }

  void setAttitude(const AttitudeData &att) override;
  void setAttitude(float roll, float pitch, float yaw) override;

protected:
  void paintScene(QPainter &p) override;
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

private:
  AttitudeScene m_scene;
  QTimer *m_smoothTimer = nullptr;
};
