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

#include <QLabel>
#include <QWidget>

/**
 * Home "numbers" panel (homepanel::kNumbers): the roll / pitch / yaw readouts,
 * each with its rolling-sigma line, as three equal-stretch columns.
 */
class NumbersPanel : public QWidget {
  Q_OBJECT

public:
  explicit NumbersPanel(QWidget *parent = nullptr);

  QLabel *rollLabel() const { return m_roll; }
  QLabel *pitchLabel() const { return m_pitch; }
  QLabel *yawLabel() const { return m_yaw; }
  QLabel *rollStd() const { return m_rollStd; }
  QLabel *pitchStd() const { return m_pitchStd; }
  QLabel *yawStd() const { return m_yawStd; }

private:
  QLabel *m_roll = nullptr;
  QLabel *m_pitch = nullptr;
  QLabel *m_yaw = nullptr;
  QLabel *m_rollStd = nullptr;
  QLabel *m_pitchStd = nullptr;
  QLabel *m_yawStd = nullptr;
};
