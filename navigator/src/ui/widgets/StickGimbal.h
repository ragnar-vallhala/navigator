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

#include <QString>
#include <QWidget>

// RC stick-gimbal crosshair (mockup .gimbal): a square box with a dashed centre
// crosshair and a glowing dot showing the current stick position. setPos takes
// normalised axes in [-1, 1] (x right+, y up+); a caption labels the pair.
class StickGimbal : public QWidget {
  Q_OBJECT

public:
  explicit StickGimbal(const QString &caption, QWidget *parent = nullptr);

  // x,y in [-1, 1]. NaN -> no data (dot hidden, "N/A" shown).
  void setPos(float x, float y);

protected:
  void paintEvent(QPaintEvent *event) override;
  QSize sizeHint() const override { return {134, 152}; }

private:
  QString m_caption;
  float m_x = 0.0f;
  float m_y = 0.0f;
  bool m_haveData = false;
};
