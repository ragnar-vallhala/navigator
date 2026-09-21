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

#include <QWidget>

// Compact green-HUD artificial horizon: sky/ground fill rotated by roll with a
// pitch ladder, roll arc + pointer, boresight and a heading readout. Styled
// like the in-sim SimHudWidget horizon, sized for a corner overlay. Driven by
// roll/pitch/yaw in degrees (NED aerospace convention).
class HorizonHud : public QWidget {
  Q_OBJECT
public:
  explicit HorizonHud(QWidget *parent = nullptr);

public slots:
  void setAttitude(float rollDeg, float pitchDeg, float yawDeg);

protected:
  void paintEvent(QPaintEvent *) override;

private:
  float roll_ = 0.0f, pitch_ = 0.0f, yaw_ = 0.0f;
};
