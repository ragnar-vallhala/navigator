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

#include "../../core/Types.h"
#include "AuxSwitch.h"
#include "RealTimeGraph.h"
#include "StickGimbal.h"
#include <QLabel>
#include <QProgressBar>
#include <QVector>
#include <QWidget>

// RC page — matches the mockup layout: a top row of Sticks (dual gimbals),
// Link Health, and Switches, then the named channel bars. Link-health fields
// have no telemetry source yet, so they show the mockup's default constants
// (per the "keep the widget, default constant / N-A" policy).
class RcChannelsWidget : public QWidget {
  Q_OBJECT

public:
  explicit RcChannelsWidget(QWidget *parent = nullptr);

signals:
  void backToHomeRequested();

public slots:
  void updateChannels(const RcData &data);

private:
  StickGimbal *m_stickL = nullptr; // Throttle / Yaw
  StickGimbal *m_stickR = nullptr; // Pitch / Roll
  AuxSwitch *m_aux[4] = {nullptr, nullptr, nullptr, nullptr};
  QVector<QProgressBar *> m_bars; // 8 named channels
  QVector<QLabel *> m_labels;
  RealTimeGraph *m_history = nullptr; // 8-trace channel history (aux dotted)
};
