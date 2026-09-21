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

#include <QElapsedTimer>
#include <QLabel>
#include <QMap>
#include <QTimer>
#include <QWidget>

class DroneProtocol;

/**
 * A horizontal ribbon that displays the frequency (Hz) of various received
 * packets. Includes smoothing (EMA) and a premium "pill" aesthetic.
 */
class FrequencyRibbon : public QWidget {
  Q_OBJECT

public:
  explicit FrequencyRibbon(QWidget *parent = nullptr);
  void setProtocol(DroneProtocol *protocol);

public slots:
  void packetReceived(const QString &pktType);

private slots:
  void onUpdateTimer();

private:
  struct PacketStats {
    uint32_t count = 0;
    float currentHz = 0.0f;
    float smoothedHz = 0.0f;
    QLabel *label = nullptr;
    QString color;
    QString displayName;
  };

  void addPacketType(const QString &key, const QString &displayName,
                     const QString &color);
  void updateLabels();
  QString getPillStyle(const QString &color);

  QMap<QString, PacketStats> m_stats;
  QTimer *m_timer = nullptr;
  QElapsedTimer m_elapsed;
  qint64 m_lastUpdateTime = 0;

  // Smoothing factor (0.0 to 1.0). Higher = more responsive, lower = smoother.
  const float m_alpha = 0.2f;
};
