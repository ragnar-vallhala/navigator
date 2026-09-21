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

#include <QHostAddress>
#include <QObject>
#include <QTimer>
#include <QUdpSocket>

/**
 * @brief UDP telemetry source — the network twin of SerialManager.
 *
 * Binds a local UDP port and emits each received datagram's bytes via
 * dataReceived(), so DroneProtocol parses a WiFi/UDP stream (from the ESP8266
 * telemetry bridge) exactly like the serial stream. On bind it sends a small
 * "hello" datagram to the broadcast address so the bridge can learn this GCS's
 * address and switch from broadcast to unicast.
 */
class UdpManager : public QObject {
  Q_OBJECT

public:
  explicit UdpManager(QObject *parent = nullptr);

  bool bind(quint16 port);
  void close();
  bool isOpen() const;
  quint16 port() const { return m_port; }

  // Send a command frame back to the bridge (GCS -> FC). Unicasts to the peer
  // learned from received telemetry; broadcasts until one is seen. Mirrors
  // SerialManager::write so MainWindow can route to either transport.
  bool write(const QByteArray &data);

signals:
  void dataReceived(const QByteArray &data);
  void dataSent(const QByteArray &data); // mirrors SerialManager::dataSent
  void connectionStateChanged(bool connected);
  void errorOccurred(const QString &message);

private slots:
  void onReadyRead();

private:
  QUdpSocket *m_sock = nullptr;
  quint16 m_port = 0;
  QHostAddress m_peer; // bridge address, learned from received telemetry
  bool m_havePeer = false;
  QTimer *m_hello =
      nullptr; // periodic re-announce so the bridge keeps unicasting
};
