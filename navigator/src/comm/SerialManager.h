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

#include <QByteArray>
#include <QObject>
#include <QSerialPort>
#include <QString>
#include <QTimer>

class SerialManager : public QObject {
  Q_OBJECT

public:
  explicit SerialManager(QObject *parent = nullptr);
  ~SerialManager() override;

  bool open(const QString &portName, qint32 baudRate);
  void close();
  bool write(const QByteArray &data);

  bool isOpen() const;
  QString currentPort() const { return m_port.portName(); }

  // Auto-reconnect on transient serial errors (FR-CONN-05 / FR-UX-?).
  // When enabled and the port closes due to an error, we schedule a
  // re-open on the last (port, baud) with a small backoff. Capped at
  // kMaxRetries to avoid spinning if the device is genuinely gone.
  void setAutoReconnect(bool on) { m_autoReconnect = on; }
  bool autoReconnect() const { return m_autoReconnect; }

  // Base retry delay for auto-reconnect (the backoff doubles from here). Lets
  // the Settings page tune the cadence; clamped to a sane floor.
  void setReconnectIntervalMs(int ms) { m_initialDelayMs = qMax(100, ms); }

  // Static helpers
  static QStringList availablePorts();

signals:
  void dataReceived(const QByteArray &data);
  void dataSent(const QByteArray &data);
  void connectionStateChanged(bool connected);
  void errorOccurred(const QString &message);
  // Optional UX hook — fires before each scheduled retry.
  void reconnectAttempt(int attempt, int maxAttempts);

private slots:
  void onReadyRead();
  void onErrorOccurred(QSerialPort::SerialPortError error);
  void tryReconnect();

private:
  QSerialPort m_port;
  QByteArray m_buffer;

  // Auto-reconnect bookkeeping.
  bool m_autoReconnect = false;
  bool m_userClose = false; // true when close() came from UI
  QString m_lastPort;
  qint32 m_lastBaud = 0;
  int m_retryCount = 0;
  QTimer m_retryTimer;
  int m_initialDelayMs = kInitialDelayMs; // configurable base retry delay

  static constexpr int kInitialDelayMs = 1000;
  static constexpr int kMaxDelayMs = 8000;
  static constexpr int kMaxRetries = 5;
};
