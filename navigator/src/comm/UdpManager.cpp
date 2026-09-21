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
#include "UdpManager.h"

#include <QHostAddress>
#include <QNetworkDatagram>

UdpManager::UdpManager(QObject *parent) : QObject(parent) {
  // Re-broadcast a hello every second. The bridge sets its telemetry
  // destination from whoever it last heard from, so this keeps it locked onto
  // us with reliable (MAC-acked, retried) unicast instead of falling back to
  // lossy broadcast — and recovers within ~1 s if the bridge reboots.
  m_hello = new QTimer(this);
  m_hello->setInterval(1000);
  connect(m_hello, &QTimer::timeout, this, [this]() {
    if (!m_sock)
      return;
    // Once we've heard the bridge's telemetry we know its address, so unicast
    // the hello straight to it — a limited broadcast (255.255.255.255) often
    // egresses the wrong interface on a multi-homed host and never reaches it.
    // Broadcast only as the initial bootstrap, before we've learned the peer.
    const QHostAddress dst =
        m_havePeer ? m_peer : QHostAddress(QHostAddress::Broadcast);
    m_sock->writeDatagram(QByteArrayLiteral("GCS-HELLO"), dst, m_port);
  });
}

bool UdpManager::bind(quint16 port) {
  close();
  m_sock = new QUdpSocket(this);
  if (!m_sock->bind(QHostAddress::AnyIPv4, port,
                    QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint)) {
    emit errorOccurred(
        QString("bind :%1 failed: %2").arg(port).arg(m_sock->errorString()));
    m_sock->deleteLater();
    m_sock = nullptr;
    return false;
  }
  m_port = port;
  connect(m_sock, &QUdpSocket::readyRead, this, &UdpManager::onReadyRead);

  // Announce ourselves so the bridge learns our address and can unicast to us
  // (and works on networks where it would otherwise only broadcast).
  m_sock->writeDatagram(QByteArrayLiteral("GCS-HELLO"), QHostAddress::Broadcast,
                        port);
  m_hello->start();

  emit connectionStateChanged(true);
  return true;
}

void UdpManager::close() {
  if (m_sock) {
    m_hello->stop();
    m_sock->close();
    m_sock->deleteLater();
    m_sock = nullptr;
    m_port = 0;
    emit connectionStateChanged(false);
  }
}

bool UdpManager::isOpen() const {
  return m_sock && m_sock->state() == QAbstractSocket::BoundState;
}

void UdpManager::onReadyRead() {
  while (m_sock && m_sock->hasPendingDatagrams()) {
    const QNetworkDatagram dg = m_sock->receiveDatagram();
    const QByteArray data = dg.data();
    // Our own broadcast hello loops back to us — ignore it so it neither
    // pollutes the parser/metrics nor mislearns the peer as ourselves.
    if (data == QByteArrayLiteral("GCS-HELLO"))
      continue;
    if (!dg.senderAddress().isNull()) {
      m_peer = dg.senderAddress(); // remember the bridge for the return path
      m_havePeer = true;
    }
    if (!data.isEmpty())
      emit dataReceived(data);
  }
}

bool UdpManager::write(const QByteArray &data) {
  if (!m_sock)
    return false;
  const QHostAddress dst =
      m_havePeer ? m_peer : QHostAddress(QHostAddress::Broadcast);
  const bool ok = m_sock->writeDatagram(data, dst, m_port) == data.size();
  if (ok)
    emit dataSent(data);
  return ok;
}
