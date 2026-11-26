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
#include "ParamService.h"

namespace params {

namespace {
constexpr int kTickMs = 50;
// HEARTBEAT.nav_state: ARMED 4, IN_AIR 5, FAILSAFE 6 are armed; 0..3, 7 and 8
// (CALIBRATING) are not. Anything above 8 is a state this build does not know.
bool armedFor(quint8 navState) { return navState >= 4 && navState != 7 && navState != 8; }
} // namespace

ParamService::ParamService(QObject *parent) : QObject(parent) {
  qRegisterMetaType<Client::WriteEvent>("params::Client::WriteEvent");
  m_elapsed.start();
  m_client.setCatalog(&m_cat);
  m_client.setArmed(true); // unknown counts as armed
  m_client.setLinkSecure(false);
  m_client.setSend([this](const QByteArray &f) {
    if (m_send)
      m_send(f);
  });
  m_client.setLog([this](const QString &s) { emit logLine(s); });
  m_client.setOnWrite([this](const Client::WriteEvent &e) { emit writeFinished(e); });
  m_client.setOnChanged([this] { emit changed(); });
  m_timer.setInterval(kTickMs);
  connect(&m_timer, &QTimer::timeout, this, &ParamService::tick);
}

qint64 ParamService::now() const { return m_clock ? m_clock() : m_elapsed.elapsed(); }

bool ParamService::loadCatalog(const QString &path, QString *err) {
  QString e;
  const bool ok = Catalog::loadFile(path, m_cat, &e);
  if (!ok) {
    emit logLine(QStringLiteral("parameters: catalog not loaded: %1").arg(e));
    if (err)
      *err = e;
  }
  return ok;
}

void ParamService::follow() {
  const bool want = m_client.state() != Client::State::NoCaps;
  if (want && !m_timer.isActive())
    m_timer.start();
  else if (!want && m_timer.isActive())
    m_timer.stop();
}

void ParamService::tick() {
  m_client.tick(now());
  follow();
}

void ParamService::onHeartbeat(quint8 navState, quint32 capabilities) {
  const qint64 t = now();
  // A long silence and then a heartbeat: the vehicle went away and came back
  // (rebooted), and its table, generation and clock are new. Start from scratch.
  // The same if the vehicle says it is starting up (UNINITIALIZED 0 or INIT 1)
  // after it had been past that: a reboot too quick to leave a heartbeat gap.
  const bool gap = m_lastHbMs >= 0 && t - m_lastHbMs > kHbGapMs;
  const bool restarted = m_lastNav >= 2 && navState <= 1;
  const bool returned = gap || restarted;
  m_lastHbMs = t;
  m_lastNav = int(navState);
  if (returned) {
    m_client.onSynced(false, t);
    m_client.onCapabilities(0, t);
  }
  const bool wasArmed = m_client.armed();
  m_client.setArmed(armedFor(navState));
  if (wasArmed != m_client.armed())
    emit changed(); // the panel greys or frees the locked parameters
  // The client is told what Navigator can use: no secure-link session exists.
  m_client.onCapabilities(capabilities & ~Client::kCapSecureLink, t);
  if (returned && m_synced)
    m_client.onSynced(true, t);
  follow();
}

void ParamService::onParamValue(const ParamCodec::ValueMsg &m) {
  m_client.onParamValue(m, now());
  follow();
}

void ParamService::onCommandAck(quint32 command, quint8 reqSeq, quint8 result) {
  m_client.onCommandAck(command, reqSeq, result, now());
  follow();
}

void ParamService::onSynced(bool synced) {
  m_synced = synced;
  m_client.onSynced(synced, now());
  follow();
}

void ParamService::linkDown() {
  m_lastHbMs = -1;
  m_lastNav = -1;
  m_synced = false;
  m_client.setArmed(true);
  m_client.onSynced(false, now());
  m_client.onCapabilities(0, now());
  follow();
}

bool ParamService::set(const QString &id, double value, QString *why) {
  const bool ok = m_client.set(id, value, now(), why);
  follow();
  return ok;
}

bool ParamService::commit(int group, QString *why) {
  const bool ok = m_client.commit(group, now(), why);
  follow();
  return ok;
}

bool ParamService::discard(int group, QString *why) {
  const bool ok = m_client.discard(group, now(), why);
  follow();
  return ok;
}

void ParamService::refresh() {
  m_client.refresh(now());
  follow();
}

} // namespace params
