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

#include "ParamCatalog.h"
#include "ParamClient.h"

#include <QElapsedTimer>
#include <QObject>
#include <QTimer>
#include <functional>

namespace params {

// The GUI-thread owner of the parameter client: it holds the catalog (a Qt
// resource built from the pinned navlink's params_catalog.json), keeps the
// clock, ticks the client while it has anything to do, and turns link events
// into the client's inputs. It has no widgets; the panel reads it.
//
// Armed: taken from HEARTBEAT.nav_state (ARMED, IN_AIR, FAILSAFE), the field the
// UI shows. Until a heartbeat has been seen, and for a nav_state it does not
// know, the vehicle counts as ARMED: the locked-group check fails safe.
//
// Secure link: Navigator has no secure-link session yet, so the SECURE_LINK
// capability is hidden from the client and a refusal is reported in its generic
// words, never with a secure-link cause nobody can act on.
class ParamService : public QObject {
  Q_OBJECT
public:
  // No heartbeat for this long and the vehicle is taken to have gone away (a
  // reboot): when heartbeats resume the table is read again from scratch.
  static constexpr qint64 kHbGapMs = 3000;
  static constexpr const char *kResourcePath = ":/params/params_catalog.json";

  explicit ParamService(QObject *parent = nullptr);

  // Loads the catalog; with no argument the one built into the app. A bad
  // catalog leaves the service in NoCaps (nothing is read or written).
  bool loadCatalog(const QString &path = QString::fromLatin1(kResourcePath),
                   QString *err = nullptr);
  void setSendFrame(std::function<void(const QByteArray &)> f) { m_send = std::move(f); }
  // Tests: replace the clock (ms).
  void setClock(std::function<qint64()> c) { m_clock = std::move(c); }

  const Catalog &catalog() const { return m_cat; }
  const Client &client() const { return m_client; }

  // Requests (the panel). Same contract as Client.
  // While the service is locked (an apply is running, see ParamApplier) every
  // request is refused with the lock's reason, except the owner's (`owner`).
  bool set(const QString &id, double value, QString *why = nullptr, bool owner = false);
  bool commit(int group, QString *why = nullptr, bool owner = false);
  bool discard(int group, QString *why = nullptr, bool owner = false);
  void lockFor(const QString &reason); // empty: unlock
  bool locked() const { return !m_lock.isEmpty(); }
  void refresh();
  bool ticking() const { return m_timer.isActive(); }
  // What the panel greys its buttons with (see Client).
  QString whyNotCommit(int group) const { return m_lock.isEmpty() ? m_client.whyNotCommit(group) : m_lock; }
  QString whyNotDiscard(int group) const { return m_lock.isEmpty() ? m_client.whyNotDiscard(group) : m_lock; }
  bool armed() const { return m_client.armed(); }
  void tick(); // the timer calls it; tests drive it with their own clock

public slots:
  void onHeartbeat(quint8 navState, quint32 capabilities);
  void onParamValue(const params::ParamCodec::ValueMsg &m);
  void onCommandAck(quint32 command, quint8 reqSeq, quint8 result);
  void onSynced(bool synced);
  void linkDown(); // transport gone: nothing is known about the vehicle any more

signals:
  void changed();
  void writeFinished(const params::Client::WriteEvent &e);
  void logLine(const QString &line);

private:
  qint64 now() const;
  void follow(); // run the timer exactly while the client can have work

  Catalog m_cat;
  Client m_client;
  QTimer m_timer;
  QElapsedTimer m_elapsed;
  std::function<qint64()> m_clock;
  std::function<void(const QByteArray &)> m_send;
  qint64 m_lastHbMs = -1; // -1: none seen on this link
  int m_lastNav = -1;     // the last nav_state, -1: none
  bool m_synced = false;  // the last sync state the app reported
  QString m_lock;         // why requests are refused right now; empty: not locked
};

} // namespace params

Q_DECLARE_METATYPE(params::Client::WriteEvent)
