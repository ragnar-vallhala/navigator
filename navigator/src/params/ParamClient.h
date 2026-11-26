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
#include "ParamCodec.h"

#include <QHash>
#include <QMap>
#include <QString>
#include <QVector>
#include <functional>

// The GCS side of the parameter service (NavLink spec 12.2.1) as a pure state
// machine: no event loop, no clock. The caller feeds it what arrives (PARAM_VALUE,
// COMMAND_ACK, capability and sync changes), calls tick(nowMs) regularly, and
// passes the frames it wants sent through the Send callback. That makes every
// path testable against a fake flight controller.
//
// What it guarantees:
//  - Nothing is sent unless the vehicle sets capability PARAMS, the link is
//    synchronised and the catalog loaded.
//  - The table CRC is read first. A CRC, a parameter count, an id or a type that
//    differs from the catalog is Mismatch: values are shown raw and nothing can
//    be written.
//  - A write is validated against the catalog before it is sent. It is never
//    shown as applied from the ACK alone: the PARAM_VALUE echo is the truth.
//  - ACKs are matched by (command, req_seq); a stale one is ignored.
//  - A write refused as "not synchronised" is retried once, soon after the sync
//    edge, and only if the user has not edited since.
//  - A gap in the list is requested again, a bounded number of times, then
//    reported: the state is Incomplete, never a silently short table.
namespace params {

class Client {
public:
  enum class State {
    NoCaps,     // the vehicle does not offer PARAMS (or none seen yet)
    Unsynced,   // waiting for the time-sync edge
    ReadingCrc, // asked for SYS_PTBL_CRC
    NoAnswer,   // the CRC read got no answer; refresh() or the next sync edge retries
    Listing,    // reading the table
    Ready,      // table complete and equal to the catalog
    Mismatch,   // vehicle table differs from the catalog: read-only, raw
    Incomplete, // some values never arrived after the retries
  };
  enum class Op { Set, Commit, Discard };
  // Unknown: the link or the capability went away with the request in flight, so
  // the vehicle may or may not have applied it. The values are read again.
  enum class Outcome { Applied, Refused, Rejected, Failed, TimedOut, Unknown };

  struct WriteEvent {
    Op op = Op::Set;
    QString id;     // the parameter (Set)
    int group = 0;  // the group (Commit / Discard)
    Outcome outcome = Outcome::Applied;
    QString text;
  };
  struct Value {
    double v = 0;
    quint8 flags = 0; // ValueFlag bits
    quint32 generation = 0;
    bool have = false;
  };

  using Send = std::function<void(const QByteArray &)>;
  using Log = std::function<void(const QString &)>;
  using OnWrite = std::function<void(const WriteEvent &)>;
  using OnChanged = std::function<void()>;

  // Timing (ms).
  static constexpr qint64 kCrcRetryMs = 1000;
  static constexpr int kCrcTries = 3;
  static constexpr qint64 kQuietMs = 500;      // no PARAM_VALUE for this long
  static constexpr int kReadRetries = 3;       // per missing index
  static constexpr int kReadBatch = 8;         // re-requests per quiet window
  static constexpr qint64 kAckMs = 1500;       // resend once, then give up
  // A commit or discard is answered after the vehicle has saved (it may take
  // seconds, and says IN_PROGRESS while it works): wait this long for the ack,
  // without resending, before giving up.
  static constexpr qint64 kCommitAckMs = 5000;
  static constexpr qint64 kEchoMs = 1000;      // ACK seen, echo not: read it
  static constexpr qint64 kRetryWindowMs = 5000; // after the sync edge
  static constexpr qint64 kSettleMs = 2000;    // generation jumps we caused
  static constexpr qint64 kRelistGapMs = 5000; // at most one relist per this

  // The ACK result codes (dialect command_result).
  static constexpr int kAccepted = 0, kTempRejected = 1, kDenied = 2,
                       kUnsupported = 3, kFailed = 4, kInProgress = 5;
  static constexpr quint32 kCapSecureLink = 4, kCapParams = 8;
  static constexpr quint32 kMsgidParamSet = 12291, kMsgidParamCommit = 8208;

  void setCatalog(const Catalog *c) { m_cat = c; }
  void setSend(Send s) { m_send = std::move(s); }
  void setLog(Log l) { m_log = std::move(l); }
  void setOnWrite(OnWrite w) { m_onWrite = std::move(w); }
  void setOnChanged(OnChanged c) { m_onChanged = std::move(c); }
  void setDevId(quint8 id) { m_dev = id; }

  // Inputs.
  void onCapabilities(quint32 caps, qint64 nowMs);
  void onSynced(bool synced, qint64 nowMs);
  void setArmed(bool armed) { m_armed = armed; }
  void setLinkSecure(bool secure) { m_secure = secure; }
  void onParamValue(const ParamCodec::ValueMsg &m, qint64 nowMs);
  void onCommandAck(quint32 command, quint8 reqSeq, quint8 result, qint64 nowMs);
  void tick(qint64 nowMs);

  // Requests. They return false (with the reason in *why) when refused here;
  // true means the frame was sent and a WriteEvent will follow.
  bool set(const QString &id, double value, qint64 nowMs, QString *why = nullptr);
  bool commit(int group, qint64 nowMs, QString *why = nullptr);
  bool discard(int group, qint64 nowMs, QString *why = nullptr);
  void refresh(qint64 nowMs); // read the whole table again
  // Why a commit / discard of `group` would be refused here (empty: it would be
  // sent). The panel greys its buttons with this; commit()/discard() use it too.
  QString whyNotCommit(int group) const;
  QString whyNotDiscard(int group) const;

  // Outputs.
  State state() const { return m_state; }
  QString stateText() const;
  const Value *value(const QString &id) const;
  const QMap<int, ParamCodec::ValueMsg> &raw() const { return m_raw; }
  QVector<int> missing() const; // indexes never received (Listing/Incomplete)
  int pendingCount() const { return m_pend.size(); }
  int staleAcks() const { return m_staleAcks; }
  bool hasSecureLinkCap() const { return m_caps & kCapSecureLink; }
  bool armed() const { return m_armed; }

private:
  struct Pending {
    Op op = Op::Set;
    quint8 reqSeq = 0;
    int entry = -1;
    int group = 0;
    double value = 0;
    quint32 serial = 0;   // edit serial of the entry (Set) / of the user (Commit)
    qint64 sentAt = 0;
    bool resent = false;  // the same frame sent once more after kAckMs
    bool retried = false; // re-sent after TEMPORARILY_REJECTED
    bool acked = false;   // ACCEPTED seen, waiting for the echo
    qint64 ackedAt = 0;
    bool echoed = false;
    bool readSent = false;
    double echoValue = 0;
    quint8 echoFlags = 0;
  };

  void start(qint64 nowMs);
  void toMismatch(const QString &why);
  void event(Op op, const QString &id, int group, Outcome o, const QString &text);
  void failAllPending(Outcome o, const QString &text);
  void changed() { if (m_onChanged) m_onChanged(); }
  void log(const QString &t) { if (m_log) m_log(t); }
  void send(const QByteArray &f) { if (!f.isEmpty() && m_send) m_send(f); }
  QByteArray frameFor(const Pending &p) const;
  quint8 allocSeq();
  Pending *findPending(quint32 command, quint8 reqSeq);
  void finishSet(int i, qint64 now);
  void startRelist(qint64 nowMs);
  void listTick(qint64 nowMs);
  void checkListComplete();
  bool anyStaged(int group, bool orUnsaved) const;
  bool anyLocked(int group) const;
  QString refusalText(const Pending &p) const;

  const Catalog *m_cat = nullptr;
  Send m_send;
  Log m_log;
  OnWrite m_onWrite;
  OnChanged m_onChanged;
  quint8 m_dev = 42;

  State m_state = State::NoCaps;
  quint32 m_caps = 0;
  bool m_synced = false, m_armed = false, m_secure = false;
  qint64 m_syncEdgeAt = 0;

  QVector<Value> m_vals;
  QMap<int, ParamCodec::ValueMsg> m_raw;
  quint32 m_generation = 0;
  bool m_haveGeneration = false;
  qint64 m_settleUntil = 0;
  qint64 m_lastRelist = -1000000;

  // CRC read.
  qint64 m_crcSentAt = 0;
  int m_crcTries = 0;
  // Listing.
  bool m_listing = false;
  bool m_loggedBackGen = false; // the backwards-generation line is logged once per offer of parameters
  bool m_relistOwed = false; // someone else changed values while a write of ours was pending
  bool m_initialList = false;
  QVector<bool> m_seen;
  QHash<int, int> m_readTries;
  qint64 m_lastRx = 0;

  QVector<Pending> m_pend;
  QVector<quint32> m_editSerial;
  quint32 m_opSerial = 0;
  quint8 m_nextSeq = 0;
  int m_staleAcks = 0;
};

} // namespace params
