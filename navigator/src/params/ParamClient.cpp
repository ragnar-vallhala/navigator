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
#include "ParamClient.h"

#include <QStringList>

#include <algorithm>
#include <cmath>

namespace params {

namespace {
bool sameValue(Type t, double a, double b) {
  return t == Type::F32 ? float(a) == float(b) : a == b;
}
} // namespace

QString Client::stateText() const {
  switch (m_state) {
  case State::NoCaps: return QStringLiteral("the vehicle does not offer parameters");
  case State::Unsynced: return QStringLiteral("waiting for the link to synchronise");
  case State::ReadingCrc: return QStringLiteral("checking the parameter table");
  case State::NoAnswer:
    return QStringLiteral("the vehicle did not answer the parameter table check; refresh to try again");
  case State::Listing: return QStringLiteral("reading parameters");
  case State::Ready: return QStringLiteral("ready");
  case State::Mismatch:
    return QStringLiteral("the vehicle's parameter table differs from this catalog: read-only");
  case State::Incomplete: return QStringLiteral("some parameters did not arrive");
  }
  return {};
}

void Client::event(Op op, const QString &id, int group, Outcome o, const QString &text) {
  if (!m_onWrite)
    return;
  WriteEvent e;
  e.op = op;
  e.id = id;
  e.group = group;
  e.outcome = o;
  e.text = text;
  m_onWrite(e);
}

void Client::failAllPending(Outcome o, const QString &text) {
  const QVector<Pending> p = m_pend;
  m_pend.clear();
  for (const Pending &x : p)
    event(x.op, x.entry >= 0 && m_cat ? m_cat->entries()[x.entry].id : QString(),
          x.group, o, text);
}

quint8 Client::allocSeq() {
  for (int i = 0; i < 255; ++i) {
    m_nextSeq = quint8(m_nextSeq % 255 + 1); // 1..255, never 0
    bool used = false;
    for (const Pending &p : m_pend)
      used = used || p.reqSeq == m_nextSeq;
    if (!used)
      return m_nextSeq;
  }
  return 0; // 255 writes in flight: refused by the caller
}

Client::Pending *Client::findPending(quint32 command, quint8 reqSeq) {
  for (Pending &p : m_pend) {
    const quint32 cmd = p.op == Op::Set ? kMsgidParamSet : kMsgidParamCommit;
    if (cmd == command && p.reqSeq == reqSeq)
      return &p;
  }
  return nullptr;
}

QByteArray Client::frameFor(const Pending &p) const {
  if (p.op == Op::Set)
    return ParamCodec::encodeSet(m_cat->entries()[p.entry].id,
                                 m_cat->entries()[p.entry].type, p.value, p.reqSeq, m_dev);
  return ParamCodec::encodeCommit(p.reqSeq, p.op == Op::Commit ? 0 : 1, quint8(p.group), m_dev);
}

void Client::onCapabilities(quint32 caps, qint64 nowMs) {
  m_caps = caps;
  if (!(caps & kCapParams)) {
    if (m_state != State::NoCaps) {
      failAllPending(Outcome::Unknown, QStringLiteral("outcome unknown; the vehicle no longer offers parameters"));
      m_state = State::NoCaps;
      changed();
    }
    return;
  }
  if (m_state == State::NoCaps) {
    if (m_synced)
      start(nowMs);
    else {
      m_state = State::Unsynced;
      changed();
    }
  }
}

void Client::onSynced(bool synced, qint64 nowMs) {
  if (synced == m_synced)
    return;
  m_synced = synced;
  if (synced) {
    m_syncEdgeAt = nowMs;
    if (m_caps & kCapParams)
      start(nowMs);
    return;
  }
  if (m_state == State::NoCaps)
    return;
  failAllPending(Outcome::Unknown,
                 QStringLiteral("outcome unknown; the values are read again on reconnect"));
  m_listing = false;
  m_state = State::Unsynced;
  changed();
}

void Client::start(qint64 nowMs) {
  if (!m_cat || m_cat->isEmpty()) {
    log(QStringLiteral("parameters: no catalog loaded, nothing is read"));
    m_state = State::NoCaps;
    changed();
    return;
  }
  m_vals = QVector<Value>(m_cat->count());
  m_editSerial = QVector<quint32>(m_cat->count(), 0);
  m_raw.clear();
  m_seen = QVector<bool>(m_cat->count(), false);
  m_readTries.clear();
  m_listing = false;
  m_relistOwed = false;
  m_haveGeneration = false;
  m_state = State::ReadingCrc;
  m_crcTries = 1;
  m_crcSentAt = nowMs;
  send(ParamCodec::encodeRequestReadById(QString::fromLatin1(Catalog::kCrcId), m_dev));
  changed();
}

void Client::toMismatch(const QString &why) {
  log(QStringLiteral("parameters: %1. Values are shown read-only.").arg(why));
  const bool wasMismatch = m_state == State::Mismatch;
  m_state = State::Mismatch;
  m_listing = false;
  failAllPending(Outcome::Rejected, QStringLiteral("the vehicle's parameter table differs from the catalog"));
  if (!wasMismatch)
    send(ParamCodec::encodeRequestList(m_dev)); // so the raw values can be shown
  changed();
}

void Client::onParamValue(const ParamCodec::ValueMsg &m, qint64 nowMs) {
  if (m_state == State::NoCaps || m_state == State::Unsynced || !m_cat)
    return;
  bool sendListAfter = false;
  m_raw.insert(m.index, m);
  if (m_state == State::Mismatch) {
    changed();
    return;
  }
  if (m.count != m_cat->count()) {
    toMismatch(QStringLiteral("the vehicle has %1 parameters, the catalog %2")
                   .arg(m.count).arg(m_cat->count()));
    return;
  }
  if (m_state == State::ReadingCrc) {
    if (m.id != QLatin1String(Catalog::kCrcId))
      return;
    double v = 0;
    if (Type(m.type) != Type::U32 || !ParamCodec::valueOf(m, v)) {
      toMismatch(QStringLiteral("the table CRC is not a 32-bit value"));
      return;
    }
    if (quint32(v) != m_cat->tableCrc()) {
      toMismatch(QStringLiteral("the vehicle's table CRC %1 is not the catalog's %2")
                     .arg(quint32(v), 8, 16, QLatin1Char('0'))
                     .arg(m_cat->tableCrc(), 8, 16, QLatin1Char('0')));
      return;
    }
    m_state = State::Listing;
    m_listing = true;
    m_initialList = true;
    m_lastRx = nowMs;
    sendListAfter = true; // sent once this message is processed
  }
  const Entry *e = m_cat->byIndex(m.index);
  double v = 0;
  if (!e || e->id != m.id || quint8(e->type) != m.type || !ParamCodec::valueOf(m, v)) {
    toMismatch(QStringLiteral("entry %1 (%2) is not the catalog's").arg(m.index).arg(m.id));
    return;
  }
  // A generation that goes BACKWARDS outside a re-list is a vehicle that
  // restarted (its counter began again): what we hold is from another life.
  // Read everything again from the CRC; what was in flight may or may not have
  // been applied.
  if (m_state == State::Ready && !m_listing && m_haveGeneration &&
      qint32(m.generation - m_generation) < 0) {
    log(QStringLiteral("parameters: the vehicle's generation went back (%1 -> %2): "
                       "it restarted, the table is read again")
            .arg(m_generation)
            .arg(m.generation));
    failAllPending(Outcome::Unknown,
                   QStringLiteral("outcome unknown; the vehicle restarted"));
    start(nowMs);
    return;
  }
  Value &slot = m_vals[m.index];
  slot.v = v;
  slot.flags = m.flags;
  slot.generation = m.generation;
  slot.have = true;
  m_lastRx = nowMs;
  if (m_listing) {
    m_seen[m.index] = true;
    m_generation = m.generation;
    m_haveGeneration = true;
  } else if (!m_haveGeneration) {
    m_generation = m.generation;
    m_haveGeneration = true;
  } else if (m.generation != m_generation) {
    m_generation = m.generation;
    // A jump we caused (a write or commit of ours, or its settling) is expected.
    // Any other means someone else changed values: read the table again, but
    // never more often than kRelistGapMs.
    if (m_pend.isEmpty() && nowMs >= m_settleUntil && nowMs - m_lastRelist >= kRelistGapMs) {
      startRelist(nowMs);
    } else {
      // Not now (a write is pending, we are settling, or one was just made):
      // note it and read again from tick() when it is allowed. The echo of our
      // own pending write carries the new generation and is not "someone else".
      // A pending commit or discard bumps it too, so those are ours as well.
      // Inside our own settle window with nothing pending the jump is the
      // read-back of what we just did.
      bool ours = m_pend.isEmpty() && nowMs < m_settleUntil;
      for (const Pending &p : m_pend)
        ours = ours || p.op != Op::Set || p.entry == m.index;
      if (!ours)
        m_relistOwed = true;
    }
  }
  for (int i = 0; i < m_pend.size(); ++i) {
    Pending &p = m_pend[i];
    if (p.op == Op::Set && p.entry == m.index) {
      p.echoed = true;
      p.echoValue = v;
      p.echoFlags = m.flags;
      if (p.acked) {
        finishSet(i, nowMs);
        break;
      }
    }
  }
  if (m_listing)
    checkListComplete();
  changed();
  if (sendListAfter)
    send(ParamCodec::encodeRequestList(m_dev));
}

void Client::finishSet(int i, qint64 now) {
  const Pending p = m_pend[i];
  m_pend.removeAt(i);
  m_settleUntil = now + kSettleMs;
  const Entry &e = m_cat->entries()[p.entry];
  QString text;
  if (p.echoFlags & Staged)
    text = QStringLiteral("staged; commit the group to apply it");
  else if (!sameValue(e.type, p.echoValue, p.value))
    text = QStringLiteral("applied as %1").arg(p.echoValue);
  else
    text = QStringLiteral("applied");
  if (p.echoFlags & RebootPending)
    text += QStringLiteral(" (takes effect after a reboot)");
  event(Op::Set, e.id, e.group, Outcome::Applied, text);
}

void Client::startRelist(qint64 nowMs) {
  m_relistOwed = false;
  m_lastRelist = nowMs;
  m_listing = true;
  m_initialList = false;
  m_seen = QVector<bool>(m_cat->count(), false);
  m_readTries.clear();
  m_lastRx = nowMs;
  send(ParamCodec::encodeRequestList(m_dev));
}

void Client::refresh(qint64 nowMs) {
  if (m_state == State::NoAnswer) {
    start(nowMs);
  } else if (m_state == State::Ready) {
    startRelist(nowMs);
    changed();
  } else if (m_state == State::Incomplete) {
    m_state = State::Listing; // a fresh initial read: Ready or Incomplete again
    startRelist(nowMs);
    m_initialList = true;
    changed();
  }
}

QVector<int> Client::missing() const {
  QVector<int> r;
  if (m_seen.size() != (m_cat ? m_cat->count() : 0))
    return r;
  if (m_state != State::Listing && m_state != State::Incomplete && !m_listing)
    return r;
  for (int i = 0; i < m_seen.size(); ++i)
    if (!m_seen[i])
      r.push_back(i);
  return r;
}

void Client::checkListComplete() {
  for (bool s : m_seen)
    if (!s)
      return;
  m_listing = false;
  if (m_state == State::Listing || m_state == State::Incomplete)
    m_state = State::Ready;
  changed();
}

void Client::listTick(qint64 nowMs) {
  if (!m_listing || nowMs - m_lastRx < kQuietMs)
    return;
  int sent = 0, open = 0;
  for (int i = 0; i < m_seen.size(); ++i) {
    if (m_seen[i])
      continue;
    ++open;
    int &tries = m_readTries[i];
    if (tries < kReadRetries && sent < kReadBatch) {
      send(ParamCodec::encodeRequestReadByIndex(quint16(i), m_dev));
      ++tries;
      ++sent;
    }
  }
  if (open == 0) {
    checkListComplete();
    return;
  }
  if (sent > 0) {
    m_lastRx = nowMs;
    return;
  }
  // Every missing index was asked for kReadRetries times already.
  bool anyLeft = false;
  for (int i = 0; i < m_seen.size(); ++i)
    anyLeft = anyLeft || (!m_seen[i] && m_readTries.value(i) < kReadRetries);
  if (anyLeft)
    return;
  QStringList names;
  for (int i = 0; i < m_seen.size() && names.size() < 6; ++i)
    if (!m_seen[i])
      names << m_cat->entries()[i].id;
  log(QStringLiteral("parameters: %1 value(s) never arrived (%2%3)")
          .arg(open).arg(names.join(QLatin1String(", ")))
          .arg(open > names.size() ? QStringLiteral(", ...") : QString()));
  m_listing = false;
  if (m_initialList)
    m_state = State::Incomplete;
  changed();
}

void Client::tick(qint64 nowMs) {
  if (!m_cat)
    return;
  if (m_state == State::ReadingCrc && nowMs - m_crcSentAt >= kCrcRetryMs) {
    if (m_crcTries < kCrcTries) {
      ++m_crcTries;
      m_crcSentAt = nowMs;
      send(ParamCodec::encodeRequestReadById(QString::fromLatin1(Catalog::kCrcId), m_dev));
    } else {
      m_state = State::NoAnswer; // terminal: refresh() or the next sync edge retries
      log(QStringLiteral("parameters: no answer to the table CRC read"));
      changed();
    }
  }
  if (m_relistOwed && m_state == State::Ready && !m_listing && m_pend.isEmpty() &&
      nowMs >= m_settleUntil && nowMs - m_lastRelist >= kRelistGapMs) {
    startRelist(nowMs);
    changed();
  }
  listTick(nowMs);
  for (int i = 0; i < m_pend.size();) {
    Pending &p = m_pend[i];
    if (!p.acked) {
      if (nowMs - p.sentAt >= kAckMs) {
        if (!p.resent) {
          p.resent = true;
          p.sentAt = nowMs;
          send(frameFor(p));
        } else {
          const Pending d = p;
          m_pend.removeAt(i);
          if (d.op == Op::Set)
            send(ParamCodec::encodeRequestReadByIndex(quint16(d.entry), m_dev));
          event(d.op, d.entry >= 0 ? m_cat->entries()[d.entry].id : QString(), d.group,
                Outcome::TimedOut, QStringLiteral("no acknowledgement from the vehicle"));
          continue;
        }
      }
    } else if (p.op == Op::Set && !p.echoed && nowMs - p.ackedAt >= kEchoMs) {
      if (!p.readSent) {
        p.readSent = true;
        p.ackedAt = nowMs;
        send(ParamCodec::encodeRequestReadByIndex(quint16(p.entry), m_dev));
      } else {
        const Pending d = p;
        m_pend.removeAt(i);
        event(Op::Set, m_cat->entries()[d.entry].id, d.group, Outcome::Failed,
              QStringLiteral("accepted, but the vehicle did not confirm the value"));
        continue;
      }
    }
    ++i;
  }
}

QString Client::refusalText(const Pending &p) const {
  Q_UNUSED(p);
  if (hasSecureLinkCap() && !m_secure)
    return QStringLiteral("secure link required: this vehicle only accepts parameter "
                          "changes on a secured link, and this link is not secured");
  return QStringLiteral("refused by the vehicle (outside its range, locked while armed, "
                        "or not allowed now)");
}

void Client::onCommandAck(quint32 command, quint8 reqSeq, quint8 result, qint64 nowMs) {
  Pending *pp = findPending(command, reqSeq);
  if (!pp) {
    ++m_staleAcks; // not ours, or already settled
    return;
  }
  if (pp->acked) {
    ++m_staleAcks; // already acknowledged: a repeat settles nothing
    return;
  }
  Pending &p = *pp;
  const QString id = p.entry >= 0 ? m_cat->entries()[p.entry].id : QString();
  const Op op = p.op;
  const int group = p.group;
  const QString refusal = refusalText(p); // before p can be erased
  auto drop = [&]() {
    for (int i = 0; i < m_pend.size(); ++i)
      if (m_pend[i].reqSeq == reqSeq && &m_pend[i] == pp) {
        m_pend.removeAt(i);
        return;
      }
  };
  switch (result) {
  case kInProgress:
    p.sentAt = nowMs; // alive: keep waiting
    return;
  case kAccepted:
    if (op == Op::Set) {
      p.acked = true;
      p.ackedAt = nowMs;
      if (p.echoed) {
        for (int i = 0; i < m_pend.size(); ++i)
          if (&m_pend[i] == pp) {
            finishSet(i, nowMs);
            break;
          }
      }
      return;
    }
    drop();
    event(op, id, group, Outcome::Applied,
          op == Op::Commit ? QStringLiteral("committed") : QStringLiteral("discarded"));
    m_settleUntil = nowMs + kSettleMs;
    if (group == 0)
      startRelist(nowMs);
    else
      for (int idx : m_cat->membersOf(group))
        send(ParamCodec::encodeRequestReadByIndex(quint16(idx), m_dev));
    return;
  case kTempRejected: {
    const bool serialSame = op == Op::Set ? m_editSerial.value(p.entry) == p.serial
                                          : m_opSerial == p.serial;
    if (!p.retried && serialSame && nowMs - m_syncEdgeAt <= kRetryWindowMs) {
      const quint8 ns = allocSeq();
      if (ns) {
        p.reqSeq = ns;
        p.retried = true;
        p.resent = false;
        p.sentAt = nowMs;
        send(frameFor(p));
        return;
      }
    }
    drop();
    event(op, id, group, Outcome::Rejected,
          QStringLiteral("not applied: the vehicle was not synchronised yet"));
    return;
  }
  case kDenied:
    drop();
    event(op, id, group, Outcome::Refused,
          op == Op::Set ? refusal : refusal + QStringLiteral("; the staged values are kept"));
    return;
  case kUnsupported:
    drop();
    event(op, id, group, Outcome::Refused,
          QStringLiteral("the vehicle does not know this %1 (catalog mismatch)")
              .arg(op == Op::Set ? QStringLiteral("parameter") : QStringLiteral("group")));
    return;
  case kFailed:
    drop();
    event(op, id, group, Outcome::Failed,
          QStringLiteral("the vehicle could not store it"));
    return;
  default:
    drop();
    event(op, id, group, Outcome::Failed, QStringLiteral("unknown result %1").arg(result));
    return;
  }
}

const Client::Value *Client::value(const QString &id) const {
  if (!m_cat)
    return nullptr;
  const Entry *e = m_cat->byId(id);
  if (!e || e->index >= m_vals.size() || !m_vals[e->index].have)
    return nullptr;
  return &m_vals[e->index];
}

bool Client::set(const QString &id, double v, qint64 nowMs, QString *why) {
  auto no = [&](const QString &m) {
    if (why)
      *why = m;
    return false;
  };
  if (m_state != State::Ready)
    return no(stateText());
  const Entry *e = m_cat->byId(id);
  if (!e)
    return no(QStringLiteral("unknown parameter %1").arg(id));
  if (e->flags & ReadOnly)
    return no(QStringLiteral("%1 is read-only").arg(id));
  QString r;
  if (!Catalog::accepts(*e, v, &r))
    return no(QStringLiteral("%1: %2").arg(id, r));
  if ((e->flags & LockedWhenArmed) && m_armed)
    return no(QStringLiteral("%1 cannot be changed while armed").arg(id));
  if (!m_vals[e->index].have)
    return no(QStringLiteral("%1 has not been read yet").arg(id));
  for (const Pending &p : m_pend)
    if (p.op == Op::Set && p.entry == e->index)
      return no(QStringLiteral("a write to %1 is still in progress").arg(id));
  Pending p;
  p.op = Op::Set;
  p.entry = e->index;
  p.group = e->group;
  p.value = v;
  p.reqSeq = allocSeq();
  if (!p.reqSeq)
    return no(QStringLiteral("too many writes in flight"));
  p.serial = ++m_editSerial[e->index];
  ++m_opSerial;
  p.sentAt = nowMs;
  const QByteArray f = frameFor(p);
  if (f.isEmpty())
    return no(QStringLiteral("%1 cannot be encoded as %2").arg(id, typeName(e->type)));
  m_pend.push_back(p);
  send(f);
  return true;
}

bool Client::anyStaged(int group, bool orUnsaved) const {
  for (const Entry &e : m_cat->entries()) {
    const bool inScope = group == 0 ? (orUnsaved || e.group != 0) : e.group == group;
    if (!inScope || !m_vals[e.index].have)
      continue;
    const quint8 f = m_vals[e.index].flags;
    if ((f & Staged) || (orUnsaved && (f & Unsaved)))
      return true;
  }
  return false;
}

bool Client::anyLocked(int group) const {
  for (const Entry &e : m_cat->entries()) {
    const bool inScope = group == 0 ? e.group != 0 : e.group == group;
    if (inScope && (e.flags & LockedWhenArmed) && (m_vals[e.index].flags & Staged))
      return true;
  }
  return false;
}

QString Client::whyNotCommit(int group) const {
  if (m_state != State::Ready)
    return stateText();
  if (group < 0 || group > 255 || (group != 0 && !m_cat->group(group)))
    return QStringLiteral("unknown group %1").arg(group);
  if (!anyStaged(group, group == 0))
    return QStringLiteral("nothing is staged to commit");
  if (m_armed && anyLocked(group))
    return QStringLiteral("these parameters cannot be committed while armed");
  for (const Pending &p : m_pend)
    if (p.op != Op::Set)
      return QStringLiteral("a commit is still in progress");
  return {};
}

QString Client::whyNotDiscard(int group) const {
  if (m_state != State::Ready)
    return stateText();
  if (group < 0 || group > 255 || (group != 0 && !m_cat->group(group)))
    return QStringLiteral("unknown group %1").arg(group);
  if (!anyStaged(group, false))
    return QStringLiteral("nothing is staged to discard");
  for (const Pending &p : m_pend)
    if (p.op != Op::Set)
      return QStringLiteral("a commit is still in progress");
  return {};
}

bool Client::commit(int group, qint64 nowMs, QString *why) {
  const QString block = whyNotCommit(group);
  if (!block.isEmpty()) {
    if (why)
      *why = block;
    return false;
  }
  Pending p;
  p.op = Op::Commit;
  p.group = group;
  p.reqSeq = allocSeq();
  if (!p.reqSeq) {
    if (why)
      *why = QStringLiteral("too many writes in flight");
    return false;
  }
  p.serial = ++m_opSerial;
  p.sentAt = nowMs;
  m_pend.push_back(p);
  send(frameFor(p));
  return true;
}

bool Client::discard(int group, qint64 nowMs, QString *why) {
  const QString block = whyNotDiscard(group);
  if (!block.isEmpty()) {
    if (why)
      *why = block;
    return false;
  }
  Pending p;
  p.op = Op::Discard;
  p.group = group;
  p.reqSeq = allocSeq();
  if (!p.reqSeq) {
    if (why)
      *why = QStringLiteral("too many writes in flight");
    return false;
  }
  p.serial = ++m_opSerial;
  p.sentAt = nowMs;
  m_pend.push_back(p);
  send(frameFor(p));
  return true;
}

} // namespace params
