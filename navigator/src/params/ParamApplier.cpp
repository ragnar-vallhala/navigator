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
#include "ParamApplier.h"

#include <QMetaObject>

namespace params {

ParamApplier::ParamApplier(ParamService *svc, QObject *parent) : QObject(parent), m_svc(svc) {
  connect(m_svc, &ParamService::writeFinished, this, &ParamApplier::onWrite);
  connect(m_svc, &ParamService::changed, this, &ParamApplier::onChanged);
}

QString ParamApplier::groupName(int g) const {
  const Group *grp = m_svc->catalog().group(g);
  return grp ? grp->name : QString::number(g);
}

ParamApplier::GroupResult &ParamApplier::entry(int g) {
  GroupResult &r = m_res[g];
  r.group = g;
  r.name = groupName(g);
  return r;
}

bool ParamApplier::apply(const QVector<Item> &items, QString *why) {
  auto no = [&](const QString &m) {
    if (why)
      *why = m;
    return false;
  };
  if (busy())
    return no(QStringLiteral("an apply is already in progress"));
  if (items.isEmpty())
    return no(QStringLiteral("nothing to apply"));
  const Client &c = m_svc->client();
  if (c.state() != Client::State::Ready)
    return no(c.stateText());
  if (m_svc->locked())
    return no(m_svc->whyNotCommit(0));
  const Catalog &cat = m_svc->catalog();
  QSet<QString> seen;
  for (const Item &it : items) {
    const Entry *e = cat.byId(it.id);
    if (!e)
      return no(QStringLiteral("unknown parameter %1").arg(it.id));
    if (seen.contains(it.id))
      return no(QStringLiteral("%1 is given twice").arg(it.id));
    seen.insert(it.id);
    if (e->flags & ReadOnly)
      return no(QStringLiteral("%1 is read-only").arg(it.id));
    QString r;
    if (!Catalog::accepts(*e, it.value, &r))
      return no(QStringLiteral("%1: %2").arg(it.id, r));
    if ((e->flags & LockedWhenArmed) && c.armed())
      return no(QStringLiteral("%1 cannot be changed while armed").arg(it.id));
    const Client::Value *v = c.value(it.id);
    if (!v || !v->have)
      return no(QStringLiteral("%1 has not been read yet").arg(it.id));
  }
  // A commit sweeps up everything staged in its groups (group 0: in all of
  // them): refuse rather than commit a change nobody asked for here.
  {
    QSet<int> touched;
    QSet<QString> ids;
    for (const Item &it : items) {
      ids.insert(it.id);
      touched.insert(cat.byId(it.id)->group);
    }
    for (const Entry &e : cat.entries()) {
      const Client::Value *v = c.value(e.id);
      if (e.group == 0 || ids.contains(e.id) || !v || !v->have || !(v->flags & Staged))
        continue;
      if (m_groupZero || touched.contains(e.group))
        return no(QStringLiteral("%1 has a staged change that this would also commit; "
                                 "commit or discard it first")
                      .arg(e.id));
    }
  }
  m_todo = items;
  m_inflight.clear();
  m_groups.clear();
  m_toDiscard.clear();
  m_res.clear();
  m_events.clear();
  m_failure.clear();
  m_commitIdx = 0;
  m_count = items.size();
  m_liveCount = 0;
  for (const Item &it : items) {
    const int g = cat.byId(it.id)->group;
    if (g == 0)
      ++m_liveCount;
    else if (!m_groups.contains(g))
      m_groups.push_back(g);
  }
  std::sort(m_groups.begin(), m_groups.end());
  m_phase = Phase::Setting;
  m_svc->lockFor(QStringLiteral("a tuning apply is in progress"));
  pump();
  return true;
}

void ParamApplier::pump() {
  while (m_phase == Phase::Setting && !m_todo.isEmpty() && m_inflight.size() < kWindow) {
    const Item it = m_todo.takeFirst();
    QString why;
    if (!m_svc->set(it.id, it.value, &why, true)) {
      fail(QStringLiteral("%1 was not sent (%2)").arg(it.id, why));
      return;
    }
    m_inflight.insert(it.id);
  }
}

void ParamApplier::fail(const QString &text) {
  if (m_failure.isEmpty())
    m_failure = text;
  m_todo.clear(); // nothing further is sent; what is staged is discarded below
  if (m_inflight.isEmpty() && m_phase != Phase::Discarding)
    startDiscard();
}

void ParamApplier::onWrite(const Client::WriteEvent &e) {
  if (m_phase == Phase::Idle)
    return;
  // The client is in the middle of handling a frame when it reports: react
  // from the event loop, never from inside it.
  m_events << e;
  if (!m_processing) {
    m_processing = true;
    QMetaObject::invokeMethod(this, "process", Qt::QueuedConnection);
  }
}

void ParamApplier::onChanged() {
  if (m_phase == Phase::Idle)
    return;
  const Client::State s = m_svc->client().state();
  if (s == Client::State::NoCaps || s == Client::State::Unsynced) {
    // The link or the capability went away without a request in flight to
    // say so: the vehicle may have applied part of this.
    finish(false, true, QStringLiteral("outcome unknown; the vehicle stopped offering parameters"));
  }
}

void ParamApplier::process() {
  m_processing = false;
  while (!m_events.isEmpty() && m_phase != Phase::Idle) {
    const Client::WriteEvent e = m_events.takeFirst();
    if (e.outcome == Client::Outcome::Unknown) {
      finish(false, true, e.text);
      return;
    }
    const bool ok = e.outcome == Client::Outcome::Applied;
    switch (m_phase) {
    case Phase::Setting:
      if (e.op != Client::Op::Set || !m_inflight.contains(e.id))
        break;
      m_inflight.remove(e.id);
      if (!ok)
        fail(QStringLiteral("%1: %2").arg(e.id, e.text));
      else if (m_failure.isEmpty())
        pump();
      if (m_phase == Phase::Setting && m_failure.isEmpty() && m_todo.isEmpty() && m_inflight.isEmpty())
        startCommit();
      else if (!m_failure.isEmpty() && m_inflight.isEmpty() && m_phase == Phase::Setting)
        startDiscard();
      break;
    case Phase::Committing:
      if (e.op != Client::Op::Commit)
        break;
      if (m_groupZero) {
        for (int g : std::as_const(m_groups)) {
          GroupResult &r = entry(g);
          if (ok) {
            r.state = GroupResult::Applied;
            r.text = QStringLiteral("applied");
          }
        }
        if (ok) {
          finish(true, false, QString());
          return;
        }
        m_toDiscard = m_groups;
        fail(QStringLiteral("the commit was not accepted: %1").arg(e.text));
      } else {
        const int g = m_groups.value(m_commitIdx);
        GroupResult &r = entry(g);
        if (ok) {
          r.state = GroupResult::Applied;
          r.text = QStringLiteral("applied");
          ++m_commitIdx;
          commitNext();
        } else {
          m_toDiscard = m_groups.mid(m_commitIdx); // this one and the rest
          fail(QStringLiteral("%1: the commit was not accepted: %2").arg(r.name, e.text));
        }
      }
      break;
    case Phase::Discarding:
      if (e.op != Client::Op::Discard)
        break;
      if (!m_toDiscard.isEmpty()) {
        GroupResult &r = entry(m_toDiscard.takeFirst());
        r.state = ok ? GroupResult::Discarded : GroupResult::NotSent;
        r.text = ok ? QStringLiteral("not applied, discarded; the vehicle keeps its values")
                    : QStringLiteral("not applied, and the discard failed: %1").arg(e.text);
      }
      discardNext();
      break;
    case Phase::Idle:
      break;
    }
  }
}

void ParamApplier::startCommit() {
  m_phase = Phase::Committing;
  if (m_groups.isEmpty()) { // only live values: applied at once, nothing to commit
    finish(true, false, QString());
    return;
  }
  if (m_groupZero) {
    QString why;
    if (!m_svc->commit(0, &why, true)) {
      m_toDiscard = m_groups;
      fail(QStringLiteral("the commit was not sent (%1)").arg(why));
    }
    return;
  }
  commitNext();
}

void ParamApplier::commitNext() {
  if (m_commitIdx >= m_groups.size()) {
    finish(true, false, QString());
    return;
  }
  QString why;
  if (!m_svc->commit(m_groups.at(m_commitIdx), &why, true)) {
    const int g = m_groups.at(m_commitIdx);
    m_toDiscard = m_groups.mid(m_commitIdx);
    fail(QStringLiteral("%1: the commit was not sent (%2)").arg(groupName(g), why));
  }
}

void ParamApplier::startDiscard() {
  if (m_phase != Phase::Discarding) {
    m_phase = Phase::Discarding;
    if (m_toDiscard.isEmpty()) // a failure while setting: everything touched is staged
      m_toDiscard = m_groups;
  }
  discardNext();
}

void ParamApplier::discardNext() {
  while (!m_toDiscard.isEmpty()) {
    QString why;
    if (m_svc->discard(m_toDiscard.first(), &why, true))
      return; // the answer arrives as an event
    GroupResult &r = entry(m_toDiscard.takeFirst()); // nothing staged to discard
    r.state = GroupResult::NotSent;
    r.text = QStringLiteral("not applied (%1)").arg(why);
  }
  finish(false, false, m_failure);
}

void ParamApplier::finish(bool ok, bool unknown, const QString &text) {
  Result r;
  r.ok = ok;
  r.unknown = unknown;
  for (int g : std::as_const(m_groups)) {
    GroupResult gr = entry(g);
    if (unknown && gr.state == GroupResult::NotSent && gr.text.isEmpty())
    {
      gr.state = GroupResult::Unknown;
      gr.text = QStringLiteral("outcome unknown");
    }
    r.groups.push_back(gr);
  }
  if (ok) {
    r.text = QStringLiteral("applied %1 value(s)").arg(m_count);
    if (m_liveCount)
      r.text += QStringLiteral(", %1 of them live and not yet saved").arg(m_liveCount);
  } else {
    r.text = text.isEmpty() ? m_failure : text;
    QStringList lines;
    for (const GroupResult &g : std::as_const(r.groups))
      lines << QStringLiteral("%1: %2").arg(g.name, g.text);
    if (!lines.isEmpty())
      r.text += QStringLiteral(" [%1]").arg(lines.join(QStringLiteral("; ")));
  }
  m_phase = Phase::Idle;
  m_todo.clear();
  m_inflight.clear();
  m_toDiscard.clear();
  m_events.clear();
  m_svc->lockFor(QString());
  emit finished(r);
}

} // namespace params
