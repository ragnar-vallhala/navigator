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

#include "ParamService.h"

#include <QMap>
#include <QObject>
#include <QSet>
#include <QVector>

namespace params {

// Applies a set of parameter values to the vehicle as one operation, for the
// tuning screens (autotune gains, notch): every value is checked before
// anything is sent, the values are staged, each group they belong to is
// committed, and ONE result says what the vehicle now has.
//
//  - Nothing is sent unless every value is acceptable (known, writable, in
//    range, not locked by armed, read) and the table is Ready.
//  - While an apply runs the service is locked: an edit in the panel, or a
//    second apply, is refused with the reason.
//  - A group that does not commit is discarded, so the vehicle is left as it
//    was. By default ONE CMD_PARAM_COMMIT group 0 commits every staged group,
//    and the firmware validates all of them before applying any, so a result
//    can never land half applied (confirmed for firmware #147). With
//    setGroupZeroCommit(false) the groups commit one by one and can differ;
//    the result then lists every group and never claims more.
//  - Values staged earlier (by the panel) that this would also commit make it
//    refuse: commit or discard them first.
//  - Values of group 0 apply at once. With the group-0 commit (the default) they
//    are saved with it, even when they are all there is; with per-group commits
//    they are not committed here and the result says "not saved".
class ParamApplier : public QObject {
  Q_OBJECT
public:
  struct Item {
    QString id;
    double value = 0;
  };
  struct GroupResult {
    enum State { Applied, Discarded, NotSent, Unknown } state = NotSent;
    int group = 0;
    QString name, text;
  };
  struct Result {
    bool ok = false;
    bool unknown = false; // the link went away: the vehicle may have applied it
    QString text;
    QVector<GroupResult> groups;
  };
  static constexpr int kWindow = 4; // sets in flight at once

  explicit ParamApplier(ParamService *svc, QObject *parent = nullptr);
  ~ParamApplier() override; // never leaves the service locked

  void setGroupZeroCommit(bool on) { m_groupZero = on; }
  bool groupZeroCommit() const { return m_groupZero; }
  bool busy() const { return m_phase != Phase::Idle; }
  // False, with the reason, when nothing was sent.
  bool apply(const QVector<Item> &items, QString *why = nullptr);

signals:
  void finished(const params::ParamApplier::Result &r);

private slots:
  void onWrite(const params::Client::WriteEvent &e);
  void onChanged();
  void process();

private:
  enum class Phase { Idle, Setting, Committing, Discarding };
  void pump();
  void startCommit();
  void commitNext();
  void startDiscard();
  void discardNext();
  void finish(bool ok, bool unknown, const QString &text);
  void fail(const QString &text);
  QString groupName(int g) const;
  GroupResult &entry(int g);

  ParamService *m_svc;
  bool m_groupZero = true;
  Phase m_phase = Phase::Idle;
  QVector<Item> m_todo;
  QSet<QString> m_inflight;
  QVector<int> m_groups;     // touched groups above 0, ascending
  QVector<int> m_toDiscard;  // groups still to discard
  QMap<int, GroupResult> m_res;
  int m_commitIdx = 0;
  int m_count = 0, m_liveCount = 0;
  QString m_failure;
  QVector<Client::WriteEvent> m_events;
  bool m_processing = false;
};

} // namespace params
