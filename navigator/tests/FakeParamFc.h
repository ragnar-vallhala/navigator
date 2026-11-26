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

// A fake flight controller for the parameter service: it implements NavLink
// spec 12.2.1 as written, so the GCS client is tested against the protocol and
// not against its own idea of it. It receives the client's real frames through
// the real parser and answers with a real encoded PARAM_VALUE decoded by the
// real unpack, so the codec is on the path both ways. Fault knobs let a test
// break the vehicle in specific ways.

#include "ParamCatalog.h"
#include "ParamClient.h"
#include "ParamCodec.h"

#include <QByteArray>
#include <QSet>
#include <QStringList>
#include <QVector>
#include <cmath>
#include <cstring>
#include <functional>

extern "C" {
#include "navlink_msgs.h"
}

namespace params {

class FakeParamFc {
public:
  struct Cell {
    double live = 0, staged = 0;
    bool isStaged = false, unsaved = false;
  };

  // --- the vehicle's state ---
  const Catalog *cat = nullptr; // what its table is built from
  QVector<Cell> cells;
  quint32 crc = 0;   // SYS_PTBL_CRC it reports
  int count = 0;     // PARAM_VALUE.count it reports
  quint32 generation = 1;
  bool synced = true; // false: requests dropped, writes TEMPORARILY_REJECTED
  bool armed = false;
  bool strict = false; // keyed vehicle, unsecured link: refuses writes (DENIED)
  bool saveFails = false;
  int tempRejectWrites = 0; // answer the next N writes TEMPORARILY_REJECTED

  // --- fault knobs ---
  QSet<int> dropOnList;  // indexes the first list skips
  QSet<int> dropAlways;  // indexes never sent
  int dropAcks = 0;      // swallow this many COMMAND_ACKs
  bool holdAcks = false; // queue acks until flush()
  bool dropEcho = false; // accept a set but send no PARAM_VALUE
  bool echoBeforeAck = false;
  bool duplicateAck = false;

  // --- what the client sent (decoded) ---
  int listRequests = 0, sets = 0, commits = 0, discards = 0;
  QVector<int> readIndexes;
  QStringList readIds;
  struct SetRx { QString id; double v; quint8 reqSeq; };
  QVector<SetRx> setLog;
  QVector<quint8> commitReqSeqs;

  // --- wiring ---
  std::function<void(const ParamCodec::ValueMsg &)> toClientValue;
  std::function<void(quint32, quint8, quint8)> toClientAck;
  QVector<std::function<void()>> held;

  void load(const Catalog *c) {
    cat = c;
    cells = QVector<Cell>(c->count());
    for (const Entry &e : c->entries())
      cells[e.index].live = e.def;
    crc = c->tableCrc();
    count = c->count();
  }
  int idx(const QString &id) const { return cat->byId(id)->index; }
  double live(const QString &id) const { return cells[idx(id)].live; }
  // A change the client did not make.
  void setLive(const QString &id, double v) {
    cells[idx(id)].live = v;
    ++generation;
  }
  // The storage owner finishing a save: clears UNSAVED and bumps the generation.
  void landSave() {
    ++generation;
    for (int i = 0; i < cells.size(); ++i)
      if (cells[i].unsaved) {
        cells[i].unsaved = false;
        sendValue(i);
      }
  }
  void flush() {
    const auto d = held;
    held.clear();
    for (const auto &f : d)
      f();
  }

  // The client's Send callback.
  void receive(const QByteArray &frame) {
    navlink_parser_t p;
    navlink_parser_init(&p);
    navlink_handlers_t h{};
    h.ctx = this;
    h.on_param_request_list = [](void *c, const navlink_frame_hdr_t *,
                                 const navlink_param_request_list_t *) {
      static_cast<FakeParamFc *>(c)->onList();
    };
    h.on_param_request_read = [](void *c, const navlink_frame_hdr_t *,
                                 const navlink_param_request_read_t *m) {
      static_cast<FakeParamFc *>(c)->onRead(*m);
    };
    h.on_param_set = [](void *c, const navlink_frame_hdr_t *, const navlink_param_set_t *m) {
      return static_cast<FakeParamFc *>(c)->onSet(*m);
    };
    h.on_cmd_param_commit = [](void *c, const navlink_frame_hdr_t *,
                               const navlink_cmd_param_commit_t *m) {
      return static_cast<FakeParamFc *>(c)->onCommit(*m);
    };
    navlink_parser_push(&p, &h, reinterpret_cast<const uint8_t *>(frame.constData()),
                        size_t(frame.size()));
  }

  void sendValue(int i) {
    if (!synced || dropAlways.contains(i) || !toClientValue)
      return;
    navlink_param_value_t m{};
    const QByteArray id = cat->entries()[i].id.toLatin1();
    std::memcpy(m.param_id, id.constData(), size_t(id.size()));
    m.index = quint16(i);
    m.count = quint16(count);
    m.generation = generation;
    m.type = quint8(cat->entries()[i].type);
    const bool isCrc = cat->entries()[i].id == QLatin1String(Catalog::kCrcId);
    // A staged parameter reports its staged value, flagged STAGED. (An
    // assumption about the firmware: the client does not depend on it, it shows
    // what is reported with the flag.)
    ParamCodec::toWire(cat->entries()[i].type,
                       isCrc ? double(crc) : (cells[i].isStaged ? cells[i].staged : cells[i].live),
                       m.value);
    m.flags = flags(i);
    uint8_t buf[NAVLINK_MAX_FRAME];
    const size_t n = navlink_param_value_encode(buf, &m, 1, 1, 1);
    navlink_parser_t p;
    navlink_parser_init(&p);
    navlink_handlers_t h{};
    h.ctx = this;
    h.on_param_value = [](void *c, const navlink_frame_hdr_t *, const navlink_param_value_t *w) {
      auto *fc = static_cast<FakeParamFc *>(c);
      fc->toClientValue(ParamCodec::fromWire(*w));
    };
    navlink_parser_push(&p, &h, buf, n);
  }

private:
  quint8 flags(int i) const {
    const Cell &c = cells[i];
    const Entry &e = cat->entries()[i];
    quint8 f = 0;
    if (c.isStaged) f |= Staged;
    if (c.unsaved) f |= Unsaved;
    if (c.live != e.def || c.isStaged) f |= NonDefault;
    return f;
  }

  void ack(quint32 cmd, quint8 req, quint8 res) {
    if (req == 0 || !toClientAck)
      return; // 0: no acknowledgement wanted
    if (dropAcks > 0) {
      --dropAcks;
      return;
    }
    auto go = [this, cmd, req, res] {
      toClientAck(cmd, req, res);
      if (duplicateAck)
        toClientAck(cmd, req, res);
    };
    if (holdAcks)
      held.push_back(go);
    else
      go();
  }

  void onList() {
    if (!synced)
      return;
    ++listRequests;
    for (int i = 0; i < cells.size(); ++i) {
      if (dropOnList.contains(i)) {
        dropOnList.remove(i);
        continue;
      }
      sendValue(i);
    }
  }

  void onRead(const navlink_param_request_read_t &m) {
    if (!synced)
      return;
    int i = m.index;
    if (i < 0) {
      const QString id = QString::fromLatin1(m.param_id, int(qstrnlen(m.param_id, 16)));
      readIds << id;
      const Entry *e = cat->byId(id);
      i = e ? e->index : -1;
    } else {
      readIndexes << i;
    }
    if (i >= 0 && i < cells.size())
      sendValue(i);
  }

  navlink_ack_t reply(quint32 cmd, quint8 req, quint8 res) {
    ack(cmd, req, res);
    navlink_ack_t a{};
    a.result = res;
    return a;
  }

  navlink_ack_t onSet(const navlink_param_set_t &m) {
    const quint32 cmd = NAVLINK_MSGID_PARAM_SET;
    const QString id = QString::fromLatin1(m.param_id, int(qstrnlen(m.param_id, 16)));
    ParamCodec::ValueMsg vm;
    vm.type = m.type;
    std::memcpy(vm.raw, m.value, 8);
    double v = 0;
    const bool decoded = ParamCodec::valueOf(vm, v);
    setLog.push_back({id, v, m.req_seq});
    ++sets;
    if (!synced || tempRejectWrites > 0) {
      if (tempRejectWrites > 0)
        --tempRejectWrites;
      return reply(cmd, m.req_seq, 1);
    }
    const Entry *e = cat->byId(id);
    if (!e || !decoded || quint8(e->type) != m.type)
      return reply(cmd, m.req_seq, 3);
    if (strict)
      return reply(cmd, m.req_seq, 2);
    if ((e->flags & ReadOnly) || ((e->flags & LockedWhenArmed) && armed) ||
        !Catalog::accepts(*e, v))
      return reply(cmd, m.req_seq, 2);
    Cell &c = cells[e->index];
    if (e->group == 0) {
      c.live = v;
      c.unsaved = (e->flags & Persisted) != 0;
    } else {
      c.staged = v;
      c.isStaged = true;
    }
    ++generation;
    auto echo = [&] {
      if (!dropEcho)
        sendValue(e->index);
    };
    if (echoBeforeAck) {
      echo();
      return reply(cmd, m.req_seq, 0);
    }
    const auto r = reply(cmd, m.req_seq, 0);
    echo();
    return r;
  }

  navlink_ack_t onCommit(const navlink_cmd_param_commit_t &m) {
    const quint32 cmd = NAVLINK_MSGID_CMD_PARAM_COMMIT;
    commitReqSeqs.push_back(m.req_seq);
    if (!synced || tempRejectWrites > 0) {
      if (tempRejectWrites > 0)
        --tempRejectWrites;
      return reply(cmd, m.req_seq, 1);
    }
    if (m.group != 0 && !cat->group(m.group))
      return reply(cmd, m.req_seq, 3);
    if (strict)
      return reply(cmd, m.req_seq, 2);
    QVector<int> members;
    for (const Entry &e : cat->entries())
      if (m.group == 0 ? true : e.group == m.group)
        members.push_back(e.index);
    if (m.op == 1) {
      ++discards;
      for (int i : members)
        if (cells[i].isStaged)
          cells[i].isStaged = false;
    } else {
      ++commits;
      for (int i : members)
        if (cells[i].isStaged && (cat->entries()[i].flags & LockedWhenArmed) && armed)
          return reply(cmd, m.req_seq, 2); // staged values are kept
      if (saveFails)
        return reply(cmd, m.req_seq, 4);
      for (int i : members)
        if (cells[i].isStaged) {
          cells[i].live = cells[i].staged;
          cells[i].isStaged = false;
          cells[i].unsaved = true;
        }
    }
    ++generation;
    const auto r = reply(cmd, m.req_seq, 0);
    for (int i : members)
      sendValue(i);
    return r;
  }
};

} // namespace params
