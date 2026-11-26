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
#include <QtTest>

#include "FakeParamFc.h"
#include "ParamCatalog.h"
#include "ParamClient.h"
#include "ParamCodec.h"

#include <cmath>
#include <limits>

using namespace params;

namespace {

// The generated catalog the build made; PARAMS_CATALOG_JSON overrides it so the
// test can run away from the build tree (a copy of the file is enough).
QString generatedCatalogPath() {
  return qEnvironmentVariable("PARAMS_CATALOG_JSON",
                              QStringLiteral(NAVLINK_GEN_DIR "/params_catalog.json"));
}

// A small catalog that has what the generated one does not: writable parameters
// with no group (applied at once). Same shape as params_catalog.json.
QByteArray synthJson(const QString &extra = {}) {
  QString j = QStringLiteral(R"({
 "catalog": 1, "table_crc": 3735928559,
 "groups": {"1": {"name":"G1","doc":"d"}, "2": {"name":"LOCKED","doc":"d"}},
 "params": [
  {"id":"SYS_PTBL_CRC","type":"U32","min":0,"max":4294967295,"default":0,"flags_byte":1,"group":0,"index":0},
  {"id":"A_LIVE","type":"F32","min":0,"max":10,"default":1,"flags_byte":8,"group":0,"index":1},
  {"id":"B_U8","type":"U8","min":0,"max":200,"default":5,"flags_byte":0,"group":0,"index":2},
  {"id":"G1_X","type":"F32","min":0,"max":10,"default":1,"flags_byte":8,"group":1,"index":3},
  {"id":"G1_Y","type":"F32","min":0,"max":10,"default":2,"flags_byte":8,"group":1,"index":4},
  {"id":"L2_Z","type":"F32","min":0,"max":10,"default":3,"flags_byte":10,"group":2,"index":5}
 ]})");
  if (!extra.isEmpty())
    j = extra;
  return j.toUtf8();
}

// A catalog + fake FC + client wired together, with a clock.
struct Rig {
  Catalog cat;
  FakeParamFc fc;
  Client cl;
  qint64 now = 1000;
  QVector<Client::WriteEvent> ev;
  QStringList logs;

  explicit Rig(const QByteArray &json = synthJson()) {
    QString err;
    const bool ok = Catalog::load(json, cat, &err);
    if (!ok)
      qFatal("catalog: %s", qPrintable(err));
    fc.load(&cat);
    cl.setCatalog(&cat);
    cl.setSend([this](const QByteArray &f) { fc.receive(f); });
    cl.setLog([this](const QString &s) { logs << s; });
    cl.setOnWrite([this](const Client::WriteEvent &e) { ev << e; });
    fc.toClientValue = [this](const ParamCodec::ValueMsg &m) { cl.onParamValue(m, now); };
    fc.toClientAck = [this](quint32 c, quint8 r, quint8 res) { cl.onCommandAck(c, r, res, now); };
  }
  void step(qint64 ms) {
    const qint64 end = now + ms;
    while (now < end) {
      now += 50;
      cl.tick(now);
    }
  }
  void up(quint32 caps = 8) {
    cl.onCapabilities(caps, now);
    cl.onSynced(true, now);
    step(100);
  }
  const Client::WriteEvent &last() const { return ev.last(); }
};

} // namespace

class TstParamClient : public QObject {
  Q_OBJECT
private slots:
  // ---- catalog
  void generatedCatalogLoads();
  void catalogRefusesWhatTheGeneratorWouldRefuse();
  void acceptsChecksRangeAndType();
  // ---- codec
  void wireValuesRoundTripEveryType();
  void setAndCommitFramesDecode();
  // ---- client: reading
  void nothingIsSentWithoutTheCapability();
  void waitsForSyncThenChecksCrcThenLists();
  void aDifferentCrcIsReadOnly();
  void aDifferentCountIsReadOnly();
  void droppedValuesAreAskedForAgain();
  void aValueThatNeverArrivesIsReportedNotHidden();
  void losingTheCapabilityOrSyncStopsEverything();
  // ---- client: writes
  void aLiveWriteIsConfirmedByTheEchoNotTheAck();
  void anAckWithoutAnEchoIsNotApplied();
  void anEchoBeforeTheAckIsFine();
  void reqSeqIsNeverZeroAndWraps();
  void aStaleOrDuplicateAckIsIgnored();
  void localChecksRefuseWithoutSending();
  void aVehicleRefusalIsSurfacedAndNothingChanges();
  void aWriteWithoutAnAckIsSentAgainOnceThenReported();
  void notSynchronisedIsRetriedOnceInTheWindow();
  void aRetryIsDroppedIfTheUserActedSince();
  void strictModeRefusalSaysSecureLinkRequired();
  // ---- client: groups
  void aGroupIsStagedThenCommitted();
  void discardDropsTheStagedValues();
  void aDeniedCommitKeepsTheStagedValues();
  void anArmedLockIsCheckedBeforeCommit();
  void someoneElsesChangeTriggersOneRelist();
  void theRealCatalogEndToEnd();
};

static quint32 caps(bool secure = false) { return 8u | (secure ? 4u : 0u); }

// ---------------------------------------------------------------- catalog
void TstParamClient::generatedCatalogLoads() {
  Catalog c;
  QString err;
  QVERIFY2(Catalog::loadFile(generatedCatalogPath(), c, &err),
           qPrintable(err));
  QVERIFY(c.count() >= 39);
  QVERIFY(c.tableCrc() != 0);
  const Entry *crc = c.byId(QStringLiteral("SYS_PTBL_CRC"));
  QVERIFY(crc);
  QVERIFY(crc->flags & ReadOnly);
  QCOMPARE(crc->type, Type::U32);
  for (int i = 0; i < c.count(); ++i)
    QCOMPARE(c.entries()[i].index, i);
  // Every group is non-empty and its members agree on the armed lock.
  QVERIFY(c.groups().size() >= 6);
  for (const Group &g : c.groups()) {
    const auto m = c.membersOf(g.id);
    QVERIFY2(!m.isEmpty(), qPrintable(g.name));
    for (int i : m)
      QCOMPARE(bool(c.entries()[i].flags & LockedWhenArmed),
               bool(c.entries()[m[0]].flags & LockedWhenArmed));
  }
  const Entry *kp = c.byId(QStringLiteral("RATE_ROLL_KP"));
  QVERIFY(kp);
  QVERIFY(kp->min > 0 && kp->min < kp->def && kp->def < kp->max); // never loop-off
}

void TstParamClient::catalogRefusesWhatTheGeneratorWouldRefuse() {
  auto refused = [](const QString &from, const QString &to, const char *why) {
    QString j = QString::fromUtf8(synthJson());
    QVERIFY2(j.contains(from), why);
    j.replace(from, to);
    Catalog c;
    QString err;
    QVERIFY2(!Catalog::load(j.toUtf8(), c, &err), why);
    QVERIFY2(!err.isEmpty(), why);
    QVERIFY(c.isEmpty()); // nothing half-loaded
  };
  refused("\"id\":\"B_U8\"", "\"id\":\"A_LIVE\"", "duplicate id");
  refused("\"id\":\"B_U8\"", "\"id\":\"B_VERY_LONG_NAME_XYZ\"", "id over 16 chars");
  refused("\"id\":\"B_U8\"", "\"id\":\"b_u8\"", "lower-case id");
  refused("\"min\":0,\"max\":10,\"default\":1,\"flags_byte\":8,\"group\":0",
          "\"min\":5,\"max\":1,\"default\":1,\"flags_byte\":8,\"group\":0", "min above max");
  refused("\"max\":10,\"default\":1,\"flags_byte\":8,\"group\":0",
          "\"max\":10,\"default\":50,\"flags_byte\":8,\"group\":0", "default out of range");
  refused("\"max\":200,\"default\":5", "\"max\":300,\"default\":5", "range wider than U8");
  refused("\"max\":200,\"default\":5", "\"max\":200,\"default\":5.5", "fractional integer default");
  refused("\"group\":1,\"index\":3", "\"group\":7,\"index\":3", "undefined group");
  refused("\"flags_byte\":10,\"group\":2", "\"flags_byte\":10,\"group\":1", "locked and unlocked in a group");
  refused("\"flags_byte\":8,\"group\":1,\"index\":4", "\"flags_byte\":9,\"group\":1,\"index\":4", "read-only in a group");
  refused("\"index\":5", "\"index\":9", "index is not the position");
  refused("\"table_crc\": 3735928559", "\"table_crc\": -1", "crc out of range");
  refused("\"catalog\": 1", "\"catalog\": 2", "unknown version");
  refused("\"flags_byte\":1,\"group\":0,\"index\":0", "\"flags_byte\":0,\"group\":0,\"index\":0", "CRC entry writable");
  Catalog c;
  QString err;
  QVERIFY(!Catalog::load("not json", c, &err));
  QVERIFY(!Catalog::loadFile(QStringLiteral("/nonexistent/params_catalog.json"), c, &err));
}

void TstParamClient::acceptsChecksRangeAndType() {
  Rig r;
  const Entry &f = *r.cat.byId(QStringLiteral("A_LIVE"));
  const Entry &u = *r.cat.byId(QStringLiteral("B_U8"));
  QString why;
  QVERIFY(Catalog::accepts(f, 0.0));
  QVERIFY(Catalog::accepts(f, 10.0));
  QVERIFY(!Catalog::accepts(f, -0.001, &why));
  QVERIFY(why.contains(QLatin1String("outside")));
  QVERIFY(!Catalog::accepts(f, 10.001));
  QVERIFY(!Catalog::accepts(f, std::numeric_limits<double>::quiet_NaN(), &why));
  QVERIFY(why.contains(QLatin1String("finite")));
  QVERIFY(!Catalog::accepts(f, std::numeric_limits<double>::infinity()));
  QVERIFY(Catalog::accepts(u, 200));
  QVERIFY(!Catalog::accepts(u, 201));
  QVERIFY(!Catalog::accepts(u, 3.5, &why));
  QVERIFY(why.contains(QLatin1String("whole")));
}

// ------------------------------------------------------------------ codec
void TstParamClient::wireValuesRoundTripEveryType() {
  struct C { Type t; double v; bool ok; };
  const C cs[] = {
      {Type::U8, 0, true},       {Type::U8, 255, true},        {Type::U8, 256, false},
      {Type::U8, -1, false},     {Type::I8, -128, true},       {Type::I8, 127, true},
      {Type::I8, 128, false},    {Type::U16, 65535, true},     {Type::U16, 65536, false},
      {Type::I16, -32768, true}, {Type::I16, 32768, false},    {Type::U32, 4294967295.0, true},
      {Type::U32, 4294967296.0, false}, {Type::I32, -2147483648.0, true},
      {Type::I32, 2147483647.0, true},  {Type::I32, 2147483648.0, false},
      {Type::F32, 0.003, true},  {Type::F32, -1e30, true},     {Type::F32, 1e39, false},
      {Type::U8, 1.5, false},    {Type::F32, std::nan(""), false},
  };
  for (const C &c : cs) {
    quint8 w[8];
    const bool ok = ParamCodec::toWire(c.t, c.v, w);
    QCOMPARE(ok, c.ok);
    if (!ok)
      continue;
    ParamCodec::ValueMsg m;
    m.type = quint8(c.t);
    std::memcpy(m.raw, w, 8);
    double back = 0;
    QVERIFY(ParamCodec::valueOf(m, back));
    if (c.t == Type::F32)
      QCOMPARE(float(back), float(c.v));
    else
      QCOMPARE(back, c.v);
    for (int i = typeSize(c.t); i < 8; ++i)
      QCOMPARE(int(w[i]), 0); // zero padded
  }
  ParamCodec::ValueMsg bad;
  bad.type = 99;
  double v;
  QVERIFY(!ParamCodec::valueOf(bad, v));
}

void TstParamClient::setAndCommitFramesDecode() {
  // Through the real parser: the bytes on the wire say what the client meant.
  struct Got { QString id; quint8 type = 0, req = 0; double v = 0; quint8 op = 9, group = 9; int lists = 0, reads = 0; short ri = 0; QString rid; } g;
  auto decode = [&g](const QByteArray &f) {
    navlink_parser_t p;
    navlink_parser_init(&p);
    navlink_handlers_t h{};
    h.ctx = &g;
    h.on_param_set = [](void *c, const navlink_frame_hdr_t *, const navlink_param_set_t *m) {
      auto *x = static_cast<Got *>(c);
      x->id = QString::fromLatin1(m->param_id, int(qstrnlen(m->param_id, 16)));
      x->type = m->type;
      x->req = m->req_seq;
      ParamCodec::ValueMsg vm;
      vm.type = m->type;
      std::memcpy(vm.raw, m->value, 8);
      ParamCodec::valueOf(vm, x->v);
      return navlink_ack_t{};
    };
    h.on_cmd_param_commit = [](void *c, const navlink_frame_hdr_t *, const navlink_cmd_param_commit_t *m) {
      auto *x = static_cast<Got *>(c);
      x->req = m->req_seq;
      x->op = m->op;
      x->group = m->group;
      return navlink_ack_t{};
    };
    h.on_param_request_list = [](void *c, const navlink_frame_hdr_t *, const navlink_param_request_list_t *) {
      static_cast<Got *>(c)->lists++;
    };
    h.on_param_request_read = [](void *c, const navlink_frame_hdr_t *, const navlink_param_request_read_t *m) {
      auto *x = static_cast<Got *>(c);
      x->reads++;
      x->ri = m->index;
      x->rid = QString::fromLatin1(m->param_id, int(qstrnlen(m->param_id, 16)));
    };
    navlink_parser_push(&p, &h, reinterpret_cast<const uint8_t *>(f.constData()), size_t(f.size()));
  };
  decode(ParamCodec::encodeSet(QStringLiteral("RATE_ROLL_KP"), Type::F32, 0.006, 7));
  QCOMPARE(g.id, QStringLiteral("RATE_ROLL_KP"));
  QCOMPARE(g.type, quint8(9));
  QCOMPARE(g.req, quint8(7));
  QCOMPARE(float(g.v), 0.006f);
  decode(ParamCodec::encodeCommit(12, 1, 3));
  QCOMPARE(g.req, quint8(12));
  QCOMPARE(g.op, quint8(1));
  QCOMPARE(g.group, quint8(3));
  decode(ParamCodec::encodeRequestList());
  QCOMPARE(g.lists, 1);
  decode(ParamCodec::encodeRequestReadById(QStringLiteral("SYS_PTBL_CRC")));
  QCOMPARE(g.rid, QStringLiteral("SYS_PTBL_CRC"));
  QCOMPARE(int(g.ri), -1);
  decode(ParamCodec::encodeRequestReadByIndex(17));
  QCOMPARE(int(g.ri), 17);
  // Never put an unencodable value or a "no ack" req_seq on the wire.
  QVERIFY(ParamCodec::encodeSet(QStringLiteral("B_U8"), Type::U8, 300, 1).isEmpty());
  QVERIFY(ParamCodec::encodeSet(QStringLiteral("B_U8"), Type::U8, 3, 0).isEmpty());
  QVERIFY(ParamCodec::encodeCommit(0, 0, 0).isEmpty());
  QVERIFY(ParamCodec::encodeCommit(1, 2, 0).isEmpty());
}

// ---------------------------------------------------------------- reading
void TstParamClient::nothingIsSentWithoutTheCapability() {
  Rig r;
  QCOMPARE(r.cl.state(), Client::State::NoCaps);
  r.cl.onSynced(true, r.now); // synced, but no capability seen
  r.step(5000);
  QCOMPARE(r.cl.state(), Client::State::NoCaps);
  QCOMPARE(r.fc.listRequests, 0);
  QVERIFY(r.fc.readIds.isEmpty());
  r.cl.onCapabilities(1u | 2u | 4u, r.now); // other capabilities, not PARAMS
  r.step(5000);
  QCOMPARE(r.cl.state(), Client::State::NoCaps);
  QVERIFY(r.fc.readIds.isEmpty());
  QString why;
  QVERIFY(!r.cl.set(QStringLiteral("A_LIVE"), 2, r.now, &why));
  QVERIFY(!why.isEmpty());
  QCOMPARE(r.fc.sets, 0);
}

void TstParamClient::waitsForSyncThenChecksCrcThenLists() {
  Rig r;
  r.cl.onCapabilities(caps(), r.now);
  QCOMPARE(r.cl.state(), Client::State::Unsynced); // capability alone sends nothing
  QVERIFY(r.fc.readIds.isEmpty());
  r.cl.onSynced(true, r.now);
  QCOMPARE(r.fc.readIds, QStringList{QStringLiteral("SYS_PTBL_CRC")}); // CRC first
  r.step(100);
  QCOMPARE(r.cl.state(), Client::State::Ready);
  QCOMPARE(r.fc.listRequests, 1);
  for (const Entry &e : r.cat.entries()) {
    const Client::Value *v = r.cl.value(e.id);
    QVERIFY2(v, qPrintable(e.id));
    if (e.id != QLatin1String("SYS_PTBL_CRC"))
      QCOMPARE(float(v->v), float(e.def));
  }
  QCOMPARE(r.cl.pendingCount(), 0);
  QVERIFY(r.cl.missing().isEmpty());
}

void TstParamClient::aDifferentCrcIsReadOnly() {
  Rig r;
  r.fc.crc ^= 1u;
  r.up();
  QCOMPARE(r.cl.state(), Client::State::Mismatch);
  QVERIFY(r.cl.stateText().contains(QLatin1String("read-only")));
  QVERIFY(!r.logs.isEmpty());
  QCOMPARE(r.cl.raw().size(), r.cat.count()); // raw values are still shown
  QString why;
  QVERIFY(!r.cl.set(QStringLiteral("A_LIVE"), 2, r.now, &why));
  QVERIFY(!r.cl.commit(1, r.now, &why));
  QCOMPARE(r.fc.sets, 0);
  QCOMPARE(r.fc.commits, 0);
}

void TstParamClient::aDifferentCountIsReadOnly() {
  Rig r;
  r.fc.count = r.cat.count() - 1; // same CRC, different count: belt and braces
  r.up();
  QCOMPARE(r.cl.state(), Client::State::Mismatch);
  QString why;
  QVERIFY(!r.cl.set(QStringLiteral("A_LIVE"), 2, r.now, &why));
  QCOMPARE(r.fc.sets, 0);
}

void TstParamClient::droppedValuesAreAskedForAgain() {
  Rig r;
  r.fc.dropOnList = {3, 5};
  r.up();
  QCOMPARE(r.cl.state(), Client::State::Listing); // gap: not Ready yet
  QCOMPARE(r.cl.missing(), (QVector<int>{3, 5}));
  r.step(700);
  QCOMPARE(r.cl.state(), Client::State::Ready);
  QVERIFY(r.fc.readIndexes.contains(3));
  QVERIFY(r.fc.readIndexes.contains(5));
  QVERIFY(r.cl.missing().isEmpty());
}

void TstParamClient::aValueThatNeverArrivesIsReportedNotHidden() {
  Rig r;
  r.fc.dropAlways = {4};
  r.up();
  r.step(5000);
  QCOMPARE(r.cl.state(), Client::State::Incomplete);
  QCOMPARE(r.cl.missing(), (QVector<int>{4}));
  QCOMPARE(r.fc.readIndexes.count(4), Client::kReadRetries); // bounded
  bool said = false;
  for (const QString &l : r.logs)
    said = said || (l.contains(QLatin1String("G1_Y")) && l.contains(QLatin1String("never arrived")));
  QVERIFY(said);
  QString why;
  QVERIFY(!r.cl.set(QStringLiteral("A_LIVE"), 2, r.now, &why)); // no writes on a short table
  QCOMPARE(r.fc.sets, 0);
  // The value comes back (the vehicle recovers): a refresh completes the table.
  r.fc.dropAlways.clear();
  r.cl.refresh(r.now);
  r.step(200);
  QCOMPARE(r.cl.state(), Client::State::Ready);
}

void TstParamClient::losingTheCapabilityOrSyncStopsEverything() {
  Rig r;
  r.up();
  r.fc.holdAcks = true;
  QVERIFY(r.cl.set(QStringLiteral("A_LIVE"), 2, r.now));
  QCOMPARE(r.cl.pendingCount(), 1);
  r.cl.onSynced(false, r.now);
  QCOMPARE(r.cl.state(), Client::State::Unsynced);
  QCOMPARE(r.cl.pendingCount(), 0);
  QCOMPARE(r.last().outcome, Client::Outcome::Rejected);
  r.fc.flush(); // the late ack finds nothing pending
  QCOMPARE(r.cl.staleAcks(), 1);
  r.fc.holdAcks = false;
  r.cl.onSynced(true, r.now);
  r.step(100);
  QCOMPARE(r.cl.state(), Client::State::Ready);
  QCOMPARE(r.fc.listRequests, 2); // read again from scratch
  r.cl.onCapabilities(0, r.now);
  QCOMPARE(r.cl.state(), Client::State::NoCaps);
  QVERIFY(!r.cl.set(QStringLiteral("A_LIVE"), 3, r.now));
}

// ----------------------------------------------------------------- writes
void TstParamClient::aLiveWriteIsConfirmedByTheEchoNotTheAck() {
  Rig r;
  r.up();
  r.fc.holdAcks = true;
  r.fc.dropEcho = true; // the echo is held back: only the ack can arrive
  QVERIFY(r.cl.set(QStringLiteral("A_LIVE"), 2.5, r.now));
  r.fc.flush();
  QVERIFY2(r.ev.isEmpty(), "applied from the ack alone");
  QCOMPARE(float(r.cl.value(QStringLiteral("A_LIVE"))->v), 1.0f); // shows the last read
  r.step(1100); // no echo: the client reads it itself
  QCOMPARE(r.ev.size(), 1);
  QCOMPARE(r.last().outcome, Client::Outcome::Applied);
  QCOMPARE(float(r.cl.value(QStringLiteral("A_LIVE"))->v), 2.5f);
  QVERIFY(r.cl.value(QStringLiteral("A_LIVE"))->flags & Unsaved);
  QCOMPARE(r.cl.pendingCount(), 0);
}

void TstParamClient::anAckWithoutAnEchoIsNotApplied() {
  Rig r;
  r.up();
  r.fc.dropEcho = true;
  r.fc.dropAlways = {r.fc.idx(QStringLiteral("A_LIVE"))}; // and it cannot be read back
  QVERIFY(r.cl.set(QStringLiteral("A_LIVE"), 2.5, r.now));
  QVERIFY(r.ev.isEmpty());
  r.step(2300);
  QCOMPARE(r.ev.size(), 1);
  QCOMPARE(r.last().outcome, Client::Outcome::Failed);
  QVERIFY(r.last().text.contains(QLatin1String("did not confirm")));
  QCOMPARE(float(r.cl.value(QStringLiteral("A_LIVE"))->v), 1.0f); // never shown as written
}

void TstParamClient::anEchoBeforeTheAckIsFine() {
  Rig r;
  r.up();
  r.fc.echoBeforeAck = true;
  QVERIFY(r.cl.set(QStringLiteral("B_U8"), 9, r.now));
  QCOMPARE(r.ev.size(), 1);
  QCOMPARE(r.last().outcome, Client::Outcome::Applied);
  QCOMPARE(r.cl.value(QStringLiteral("B_U8"))->v, 9.0);
}

void TstParamClient::reqSeqIsNeverZeroAndWraps() {
  Rig r;
  r.up();
  for (int i = 0; i < 600; ++i)
    QVERIFY(r.cl.set(QStringLiteral("B_U8"), double(i % 100), r.now));
  QCOMPARE(r.fc.setLog.size(), 600);
  for (int i = 0; i < r.fc.setLog.size(); ++i) {
    QVERIFY(r.fc.setLog[i].reqSeq != 0);
    if (i > 0) {
      const int prev = r.fc.setLog[i - 1].reqSeq;
      QCOMPARE(int(r.fc.setLog[i].reqSeq), prev == 255 ? 1 : prev + 1); // 255 -> 1, never 0
    }
  }
  QCOMPARE(r.ev.size(), 600);
}

void TstParamClient::aStaleOrDuplicateAckIsIgnored() {
  Rig r;
  r.up();
  r.fc.duplicateAck = true;
  QVERIFY(r.cl.set(QStringLiteral("B_U8"), 9, r.now));
  QCOMPARE(r.ev.size(), 1); // one event, not two
  QCOMPARE(r.cl.staleAcks(), 1);
  // An ack for a request we never made, and one for another command.
  r.cl.onCommandAck(Client::kMsgidParamSet, 200, 0, r.now);
  r.cl.onCommandAck(1234, 1, 0, r.now);
  QCOMPARE(r.cl.staleAcks(), 3);
  QCOMPARE(r.ev.size(), 1);
  // An ack with the right req_seq but the wrong command is not the write's.
  r.fc.duplicateAck = false;
  r.fc.holdAcks = true;
  QVERIFY(r.cl.set(QStringLiteral("B_U8"), 10, r.now));
  const quint8 seq = r.fc.setLog.last().reqSeq;
  r.cl.onCommandAck(Client::kMsgidParamCommit, seq, 0, r.now);
  QCOMPARE(r.cl.pendingCount(), 1);
  QCOMPARE(r.ev.size(), 1);
}

void TstParamClient::localChecksRefuseWithoutSending() {
  Rig r;
  r.up();
  QString why;
  auto no = [&](const QString &id, double v, const char *needle) {
    why.clear();
    QVERIFY2(!r.cl.set(id, v, r.now, &why), needle);
    QVERIFY2(why.contains(QLatin1String(needle)), qPrintable(why));
  };
  no(QStringLiteral("NOPE"), 1, "unknown");
  no(QStringLiteral("SYS_PTBL_CRC"), 1, "read-only");
  no(QStringLiteral("A_LIVE"), 11, "outside");
  no(QStringLiteral("A_LIVE"), -1, "outside");
  no(QStringLiteral("A_LIVE"), std::numeric_limits<double>::quiet_NaN(), "finite");
  no(QStringLiteral("B_U8"), 3.5, "whole");
  r.cl.setArmed(true);
  no(QStringLiteral("L2_Z"), 4, "armed");
  QCOMPARE(r.fc.sets, 0); // nothing reached the wire
  r.cl.setArmed(false);
  r.fc.holdAcks = true;
  QVERIFY(r.cl.set(QStringLiteral("A_LIVE"), 2, r.now));
  no(QStringLiteral("A_LIVE"), 3, "in progress"); // one write per parameter at a time
}

void TstParamClient::aVehicleRefusalIsSurfacedAndNothingChanges() {
  Rig r;
  r.up();
  r.fc.armed = true; // the vehicle is armed, the client has not heard yet
  QVERIFY(r.cl.set(QStringLiteral("L2_Z"), 4, r.now));
  QCOMPARE(r.ev.size(), 1);
  QCOMPARE(r.last().outcome, Client::Outcome::Refused);
  QCOMPARE(r.last().id, QStringLiteral("L2_Z"));
  QVERIFY(r.last().text.contains(QLatin1String("refused")));
  QCOMPARE(float(r.cl.value(QStringLiteral("L2_Z"))->v), 3.0f);
  QCOMPARE(r.cl.pendingCount(), 0);
}

void TstParamClient::aWriteWithoutAnAckIsSentAgainOnceThenReported() {
  Rig r;
  r.up();
  r.fc.dropAcks = 5;
  QVERIFY(r.cl.set(QStringLiteral("B_U8"), 9, r.now));
  r.step(Client::kAckMs + 100);
  QCOMPARE(r.fc.sets, 2); // sent again, once
  QCOMPARE(r.fc.setLog[0].reqSeq, r.fc.setLog[1].reqSeq);
  QVERIFY(r.ev.isEmpty());
  r.step(Client::kAckMs + 100);
  QCOMPARE(r.fc.sets, 2); // not a third time
  QCOMPARE(r.ev.size(), 1);
  QCOMPARE(r.last().outcome, Client::Outcome::TimedOut);
  QCOMPARE(r.cl.pendingCount(), 0);
}

void TstParamClient::notSynchronisedIsRetriedOnceInTheWindow() {
  Rig r;
  r.up();
  r.fc.tempRejectWrites = 1;
  QVERIFY(r.cl.set(QStringLiteral("B_U8"), 9, r.now));
  QCOMPARE(r.fc.sets, 2);
  QVERIFY(r.fc.setLog[0].reqSeq != r.fc.setLog[1].reqSeq); // a new request
  QCOMPARE(r.ev.size(), 1);
  QCOMPARE(r.last().outcome, Client::Outcome::Applied);
  // Two refusals in a row: one retry, then it is reported.
  r.fc.tempRejectWrites = 2;
  r.ev.clear();
  QVERIFY(r.cl.set(QStringLiteral("B_U8"), 10, r.now));
  QCOMPARE(r.fc.sets, 4);
  QCOMPARE(r.ev.size(), 1);
  QCOMPARE(r.last().outcome, Client::Outcome::Rejected);
  // Outside the window after the sync edge: no retry at all.
  r.step(Client::kRetryWindowMs + 500);
  r.fc.tempRejectWrites = 1;
  r.ev.clear();
  const int before = r.fc.sets;
  QVERIFY(r.cl.set(QStringLiteral("B_U8"), 11, r.now));
  QCOMPARE(r.fc.sets, before + 1);
  QCOMPARE(r.last().outcome, Client::Outcome::Rejected);
}

void TstParamClient::aRetryIsDroppedIfTheUserActedSince() {
  Rig r;
  r.up();
  QVERIFY(r.cl.set(QStringLiteral("G1_X"), 4, r.now)); // stage something to commit
  r.ev.clear();
  r.fc.holdAcks = true;
  r.fc.tempRejectWrites = 1;
  QVERIFY(r.cl.commit(1, r.now));      // its ack will be TEMPORARILY_REJECTED
  QVERIFY(r.cl.set(QStringLiteral("B_U8"), 8, r.now)); // the user acts again meanwhile
  r.fc.holdAcks = false;
  r.fc.flush();
  QCOMPARE(r.fc.commitReqSeqs.size(), 1); // not sent a second time: a stale commit would surprise
  QCOMPARE(r.fc.commits, 0);              // and never applied
  bool rejected = false;
  for (const auto &e : r.ev)
    rejected = rejected || (e.op == Client::Op::Commit && e.outcome == Client::Outcome::Rejected);
  QVERIFY(rejected);
}

void TstParamClient::strictModeRefusalSaysSecureLinkRequired() {
  Rig r;
  r.up(caps(true)); // the vehicle is keyed (SECURE_LINK) ...
  r.fc.strict = true; // ... and refuses an unsecured link
  r.cl.setLinkSecure(false);
  QVERIFY(r.cl.set(QStringLiteral("A_LIVE"), 2, r.now));
  QCOMPARE(r.last().outcome, Client::Outcome::Refused);
  QVERIFY2(r.last().text.contains(QLatin1String("secure link required")), qPrintable(r.last().text));
  // On a secured link the same denial is the generic one.
  r.cl.setLinkSecure(true);
  QVERIFY(r.cl.set(QStringLiteral("A_LIVE"), 3, r.now));
  QVERIFY(!r.last().text.contains(QLatin1String("secure link required")));
  // A vehicle without the secure-link capability never gets that message.
  Rig q;
  q.up(caps(false));
  q.fc.strict = true;
  QVERIFY(q.cl.set(QStringLiteral("A_LIVE"), 2, q.now));
  QVERIFY(!q.last().text.contains(QLatin1String("secure link required")));
}

// ----------------------------------------------------------------- groups
void TstParamClient::aGroupIsStagedThenCommitted() {
  Rig r;
  r.up();
  QVERIFY(r.cl.set(QStringLiteral("G1_X"), 4, r.now));
  QCOMPARE(r.last().outcome, Client::Outcome::Applied);
  QVERIFY(r.last().text.contains(QLatin1String("staged")));
  QVERIFY(r.cl.value(QStringLiteral("G1_X"))->flags & Staged);
  QVERIFY(r.cl.set(QStringLiteral("G1_Y"), 5, r.now));
  QCOMPARE(r.fc.commits, 0); // nothing is live until the group is committed
  QCOMPARE(float(r.fc.live(QStringLiteral("G1_X"))), 1.0f);
  r.ev.clear();
  QString why;
  QVERIFY(!r.cl.commit(2, r.now, &why)); // group 2 has nothing staged
  QVERIFY(why.contains(QLatin1String("nothing")));
  QVERIFY(r.cl.commit(1, r.now));
  QCOMPARE(r.fc.commits, 1);
  QCOMPARE(r.last().op, Client::Op::Commit);
  QCOMPARE(r.last().outcome, Client::Outcome::Applied);
  QCOMPARE(float(r.fc.live(QStringLiteral("G1_X"))), 4.0f);
  QCOMPARE(float(r.fc.live(QStringLiteral("G1_Y"))), 5.0f);
  QCOMPARE(float(r.cl.value(QStringLiteral("G1_X"))->v), 4.0f);
  QVERIFY(!(r.cl.value(QStringLiteral("G1_X"))->flags & Staged));
  QVERIFY(r.cl.value(QStringLiteral("G1_X"))->flags & Unsaved); // live, not yet saved
  // The save landing clears UNSAVED: the client notices the generation change.
  r.step(Client::kSettleMs + 100);
  const int lists = r.fc.listRequests;
  r.fc.landSave();
  r.step(200);
  QVERIFY(!(r.cl.value(QStringLiteral("G1_X"))->flags & Unsaved));
  QCOMPARE(r.fc.listRequests, lists + 1);
}

void TstParamClient::discardDropsTheStagedValues() {
  Rig r;
  r.up();
  QVERIFY(r.cl.set(QStringLiteral("G1_X"), 4, r.now));
  r.ev.clear();
  QVERIFY(r.cl.discard(1, r.now));
  QCOMPARE(r.fc.discards, 1);
  QCOMPARE(r.last().op, Client::Op::Discard);
  QCOMPARE(r.last().outcome, Client::Outcome::Applied);
  QVERIFY(!(r.cl.value(QStringLiteral("G1_X"))->flags & Staged));
  QCOMPARE(float(r.cl.value(QStringLiteral("G1_X"))->v), 1.0f);
  QString why;
  QVERIFY(!r.cl.discard(1, r.now, &why)); // nothing left to discard
}

void TstParamClient::aDeniedCommitKeepsTheStagedValues() {
  Rig r;
  r.up();
  QVERIFY(r.cl.set(QStringLiteral("L2_Z"), 4, r.now));
  r.fc.armed = true; // armed since: the vehicle refuses the locked group
  r.ev.clear();
  QVERIFY(r.cl.commit(2, r.now));
  QCOMPARE(r.last().outcome, Client::Outcome::Refused);
  QVERIFY(r.last().text.contains(QLatin1String("staged values are kept")));
  QVERIFY(r.cl.value(QStringLiteral("L2_Z"))->flags & Staged);
  QCOMPARE(float(r.fc.live(QStringLiteral("L2_Z"))), 3.0f);
}

void TstParamClient::anArmedLockIsCheckedBeforeCommit() {
  Rig r;
  r.up();
  QVERIFY(r.cl.set(QStringLiteral("L2_Z"), 4, r.now));
  r.cl.setArmed(true);
  QString why;
  QVERIFY(!r.cl.commit(2, r.now, &why));
  QVERIFY(why.contains(QLatin1String("armed")));
  QCOMPARE(r.fc.commits, 0);
  QVERIFY(!r.cl.commit(9, r.now, &why)); // a group the catalog does not have
  QVERIFY(why.contains(QLatin1String("unknown group")));
  r.cl.setArmed(false);
  QVERIFY(r.cl.commit(2, r.now));
  QCOMPARE(r.fc.commits, 1);
}

void TstParamClient::someoneElsesChangeTriggersOneRelist() {
  Rig r;
  r.up();
  r.step(Client::kSettleMs + 100);
  QCOMPARE(r.fc.listRequests, 1);
  // Another ground station changes a value; the vehicle sends it (new generation).
  r.fc.setLive(QStringLiteral("A_LIVE"), 7);
  r.fc.sendValue(r.fc.idx(QStringLiteral("A_LIVE")));
  r.step(200);
  QCOMPARE(r.fc.listRequests, 2);
  QCOMPARE(float(r.cl.value(QStringLiteral("A_LIVE"))->v), 7.0f);
  // A burst of further changes does not relist again within kRelistGapMs.
  r.fc.setLive(QStringLiteral("B_U8"), 6);
  r.fc.sendValue(r.fc.idx(QStringLiteral("B_U8")));
  r.step(200);
  QCOMPARE(r.fc.listRequests, 2);
  // Our own write's generation jump is not "someone else".
  r.step(Client::kRelistGapMs + 100);
  const int n = r.fc.listRequests;
  QVERIFY(r.cl.set(QStringLiteral("B_U8"), 12, r.now));
  r.step(300);
  QCOMPARE(r.fc.listRequests, n);
}

void TstParamClient::theRealCatalogEndToEnd() {
  QByteArray json;
  {
    QFile f(generatedCatalogPath());
    QVERIFY(f.open(QIODevice::ReadOnly));
    json = f.readAll();
  }
  Rig r(json);
  r.up();
  QCOMPARE(r.cl.state(), Client::State::Ready);
  QVERIFY(r.cat.count() >= 39);
  QString why;
  // The catalog's own cap: a gain far above it is refused here, before the wire.
  QVERIFY(!r.cl.set(QStringLiteral("RATE_ROLL_KP"), 0.5, r.now, &why));
  QVERIFY(why.contains(QLatin1String("outside")));
  QVERIFY(!r.cl.set(QStringLiteral("RATE_ROLL_KP"), 0.0, r.now, &why)); // loop off
  QCOMPARE(r.fc.sets, 0);
  const Entry &kp = *r.cat.byId(QStringLiteral("RATE_ROLL_KP"));
  QVERIFY(r.cl.set(kp.id, kp.def * 2, r.now));
  QVERIFY(r.cl.commit(kp.group, r.now));
  QCOMPARE(r.last().outcome, Client::Outcome::Applied);
  QCOMPARE(float(r.cl.value(kp.id)->v), float(kp.def * 2));
}

QTEST_MAIN(TstParamClient)
#include "tst_param_client.moc"
