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
#include "ParamApplier.h"

#include "FakeParamFc.h"
#include "NavlinkRouter.h"
#include "ParamService.h"

#include <QTemporaryFile>
#include <QThread>

extern "C" {
#include "navlink_msgs.h"
}

using namespace params;

namespace {

const char *kSynth = R"({
 "catalog": 1, "table_crc": 3735928559,
 "groups": {"1": {"name":"G1","doc":"d"}, "2": {"name":"LOCKED","doc":"d"}},
 "params": [
  {"id":"SYS_PTBL_CRC","type":"U32","min":0,"max":4294967295,"default":0,"flags_byte":1,"group":0,"index":0},
  {"id":"A_LIVE","type":"F32","min":0,"max":10,"default":1,"flags_byte":8,"group":0,"index":1},
  {"id":"B_U8","type":"U8","min":0,"max":200,"default":5,"flags_byte":0,"group":0,"index":2},
  {"id":"G1_X","type":"F32","min":0,"max":10,"default":1,"flags_byte":8,"group":1,"index":3},
  {"id":"G1_Y","type":"F32","min":0,"max":10,"default":2,"flags_byte":8,"group":1,"index":4},
  {"id":"L2_Z","type":"F32","min":0,"max":10,"default":3,"flags_byte":10,"group":2,"index":5}
 ]})";

// The worker-thread side of the app: a router fed with whole frames, whose
// decoded leaves leave as signals (queued into the GUI thread, as DroneProtocol
// does).
class Wire : public QObject {
  Q_OBJECT
public:
  Wire() {
    router.onHeartbeatCaps = [this](uint8_t s, uint32_t c) { emit heartbeat(s, c); };
    router.onParamValue = [this](const ParamCodec::ValueMsg &m) { emit value(m); };
    router.onCommandAck = [this](uint32_t c, uint8_t r, uint8_t res) { emit ack(c, r, res); };
  }
  NavlinkRouter router;
public slots:
  void feed(const QByteArray &f) { router.feed(f); }
signals:
  void heartbeat(quint8, quint32);
  void value(const params::ParamCodec::ValueMsg &);
  void ack(quint32, quint8, quint8);
};

QByteArray frameOf(size_t n, const uint8_t *buf) {
  return QByteArray(reinterpret_cast<const char *>(buf), int(n));
}

struct Rig {
  QTemporaryFile file;
  QThread th;
  Wire wire;
  ParamService svc;
  FakeParamFc fc;
  qint64 clk = 1000;
  quint8 seq = 0;
  QVector<Client::WriteEvent> ev;

  explicit Rig(const QByteArray &json = QByteArray(kSynth)) {
    file.setFileTemplate(QDir::tempPath() + QStringLiteral("/params_XXXXXX.json"));
    if (!file.open())
      qFatal("temp file");
    file.write(json);
    file.flush();
    if (!svc.loadCatalog(file.fileName()))
      qFatal("catalog");
    fc.load(&svc.catalog());
    fc.synced = true;
    svc.setClock([this] { return clk; });
    svc.setSendFrame([this](const QByteArray &f) { fc.receive(f); });
    fc.toClientValue = [this](const ParamCodec::ValueMsg &m) {
      navlink_param_value_t w{};
      std::memcpy(w.param_id, m.id.toLatin1().constData(), size_t(qMin(16, int(m.id.size()))));
      w.index = m.index;
      w.count = m.count;
      w.generation = m.generation;
      w.type = m.type;
      std::memcpy(w.value, m.raw, 8);
      w.flags = m.flags;
      uint8_t buf[NAVLINK_MAX_FRAME];
      send(frameOf(navlink_param_value_encode(buf, &w, seq++, 1, 1), buf));
    };
    fc.toClientAck = [this](quint32 c, quint8 r, quint8 res) {
      navlink_command_ack_t a{};
      a.command = c;
      a.req_seq = r;
      a.result = res;
      uint8_t buf[NAVLINK_MAX_FRAME];
      send(frameOf(navlink_command_ack_encode(buf, &a, seq++, 1, 1), buf));
    };
    wire.moveToThread(&th);
    th.start();
    QObject::connect(&wire, &Wire::heartbeat, &svc, &ParamService::onHeartbeat);
    QObject::connect(&wire, &Wire::value, &svc, &ParamService::onParamValue);
    QObject::connect(&wire, &Wire::ack, &svc, &ParamService::onCommandAck);
    QObject::connect(&svc, &ParamService::writeFinished, &svc,
            [this](const Client::WriteEvent &e) { ev << e; });
  }
  ~Rig() {
    th.quit();
    th.wait();
  }
  void send(const QByteArray &f) {
    QMetaObject::invokeMethod(&wire, "feed", Qt::QueuedConnection, Q_ARG(QByteArray, f));
  }
  // The vehicle's heartbeat, through the router and the thread hop.
  void heartbeat(quint8 navState, quint32 caps) {
    navlink_heartbeat_t h{};
    h.nav_state = navState;
    h.capabilities = caps;
    uint8_t buf[NAVLINK_MAX_FRAME];
    send(frameOf(navlink_heartbeat_encode(buf, &h, seq++, 1, 1), buf));
  }
  // Time passes: the app's 50 ms timer ticks and the worker thread delivers.
  void run(qint64 ms) {
    for (qint64 t = 0; t < ms; t += 50) {
      clk += 50;
      svc.tick();
      QTest::qWait(6); // the frames cross the worker thread both ways
    }
  }
  void up(quint8 nav = 2, quint32 caps = 8) {
    heartbeat(nav, caps);
    run(100);
    svc.onSynced(true);
    run(300);
  }
};

} // namespace

class TstParamApplier : public QObject {
  Q_OBJECT
private slots:
  void everyGroupIsStagedCommittedAndReportedOnce();
  void oneGroupZeroCommitByDefault();
  void liveValuesAreSavedByTheGroupZeroCommit();
  void aFailedGroupZeroCommitIsOneDiscard();
  void destroyingItMidApplyUnlocksTheService();
  void staleStagedValuesMakeItRefuse();
  void oneBadValueSendsNothing();
  void armedRefusesBeforeAnythingIsSent();
  void aRefusedCommitIsDiscardedAndSaidSo();
  void editsAndASecondApplyAreRefusedWhileItRuns();
  void onlyAWindowOfSetsIsInFlight();
  void aLostLinkMidApplyIsReportedUnknownAndUnlocks();
};

namespace {
using Item = ParamApplier::Item;
struct Done {
  QVector<ParamApplier::Result> r;
};
} // namespace

#define APPLIER(rig, name, done)                                                  \
  ParamApplier name(&rig.svc);                                                    \
  Done done;                                                                      \
  QObject::connect(&name, &ParamApplier::finished, &name,                         \
                   [&done](const ParamApplier::Result &x) { done.r << x; })

void TstParamApplier::everyGroupIsStagedCommittedAndReportedOnce() {
  Rig r;
  r.up();
  APPLIER(r, ap, done);
  ap.setGroupZeroCommit(false); // group by group
  QString why;
  QVERIFY2(ap.apply({{"G1_X", 4}, {"G1_Y", 5}, {"L2_Z", 6}, {"A_LIVE", 2}}, &why), qPrintable(why));
  QVERIFY(r.svc.locked());
  r.run(1500);
  QCOMPARE(done.r.size(), 1);
  QVERIFY2(done.r[0].ok, qPrintable(done.r[0].text));
  QCOMPARE(r.fc.commits, 2); // groups 1 and 2, one commit each
  QCOMPARE(done.r[0].groups.size(), 2);
  QVERIFY(done.r[0].text.contains(QLatin1String("1 of them live")));
  QVERIFY(!r.svc.locked());
  QCOMPARE(r.svc.client().value(QStringLiteral("G1_Y"))->v, 5.0);
}

void TstParamApplier::oneGroupZeroCommitByDefault() {
  Rig r;
  r.up();
  APPLIER(r, ap, done);
  QVERIFY(ap.groupZeroCommit());
  QVERIFY(ap.apply({{"G1_X", 4}, {"L2_Z", 6}}));
  r.run(1500);
  QCOMPARE(done.r.size(), 1);
  QVERIFY2(done.r[0].ok, qPrintable(done.r[0].text));
  QCOMPARE(r.fc.commits, 1);
}

void TstParamApplier::liveValuesAreSavedByTheGroupZeroCommit() {
  Rig r;
  r.up();
  APPLIER(r, ap, done);
  QVERIFY(ap.apply({{"A_LIVE", 2}})); // only a live value
  r.run(1500);
  QCOMPARE(done.r.size(), 1);
  QVERIFY2(done.r[0].ok, qPrintable(done.r[0].text));
  QCOMPARE(r.fc.commits, 1); // commit 0 saves it
  QVERIFY(done.r[0].text.contains(QLatin1String("live and saved")));
  r.fc.commits = 0;
  APPLIER(r, ap2, done2);
  ap2.setGroupZeroCommit(false); // per group: nothing to commit, and it says so
  QVERIFY(ap2.apply({{"A_LIVE", 3}}));
  r.run(800);
  QCOMPARE(done2.r.size(), 1);
  QCOMPARE(r.fc.commits, 0);
  QVERIFY(done2.r[0].text.contains(QLatin1String("not saved")));
}

void TstParamApplier::aFailedGroupZeroCommitIsOneDiscard() {
  Rig r;
  r.up();
  APPLIER(r, ap, done);
  r.fc.saveFails = true;
  QVERIFY(ap.apply({{"G1_X", 4}, {"L2_Z", 6}}));
  r.run(3000);
  QCOMPARE(done.r.size(), 1);
  QVERIFY(!done.r[0].ok);
  QCOMPARE(r.fc.discards, 1); // one discard of group 0, not one per group
  QCOMPARE(done.r[0].groups.size(), 2);
  for (const auto &g : done.r[0].groups)
    QCOMPARE(g.state, ParamApplier::GroupResult::Discarded);
}

void TstParamApplier::destroyingItMidApplyUnlocksTheService() {
  Rig r;
  r.up();
  r.fc.holdAcks = true;
  {
    ParamApplier ap(&r.svc);
    QVERIFY(ap.apply({{"G1_X", 4}}));
    QVERIFY(r.svc.locked());
  }
  QVERIFY(!r.svc.locked());
}

void TstParamApplier::staleStagedValuesMakeItRefuse() {
  Rig r;
  r.up();
  QString why;
  QVERIFY(r.svc.set(QStringLiteral("G1_Y"), 3, &why)); // staged by an earlier edit
  r.run(300);
  APPLIER(r, ap, done);
  QVERIFY(!ap.apply({{"L2_Z", 6}}, &why)); // a group-0 commit would apply G1_Y too
  QVERIFY2(why.contains(QLatin1String("G1_Y")), qPrintable(why));
  ap.setGroupZeroCommit(false);
  QVERIFY2(ap.apply({{"L2_Z", 6}}, &why), qPrintable(why)); // group 2 only: G1_Y untouched
  r.run(1500);
  QCOMPARE(done.r.size(), 1);
  QVERIFY2(done.r[0].ok, qPrintable(done.r[0].text));
  QCOMPARE(r.fc.commits, 1);
  QVERIFY(r.svc.client().value(QStringLiteral("G1_Y"))->flags & Staged); // still staged
}

void TstParamApplier::oneBadValueSendsNothing() {
  Rig r;
  r.up();
  APPLIER(r, ap, done);
  QString why;
  QVERIFY(!ap.apply({{"G1_X", 4}, {"G1_Y", 99}}, &why)); // G1_Y is 0..10
  QVERIFY2(why.contains(QLatin1String("G1_Y")), qPrintable(why));
  QVERIFY(!ap.apply({{"NOPE", 1}}, &why));
  QVERIFY(!ap.apply({{"G1_X", 1}, {"G1_X", 2}}, &why)); // given twice
  QVERIFY(!ap.apply({{"SYS_PTBL_CRC", 1}}, &why));      // read-only
  r.run(300);
  QCOMPARE(r.fc.sets, 0);
  QVERIFY(!r.svc.locked());
  QVERIFY(done.r.isEmpty());
}

void TstParamApplier::armedRefusesBeforeAnythingIsSent() {
  Rig r;
  r.up();
  r.heartbeat(4, 8);
  r.run(100);
  APPLIER(r, ap, done);
  QString why;
  QVERIFY(!ap.apply({{"G1_X", 4}, {"L2_Z", 6}}, &why)); // L2_Z is locked when armed
  QVERIFY2(why.contains(QLatin1String("armed")), qPrintable(why));
  r.run(300);
  QCOMPARE(r.fc.sets, 0);
}

void TstParamApplier::aRefusedCommitIsDiscardedAndSaidSo() {
  Rig r;
  r.up();
  APPLIER(r, ap, done);
  r.fc.saveFails = true; // the vehicle's storage refuses the commit
  QVERIFY(ap.apply({{"G1_X", 4}, {"G1_Y", 5}}));
  r.run(2500);
  QCOMPARE(done.r.size(), 1);
  QVERIFY(!done.r[0].ok);
  QVERIFY(!done.r[0].unknown);
  QCOMPARE(done.r[0].groups.size(), 1);
  QCOMPARE(done.r[0].groups[0].state, ParamApplier::GroupResult::Discarded);
  QVERIFY(done.r[0].text.contains(QLatin1String("G1")));
  QCOMPARE(r.fc.discards, 1);
  QVERIFY(!r.svc.locked());
  // The vehicle keeps what it had.
  r.run(500);
  QCOMPARE(r.svc.client().value(QStringLiteral("G1_X"))->v, 1.0);
}

void TstParamApplier::editsAndASecondApplyAreRefusedWhileItRuns() {
  Rig r;
  r.up();
  APPLIER(r, ap, done);
  r.fc.holdAcks = true;
  QVERIFY(ap.apply({{"G1_X", 4}}));
  QString why;
  QVERIFY(!r.svc.set(QStringLiteral("B_U8"), 9, &why)); // an edit in the panel
  QVERIFY2(why.contains(QLatin1String("apply")), qPrintable(why));
  why.clear();
  QVERIFY(!r.svc.commit(1, &why));
  QVERIFY(!why.isEmpty());
  QVERIFY(!r.svc.discard(1, &why));
  QVERIFY(!r.svc.whyNotCommit(1).isEmpty()); // the buttons are greyed with it
  QVERIFY(!ap.apply({{"G1_Y", 3}}, &why));
  QVERIFY2(why.contains(QLatin1String("already")), qPrintable(why));
  r.fc.holdAcks = false;
  r.fc.flush();
  r.run(1500);
  QCOMPARE(done.r.size(), 1);
  QVERIFY(done.r[0].ok);
  QVERIFY(r.svc.set(QStringLiteral("B_U8"), 9, &why)); // free again
}

void TstParamApplier::onlyAWindowOfSetsIsInFlight() {
  Rig r;
  r.up();
  APPLIER(r, ap, done);
  r.fc.holdAcks = true;
  QVERIFY(ap.apply({{"G1_X", 4}, {"G1_Y", 5}, {"L2_Z", 6}, {"A_LIVE", 2}, {"B_U8", 7}}));
  r.run(200);
  QCOMPARE(r.fc.sets, ParamApplier::kWindow);
  r.fc.holdAcks = false;
  r.fc.flush();
  r.run(2500);
  QCOMPARE(r.fc.sets, 5);
  QCOMPARE(done.r.size(), 1);
  QVERIFY2(done.r[0].ok, qPrintable(done.r[0].text));
}

void TstParamApplier::aLostLinkMidApplyIsReportedUnknownAndUnlocks() {
  Rig r;
  r.up();
  APPLIER(r, ap, done);
  r.fc.holdAcks = true;
  QVERIFY(ap.apply({{"G1_X", 4}, {"G1_Y", 5}}));
  r.run(100);
  r.svc.linkDown();
  QTest::qWait(20);
  QCOMPARE(done.r.size(), 1);
  QVERIFY(!done.r[0].ok);
  QVERIFY(done.r[0].unknown);
  QVERIFY(!r.svc.locked());
}

QTEST_MAIN(TstParamApplier)
#include "tst_param_applier.moc"
