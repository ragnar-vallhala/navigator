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

class TstParamService : public QObject {
  Q_OBJECT
private slots:
  void builtInCatalogLoads();
  void aCatalogThatFailsLeavesNothingRunning();
  void readsOnlyAfterTheHeartbeatAndTheSyncEdge();
  void aWriteIsConfirmedWhicheverWayAckAndEchoArrive();
  void armedFollowsNavStateAndUnknownCountsAsArmed();
  void theSecureLinkCapabilityIsNotPassedOn();
  void aSilentVehicleThatReturnsIsReadFromScratch();
  void linkDownStopsEverything();
};

void TstParamService::builtInCatalogLoads() {
  ParamService s;
  QVERIFY(s.loadCatalog()); // the resource the app is built with
  QVERIFY(s.catalog().count() >= 39);
  QVERIFY(s.catalog().byId(QStringLiteral("SYS_PTBL_CRC")));
  QVERIFY(!s.ticking());
}

void TstParamService::aCatalogThatFailsLeavesNothingRunning() {
  ParamService s;
  QString err;
  QVERIFY(!s.loadCatalog(QStringLiteral("/nonexistent/params_catalog.json"), &err));
  QVERIFY(!err.isEmpty());
  s.onSynced(true);
  s.onHeartbeat(2, 8);
  QCOMPARE(s.client().state(), Client::State::NoCaps);
  QVERIFY(!s.ticking());
}

void TstParamService::readsOnlyAfterTheHeartbeatAndTheSyncEdge() {
  Rig r;
  r.run(500);
  QCOMPARE(r.fc.readIds.size(), 0); // nothing from a service that has heard nothing
  QVERIFY(!r.svc.ticking());
  r.svc.onSynced(true); // synced, but no heartbeat yet
  r.run(500);
  QCOMPARE(r.fc.readIds.size(), 0);
  r.heartbeat(2, 8); // the vehicle says PARAMS
  r.run(400);
  QVERIFY(r.svc.ticking());
  QCOMPARE(r.svc.client().state(), Client::State::Ready);
  QCOMPARE(r.fc.listRequests, 1);
  QVERIFY(r.svc.client().value(QStringLiteral("A_LIVE")));
  // A heartbeat without PARAMS: nothing is read.
  Rig q;
  q.svc.onSynced(true);
  q.heartbeat(2, 1 | 2);
  q.run(300);
  QCOMPARE(q.svc.client().state(), Client::State::NoCaps);
  QCOMPARE(q.fc.readIds.size(), 0);
}

void TstParamService::aWriteIsConfirmedWhicheverWayAckAndEchoArrive() {
  for (bool echoFirst : {false, true}) {
    Rig r;
    r.up();
    r.fc.echoBeforeAck = echoFirst;
    QString why;
    QVERIFY2(r.svc.set(QStringLiteral("A_LIVE"), 2.5, &why), qPrintable(why));
    r.run(200);
    QCOMPARE(r.ev.size(), 1);
    QCOMPARE(r.ev.last().outcome, Client::Outcome::Applied);
    QCOMPARE(float(r.svc.client().value(QStringLiteral("A_LIVE"))->v), 2.5f);
    QCOMPARE(r.svc.client().pendingCount(), 0);
  }
}

void TstParamService::armedFollowsNavStateAndUnknownCountsAsArmed() {
  Rig r;
  r.up(2); // STANDBY
  QString why;
  QVERIFY2(r.svc.set(QStringLiteral("L2_Z"), 4, &why), qPrintable(why)); // locked, disarmed: ok
  r.run(200);
  QCOMPARE(r.ev.last().outcome, Client::Outcome::Applied);
  for (quint8 armed : {quint8(4), quint8(5), quint8(6), quint8(99)}) { // ARMED, IN_AIR, FAILSAFE, unknown
    r.heartbeat(armed, 8);
    r.run(100);
    why.clear();
    QVERIFY2(!r.svc.set(QStringLiteral("L2_Z"), 5, &why), qPrintable(QString::number(armed)));
    QVERIFY(why.contains(QLatin1String("armed")));
  }
  r.heartbeat(8, 8); // CALIBRATING is not armed
  r.run(100);
  QVERIFY(r.svc.set(QStringLiteral("L2_Z"), 5));
  r.run(200);
  // An unlocked parameter is writable while armed.
  r.heartbeat(4, 8);
  r.run(100);
  QVERIFY(r.svc.set(QStringLiteral("A_LIVE"), 2));
}

void TstParamService::theSecureLinkCapabilityIsNotPassedOn() {
  Rig r;
  r.fc.strict = true; // the vehicle refuses an unsecured link
  r.up(2, 8 | 4);     // and says it has the secure-link capability
  QVERIFY(r.svc.set(QStringLiteral("A_LIVE"), 2));
  r.run(200);
  QCOMPARE(r.ev.last().outcome, Client::Outcome::Refused);
  QVERIFY2(!r.ev.last().text.contains(QLatin1String("secure link")), qPrintable(r.ev.last().text));
}

void TstParamService::aSilentVehicleThatReturnsIsReadFromScratch() {
  Rig r;
  r.up();
  QCOMPARE(r.svc.client().state(), Client::State::Ready);
  QCOMPARE(r.fc.listRequests, 1);
  r.fc.holdAcks = true;
  QVERIFY(r.svc.set(QStringLiteral("A_LIVE"), 3)); // in flight when it goes silent
  // No heartbeat for longer than the gap, then it is back.
  r.clk += ParamService::kHbGapMs + 500;
  r.heartbeat(2, 8);
  r.run(100);
  QCOMPARE(r.ev.size(), 1);
  QCOMPARE(r.ev.last().outcome, Client::Outcome::Unknown); // it may have been applied
  r.fc.holdAcks = false;
  r.run(300);
  QCOMPARE(r.svc.client().state(), Client::State::Ready);
  QCOMPARE(r.fc.listRequests, 2); // the table was read again
  // A short pause is not a reboot.
  r.clk += ParamService::kHbGapMs - 500;
  r.heartbeat(2, 8);
  r.run(300);
  QCOMPARE(r.fc.listRequests, 2);
}

void TstParamService::linkDownStopsEverything() {
  Rig r;
  r.up();
  QVERIFY(r.svc.ticking());
  r.svc.linkDown();
  QCOMPARE(r.svc.client().state(), Client::State::NoCaps);
  QVERIFY(!r.svc.ticking());
  QString why;
  QVERIFY(!r.svc.set(QStringLiteral("A_LIVE"), 2, &why));
  const int sets = r.fc.sets;
  r.run(2000);
  QCOMPARE(r.fc.sets, sets);
  // The next link starts clean, and an unknown armed state counts as armed.
  r.up(2);
  QCOMPARE(r.svc.client().state(), Client::State::Ready);
}

QTEST_MAIN(TstParamService)
#include "tst_param_service.moc"
