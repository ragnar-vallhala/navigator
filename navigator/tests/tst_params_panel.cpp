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
#include "ParamsPanel.h"
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QTreeWidget>

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

class TstParamsPanel : public QObject {
  Q_OBJECT
private slots:
  void nothingIsEditableUntilReady();
  void aValidEditIsSentAndTheVehiclesValueShownBesideIt();
  void badEntriesNeverLeaveTheEditor();
  void saveAllAppearsForUnsavedLiveValues();
  void aGroupIsCommittedAndDiscardIsGreyedWithAReason();
  void aLockedGroupIsNotEditableWhileArmed();
};

namespace {
QTreeWidgetItem *rowOf(params::ParamsPanel &p, const char *id) {
  auto *tree = p.findChild<QTreeWidget *>(QStringLiteral("paramsTree"));
  const QList<QTreeWidgetItem *> hits =
      tree->findItems(QString::fromLatin1(id), Qt::MatchExactly | Qt::MatchRecursive, 0);
  return hits.isEmpty() ? nullptr : hits.first();
}
} // namespace

void TstParamsPanel::nothingIsEditableUntilReady() {
  Rig r;
  ParamsPanel p(&r.svc);
  QTreeWidgetItem *it = rowOf(p, "A_LIVE");
  QVERIFY(it);
  QVERIFY(!(it->flags() & Qt::ItemIsEditable));
  r.up();
  QTest::qWait(10);
  QVERIFY(it->flags() & Qt::ItemIsEditable);
  QCOMPARE(it->text(1), QStringLiteral("1"));
}

void TstParamsPanel::aValidEditIsSentAndTheVehiclesValueShownBesideIt() {
  Rig r;
  ParamsPanel p(&r.svc);
  r.up();
  r.fc.holdAcks = true;
  p.submitEdit(r.svc.catalog().byId(QStringLiteral("A_LIVE"))->index, QStringLiteral("2.5"));
  QTest::qWait(10);
  QTreeWidgetItem *it = rowOf(p, "A_LIVE");
  QCOMPARE(it->text(1), QStringLiteral("2.5")); // what was typed
  QVERIFY(!it->text(2).isEmpty());              // the vehicle's own value, beside it
  r.fc.holdAcks = false;
  r.run(1500);
  QTest::qWait(10);
  QCOMPARE(it->text(2), QString());
}

void TstParamsPanel::badEntriesNeverLeaveTheEditor() {
  Rig r;
  ParamsPanel p(&r.svc);
  r.up();
  const int before = r.fc.sets;
  const int idx = r.svc.catalog().byId(QStringLiteral("A_LIVE"))->index;
  for (const char *t : {"abc", "nan", "inf", "11", "-1", ""})
    p.submitEdit(idx, QString::fromLatin1(t));
  r.run(200);
  QCOMPARE(r.fc.sets, before);
}

void TstParamsPanel::saveAllAppearsForUnsavedLiveValues() {
  Rig r;
  ParamsPanel p(&r.svc);
  r.up();
  auto *save = p.findChild<QPushButton *>(QStringLiteral("paramsSaveAll"));
  QVERIFY(save);
  QVERIFY(!save->isVisibleTo(&p));
  p.submitEdit(r.svc.catalog().byId(QStringLiteral("A_LIVE"))->index, QStringLiteral("2"));
  r.run(300);
  QTest::qWait(10);
  QVERIFY(save->isVisibleTo(&p));
  QVERIFY(save->isEnabled());
  save->click();
  r.run(1500);
  QCOMPARE(r.fc.commits, 1); // CMD_PARAM_COMMIT group 0 went out
  r.fc.landSave();           // the vehicle's storage finishes: UNSAVED clears
  r.run(300);
  QTest::qWait(10);
  QVERIFY(!save->isVisibleTo(&p));
}

void TstParamsPanel::aGroupIsCommittedAndDiscardIsGreyedWithAReason() {
  Rig r;
  ParamsPanel p(&r.svc);
  r.up();
  auto *commit = p.findChild<QPushButton *>(QStringLiteral("paramsCommit1"));
  auto *discard = p.findChild<QPushButton *>(QStringLiteral("paramsDiscard1"));
  QVERIFY(commit && discard);
  QVERIFY(!commit->isEnabled());
  QVERIFY(commit->toolTip().contains(QLatin1String("nothing")));
  p.submitEdit(r.svc.catalog().byId(QStringLiteral("G1_X"))->index, QStringLiteral("4"));
  r.run(300);
  QTest::qWait(10);
  QVERIFY(commit->isEnabled());
  QVERIFY(discard->isEnabled());
  commit->click();
  r.run(500);
  QTest::qWait(10);
  QVERIFY(!commit->isEnabled());
  auto *list = p.findChild<QListWidget *>(QStringLiteral("paramsOutcomes"));
  QVERIFY(list->count() >= 2);
}

void TstParamsPanel::aLockedGroupIsNotEditableWhileArmed() {
  Rig r;
  ParamsPanel p(&r.svc);
  r.up(2);
  QTreeWidgetItem *it = rowOf(p, "L2_Z");
  QVERIFY(it->flags() & Qt::ItemIsEditable);
  r.heartbeat(4, 8);
  r.run(100);
  QTest::qWait(10);
  QVERIFY(!(it->flags() & Qt::ItemIsEditable));
  QVERIFY(it->text(5).contains(QLatin1String("armed")));
}

QTEST_MAIN(TstParamsPanel)
#include "tst_params_panel.moc"
