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
#include "GyroNotchWidget.h"

#include "CommandCodec.h"
#include "FakeParamFc.h"
#include "NavlinkRouter.h"
#include "ParamService.h"

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QSignalSpy>
#include <QThread>

extern "C" {
#include "navlink_msgs.h"
}

using namespace params;

namespace {

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
  QThread th;
  Wire wire;
  ParamService svc;
  FakeParamFc fc;
  qint64 clk = 1000;
  quint8 seq = 0;
  QVector<Client::WriteEvent> ev;

  Rig() {
    if (!svc.loadCatalog()) // the catalog the app is built with
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


quint32 msgidOf(const QByteArray &f) {
  return quint8(f[7]) | (quint32(quint8(f[8])) << 8) | (quint32(quint8(f[9])) << 16);
}

// The notch panel with its parts found by what they are.
struct Panel {
  GyroNotchWidget w;
  QCheckBox *enable = nullptr, *autoband = nullptr;
  QDoubleSpinBox *q = nullptr, *fmin = nullptr, *fmax = nullptr, *ratio = nullptr;
  QPushButton *apply = nullptr;
  Panel() {
    const auto checks = w.findChildren<QCheckBox *>();
    enable = checks.at(0);
    autoband = checks.at(1);
    const auto spins = w.findChildren<QDoubleSpinBox *>();
    q = spins.at(0);
    fmin = spins.at(1);
    fmax = spins.at(2);
    ratio = spins.at(3);
    for (QPushButton *b : w.findChildren<QPushButton *>())
      if (b->text().startsWith(QLatin1String("Apply")))
        apply = b;
    w.setConnected(true);
  }
};

int g_abSeq = -1;
navlink_ack_t onAutoband(void *, const navlink_frame_hdr_t *, const navlink_cmd_notch_autoband_t *m) {
  g_abSeq = m->req_seq;
  return navlink_ack_t{};
}
// The req_seq a CMD_NOTCH_AUTOBAND frame carries; -1 when it does not decode.
int autobandSeqOf(const QByteArray &f) {
  g_abSeq = -1;
  navlink_handlers_t h{};
  h.on_cmd_notch_autoband = onAutoband;
  navlink_parser_t p;
  navlink_parser_init(&p);
  navlink_parser_push(&p, &h, reinterpret_cast<const uint8_t *>(f.constData()), size_t(f.size()));
  return g_abSeq;
}

} // namespace

class TstGyroNotchWidget : public QObject {
  Q_OBJECT
private slots:
  void boxesShowTheVehiclesValuesWhenReady();
  void aUsersEditIsNotOverwritten();
  void applyWritesTheNotchGroupAsOneCommit();
  void theBoxesAreLockedWhileItRuns();
  void autobandIsArmedOnlyAfterTheSettingsAreIn();
  void autobandSaysArmedOnlyWhenTheVehicleAcksIt();
  void aFailedApplyDoesNotArmAutoband();
  void aBandThatIsNotABandIsRefused();
  void olderFirmwareKeepsTheOldCommand();
  void aTableThatIsNotLoadedIsRefused();
};

void TstGyroNotchWidget::boxesShowTheVehiclesValuesWhenReady() {
  Rig r;
  r.fc.setLive("NOTCH_Q", 12.0);
  r.fc.setLive("NOTCH_FMAX", 300.0);
  r.fc.setLive("NOTCH_EN", 1.0);
  Panel p;
  p.w.setParams(&r.svc);
  QCOMPARE(p.q->value(), 8.0); // the built-in defaults before the table is read
  r.up();
  QCOMPARE(p.q->value(), 12.0);
  QCOMPARE(p.fmax->value(), 300.0);
  QVERIFY(p.enable->isChecked());
  QCOMPARE(p.fmin->minimum(), 20.0); // the catalog's range, not the old one
  QCOMPARE(p.fmin->maximum(), 500.0);
  QCOMPARE(p.q->maximum(), 20.0);
}

void TstGyroNotchWidget::aUsersEditIsNotOverwritten() {
  Rig r;
  Panel p;
  p.w.setParams(&r.svc);
  r.up();
  p.q->setValue(3.0); // the user
  r.fc.setLive("NOTCH_Q", 9.0);
  r.svc.refresh();
  r.run(1500);
  QCOMPARE(p.q->value(), 3.0);
}

void TstGyroNotchWidget::applyWritesTheNotchGroupAsOneCommit() {
  Rig r;
  Panel p;
  QSignalSpy cmd(&p.w, &GyroNotchWidget::commandRequested);
  p.w.setParams(&r.svc);
  r.up();
  p.enable->setChecked(true);
  p.q->setValue(10.0);
  p.fmin->setValue(80.0);
  p.fmax->setValue(400.0);
  p.ratio->setValue(5.0);
  r.fc.sets = 0;
  p.apply->click();
  r.run(2500);
  QCOMPARE(r.fc.sets, 5);
  QCOMPARE(r.fc.commits, 1);
  QCOMPARE(r.fc.live("NOTCH_EN"), 1.0);
  QCOMPARE(r.fc.live("NOTCH_Q"), 10.0);
  QCOMPARE(r.fc.live("NOTCH_FMIN"), 80.0);
  QCOMPARE(r.fc.live("NOTCH_FMAX"), 400.0);
  QCOMPARE(r.fc.live("NOTCH_MINRATIO"), 5.0);
  QCOMPARE(cmd.count(), 0); // no legacy command, no autoband asked for
  QVERIFY(p.apply->isEnabled());
}

void TstGyroNotchWidget::theBoxesAreLockedWhileItRuns() {
  Rig r;
  Panel p;
  p.w.setParams(&r.svc);
  r.up();
  r.fc.holdAcks = true;
  p.q->setValue(10.0);
  p.apply->click();
  r.run(300);
  QVERIFY(r.svc.locked());
  for (QWidget *w : {static_cast<QWidget *>(p.q), static_cast<QWidget *>(p.fmin),
                     static_cast<QWidget *>(p.fmax), static_cast<QWidget *>(p.ratio),
                     static_cast<QWidget *>(p.enable), static_cast<QWidget *>(p.autoband),
                     static_cast<QWidget *>(p.apply)})
    QVERIFY(!w->isEnabled());
  r.fc.holdAcks = false;
  r.fc.flush();
  r.run(2500);
  QVERIFY(!r.svc.locked());
  QVERIFY(p.q->isEnabled());
  QVERIFY(p.apply->isEnabled());
}

void TstGyroNotchWidget::autobandIsArmedOnlyAfterTheSettingsAreIn() {
  Rig r;
  Panel p;
  QSignalSpy cmd(&p.w, &GyroNotchWidget::commandRequested);
  p.w.setParams(&r.svc);
  r.up();
  p.autoband->setChecked(true);
  r.fc.holdAcks = true;
  p.apply->click();
  r.run(300);
  QCOMPARE(cmd.count(), 0); // the settings are not in yet
  r.fc.holdAcks = false;
  r.fc.flush();
  r.run(2500);
  QCOMPARE(cmd.count(), 1);
  QCOMPARE(msgidOf(cmd.at(0).at(0).toByteArray()), quint32(NAVLINK_MSGID_CMD_NOTCH_AUTOBAND));
  QVERIFY(!p.autoband->isChecked()); // a one-shot
}

void TstGyroNotchWidget::autobandSaysArmedOnlyWhenTheVehicleAcksIt() {
  const auto statusOf = [](Panel &p) {
    for (QLabel *l : p.w.findChildren<QLabel *>())
      if (l->text().contains(QLatin1String("uto-band")))
        return l->text();
    return QString();
  };
  // Sends the settings and the command; returns the req_seq the command carried.
  const auto arm = [&](Rig &r, Panel &p, QSignalSpy &cmd) -> int {
    p.w.setParams(&r.svc);
    r.up();
    p.autoband->setChecked(true);
    p.apply->click();
    r.run(2500);
    if (cmd.count() != 1)
      return -1;
    return autobandSeqOf(cmd.at(0).at(0).toByteArray());
  };
  { // accepted: armed
    Rig r;
    Panel p;
    QSignalSpy cmd(&p.w, &GyroNotchWidget::commandRequested);
    const int armed = arm(r, p, cmd);
    QVERIFY(armed >= 0);
    const quint8 seq = quint8(armed);
    QVERIFY(!statusOf(p).contains(QLatin1String("armed")) || statusOf(p).contains(QLatin1String("Arming")));
    p.w.onCommandAck(NAVLINK_MSGID_CMD_NOTCH_AUTOBAND, quint8(seq + 1), 0); // another request's ack
    QVERIFY(statusOf(p).contains(QLatin1String("Arming")));
    p.w.onCommandAck(NAVLINK_MSGID_CMD_PARAM_COMMIT, seq, 0); // another command's
    QVERIFY(statusOf(p).contains(QLatin1String("Arming")));
    p.w.onCommandAck(NAVLINK_MSGID_CMD_NOTCH_AUTOBAND, seq, 5); // in progress: keep waiting
    QVERIFY(statusOf(p).contains(QLatin1String("Arming")));
    p.w.onCommandAck(NAVLINK_MSGID_CMD_NOTCH_AUTOBAND, seq, 0);
    QCOMPARE(statusOf(p), QStringLiteral("Auto-band armed"));
  }
  { // refused: not armed
    Rig r;
    Panel p;
    QSignalSpy cmd(&p.w, &GyroNotchWidget::commandRequested);
    const int armed = arm(r, p, cmd);
    QVERIFY(armed >= 0);
    const quint8 seq = quint8(armed);
    p.w.onCommandAck(NAVLINK_MSGID_CMD_NOTCH_AUTOBAND, seq, 2);
    QCOMPARE(statusOf(p), QStringLiteral("Auto-band not armed"));
  }
  { // no answer: said so, not "armed"
    Rig r;
    Panel p;
    QSignalSpy cmd(&p.w, &GyroNotchWidget::commandRequested);
    QVERIFY(arm(r, p, cmd) >= 0);
    QTest::qWait(GyroNotchWidget::kAutobandAckMs + 300);
    QCOMPARE(statusOf(p), QStringLiteral("Auto-band: no answer"));
  }
}

void TstGyroNotchWidget::aFailedApplyDoesNotArmAutoband() {
  Rig r;
  Panel p;
  QSignalSpy cmd(&p.w, &GyroNotchWidget::commandRequested);
  p.w.setParams(&r.svc);
  r.up();
  p.autoband->setChecked(true);
  r.fc.saveFails = true;
  p.apply->click();
  r.run(4000);
  QCOMPARE(cmd.count(), 0);
  QVERIFY(p.autoband->isChecked()); // still asked for
  QVERIFY(p.apply->isEnabled());
  QVERIFY(!r.svc.locked());
}

void TstGyroNotchWidget::aBandThatIsNotABandIsRefused() {
  Rig r;
  Panel p;
  p.w.setParams(&r.svc);
  r.up();
  r.fc.sets = 0;
  p.fmin->setValue(300.0);
  p.fmax->setValue(300.0);
  p.apply->click();
  r.run(500);
  QCOMPARE(r.fc.sets, 0);
  QCOMPARE(r.fc.commits, 0);
  QVERIFY(p.apply->isEnabled());
}

void TstGyroNotchWidget::olderFirmwareKeepsTheOldCommand() {
  Rig r;
  Panel p;
  QSignalSpy cmd(&p.w, &GyroNotchWidget::commandRequested);
  p.w.setParams(&r.svc);
  r.heartbeat(2, 0); // no PARAMS capability
  r.run(300);
  p.autoband->setChecked(true);
  p.apply->click();
  r.run(300);
  QCOMPARE(cmd.count(), 1);
  QCOMPARE(msgidOf(cmd.at(0).at(0).toByteArray()), quint32(NAVLINK_MSGID_CMD_SET_GYRO_NOTCH));
  QCOMPARE(r.fc.sets, 0);
  QVERIFY(!p.autoband->isChecked());
}

void TstGyroNotchWidget::aTableThatIsNotLoadedIsRefused() {
  Rig r;
  Panel p;
  QSignalSpy cmd(&p.w, &GyroNotchWidget::commandRequested);
  p.w.setParams(&r.svc);
  r.heartbeat(2, 8); // PARAMS offered, but the table is not read (no sync edge)
  r.run(300);
  p.apply->click();
  r.run(300);
  QCOMPARE(cmd.count(), 0); // the old command would skip the staging and checks
  QCOMPARE(r.fc.sets, 0);
}

QTEST_MAIN(TstGyroNotchWidget)
#include "tst_gyro_notch_widget.moc"
