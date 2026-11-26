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
#include "ParamFileDialog.h"
#include "ParamsPanel.h"
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QTreeWidget>

#include "FakeParamFc.h"
#include "NavlinkRouter.h"
#include "ParamService.h"

#include <QDialogButtonBox>
#include <QLabel>
#include <QTableWidget>
#include <QTemporaryDir>
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
  void smallValuesAreShownInPlainNotationAndTypedBackInAnyLocale();
  void saveAllAppearsForUnsavedLiveValues();
  void aGroupIsCommittedAndDiscardIsGreyedWithAReason();
  void aLockedGroupIsNotEditableWhileArmed();
  void aSavedFileHoldsTheGroupedValuesAndIsRefusedWhileStaged();
  void aLoadedFileShowsTheDiffAndAppliesOnlyTheTickedChangedRows();
  void aFileWithProblemsOrForAnotherTableShowsButCannotBeApplied();
  void armedOrNotReadyRefusesTheLoadAndTheSave();
  void aValueThatMovesWhileTheDiffIsOpenTurnsApplyOff();
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

void TstParamsPanel::smallValuesAreShownInPlainNotationAndTypedBackInAnyLocale() {
  const QLocale keep = QLocale();
  QLocale::setDefault(QLocale(QLocale::German, QLocale::Germany)); // comma decimals
  {
    Rig r;
    ParamsPanel p(&r.svc);
    r.up();
    r.fc.holdAcks = true;
    const int idx = r.svc.catalog().byId(QStringLiteral("A_LIVE"))->index;
    p.submitEdit(idx, QStringLiteral("0.00005"));
    QTest::qWait(10);
    QTreeWidgetItem *it = rowOf(p, "A_LIVE");
    QCOMPARE(it->text(1), QStringLiteral("0.00005")); // not 5e-05, no comma
    // The editor takes what the cell shows, and "0.003", in this locale.
    auto *d = p.findChild<ParamValueDelegate *>();
    QVERIFY(d);
    auto *tree = p.findChild<QTreeWidget *>(QStringLiteral("paramsTree"));
    QStyleOptionViewItem opt;
    QWidget *w = d->createEditor(&p, opt, tree->indexFromItem(it, 1));
    auto *ed = qobject_cast<QLineEdit *>(w);
    QVERIFY(ed);
    for (const char *t : {"0.00005", "0.003", "10"}) {
      ed->setText(QString::fromLatin1(t));
      QVERIFY2(ed->hasAcceptableInput(), t);
    }
    ed->setText(QStringLiteral("10.5"));
    QVERIFY(!ed->hasAcceptableInput());
    delete w;
    r.fc.holdAcks = false;
  }
  QLocale::setDefault(keep);
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

namespace {
const char *kFileHead = R"({"format":1,"table_crc":%1,"params":[%2]})";
QString writeFileText(const QTemporaryDir &d, const char *name, const QString &crc, const QString &rows) {
  const QString path = d.filePath(QString::fromLatin1(name));
  QFile f(path);
  if (!f.open(QIODevice::WriteOnly))
    qFatal("temp file");
  f.write(QString::fromLatin1(kFileHead).arg(crc, rows).toUtf8());
  return path;
}
const QString kRows = QStringLiteral(
    R"({"id":"G1_X","value":4},{"id":"G1_Y","value":2},{"id":"L2_Z","value":3})");
ParamFileDialog *dialogOf(ParamsPanel &p) { return p.findChild<ParamFileDialog *>(); }
QPushButton *applyOf(ParamFileDialog *d) {
  return d->findChild<QPushButton *>(QStringLiteral("paramsFileApply"));
}
QLabel *resultOf(ParamsPanel &p) { return p.findChild<QLabel *>(QStringLiteral("paramsFileResult")); }
} // namespace

void TstParamsPanel::aSavedFileHoldsTheGroupedValuesAndIsRefusedWhileStaged() {
  Rig r;
  QTemporaryDir dir;
  ParamsPanel p(&r.svc);
  r.up();
  const QString path = dir.filePath(QStringLiteral("a.json"));
  QVERIFY(p.saveFile(path));
  QFile f(path);
  QVERIFY(f.open(QIODevice::ReadOnly));
  const QByteArray text = f.readAll();
  f.close();
  QVERIFY(text.contains("G1_X"));
  QVERIFY(text.contains("L2_Z"));
  QVERIFY(!text.contains("A_LIVE")); // live values are not a tune
  QVERIFY(resultOf(p)->isVisibleTo(&p));
  QVERIFY(resultOf(p)->text().contains(QLatin1String("a.json")));
  // A staged value is not what the vehicle runs: no file, and the reason is said.
  p.submitEdit(r.svc.catalog().byId(QStringLiteral("G1_X"))->index, QStringLiteral("4"));
  r.run(300);
  QTest::qWait(10);
  const QString path2 = dir.filePath(QStringLiteral("b.json"));
  QVERIFY(!p.saveFile(path2));
  QVERIFY(!QFile::exists(path2));
  QVERIFY(resultOf(p)->text().contains(QLatin1String("staged")));
}

void TstParamsPanel::aLoadedFileShowsTheDiffAndAppliesOnlyTheTickedChangedRows() {
  Rig r;
  QTemporaryDir dir;
  ParamsPanel p(&r.svc);
  r.up();
  p.loadFile(writeFileText(dir, "t.json", QStringLiteral("3735928559"), kRows));
  ParamFileDialog *d = dialogOf(p);
  QVERIFY(d);
  auto *table = d->findChild<QTableWidget *>(QStringLiteral("paramsFileTable"));
  QCOMPARE(table->rowCount(), 3);
  QVERIFY(d->applyEnabled());
  // Only the row that differs is ticked; the two that are the same are left out.
  const auto items = d->ticked();
  QCOMPARE(items.size(), 1);
  QCOMPARE(items[0].id, QStringLiteral("G1_X"));
  QCOMPARE(items[0].value, 4.0);
  QCOMPARE(r.fc.sets, 0); // opening the dialog sent nothing
  // Unticking the last row leaves nothing to apply.
  table->item(0, 0)->setCheckState(Qt::Unchecked);
  QVERIFY(!d->applyEnabled());
  table->item(0, 0)->setCheckState(Qt::Checked);
  QVERIFY(applyOf(d)->isEnabled());
  applyOf(d)->click();
  r.run(2000);
  QTest::qWait(10);
  QCOMPARE(r.fc.sets, 1);
  QCOMPARE(r.fc.commits, 1);
  QCOMPARE(r.svc.client().value(QStringLiteral("G1_X"))->v, 4.0);
  QVERIFY(resultOf(p)->text().contains(QLatin1String("loaded")));
}

void TstParamsPanel::aFileWithProblemsOrForAnotherTableShowsButCannotBeApplied() {
  Rig r;
  QTemporaryDir dir;
  ParamsPanel p(&r.svc);
  r.up();
  // An id the table does not have: listed, and nothing of the file may be applied.
  p.loadFile(writeFileText(dir, "u.json", QStringLiteral("3735928559"),
                           kRows + QStringLiteral(R"(,{"id":"NOPE","value":1})")));
  ParamFileDialog *d = dialogOf(p);
  QVERIFY(d);
  QVERIFY(d->findChild<QLabel *>(QStringLiteral("paramsFileProblems"))->text().contains(QLatin1String("NOPE")));
  QVERIFY(!applyOf(d)->isEnabled());
  applyOf(d)->click();
  r.run(500);
  QCOMPARE(r.fc.sets, 0);
  d->close();
  QTest::qWait(10);
  // Made for another table: the diff is there, the apply is not.
  p.loadFile(writeFileText(dir, "o.json", QStringLiteral("1"), kRows));
  d = dialogOf(p);
  QVERIFY(d);
  QCOMPARE(d->findChild<QTableWidget *>(QStringLiteral("paramsFileTable"))->rowCount(), 3);
  QVERIFY(d->findChild<QLabel *>(QStringLiteral("paramsFileRefusal")));
  QVERIFY(!applyOf(d)->isEnabled());
  applyOf(d)->click();
  r.run(500);
  QCOMPARE(r.fc.sets, 0);
}

void TstParamsPanel::armedOrNotReadyRefusesTheLoadAndTheSave() {
  Rig r;
  QTemporaryDir dir;
  ParamsPanel p(&r.svc);
  auto *saveBtn = p.findChild<QPushButton *>(QStringLiteral("paramsSaveFile"));
  auto *loadBtn = p.findChild<QPushButton *>(QStringLiteral("paramsLoadFile"));
  QVERIFY(saveBtn && loadBtn);
  QVERIFY(!saveBtn->isEnabled() && !loadBtn->isEnabled());
  QVERIFY(!p.saveFile(dir.filePath(QStringLiteral("n.json"))));
  QVERIFY(!QFile::exists(dir.filePath(QStringLiteral("n.json"))));
  r.up(2);
  QTest::qWait(10);
  QVERIFY(saveBtn->isEnabled() && loadBtn->isEnabled());
  const QString path = writeFileText(dir, "t.json", QStringLiteral("3735928559"), kRows);
  // Armed when the file is opened: shown, not applicable.
  r.heartbeat(4, 8);
  r.run(100);
  p.loadFile(path);
  ParamFileDialog *d = dialogOf(p);
  QVERIFY(d);
  QVERIFY(!applyOf(d)->isEnabled());
  QVERIFY(d->findChild<QLabel *>(QStringLiteral("paramsFileRefusal"))->text().contains(QLatin1String("armed")));
  d->close();
  QTest::qWait(10);
  // Disarmed when opened, armed by the time Apply is pressed: asked again, refused.
  r.heartbeat(2, 8);
  r.run(100);
  p.loadFile(path);
  d = dialogOf(p);
  QVERIFY(d && applyOf(d)->isEnabled());
  r.heartbeat(4, 8);
  r.run(100);
  applyOf(d)->click();
  r.run(500);
  QCOMPARE(r.fc.sets, 0);
  QVERIFY(resultOf(p)->text().contains(QLatin1String("armed")));
}

void TstParamsPanel::aValueThatMovesWhileTheDiffIsOpenTurnsApplyOff() {
  Rig r;
  QTemporaryDir dir;
  ParamsPanel p(&r.svc);
  r.up();
  p.loadFile(writeFileText(dir, "t.json", QStringLiteral("3735928559"), kRows));
  ParamFileDialog *d = dialogOf(p);
  QVERIFY(d && d->applyEnabled() && !d->stale());
  // Another edit (the panel's own, the vehicle's) lands on a row of the file.
  p.submitEdit(r.svc.catalog().byId(QStringLiteral("G1_Y"))->index, QStringLiteral("6"));
  r.run(300);
  QTest::qWait(10);
  QVERIFY(d->stale());
  QVERIFY(!d->applyEnabled());
  QVERIFY(d->findChild<QLabel *>(QStringLiteral("paramsFileStale"))->isVisibleTo(d));
  applyOf(d)->click();
  r.run(500);
  QCOMPARE(r.fc.commits, 0);
  // A value that is not in the file does not matter.
  d->close();
  QTest::qWait(10);
  p.loadFile(writeFileText(dir, "u.json", QStringLiteral("3735928559"),
                           QStringLiteral(R"({"id":"G1_X","value":4})")));
  d = dialogOf(p);
  QVERIFY(d && d->applyEnabled());
  p.submitEdit(r.svc.catalog().byId(QStringLiteral("L2_Z"))->index, QStringLiteral("7"));
  r.run(300);
  QTest::qWait(10);
  QVERIFY(!d->stale());
}

QTEST_MAIN(TstParamsPanel)
#include "tst_params_panel.moc"
