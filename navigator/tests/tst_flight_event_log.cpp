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
// Flight event log: line format, header, RC throttle, tolerant reader, run
// listing and retention. Temp dirs only.

#include "FlightEventLog.h"

#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTemporaryDir>
#include <QtTest>

using vsim::FlightEventLog;

namespace {
QList<QByteArray> rawLines(const QString &path) {
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly))
    return {};
  QList<QByteArray> l = f.readAll().split('\n');
  if (!l.isEmpty() && l.last().isEmpty())
    l.removeLast();
  return l;
}

void touch(const QString &path, const QByteArray &data = "x") {
  QFile f(path);
  QVERIFY2(f.open(QIODevice::WriteOnly), qPrintable(path));
  f.write(data);
}

FlightEventLog::Header header() {
  FlightEventLog::Header h;
  h.runId = "2026-11-15_10-00-00";
  h.appVersion = "9.9.9";
  h.sdkVersion = 3;
  h.vehicleSha256 = FlightEventLog::sha256Hex("geom");
  h.worldPreset = "hills";
  h.worldSeed = 1337;
  h.gtFile = "gt-2026-11-15_10-00-00.bin";
  h.t0UnixMs = 1763200800000;
  return h;
}
} // namespace

class TstFlightEventLog : public QObject {
  Q_OBJECT
private slots:
  void everyKindRoundTrips() {
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString p = dir.filePath("ev-2026-11-15_10-00-00.jsonl");
    FlightEventLog log;
    QVERIFY(log.open(p, header()));
    QVERIFY(log.start(0));
    QVERIFY(log.rc(1000, {1500, 1500, 1000, 1500, 1000, 1000}, "gamepad"));
    QVERIFY(log.arm(2000, true));
    QVERIFY(log.mode(3000, "STABILISE", "ACRO", "rc"));
    QVERIFY(log.fault(4000, "motor1", true, "motor 1 killed"));
    QVERIFY(log.fault(5000, "motor1", false));
    QVERIFY(log.marker(6000, "gust"));
    QVERIFY(log.stop(7000, "user"));
    log.close();
    QVERIFY(!log.start(8000)); // closed: nothing written

    const QList<QByteArray> lines = rawLines(p);
    QCOMPARE(lines.size(), 9);
    QList<QJsonObject> objs;
    for (const QByteArray &l : lines) {
      QJsonParseError e;
      const QJsonDocument d = QJsonDocument::fromJson(l, &e);
      QVERIFY2(e.error == QJsonParseError::NoError, l.constData());
      QVERIFY(d.isObject());
      objs.append(d.object());
    }
    const QJsonObject h = objs[0];
    QCOMPARE(h["schema"].toString(), QString("nav-flight-events/1"));
    for (const char *k :
         {"run_id", "app_version", "sdk_version", "vehicle_sha256",
          "world_preset", "world_seed", "gt_file", "t0_unix_ms",
          "module_build_id"})
      QVERIFY2(h.contains(QLatin1String(k)), k);
    QCOMPARE(h["run_id"].toString(), QString("2026-11-15_10-00-00"));
    QCOMPARE(h["sdk_version"].toInt(), 3);
    QCOMPARE(h["vehicle_sha256"].toString().size(), 64);
    QCOMPARE(h["world_seed"].toInt(), 1337);
    QCOMPARE(h["gt_file"].toString(), QString("gt-2026-11-15_10-00-00.bin"));
    QCOMPARE(h["t0_unix_ms"].toVariant().toLongLong(), 1763200800000LL);

    const QStringList kinds = {"start", "rc",    "arm",    "mode",
                               "fault", "fault", "marker", "stop"};
    qint64 prev = -1;
    for (int i = 0; i < kinds.size(); ++i) {
      const QJsonObject o = objs[i + 1];
      QCOMPARE(o["kind"].toString(), kinds[i]);
      const qint64 t = o["t_us"].toVariant().toLongLong();
      QVERIFY(t >= prev);
      prev = t;
    }
    QCOMPARE(objs[2]["channels"].toArray().size(), 6);
    QCOMPARE(objs[2]["source"].toString(), QString("gamepad"));
    QCOMPARE(objs[3]["armed"].toBool(), true);
    QCOMPARE(objs[4]["from"].toString(), QString("STABILISE"));
    QCOMPARE(objs[4]["to"].toString(), QString("ACRO"));
    QCOMPARE(objs[4]["source"].toString(), QString("rc"));
    QCOMPARE(objs[5]["id"].toString(), QString("motor1"));
    QCOMPARE(objs[5]["active"].toBool(), true);
    QCOMPARE(objs[6]["active"].toBool(), false);
    QVERIFY(objs[6]["detail"].isNull());
    QCOMPARE(objs[7]["text"].toString(), QString("gust"));
    QCOMPARE(objs[8]["reason"].toString(), QString("user"));
    QCOMPARE(log.eventCount(), qint64(8));
    QCOMPARE(FlightEventLog::countEvents(p), 8);
  }

  void missingHeaderValuesAreNullNotOmitted() {
    QTemporaryDir dir;
    const QString p = dir.filePath("ev-x.jsonl");
    FlightEventLog log;
    QVERIFY(log.open(p, FlightEventLog::Header{}));
    log.close();
    const QJsonObject h =
        QJsonDocument::fromJson(rawLines(p).value(0)).object();
    for (const char *k : {"run_id", "app_version", "sdk_version",
                          "vehicle_sha256", "world_preset", "world_seed",
                          "gt_file", "module_build_id"}) {
      QVERIFY2(h.contains(QLatin1String(k)), k);
      QVERIFY2(h[QLatin1String(k)].isNull(), k);
    }
    QCOMPARE(h["schema"].toString(), QString("nav-flight-events/1"));
  }

  void rcWrittenOnChangeAndAtTenHz() {
    QTemporaryDir dir;
    const QString p = dir.filePath("ev-rc.jsonl");
    FlightEventLog log;
    QVERIFY(log.open(p, header()));
    const QVector<int> a{1500, 1500, 1000, 1500, 1000, 1000};
    QVector<int> b = a;
    b[2] = 1200;
    QVERIFY(log.rc(0, a, "gamepad")); // first frame
    // 50 Hz identical frames for one second: only the 10 Hz ticks land.
    int written = 0;
    for (qint64 t = 20000; t <= 1000000; t += 20000)
      written += log.rc(t, a, "gamepad") ? 1 : 0;
    QCOMPARE(written, 10); // t = 100 ms ... 1000 ms
    QVERIFY(log.rc(1010000, b, "gamepad")); // change: immediate
    QVERIFY(!log.rc(1020000, b, "gamepad"));
    QVERIFY(log.rc(1110000, b, "gamepad")); // 100 ms after the last write
    log.close();
    QCOMPARE(FlightEventLog::countEvents(p), 13);
  }

  void rcDeadbandIgnoresJitterButCatchesSteps() {
    QTemporaryDir dir;
    const QString p = dir.filePath("ev-db.jsonl");
    FlightEventLog log;
    QVERIFY(log.open(p, header()));
    QVERIFY(log.rc(0, {1500, 1500, 1000, 1500, 1500, 1000}, "joystick"));
    // +-1 / +-2 jitter on analog channels, 5 ms apart for 90 ms: no lines.
    for (int i = 1; i <= 18; ++i) {
      const int j = (i % 2) ? 2 : -2;
      QVERIFY(!log.rc(i * 5000, {1500 + j, 1500 - j / 2, 1000 + j, 1500, 1500, 1000},
                      "joystick"));
    }
    // Only the 10 Hz floor writes (identical to the last written within band).
    QVERIFY(log.rc(100000, {1502, 1499, 1002, 1500, 1500, 1000}, "joystick"));
    // Drift is measured against the last WRITTEN value: 1502 -> 1505 is a step of 3.
    QVERIFY(log.rc(100100, {1505, 1499, 1002, 1500, 1500, 1000}, "joystick"));
    // A step of 3 from the last written value writes immediately.
    QVERIFY(!log.rc(100200, {1507, 1499, 1002, 1500, 1500, 1000}, "joystick"));
    QVERIFY(log.rc(100300, {1509, 1499, 1002, 1500, 1500, 1000}, "joystick"));
    // The arm channel and ch6 count on any change.
    QVERIFY(log.rc(100400, {1509, 1499, 1002, 1500, 1501, 1000}, "joystick"));
    QVERIFY(log.rc(100500, {1509, 1499, 1002, 1500, 1501, 1001}, "joystick"));
    log.close();
    QVERIFY(!FlightEventLog::rcChanged({1000}, {1002}));
    QVERIFY(FlightEventLog::rcChanged({1000}, {1003}));
    QVERIFY(FlightEventLog::rcChanged({1000}, {1000, 1000}));
  }

  void cappedCountAndLiveRun() {
    QTemporaryDir dir;
    const QString p = dir.filePath("ev-cap.jsonl");
    FlightEventLog log;
    QVERIFY(log.open(p, header()));
    for (int i = 0; i < 200; ++i)
      log.marker(i, "m");
    log.close();
    bool capped = true;
    QCOMPARE(FlightEventLog::countEvents(p, 1 << 20, &capped), 200);
    QVERIFY(!capped);
    const int n = FlightEventLog::countEvents(p, 1000, &capped);
    QVERIFY(capped);
    QVERIFY(n > 0 && n < 200);
    QVERIFY(FlightEventLog::isLiveRun("a", "a", true));
    QVERIFY(!FlightEventLog::isLiveRun("a", "a", false));
    QVERIFY(!FlightEventLog::isLiveRun("a", "b", true));
    QVERIFY(!FlightEventLog::isLiveRun("", "", true));
  }

  void backwardsTimestampIsClamped() {
    QTemporaryDir dir;
    const QString p = dir.filePath("ev-t.jsonl");
    FlightEventLog log;
    QVERIFY(log.open(p, header()));
    log.marker(500, "a");
    log.marker(100, "b");
    log.close();
    const auto ev = FlightEventLog::readEvents(p);
    QCOMPARE(ev.size(), 3);
    QCOMPARE(ev[2]["t_us"].toVariant().toLongLong(), 500LL);
  }

  void readerToleratesTruncatedLastLine() {
    QTemporaryDir dir;
    const QString p = dir.filePath("ev-trunc.jsonl");
    FlightEventLog log;
    QVERIFY(log.open(p, header()));
    log.start(0);
    log.marker(10, "hello");
    log.close();
    QFile f(p);
    QVERIFY(f.open(QIODevice::Append));
    f.write("{\"t_us\":20,\"kind\":\"mar"); // crash mid-line
    f.close();
    const auto ev = FlightEventLog::readEvents(p);
    QCOMPARE(ev.size(), 3); // header, start, marker
    QCOMPARE(ev[2]["text"].toString(), QString("hello"));
    QVERIFY(FlightEventLog::readEvents(dir.filePath("nope")).isEmpty());
  }

  void eachLineIsOnDiskBeforeClose() {
    QTemporaryDir dir;
    const QString p = dir.filePath("ev-flush.jsonl");
    FlightEventLog log;
    QVERIFY(log.open(p, header()));
    log.marker(1, "m");
    // Not closed: a crash here must still leave header + event readable.
    QCOMPARE(rawLines(p).size(), 2);
  }

  // ---- listing + retention -------------------------------------------------
  void listRunsPairsAndIgnoresOthers() {
    QTemporaryDir dir;
    const QString d = dir.path();
    touch(d + "/gt-2026-01-01_00-00-01.bin", "aaaa");
    touch(d + "/ev-2026-01-01_00-00-01.jsonl", "bb\n");
    touch(d + "/gt-2026-01-01_00-00-02.bin");
    touch(d + "/ev-2026-01-01_00-00-03.jsonl");
    touch(d + "/notes.txt");
    touch(d + "/autotune_vehicle.json");
    const auto runs = FlightEventLog::listRuns(d);
    QCOMPARE(runs.size(), 3);
    QCOMPARE(runs[0].stamp, QString("2026-01-01_00-00-03")); // newest first
    QVERIFY(runs[0].gtPath.isEmpty());
    QCOMPARE(runs[2].gtBytes, qint64(4));
    QCOMPARE(runs[2].evBytes, qint64(3));
  }

  void prunePairsTogetherOldestFirst() {
    QTemporaryDir dir;
    const QString d = dir.path();
    for (int i = 1; i <= 5; ++i) {
      const QString s = QString("2026-01-01_00-00-0%1").arg(i);
      touch(d + "/gt-" + s + ".bin");
      touch(d + "/ev-" + s + ".jsonl");
    }
    touch(d + "/readme.txt");
    touch(d + "/gt-2026-01-01_00-00-01.bin.bak");
    touch(d + "/autotune_world.json");
    QCOMPARE(FlightEventLog::prune(d, 3, "2026-01-01_00-00-05"), 4);
    QVERIFY(!QFile::exists(d + "/gt-2026-01-01_00-00-01.bin"));
    QVERIFY(!QFile::exists(d + "/ev-2026-01-01_00-00-01.jsonl"));
    QVERIFY(!QFile::exists(d + "/gt-2026-01-01_00-00-02.bin"));
    QVERIFY(!QFile::exists(d + "/ev-2026-01-01_00-00-02.jsonl"));
    for (int i = 3; i <= 5; ++i) {
      const QString s = QString("2026-01-01_00-00-0%1").arg(i);
      QVERIFY(QFile::exists(d + "/gt-" + s + ".bin"));
      QVERIFY(QFile::exists(d + "/ev-" + s + ".jsonl"));
    }
    // Unrelated names untouched.
    QVERIFY(QFile::exists(d + "/readme.txt"));
    QVERIFY(QFile::exists(d + "/gt-2026-01-01_00-00-01.bin.bak"));
    QVERIFY(QFile::exists(d + "/autotune_world.json"));
  }

  void loneFileCountsAsOwnRun() {
    QTemporaryDir dir;
    const QString d = dir.path();
    touch(d + "/gt-2026-01-01_00-00-01.bin"); // lone gt
    touch(d + "/ev-2026-01-01_00-00-02.jsonl"); // lone ev
    touch(d + "/gt-2026-01-01_00-00-03.bin");
    touch(d + "/ev-2026-01-01_00-00-03.jsonl");
    QCOMPARE(FlightEventLog::prune(d, 2, "2026-01-01_00-00-03"), 1);
    QVERIFY(!QFile::exists(d + "/gt-2026-01-01_00-00-01.bin"));
    QVERIFY(QFile::exists(d + "/ev-2026-01-01_00-00-02.jsonl"));
    QVERIFY(QFile::exists(d + "/gt-2026-01-01_00-00-03.bin"));
  }

  void currentRunNeverDeleted() {
    QTemporaryDir dir;
    const QString d = dir.path();
    // The run opening now has the OLDEST stamp (clock stepped back).
    touch(d + "/gt-2026-01-01_00-00-01.bin");
    touch(d + "/ev-2026-01-01_00-00-01.jsonl");
    touch(d + "/gt-2026-01-01_00-00-02.bin");
    touch(d + "/gt-2026-01-01_00-00-03.bin");
    FlightEventLog::prune(d, 1, "2026-01-01_00-00-01");
    QVERIFY(QFile::exists(d + "/gt-2026-01-01_00-00-01.bin"));
    QVERIFY(QFile::exists(d + "/ev-2026-01-01_00-00-01.jsonl"));
    QVERIFY(!QFile::exists(d + "/gt-2026-01-01_00-00-02.bin"));
    QVERIFY(!QFile::exists(d + "/gt-2026-01-01_00-00-03.bin"));
  }

  void zeroKeepsAll() {
    QTemporaryDir dir;
    const QString d = dir.path();
    for (int i = 1; i <= 4; ++i)
      touch(d + QString("/gt-2026-01-01_00-00-0%1.bin").arg(i));
    QCOMPARE(FlightEventLog::prune(d, 0, "2026-01-01_00-00-04"), 0);
    QCOMPARE(FlightEventLog::listRuns(d).size(), 4);
    QCOMPARE(FlightEventLog::prune(d, -1, ""), 0);
  }
};

QTEST_MAIN(TstFlightEventLog)
#include "tst_flight_event_log.moc"
