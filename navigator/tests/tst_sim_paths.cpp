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
// Simulator default paths: log dir location, and the stale world-export sweep.

#include "SimPaths.h"

#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QFile>
#include <QStandardPaths>
#include <QTemporaryDir>
#include <QtTest>

class TstSimPaths : public QObject {
  Q_OBJECT
private slots:
  void initTestCase() { QStandardPaths::setTestModeEnabled(true); }

  void logDirIsPerUserNotCheckout() {
    qunsetenv("VAYU_REPO");
    const QString dir = vsim::defaultLogDir();
    const QString appData =
        QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    QCOMPARE(dir, appData + QStringLiteral("/logs"));
    // Not under the executable's tree (the old behaviour found the checkout).
    QVERIFY(!dir.startsWith(QCoreApplication::applicationDirPath()));
    QVERIFY(!dir.contains(QStringLiteral("worktrees")));
  }

  void logDirHonoursVayuRepo() {
    qputenv("VAYU_REPO", "/some/vayu");
    QCOMPARE(vsim::defaultLogDir(), QStringLiteral("/some/vayu/logs"));
    qunsetenv("VAYU_REPO");
  }

  // ---- stale world tmp sweep (temp dir only, never the real /tmp) ----------
  static void touch(const QString &path, int ageSecs) {
    QFile f(path);
    QVERIFY(f.open(QIODevice::WriteOnly));
    f.write("x");
    f.close();
    QVERIFY(f.open(QIODevice::ReadWrite));
    QVERIFY(f.setFileTime(QDateTime::currentDateTime().addSecs(-ageSecs),
                          QFileDevice::FileModificationTime));
  }

  void sweepRemovesOnlyOldMatchingFiles() {
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString d = dir.path();
    const int old = 7200, fresh = 5;
    touch(d + "/vsim_world.bin.3.tmp", old);        // stale, no suffix
    touch(d + "/vsim_world_a1b2.bin.12.tmp", old);  // stale, with suffix
    touch(d + "/vsim_world.bin.4.tmp", fresh);      // young: a live export
    touch(d + "/vsim_world.bin", old);              // the real blob
    touch(d + "/vsim_world.bin.x.tmp", old);        // N not a number
    touch(d + "/vsim_world.bin.5.tmp.keep", old);   // wrong tail
    touch(d + "/other.bin.3.tmp", old);             // other owner
    touch(d + "/xvsim_world.bin.3.tmp", old);       // wrong prefix
    QVERIFY(QDir(d).mkdir("vsim_world.bin.9.tmp")); // a dir, not a file

    QCOMPARE(vsim::sweepStaleWorldTmp(d, 3600), 2);

    QVERIFY(!QFile::exists(d + "/vsim_world.bin.3.tmp"));
    QVERIFY(!QFile::exists(d + "/vsim_world_a1b2.bin.12.tmp"));
    for (const char *keep :
         {"vsim_world.bin.4.tmp", "vsim_world.bin", "vsim_world.bin.x.tmp",
          "vsim_world.bin.5.tmp.keep", "other.bin.3.tmp",
          "xvsim_world.bin.3.tmp", "vsim_world.bin.9.tmp"})
      QVERIFY2(QFileInfo::exists(d + "/" + keep), keep);
  }

  void sweepOnMissingDirIsHarmless() {
    QCOMPARE(vsim::sweepStaleWorldTmp("/nonexistent-dir-for-test", 3600), 0);
  }

  void worldBlobPathUsesWorldTmpDir() {
    QCOMPARE(vsim::worldBlobPath("_s"),
             vsim::worldTmpDir() + QStringLiteral("/vsim_world_s.bin"));
  }
};

QTEST_MAIN(TstSimPaths)
#include "tst_sim_paths.moc"
