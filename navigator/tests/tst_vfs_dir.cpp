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
// Headless: VFS dir resolution (user env wins, else <AppData>/vfs, created) and
// the reset helper's safety rule (never outside the app-data dir).
#include <QDir>
#include <QFile>
#include <QTemporaryDir>
#include <QtTest>

#include "VfsDir.h"
#include "VfsTestEnv.h"

class TstVfsDir : public QObject {
  Q_OBJECT
private slots:
  void userEnvWins() {
    QTemporaryDir root;
    const auto c = vsim::resolveVfsDir("/some/user/dir", root.path());
    QVERIFY(c.fromUser);
    QCOMPARE(c.dir, QString("/some/user/dir"));
    QVERIFY(!QDir(root.path() + "/vfs").exists()); // nothing created
  }
  void defaultsToAppDataAndCreates() {
    QTemporaryDir root;
    const auto c = vsim::resolveVfsDir(QString(), root.path());
    QVERIFY(!c.fromUser);
    QVERIFY(c.ok);
    QCOMPARE(c.dir, root.path() + "/vfs");
    QVERIFY(QDir(c.dir).exists());
    QVERIFY(!c.dir.startsWith("/tmp/vayu_vfs"));
  }
  void resetClearsOnlyContents() {
    QTemporaryDir root;
    const auto c = vsim::resolveVfsDir(QString(), root.path());
    QFile f(c.dir + "/0_pid.bin");
    QVERIFY(f.open(QIODevice::WriteOnly));
    f.write("x");
    f.close();
    QVERIFY(QDir(c.dir).mkpath("sub/deeper"));
    QString why;
    QVERIFY2(vsim::resetPrivateVfs(c.dir, root.path(), false, &why),
             qPrintable(why));
    QVERIFY(QDir(c.dir).exists());
    QVERIFY(QDir(c.dir).isEmpty(QDir::AllEntries | QDir::NoDotAndDotDot));
  }
  void resetRefusesUserDir() {
    QTemporaryDir root;
    const auto c = vsim::resolveVfsDir(QString(), root.path());
    QFile f(c.dir + "/keep");
    QVERIFY(f.open(QIODevice::WriteOnly));
    f.close();
    QString why;
    QVERIFY(!vsim::resetPrivateVfs(c.dir, root.path(), true, &why));
    QVERIFY(!why.isEmpty());
    QVERIFY(QFile::exists(c.dir + "/keep"));
  }
  void resetRefusesOutsideAppData() {
    QTemporaryDir root, other;
    QFile f(other.path() + "/precious");
    QVERIFY(f.open(QIODevice::WriteOnly));
    f.close();
    QString why;
    QVERIFY(!vsim::resetPrivateVfs(other.path(), root.path(), false, &why));
    QVERIFY(QFile::exists(other.path() + "/precious"));
    // the app-data root itself is not "inside" it either
    QVERIFY(!vsim::resetPrivateVfs(root.path(), root.path(), false, &why));
    // a symlink inside app data pointing outside must not be followed
    QVERIFY(QFile::link(other.path(), root.path() + "/link"));
    QVERIFY(!vsim::resetPrivateVfs(root.path() + "/link", root.path(), false,
                                   &why));
    QVERIFY(QFile::exists(other.path() + "/precious"));
  }
  void testHelperSetsEnv() {
    const QString d = testenv::isolateVfs();
    QVERIFY(!d.isEmpty());
    QCOMPARE(qEnvironmentVariable("VAYU_VFS_DIR"), d);
    const auto c = vsim::applyVfsEnvironment(); // user env now set -> wins
    QVERIFY(c.fromUser);
    QCOMPARE(c.dir, d);
  }
};

QTEST_GUILESS_MAIN(TstVfsDir)
#include "tst_vfs_dir.moc"
