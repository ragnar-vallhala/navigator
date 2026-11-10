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
// SitlModule load-failure text: "not found" only when the file is absent, the
// loader's own reason when it exists but cannot be loaded.

#include "SitlModule.h"

#include <QFile>
#include <QLibrary>
#include <QTemporaryDir>
#include <QtTest>

class TstSitlModuleError : public QObject {
  Q_OBJECT
private slots:
  void missingFileSaysNotFound() {
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString p = dir.filePath("libvayu_sitl.so");
    QLibrary lib(p);
    QVERIFY(!lib.load());
    const QString r = SitlModule::loadFailureReason(p, lib.errorString());
    QVERIFY2(r.contains("not found"), qPrintable(r));
    QVERIFY(r.contains(p));
  }

  void missingDirectorySaysSo() {
    const QString p = "/nonexistent-dir-for-test/libvayu_sitl.so";
    const QString r = SitlModule::loadFailureReason(p, QString());
    QVERIFY2(r.contains("no such directory"), qPrintable(r));
  }

  void existingNonLibraryReportsLoaderError() {
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString p = dir.filePath("libvayu_sitl.so");
    QFile f(p);
    QVERIFY(f.open(QIODevice::WriteOnly));
    f.write("this is not an ELF file\n");
    f.close();
    QLibrary lib(p);
    QVERIFY(!lib.load());
    QVERIFY(!lib.errorString().isEmpty());
    const QString r = SitlModule::loadFailureReason(p, lib.errorString());
    QVERIFY2(!r.contains("not found"), qPrintable(r));
    // The real cause of a non-ELF file, never the misleading "No such file".
    QVERIFY2(!r.contains("No such file"), qPrintable(r));
    QVERIFY2(r.contains("too short") || r.contains("ELF") ||
                 r.contains("invalid") || r.contains("exists but could not"),
             qPrintable(r));
    QVERIFY(r.startsWith(p));
  }

  void misleadingLoaderTextIsNotQuoted() {
    QTemporaryDir dir;
    const QString p = dir.filePath("x.so");
    QFile f(p);
    QVERIFY(f.open(QIODevice::WriteOnly));
    f.write("junk");
    f.close();
    // A loader error about another name, or "No such file", must not be shown.
    const QString a = SitlModule::loadFailureReason(
        p, "Cannot load library /elsewhere/libx.so: (No such file or directory)");
    QVERIFY2(!a.contains("No such file"), qPrintable(a));
    QVERIFY2(a.contains("exists but could not be loaded"), qPrintable(a));
    const QString b = SitlModule::loadFailureReason(
        p, "Cannot load library /elsewhere/libx.so: (file too short)");
    QVERIFY2(b.contains("exists but could not be loaded"), qPrintable(b));
  }

  void existingFileWithEmptyLoaderErrorIsStillNotNotFound() {
    QTemporaryDir dir;
    const QString p = dir.filePath("x.so");
    QFile f(p);
    QVERIFY(f.open(QIODevice::WriteOnly));
    f.close();
    const QString r = SitlModule::loadFailureReason(p, QString());
    QVERIFY2(!r.contains("not found"), qPrintable(r));
  }

  void directoryIsNotNotFound() {
    QTemporaryDir dir;
    const QString r = SitlModule::loadFailureReason(dir.path(), QString());
    QVERIFY2(r.contains("directory"), qPrintable(r));
  }
};

QTEST_MAIN(TstSitlModuleError)
#include "tst_sitl_module_error.moc"
