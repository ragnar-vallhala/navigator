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
#include <QProcess>
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

  // (a) Real case: libmod.so links against libdep.so.1, then libdep is removed.
  void missingDependencyKeepsDependencyName() {
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString cc = QStandardPaths::findExecutable("cc");
    if (cc.isEmpty())
      QSKIP("no C compiler available");
    const QString dsrc = dir.filePath("dep.c"), msrc = dir.filePath("mod.c");
    for (const auto &[f, body] :
         {std::pair<QString, QByteArray>{dsrc, "int dep_fn(void){return 1;}\n"},
          {msrc, "int dep_fn(void);\nint mod_fn(void){return dep_fn();}\n"}}) {
      QFile w(f);
      QVERIFY(w.open(QIODevice::WriteOnly));
      w.write(body);
    }
    const QString dep = dir.filePath("libuniquedep.so.1");
    const QString mod = dir.filePath("libvayu_sitl.so");
    auto run = [&](const QStringList &a) {
      QProcess pr;
      pr.start(cc, a);
      return pr.waitForFinished(60000) && pr.exitCode() == 0;
    };
    if (!run({"-shared", "-fPIC", "-Wl,-soname,libuniquedep.so.1", "-o", dep,
              dsrc}) ||
        !run({"-shared", "-fPIC", "-o", mod, msrc, dep}))
      QSKIP("could not build the fake module");
    QVERIFY(QFile::remove(dep));
    QLibrary lib(mod);
    QVERIFY(!lib.load());
    const QString r = SitlModule::loadFailureReason(mod, lib.errorString());
    QVERIFY2(r.contains("libuniquedep.so.1"), qPrintable(r));
    QVERIFY2(r.contains("cannot open shared object file"), qPrintable(r));
    QVERIFY(r.startsWith(mod));
  }

  // (a') the exact loader string, for hosts without a compiler.
  void missingDependencyLoaderStringIsQuoted() {
    QTemporaryDir dir;
    const QString p = dir.filePath("libvayu_sitl.so");
    QFile f(p);
    QVERIFY(f.open(QIODevice::WriteOnly));
    f.write("x");
    f.close();
    const QString err = "Cannot load library " + p +
                        ": (libdep.so.1: cannot open shared object file: No "
                        "such file or directory)";
    const QString r = SitlModule::loadFailureReason(p, err);
    QVERIFY2(r.contains("libdep.so.1"), qPrintable(r));
  }

  // (b) the module file itself absent: short message, no loader text.
  void absentModuleStaysShort() {
    QTemporaryDir dir;
    const QString p = dir.filePath("libvayu_sitl.so");
    const QString r = SitlModule::loadFailureReason(
        p, "Cannot load library " + p +
               ": cannot open shared object file: No such file or directory");
    QCOMPARE(r, p + " (not found)");
  }

  // (b') file exists but the loader's last attempt says the candidate itself
  // cannot be opened (QLibrary name-probing): still not shown.
  void selfNoSuchFileFromProbingIsHidden() {
    QTemporaryDir dir;
    const QString p = dir.filePath("libvayu_sitl.so");
    QFile f(p);
    QVERIFY(f.open(QIODevice::WriteOnly));
    f.close();
    const QString r = SitlModule::loadFailureReason(
        p, p + ": cannot open shared object file: No such file or directory");
    QVERIFY2(r.contains("exists but could not be loaded"), qPrintable(r));
  }

  // (c) other loader text about the candidate is quoted unchanged.
  void otherLoaderTextUnchanged() {
    QTemporaryDir dir;
    const QString p = dir.filePath("libvayu_sitl.so");
    QFile f(p);
    QVERIFY(f.open(QIODevice::WriteOnly));
    f.close();
    const QString err = "Cannot load library " + p + ": (file too short)";
    QCOMPARE(SitlModule::loadFailureReason(p, err), p + ": " + err);
  }
};

QTEST_MAIN(TstSitlModuleError)
#include "tst_sitl_module_error.moc"
