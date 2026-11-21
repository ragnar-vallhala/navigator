// Lookup order for the autotune backend: env override, then beside the binary,
// then the repo build trees. An installed app has no repo, so "beside the
// binary" must win over (and work without) any repo path.
#include <QtTest>
#include <QTemporaryDir>
#include <QFile>
#include "BackendLocator.h"

using namespace autotune;

namespace {
QString touchExe(const QString &path) {
  QDir().mkpath(QFileInfo(path).path());
  QFile f(path);
  if (!f.open(QIODevice::WriteOnly))
    return {};
  f.write("#!/bin/sh\n");
  f.close();
  f.setPermissions(QFile::permissions(path) | QFile::ExeOwner);
  return path;
}
#ifdef Q_OS_WIN
const char *kBin = "vayu_sitl_rtos.exe";
#else
const char *kBin = "vayu_sitl_rtos";
#endif
} // namespace

class TstBackendLocator : public QObject {
  Q_OBJECT
private slots:
  void candidatesInDocumentedOrder() {
    const QStringList c = backendCandidates("/x/env_bin", "/opt/nav", "/src/vayu");
    QCOMPARE(c.size(), 6);
    QCOMPARE(c[0], QString("/x/env_bin"));
    QVERIFY(c[1].endsWith(QString("/opt/nav/") + kBin));
    QVERIFY(c[2].endsWith(QString("/opt/nav/lib/") + kBin));
    QVERIFY(c[3].endsWith(QString("/opt/lib/") + kBin));
    QVERIFY(c[4].endsWith(QString("/src/vayu/build_sitl_rtos/") + kBin));
    QVERIFY(c[5].endsWith(QString("/src/vayu/sim/host/build_sitl_rtos/") + kBin));
    QCOMPARE(backendCandidates("", "/opt/nav", "/src/vayu").size(), 5); // no env -> no slot
  }
  void besideBinaryWorksWithNoRepo() {
    QTemporaryDir t;
    const QString app = t.path() + "/opt/navigator";
    const QString bin = touchExe(app + "/" + kBin);
    const BackendLookup r = findRtosBackend("", app, t.path() + "/no/such/repo");
    QCOMPARE(r.found, QDir(app).absoluteFilePath(kBin));
    QCOMPARE(QFileInfo(r.found).canonicalFilePath(), QFileInfo(bin).canonicalFilePath());
  }
  void envBeatsBesideBinary() {
    QTemporaryDir t;
    const QString app = t.path() + "/app";
    touchExe(app + "/" + kBin);
    // Windows treats a file as executable only by extension, so keep ".exe" there.
#ifdef Q_OS_WIN
    const QString envBin = touchExe(t.path() + "/elsewhere/custom_rtos.exe");
#else
    const QString envBin = touchExe(t.path() + "/elsewhere/custom_rtos");
#endif
    QVERIFY(!envBin.isEmpty());
    QCOMPARE(findRtosBackend(envBin, app, t.path() + "/repo").found, envBin);
  }
  void besideBinaryBeatsRepo() {
    QTemporaryDir t;
    const QString app = t.path() + "/app", repo = t.path() + "/repo";
    touchExe(repo + "/build_sitl_rtos/" + kBin);
    touchExe(app + "/" + kBin);
    QCOMPARE(findRtosBackend("", app, repo).found, QDir(app).absoluteFilePath(kBin));
  }
  void repoBuildTreesAreTheFallback() {
    QTemporaryDir t;
    const QString app = t.path() + "/app", repo = t.path() + "/repo";
    touchExe(repo + "/sim/host/build_sitl_rtos/" + kBin);
    QCOMPARE(findRtosBackend("", app, repo).found,
             QDir(repo).absoluteFilePath(QString("sim/host/build_sitl_rtos/") + kBin));
    touchExe(repo + "/build_sitl_rtos/" + kBin); // first repo path wins over the second
    QCOMPARE(findRtosBackend("", app, repo).found,
             QDir(repo).absoluteFilePath(QString("build_sitl_rtos/") + kBin));
  }
  void unusableCandidatesAreSkipped() {
    QTemporaryDir t;
    const QString app = t.path() + "/app";
    QDir().mkpath(app + "/" + kBin);                      // a directory is not a backend
#ifndef Q_OS_WIN
    QFile f(app + "/lib/" + kBin);                        // present but not executable
    QDir().mkpath(app + "/lib");
    QVERIFY(f.open(QIODevice::WriteOnly));
    f.close();
    f.setPermissions(QFile::ReadOwner | QFile::WriteOwner);
#endif
    const QString good = touchExe(t.path() + "/lib/" + kBin);   // ../lib of app
    const BackendLookup r = findRtosBackend("", app, t.path() + "/repo");
    QCOMPARE(QFileInfo(r.found).canonicalFilePath(), QFileInfo(good).canonicalFilePath());
  }
  void notFoundNamesEveryPathTried() {
    QTemporaryDir t;
    const BackendLookup r = findRtosBackend("/nope/env", t.path() + "/app", t.path() + "/repo");
    QVERIFY(r.found.isEmpty());
    QCOMPARE(r.tried.size(), 6);
    const QString msg = backendNotFoundMessage(r.tried);
    for (const QString &p : r.tried)
      QVERIFY2(msg.contains(p), qPrintable("message lacks " + p));
    QVERIFY(msg.contains("VAYU_SITL_RTOS"));
    QVERIFY(msg.contains("cmake -S sim/host"));
  }
};

QTEST_MAIN(TstBackendLocator)
#include "tst_autotune_backend_locator.moc"
