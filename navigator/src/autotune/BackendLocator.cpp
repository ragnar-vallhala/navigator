#include "BackendLocator.h"

#include <QDir>
#include <QFileInfo>

namespace autotune {

namespace {
#ifdef Q_OS_WIN
const QString kName = QStringLiteral("vayu_sitl_rtos.exe");
#else
const QString kName = QStringLiteral("vayu_sitl_rtos");
#endif
} // namespace

QStringList backendCandidates(const QString &envOverride,
                              const QString &appDir, const QString &repoRoot) {
  QStringList out;
  if (!envOverride.isEmpty())
    out << envOverride;
  const QDir app(appDir);
  out << app.absoluteFilePath(kName);
  out << app.absoluteFilePath(QStringLiteral("lib/") + kName);
  out << QDir::cleanPath(app.absoluteFilePath(QStringLiteral("../lib/") + kName));
  const QDir repo(repoRoot);
  out << repo.absoluteFilePath(QStringLiteral("build_sitl_rtos/") + kName);
  out << repo.absoluteFilePath(QStringLiteral("sim/host/build_sitl_rtos/") + kName);
  return out;
}

BackendLookup findRtosBackend(const QString &envOverride,
                              const QString &appDir, const QString &repoRoot) {
  BackendLookup r;
  r.tried = backendCandidates(envOverride, appDir, repoRoot);
  for (const QString &c : r.tried) {
    const QFileInfo fi(c);
    if (fi.isFile() && fi.isExecutable()) {
      r.found = c;
      break;
    }
  }
  return r;
}

QString backendNotFoundMessage(const QStringList &tried) {
  QString s = QStringLiteral("[error] autotune backend (vayu_sitl_rtos) not found. Looked in:");
  for (const QString &t : tried)
    s += QStringLiteral("\n  ") + t;
  s += QStringLiteral(
      "\nPut vayu_sitl_rtos next to the Navigator binary, set VAYU_SITL_RTOS to "
      "its path, or build it in a vayu checkout (set VAYU_REPO if it is not "
      "found automatically):\n"
      "  cmake -S sim/host -B build_sitl_rtos -DVAYU_SITL_RTOS_BUILD=ON\n"
      "  cmake --build build_sitl_rtos --target vayu_sitl_rtos");
  return s;
}

} // namespace autotune
