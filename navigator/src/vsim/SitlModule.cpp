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
#include "SitlModule.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QLibrary>
#include "VfsDir.h"
#include <QProcessEnvironment>

// QLibrary rather than dlopen/LoadLibrary behind an #ifdef: Qt is already a
// dependency, and it resolves the platform's own naming (libvayu_sitl.so,
// vayu_sitl.dll) from one base name.

namespace {

// Where to look, in order. An explicit override wins so a developer can point
// the GCS at a firmware build without installing it; otherwise we look beside
// the executable (how it ships), then in the build tree (how it is developed).
QStringList candidatePaths() {
  QStringList out;

  const QString override =
      QProcessEnvironment::systemEnvironment().value("VAYU_SITL_MODULE");
  if (!override.isEmpty())
    out << override;

  const QDir appDir(QCoreApplication::applicationDirPath());
  out << appDir.absoluteFilePath("lib" VAYU_SITL_MODULE_NAME);
  out << appDir.absoluteFilePath("../lib/lib" VAYU_SITL_MODULE_NAME);

  // Development layout: a sibling SITL build dir next to the source tree.
  for (const char *build : {"build_sitl_rtos", "build_sitl"})
    out << appDir.absoluteFilePath(
        QStringLiteral("../../%1/lib%2")
            .arg(QLatin1String(build), QLatin1String(VAYU_SITL_MODULE_NAME)));

  return out;
}

} // namespace

QStringList SitlModule::searchPaths() { return candidatePaths(); }

QString SitlModule::loadFailureReason(const QString &candidate,
                                      const QString &libraryError) {
  const QFileInfo fi(candidate);
  if (!fi.exists())
    return QFileInfo(fi.path()).isDir()
               ? QStringLiteral("%1 (not found)").arg(candidate)
               : QStringLiteral("%1 (no such directory)").arg(candidate);
  if (fi.isDir())
    return QStringLiteral("%1: is a directory, not a shared library")
        .arg(candidate);
  // The file exists. QLibrary tries several names around the base and its
  // errorString() reports only the LAST attempt, so it can say "No such file"
  // about a name nobody passed. Quote it only when it is about this very path
  // and is not that misleading text; otherwise say what is known.
  // Exception: a missing DEPENDENCY is reported by the loader as
  // "<candidate>: (libdep.so.1: cannot open shared object file: No such file
  // or directory)". That also contains "No such file", but it names the
  // dependency the user must install, so keep it. When the file that cannot be
  // opened is the candidate itself ("<candidate>: cannot open shared object
  // file"), the module really is absent and the short text stays.
  const bool aboutCandidate = libraryError.contains(candidate);
  const bool missingDependency =
      libraryError.contains(QStringLiteral("cannot open shared object file")) &&
      !libraryError.contains(candidate + QStringLiteral(": cannot open"));
  const bool usable =
      !libraryError.isEmpty() && aboutCandidate &&
      (missingDependency ||
       !libraryError.contains(QStringLiteral("No such file")));
  return QStringLiteral("%1: %2")
      .arg(candidate,
           usable ? libraryError
                  : QStringLiteral("exists but could not be loaded (wrong "
                                   "ABI/architecture or missing dependency)"));
}

SitlModule::SitlModule() {
  // Before the first load: the firmware reads VAYU_VFS_DIR from the process
  // environment, and its default is shared by every process on the machine.
  vsim::applyVfsEnvironment();

  QStringList tried;

  for (const QString &candidate : candidatePaths()) {
    QLibrary lib(candidate);
    // RESOLVE_ALL_SYMBOLS_HINT: fail here on a module that is not
    // self-contained, rather than on whichever vtable call first needs the
    // missing symbol.
    lib.setLoadHints(QLibrary::ResolveAllSymbolsHint);
    if (!lib.load()) {
      tried << loadFailureReason(candidate, lib.errorString());
      continue;
    }

    auto get_api = reinterpret_cast<vayu_sitl_get_api_fn>(
        lib.resolve(VAYU_SITL_ENTRY_SYMBOL));
    if (!get_api) {
      tried << QStringLiteral("%1: no %2 symbol")
                   .arg(candidate, QLatin1String(VAYU_SITL_ENTRY_SYMBOL));
      lib.unload();
      continue;
    }

    const vayu_sitl_api_t *api = get_api(VAYU_SITL_ABI_VERSION);
    if (!api) {
      // The module is there but speaks a different ABI. Say so plainly: this
      // is a firmware/GCS version mismatch, not a missing file, and the two
      // have completely different fixes.
      tried << QStringLiteral("%1: ABI mismatch (this build needs v%2)")
                   .arg(candidate)
                   .arg(VAYU_SITL_ABI_VERSION);
      lib.unload();
      continue;
    }

    // Deliberately left loaded for the process lifetime. boot() starts vaios
    // tasks that never stop, so unloading the code underneath them would
    // crash; there is nothing to gain by trying.
    api_ = api;
    path_ = QFileInfo(lib.fileName()).absoluteFilePath();
    buildId_ = api->build_id ? QString::fromUtf8(api->build_id)
                             : QStringLiteral("unknown");
    return;
  }

  error_ =
      QStringLiteral("no SITL module found (lib%1). Build the firmware, or set "
                     "VAYU_SITL_MODULE. Looked in:\n  %2")
          .arg(QLatin1String(VAYU_SITL_MODULE_NAME), tried.join("\n  "));
}

bool SitlModule::send(const QByteArray &frame) const {
  if (!api_ || frame.isEmpty())
    return false;
  api_->uart2_rx(reinterpret_cast<const uint8_t *>(frame.constData()),
                 static_cast<size_t>(frame.size()));
  return true;
}

SitlModule &SitlModule::instance() {
  static SitlModule inst;
  return inst;
}
