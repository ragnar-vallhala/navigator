#pragma once
// Test helper: point the firmware's host VFS at a fresh temp dir so no test
// reads or writes the machine-shared /tmp/vayu_vfs. Call isolateVfs() before
// anything that can load the engine module (the env is read at module load).
#include <QByteArray>
#include <QFile>
#include <QString>
#include <QTemporaryDir>
#include <QtGlobal>

namespace testenv {
inline QString isolateVfs() {
  static QTemporaryDir dir; // lives for the whole test process
  if (dir.isValid())
    qputenv("VAYU_VFS_DIR", QFile::encodeName(dir.path()));
  return dir.path();
}
} // namespace testenv
