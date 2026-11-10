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
#include "VfsDir.h"

#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QStandardPaths>
#include <QtGlobal>

namespace vsim {

VfsChoice resolveVfsDir(const QString &userEnv, const QString &appDataRoot) {
  VfsChoice c;
  if (!userEnv.isEmpty()) {
    c.dir = userEnv;
    c.fromUser = true;
    return c; // the user owns it; do not create or second-guess
  }
  c.dir = QDir(appDataRoot).absoluteFilePath(QStringLiteral("vfs"));
  c.ok = QDir().mkpath(c.dir);
  return c;
}

QString privateVfsDir() {
  return QDir(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation))
      .absoluteFilePath(QStringLiteral("vfs"));
}

VfsChoice applyVfsEnvironment() {
  const VfsChoice c = resolveVfsDir(
      qEnvironmentVariable("VAYU_VFS_DIR"),
      QStandardPaths::writableLocation(QStandardPaths::AppDataLocation));
  if (c.ok && !c.fromUser)
    qputenv("VAYU_VFS_DIR", QFile::encodeName(c.dir));
  qInfo().noquote() << "[vfs] firmware VFS dir:" << c.dir
                    << (c.fromUser ? "(from VAYU_VFS_DIR)" : "(private)")
                    << (c.ok ? "" : "- could not be created");
  return c;
}

bool resetPrivateVfs(const QString &dir, const QString &appDataRoot,
                     bool userSet, QString *why) {
  auto refuse = [&](const QString &m) {
    if (why)
      *why = m;
    return false;
  };
  if (userSet)
    return refuse(QStringLiteral(
        "VAYU_VFS_DIR is set by you; clear files in that directory yourself."));
  const QString root = QFileInfo(appDataRoot).canonicalFilePath();
  const QString d = QFileInfo(dir).canonicalFilePath();
  if (root.isEmpty() || d.isEmpty())
    return refuse(QStringLiteral("Directory does not exist: %1").arg(dir));
  if (!d.startsWith(root + QLatin1Char('/')))
    return refuse(
        QStringLiteral("%1 is outside the app data directory %2.").arg(d, root));
  const auto entries = QDir(d).entryInfoList(
      QDir::AllEntries | QDir::NoDotAndDotDot | QDir::Hidden | QDir::System);
  for (const QFileInfo &e : entries) {
    const bool ok = (e.isDir() && !e.isSymLink())
                        ? QDir(e.absoluteFilePath()).removeRecursively()
                        : QFile::remove(e.absoluteFilePath());
    if (!ok)
      return refuse(QStringLiteral("Could not remove %1").arg(e.fileName()));
  }
  return true;
}

} // namespace vsim
