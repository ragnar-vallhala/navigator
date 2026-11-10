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
#include "SimPaths.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QRegularExpression>
#include <QStandardPaths>

namespace vsim {

QString defaultLogDir() {
  const QByteArray repo = qgetenv("VAYU_REPO");
  if (!repo.isEmpty())
    return QString::fromUtf8(repo) + QStringLiteral("/logs");
  const QString base =
      QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
  return base.isEmpty() ? QDir::tempPath() + QStringLiteral("/vayu-gcs/logs")
                        : base + QStringLiteral("/logs");
}

QString worldTmpDir() {
#ifdef _WIN32
  // No /tmp; the engine only needs a path it can read, so the user's temp dir.
  return QDir::tempPath();
#else
  return QStringLiteral("/tmp");
#endif
}

QString worldBlobPath(const QString &suffix) {
  return worldTmpDir() + QStringLiteral("/vsim_world") + suffix +
         QStringLiteral(".bin");
}

int sweepStaleWorldTmp(const QString &dir, qint64 minAgeSecs) {
  // The suffix is user-controlled text (VSIM_FIFO_SUFFIX), so match on the
  // fixed prefix and the fixed tail only, and require digits for N.
  static const QRegularExpression re(
      QStringLiteral("^vsim_world.*\\.bin\\.[0-9]+\\.tmp$"));
  int removed = 0;
  const QDateTime now = QDateTime::currentDateTime();
  const QFileInfoList list = QDir(dir).entryInfoList(
      {QStringLiteral("vsim_world*.tmp")}, QDir::Files | QDir::NoSymLinks);
  for (const QFileInfo &fi : list) {
    if (!re.match(fi.fileName()).hasMatch())
      continue;
    if (fi.lastModified().secsTo(now) < minAgeSecs)
      continue;
    if (QFile::remove(fi.absoluteFilePath()))
      ++removed;
  }
  return removed;
}

} // namespace vsim
