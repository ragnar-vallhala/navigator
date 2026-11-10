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

#include <QDir>
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

} // namespace vsim
