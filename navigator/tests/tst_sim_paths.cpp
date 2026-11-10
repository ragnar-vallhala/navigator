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
// Simulator default paths: log dir location, and the stale world-export sweep.

#include "SimPaths.h"

#include <QDir>
#include <QFile>
#include <QStandardPaths>
#include <QTemporaryDir>
#include <QtTest>

class TstSimPaths : public QObject {
  Q_OBJECT
private slots:
  void initTestCase() { QStandardPaths::setTestModeEnabled(true); }

  void logDirIsPerUserNotCheckout() {
    qunsetenv("VAYU_REPO");
    const QString dir = vsim::defaultLogDir();
    const QString appData =
        QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    QCOMPARE(dir, appData + QStringLiteral("/logs"));
    // Not under the executable's tree (the old behaviour found the checkout).
    QVERIFY(!dir.startsWith(QCoreApplication::applicationDirPath()));
    QVERIFY(!dir.contains(QStringLiteral("worktrees")));
  }

  void logDirHonoursVayuRepo() {
    qputenv("VAYU_REPO", "/some/vayu");
    QCOMPARE(vsim::defaultLogDir(), QStringLiteral("/some/vayu/logs"));
    qunsetenv("VAYU_REPO");
  }
};

QTEST_MAIN(TstSimPaths)
#include "tst_sim_paths.moc"
