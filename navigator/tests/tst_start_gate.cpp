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
// StartGate: Start stays disabled, with a visible reason, once blocked.

#include "StartGate.h"

#include <QPushButton>
#include <QtTest>

class TstStartGate : public QObject {
  Q_OBJECT
private slots:
  void unblockedFollowsCaller() {
    StartGate g;
    QPushButton b;
    g.apply(&b, true, "normal");
    QVERIFY(b.isEnabled());
    QCOMPARE(b.toolTip(), QStringLiteral("normal"));
    g.apply(&b, false, "normal");
    QVERIFY(!b.isEnabled());
  }

  void blockedStaysDisabledWhateverTheCallerWants() {
    StartGate g;
    g.block("Restart Navigator");
    QVERIFY(g.blocked());
    QPushButton b;
    g.apply(&b, true, "normal"); // e.g. stopInAppSim re-enabling Start
    QVERIFY(!b.isEnabled());
    QCOMPARE(b.toolTip(), QStringLiteral("Restart Navigator"));
    g.apply(&b, true, "normal"); // and again: no way back in this process
    QVERIFY(!b.isEnabled());
  }

  void nullButtonIsHarmless() {
    StartGate g;
    g.block("x");
    g.apply(nullptr, true, "t");
  }
};

QTEST_MAIN(TstStartGate)
#include "tst_start_gate.moc"
