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
#include <QtTest>

#include "audio/PropAudio.h"

using propaudio::kIdleCloseMs;
using propaudio::kStaleMs;
using propaudio::streamWanted;
using propaudio::targetGain;

class TstPropAudioGate : public QObject {
  Q_OBJECT
private slots:
  // The reported bug: the last rpm was held forever after Stop.
  void staleFeedIsSilent() {
    const std::array<float, 4> w = {800, 800, 800, 800};
    QVERIFY(targetGain(w, 0) > 0.5f);
    QVERIFY(targetGain(w, kStaleMs) > 0.5f); // still fresh at the limit
    QCOMPARE(targetGain(w, kStaleMs + 1), 0.0f);
    QCOMPARE(targetGain(w, 60000), 0.0f); // a minute after Stop
  }

  void freshFeedScalesWithRpm() {
    QCOMPARE(targetGain({0, 0, 0, 0}, 0), 0.0f);
    QVERIFY(targetGain({300, 0, 0, 0}, 10) < targetGain({600, 0, 0, 0}, 10));
    QCOMPARE(targetGain({5000, 0, 0, 0}, 10), 1.0f); // clamped
  }

  void streamClosesAfterSustainedSilence() {
    QVERIFY(streamWanted(true, 0));
    QVERIFY(streamWanted(true, kIdleCloseMs - 1));
    QVERIFY(!streamWanted(true, kIdleCloseMs));
    QVERIFY(!streamWanted(false, 0)); // muted by the checkbox
  }

  // End to end on the real object: feed, silence, feed again. Under a stub
  // (no PulseAudio) it only has to stay safe from the GUI thread.
  void silenceThenResumeIsSafe() {
    PropAudio a;
    a.setEnabled(true);
    a.setMotors({700, 700, 700, 700});
    QTest::qWait(30);
    a.silence();
    QTest::qWait(30);
    a.setMotors({500, 500, 500, 500});
    a.silence();
    a.setEnabled(false);
  }
};

QTEST_MAIN(TstPropAudioGate)
#include "tst_prop_audio_gate.moc"
