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
// The crash path end to end, headless, against the real engine module: the
// engine's own default is "no crash", so this only passes when SimWorker sends
// the crash limit on start, the module reports the crash in the pose tail, and
// the pose read (zeroed, size-arg get_pose) hands it to the snapshot. A reset
// must clear it again. Skips, saying why, without a module (like
// tst_sitl_engine); it needs an ABI 7 module.

#include "CrashStatus.h"
#include "SimWorker.h"
#include "SitlModule.h"
#include "VfsTestEnv.h"

#include <QSignalSpy>
#include <QtTest>

class TstSitlCrash : public QObject {
  Q_OBJECT
private slots:
  void aHardFallCrashesAndAResetClearsIt();
};

void TstSitlCrash::aHardFallCrashesAndAResetClearsIt() {
  testenv::isolateVfs(); // before SitlModule::instance(): it reads the VFS dir
  if (!SitlModule::instance().available())
    QSKIP(qPrintable("no SITL engine module: " + SitlModule::instance().error()));

  vsim::SimWorker sim;
  QSignalSpy online(&sim, &vsim::SimWorker::online);
  bool crashedNow = false;
  bool everCrashedBeforeFall = false;
  bool fell = false;
  float peak = 0.0f;
  int poses = 0;
  connect(&sim, &vsim::SimWorker::poseUpdated, this,
          [&](const vsim::SimSnapshot &s) {
            ++poses;
            crashedNow = vsim::crash::isCrashed(s.crash_flags);
            peak = s.peak_impact;
            if (!fell && crashedNow)
              everCrashedBeforeFall = true;
          });
  sim.start();
  QVERIFY2(online.wait(10000), "engine never came online");
  QTRY_VERIFY_WITH_TIMEOUT(poses > 5, 10000);
  QVERIFY2(!crashedNow, "crashed while sitting on the ground");
  // Settling onto the ground is a contact too, so the peak is small but not 0.
  QVERIFY2(peak < 2.0f, "the settle at boot should not be a hard contact");

  // 20 m free fall (motors off): about 20 m/s on impact, far over the 8 m/s
  // limit. Not armed, so nothing but gravity acts.
  fell = true;
  sim.sendResetPose(0.0f, 0.0f, -20.0f, 1u);
  QTRY_VERIFY2_WITH_TIMEOUT(crashedNow, "a 20 m fall never reported a crash",
                            15000);
  QVERIFY2(peak > vsim::crash::kLimitMps,
           qPrintable(QStringLiteral("peak %1 m/s is not over the limit")
                          .arg(peak)));
  QVERIFY2(peak < 60.0f, "peak impact is not a plausible speed");
  QVERIFY(!everCrashedBeforeFall);

  // The latch holds while the vehicle lies there ...
  QTest::qWait(300);
  QVERIFY(crashedNow);

  // ... and a reset clears the latch and the peak.
  sim.sendReset(2u);
  QTRY_VERIFY2_WITH_TIMEOUT(!crashedNow, "reset did not clear the crash",
                            10000);
  // The ~20 m/s peak is gone; what remains is the settle after the reset.
  QTRY_VERIFY2_WITH_TIMEOUT(peak < 2.0f, "reset did not clear the peak", 5000);

  sim.requestStop();
  QVERIFY(sim.wait(5000));
  SitlModule::instance().api()->set_telemetry_sink(nullptr, nullptr);
}

QTEST_MAIN(TstSitlCrash)
#include "tst_sitl_crash.moc"
