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
// crash status (pose proto v5): the pose read zeroes the struct first, so a
// module with a shorter tail never leaves stale crash bytes; and the banner
// text the viewer shows once the vehicle has crashed.

#include "CrashStatus.h"

#include <QtTest>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>

// A stand-in for a module's get_pose: writes the first min(size, frameSize)
// bytes of `src` and returns the count, exactly as the ABI says.
struct FakeModule {
  vsim_pose_frame_t src{};
  size_t frameSize = sizeof(vsim_pose_frame_t);
  size_t operator()(vsim_pose_frame_t *out, size_t size) const {
    const size_t n = std::min(size, frameSize);
    std::memcpy(out, &src, n);
    return n;
  }
};

class TstCrashStatus : public QObject {
  Q_OBJECT
private slots:
  void headerIsTheCrashProtocol();
  void newModuleFillsTheCrashFields();
  void staleCrashBytesNeverSurviveAShorterModule();
  void returnsTheBytesWritten();
  void onlyBitZeroIsCrashed();
  void bannerTextWhenCrashed();
  void bannerEmptyWhenNotCrashed();
  void bannerHandlesNonsensePeak();
};

void TstCrashStatus::headerIsTheCrashProtocol() {
  // Built against an SDK without the crash tail this would not compile; this
  // pins the numbers the viewer relies on.
  QVERIFY(VSIM_PROTO_VERSION >= 5u);
  QCOMPARE(static_cast<size_t>(offsetof(vsim_pose_frame_t, crash_flags)) +
               sizeof(uint32_t),
           static_cast<size_t>(offsetof(vsim_pose_frame_t, peak_impact)));
  QVERIFY(vsim::crash::kLimitMps > 0.0f);
}

void TstCrashStatus::newModuleFillsTheCrashFields() {
  FakeModule m;
  m.src.crash_flags = VSIM_POSE_CRASHED;
  m.src.peak_impact = 9.3f;
  vsim_pose_frame_t out;
  vsim::crash::readPose(m, out);
  QVERIFY(vsim::crash::isCrashed(out.crash_flags));
  QCOMPARE(out.peak_impact, 9.3f);
}

void TstCrashStatus::staleCrashBytesNeverSurviveAShorterModule() {
  // One buffer, reused like the worker loop does. A new module reports a
  // crash; then the module in use only knows the pre-crash frame (it writes
  // just the prefix). The crash fields must read as zero, not as the last
  // call's values.
  vsim_pose_frame_t out;
  FakeModule fresh;
  fresh.src.crash_flags = VSIM_POSE_CRASHED;
  fresh.src.peak_impact = 12.5f;
  vsim::crash::readPose(fresh, out);
  QVERIFY(vsim::crash::isCrashed(out.crash_flags));

  FakeModule old;
  old.frameSize = offsetof(vsim_pose_frame_t, crash_flags); // no crash tail
  old.src.crash_flags = 0xFFFFFFFFu; // would be visible if it were copied
  old.src.peak_impact = 99.0f;
  vsim::crash::readPose(old, out);
  QCOMPARE(out.crash_flags, 0u);
  QCOMPARE(out.peak_impact, 0.0f);
}

void TstCrashStatus::returnsTheBytesWritten() {
  FakeModule full;
  vsim_pose_frame_t out;
  QCOMPARE(vsim::crash::readPose(full, out), sizeof(vsim_pose_frame_t));
  FakeModule old;
  old.frameSize = sizeof(vsim_pose_frame_t) - 8; // a v4 frame
  QCOMPARE(vsim::crash::readPose(old, out), sizeof(vsim_pose_frame_t) - 8);
}

void TstCrashStatus::onlyBitZeroIsCrashed() {
  QVERIFY(!vsim::crash::isCrashed(0u));
  QVERIFY(vsim::crash::isCrashed(VSIM_POSE_CRASHED));
  QVERIFY(vsim::crash::isCrashed(VSIM_POSE_CRASHED | 0x80u));
  // Reserved bits alone are not a crash.
  QVERIFY(!vsim::crash::isCrashed(0x2u));
  QVERIFY(!vsim::crash::isCrashed(0xFFFFFFFEu));
}

void TstCrashStatus::bannerTextWhenCrashed() {
  QCOMPARE(vsim::crash::bannerText(VSIM_POSE_CRASHED, 9.3f),
           QStringLiteral("CRASHED: impact 9.3 m/s, Reset to fly"));
  // One decimal, rounded.
  QCOMPARE(vsim::crash::bannerText(VSIM_POSE_CRASHED, 8.04f),
           QStringLiteral("CRASHED: impact 8.0 m/s, Reset to fly"));
  QCOMPARE(vsim::crash::bannerText(VSIM_POSE_CRASHED, 14.96f),
           QStringLiteral("CRASHED: impact 15.0 m/s, Reset to fly"));
}

void TstCrashStatus::bannerEmptyWhenNotCrashed() {
  // A hard-but-survivable contact has a peak and no crash: nothing to show.
  QVERIFY(vsim::crash::bannerText(0u, 7.9f).isEmpty());
  QVERIFY(vsim::crash::bannerText(0x2u, 30.0f).isEmpty());
}

void TstCrashStatus::bannerHandlesNonsensePeak() {
  const QString zero = QStringLiteral("CRASHED: impact 0.0 m/s, Reset to fly");
  QCOMPARE(vsim::crash::bannerText(VSIM_POSE_CRASHED, std::nanf("")), zero);
  QCOMPARE(vsim::crash::bannerText(VSIM_POSE_CRASHED, -3.0f), zero);
  QCOMPARE(vsim::crash::bannerText(VSIM_POSE_CRASHED, INFINITY), zero);
}

QTEST_MAIN(TstCrashStatus)
#include "tst_crash_status.moc"
