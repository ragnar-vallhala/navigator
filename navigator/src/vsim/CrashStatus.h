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
#pragma once

// Crash status of the in-process sim (pose proto v5), kept free of the engine
// and of the GUI so it can be unit tested.
//
// Two things live here:
//   * readPose(): the one correct way to read the pose frame from the module.
//     get_pose(out, size) writes min(size, the module's own frame size) bytes
//     and leaves the rest of `out` alone, so a module older than this build
//     (a shorter tail) would leave stale bytes from the previous call in the
//     crash fields. Zeroing first makes a missing tail read as "no crash".
//   * the text the viewer shows once the vehicle has crashed.

#include "vsim_proto.h"

#include <QString>

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace vsim {
namespace crash {

// The impact speed above which a contact is a crash. Sent to the engine on
// every start (the engine's own default is off, so an app that never sends it
// never crashes). 8 m/s is a hard landing for a small quad.
constexpr float kLimitMps = 8.0f;

// Zero `out`, ask the module to fill it, and return the bytes it wrote.
// `getPose` is `size_t(vsim_pose_frame_t *, size_t)`: the module's get_pose.
template <class GetPose>
inline size_t readPose(GetPose &&getPose, vsim_pose_frame_t &out) {
  std::memset(&out, 0, sizeof out);
  return getPose(&out, sizeof out);
}

inline bool isCrashed(uint32_t crashFlags) {
  return (crashFlags & VSIM_POSE_CRASHED) != 0;
}

// What the viewer shows while crashed. Empty when the vehicle has not crashed.
// A reset clears the engine's latch, so the text tells the user the one thing
// that gets them flying again.
inline QString bannerText(uint32_t crashFlags, float peakImpactMps) {
  if (!isCrashed(crashFlags))
    return QString();
  // A non-finite or negative peak would print "nan"/"-inf" in the middle of a
  // warning, and says nothing about why: show no cause at all.
  if (!(peakImpactMps >= 0.0f && peakImpactMps < 1.0e6f))
    return QStringLiteral("CRASHED, Reset to fly");
  const float peak = peakImpactMps;
  // A speed crash needs a contact faster than the limit, so a crash whose peak
  // is at or under it was the other rule: the airframe was left on its back.
  // Saying "impact 2.4 m/s" for that would read as a crash below the limit.
  if (peak <= kLimitMps)
    return QStringLiteral("CRASHED: left on its back, Reset to fly");
  return QStringLiteral("CRASHED: impact %1 m/s, Reset to fly")
      .arg(static_cast<double>(peak), 0, 'f', 1);
}

} // namespace crash
} // namespace vsim
