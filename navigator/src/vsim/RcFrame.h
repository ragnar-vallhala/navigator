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

#include <cerrno>
#include <cstdio>
#include <string>

// Pure helpers for the RcBridge wire format and failsafe policy (no I/O, no
// threads), so the policy can be unit-tested without a pty or a joystick.
//
// The firmware's host_rc_feeder treats the line "NO_SIGNAL" (what the real
// sim_bridge MCU sends after >100 ms without valid PPM) as RC loss and raises
// is_failsafe; >500 ms of silence does the same. We send the explicit token:
// it is immediate, and works the same through a pty and a Windows pipe.
namespace rcframe {

inline const char *noSignalLine() { return "NO_SIGNAL\n"; }

// True when a joystick read() errno means the device is gone (unplugged:
// ENODEV, or EIO/ENXIO/EBADF on a dead fd) rather than "no event right now".
inline bool deviceLostErrno(int e) {
  return e != EAGAIN && e != EWOULDBLOCK && e != EINTR;
}

// A source that was selected (joystick, or UART with the connect flag set)
// but cannot be opened is a LOST source, not a neutral live link: the bridge
// sends NO_SIGNAL and retries the open.
inline bool sourceLostAtOpen(bool selected, bool opened) {
  return selected && !opened;
}

// Retry timer: counts `wait_ms` down by `step_ms`; when it expires re-arms it
// to `period_ms` and returns true (time to retry the open).
inline bool reopenDue(int &wait_ms, int step_ms, int period_ms) {
  wait_ms -= step_ms;
  if (wait_ms > 0)
    return false;
  wait_ms = period_ms;
  return true;
}

// Arm channel in us. `mapped` is the already-resolved mapped/disabled value.
// Returns -1 while there is no link (no arm may be emitted at all).
//  - link lost / source lost: never arm (no frame is sent).
//  - RC disabled, or no input source (`hasSource` false: nothing selected, or
//    UART selected but not connected): the override is ignored too; the frame
//    stays neutral (arm 1500, throttle low) so neither "RC off" nor "no input
//    device" can ever arm the vehicle.
//  - otherwise: override -1 = mapped, 0 = force disarm, 1 = force arm.
inline int resolveArm(bool noSignal, bool enabled, bool hasSource,
                      int armOverride, int mapped) {
  if (noSignal)
    return -1;
  if (!enabled || !hasSource || armOverride < 0)
    return mapped;
  return armOverride ? 2000 : 1000;
}

// One 20 ms wire frame: "roll,pitch,thr,yaw,arm,ch6\n", or the feeder's
// no-signal line when there is no link.
inline std::string formatFrame(bool noSignal, int roll, int pitch, int thr,
                               int yaw, int arm, int ch6) {
  if (noSignal)
    return noSignalLine();
  char buf[96];
  const int n = std::snprintf(buf, sizeof buf, "%d,%d,%d,%d,%d,%d\n", roll,
                              pitch, thr, yaw, arm, ch6);
  return n > 0 ? std::string(buf, (size_t)n) : std::string();
}

// One line from a UART transmitter bridge: a channel frame (starts with a
// number), the bridge's NO_SIGNAL token (radio off), or anything else.
enum class Line { Frame, NoSignal, Junk };
inline Line classifyLine(const std::string &l) {
  size_t i = 0;
  while (i < l.size() && (l[i] == ' ' || l[i] == '\t'))
    ++i;
  if (l.compare(i, 9, "NO_SIGNAL") == 0)
    return Line::NoSignal;
  if (i < l.size() && ((l[i] >= '0' && l[i] <= '9') || l[i] == '-'))
    return Line::Frame;
  return Line::Junk;
}

} // namespace rcframe
