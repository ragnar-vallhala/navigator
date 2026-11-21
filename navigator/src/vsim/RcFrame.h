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

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <string>
#include <utility>

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

// Joystick axis (-32767..32767) to a channel in microseconds, 1000..2000.
inline int axisToUs(int v, bool invert) {
  if (invert)
    v = -v;
  return std::clamp(1500 + (v * 500) / 32767, 1000, 2000);
}

// The firmware refuses to arm with throttle at or above 1100 us (10 percent).
// Anything within the bottom 5 percent of the range is sent as exactly 1000,
// so a stick deadzone or an imperfect rest position cannot block arming.
constexpr int kThrottleDeadUs = 1050;

// Throttle channel. halfRange is for a stick that rests at centre (a spring-
// loaded gamepad axis): only the upper half of the travel is used, centre and
// below give 1000, full up gives 2000. Otherwise the whole travel is used, as
// for a ratcheted RC throttle that rests fully down.
inline int throttleToUs(int v, bool invert, bool halfRange) {
  if (invert)
    v = -v;
  const int us = halfRange ? 1000 + (std::max(v, 0) * 1000) / 32767
                           : 1500 + (v * 500) / 32767;
  const int c = std::clamp(us, 1000, 2000);
  return c < kThrottleDeadUs ? 1000 : c;
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

// ---- UART CSV reader ------------------------------------------------------

// Longest line the reader keeps. A real frame is ~40 characters (8 channels).
constexpr size_t kMaxLine = 128;

// Result of feeding one byte to LineAssembler.
enum class Feed { None, Line, Dropped };

// Splits a byte stream into lines on '\n' or '\r', however the bytes arrive
// (whole lines, single bytes, any chunking). A line longer than kMaxLine is
// dropped whole (reported once, at its terminator) instead of being truncated
// and parsed, so the reader resyncs on the next newline.
struct LineAssembler {
  std::string buf;
  bool overflow = false;

  Feed push(char c, std::string *line) {
    if (c == '\n' || c == '\r') {
      if (overflow) {
        overflow = false;
        buf.clear();
        return Feed::Dropped;
      }
      if (buf.empty())
        return Feed::None; // blank line, or the '\n' of "\r\n"
      *line = std::move(buf);
      buf.clear();
      return Feed::Line;
    }
    if (overflow)
      return Feed::None;
    if (buf.size() >= kMaxLine) {
      overflow = true;
      buf.clear();
      return Feed::None;
    }
    buf += c;
    return Feed::None;
  }

  void reset() {
    buf.clear();
    overflow = false;
  }
};

// Strict parse of one channel line: 5..8 decimal fields separated by commas
// (spaces allowed around them), each 800..2200 us, nothing else. Returns the
// field count, or 0 if the line is not a whole, well-formed frame. Every valid
// channel is four digits, so a dropped, doubled or shifted byte almost always
// pushes a field out of range: such a frame is rejected, never clamped into a
// wrong stick position (a one-frame arm or throttle glitch).
inline int parseStrictFrame(const std::string &l, int out[8]) {
  size_t i = 0, n = 0;
  auto skipWs = [&] {
    while (i < l.size() && (l[i] == ' ' || l[i] == '\t'))
      ++i;
  };
  while (i < l.size()) {
    skipWs();
    const size_t st = i;
    long v = 0;
    while (i < l.size() && l[i] >= '0' && l[i] <= '9') {
      v = v * 10 + (l[i] - '0');
      if (v > 99999)
        return 0;
      ++i;
    }
    if (i == st || v < 800 || v > 2200 || n >= 8)
      return 0;
    out[n++] = (int)v;
    skipWs();
    if (i < l.size()) {
      if (l[i] != ',')
        return 0;
      ++i;
      if (i == l.size())
        return 0; // trailing comma
    }
  }
  return n >= 5 ? (int)n : 0;
}

// A raw line made safe for a text log: printable ASCII as is, everything else
// as \xNN, and cut at maxLen characters.
inline std::string escapeRaw(const std::string &s, size_t maxLen = 160) {
  std::string o;
  char b[5];
  for (unsigned char c : s) {
    if (o.size() >= maxLen) {
      o += "...";
      break;
    }
    if (c >= 0x20 && c < 0x7f) {
      o += (char)c;
    } else {
      std::snprintf(b, sizeof b, "\\x%02x", c);
      o += b;
    }
  }
  return o;
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
