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

#include "BatteryEstimator.h"

#include <QtGlobal>

#include <algorithm>

// Warn / critical alerting on the pack's volts per cell, from a
// battery::Reading. Pure state machine (time passed in) so the debounce and
// hysteresis are unit-tested without a window.
//
//  - Only a valid pack (Reading::pack: PRESENT and CONVERTED, not RAILED) is
//    judged. A sample that is not a pack does not clear the alert at once (a
//    flickering PRESENT flag must not re-announce); the level is held for
//    kLostMs and then forgotten.
//  - Going to a worse level needs the condition to hold kEnterMs; going back
//    needs the voltage to be kHystV above the threshold for kExitMs. A sag
//    under throttle therefore does not flap the alert, and a pack hovering on
//    the threshold does not either.
//  - update() reports `changed` once per transition; the caller announces
//    only when `escalated`, so each step up is announced exactly once and
//    recovery is silent.
namespace battery {

enum class AlertLevel { Ok = 0, Warn = 1, Critical = 2 };

class AlertTracker {
public:
  static constexpr qint64 kEnterMs = 2000;
  static constexpr qint64 kExitMs = 4000;
  static constexpr qint64 kLostMs = 5000;
  static constexpr double kHystV = 0.05; // V per cell

  struct Result {
    AlertLevel level = AlertLevel::Ok;
    bool changed = false;
    bool escalated = false; // changed to a worse level: announce
    double perCellV = 0.0;
  };

  // Critical is never above warn: a crossed pair collapses onto warn.
  void setThresholds(double warnV, double critV) {
    warn_ = warnV;
    crit_ = std::min(critV, warnV);
  }
  double warnV() const { return warn_; }
  double critV() const { return crit_; }
  AlertLevel level() const { return level_; }

  Result update(const Reading &r, qint64 nowMs) {
    Result out;
    out.level = level_;
    if (!r.pack || r.cells <= 0) {
      noPack(nowMs);
      out.level = level_;
      return out;
    }
    lostSince_ = -1;
    const double pc = r.volts / r.cells;
    out.perCellV = pc;
    const AlertLevel want = wanted(pc);
    if (want == level_) {
      pending_ = level_;
      return out;
    }
    if (pending_ != want || pendingSince_ < 0) {
      pending_ = want;
      pendingSince_ = nowMs;
    }
    const bool up = want > level_;
    if (nowMs - pendingSince_ >= (up ? kEnterMs : kExitMs)) {
      out.changed = true;
      out.escalated = up;
      level_ = want;
      pending_ = level_;
      pendingSince_ = -1;
    }
    out.level = level_;
    return out;
  }

  // No usable reading (not a pack, or telemetry stale). Held for kLostMs.
  void noPack(qint64 nowMs) {
    pending_ = level_;
    pendingSince_ = -1;
    if (level_ == AlertLevel::Ok)
      return;
    if (lostSince_ < 0)
      lostSince_ = nowMs;
    if (nowMs - lostSince_ >= kLostMs)
      reset();
  }

  void reset() {
    level_ = pending_ = AlertLevel::Ok;
    pendingSince_ = lostSince_ = -1;
  }

private:
  // The level the voltage asks for, with hysteresis relative to the current one.
  AlertLevel wanted(double pc) const {
    if (pc < crit_)
      return AlertLevel::Critical;
    if (level_ == AlertLevel::Critical && pc < crit_ + kHystV)
      return AlertLevel::Critical;
    if (pc < warn_)
      return AlertLevel::Warn;
    if (level_ >= AlertLevel::Warn && pc < warn_ + kHystV)
      return AlertLevel::Warn;
    return AlertLevel::Ok;
  }

  double warn_ = 3.50;
  double crit_ = 3.30;
  AlertLevel level_ = AlertLevel::Ok;
  AlertLevel pending_ = AlertLevel::Ok;
  qint64 pendingSince_ = -1;
  qint64 lostSince_ = -1;
};

} // namespace battery
