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

#include "navlink_msgs.h" // NAVLINK_BATTERY_FLAGS_*

#include <QString>

#include <algorithm>
#include <cmath>
#include <cstdint>

// What the GCS shows for a BATTERY reading (NavLink msgid 1050).
//
// The firmware sends the pack voltage and flags saying whether it can be
// believed; it does not sense the cell count -- the protocol leaves that to the
// consumer. So this decides three things:
//
//  - Is it a pack? Only with both PRESENT and CONVERTED and not RAILED. A
//    disconnected pack does NOT read zero (the divider reads residual charge
//    on the ESC rail), so the voltage alone cannot say.
//  - How many cells? ceil(V / 4.25), latched at the highest count seen while
//    a pack stays present: a 4S pack sagging under load to 3.1 V/cell reads as
//    3S on its own, and the count must not flip mid-flight. It resets when the
//    pack goes away. A fixed count from Settings overrides the inference, for
//    a pack first seen nearly empty (6S at 3.3 V/cell infers as 5S).
//  - What percent? Linear from 3.5 to 4.2 V/cell.
//    ponytail: linear; a LiPo's discharge curve is not, so mid-range reads
//    pessimistic. A per-chemistry lookup table is the upgrade if it matters.
//
// PRESENT is a presence test, not a health test: a dangerously flat pack still
// reports PRESENT, and its per-cell voltage is what says so.
namespace battery {

struct Reading {
  bool pack = false; // a pack is there and the voltage is trustworthy
  QString status;    // why not, when !pack ("no pack", "ADC timeout", ...)
  double volts = 0.0;
  int cells = 0;
  bool cellsInferred = false; // true = from voltage, false = from Settings
  double percent = 0.0;       // 0..100
};

class Estimator {
public:
  static constexpr double kCellFullV = 4.20;
  static constexpr double kCellEmptyV = 3.50;
  // Just above a full cell, so a full pack infers its own count, not one more.
  static constexpr double kCellInferV = 4.25;
  static constexpr int kMaxCells = 12;

  // 0 = infer from voltage (default); 1..kMaxCells = this pack's cell count.
  void setCellsOverride(int cells) {
    override_ = std::clamp(cells, 0, kMaxCells);
  }

  Reading update(float volts, uint8_t flags) {
    Reading r;
    r.volts = volts;
    if (!(flags & NAVLINK_BATTERY_FLAGS_CONVERTED)) {
      r.status = (flags & NAVLINK_BATTERY_FLAGS_INIT_FAIL) ? "ADC init failed"
                 : (flags & NAVLINK_BATTERY_FLAGS_TIMEOUT) ? "ADC timeout"
                                                            : "no reading";
    } else if (flags & NAVLINK_BATTERY_FLAGS_RAILED) {
      r.status = "ADC railed"; // full-scale count: a wiring/reference fault
    } else if (!(flags & NAVLINK_BATTERY_FLAGS_PRESENT)) {
      r.status = "no pack";
    } else {
      r.pack = true;
    }
    if (!r.pack) {
      latched_ = 0; // the pack went away: the next one is inferred afresh
      return r;
    }

    if (override_ > 0) {
      r.cells = override_;
    } else {
      const int inferred =
          std::clamp((int)std::ceil(volts / kCellInferV), 1, kMaxCells);
      latched_ = std::max(latched_, inferred);
      r.cells = latched_;
      r.cellsInferred = true;
    }
    const double perCell = volts / r.cells;
    r.percent = std::clamp(
        (perCell - kCellEmptyV) / (kCellFullV - kCellEmptyV) * 100.0, 0.0,
        100.0);
    return r;
  }

  // The link dropped or went stale: forget the latched count.
  void reset() { latched_ = 0; }

private:
  int override_ = 0;
  int latched_ = 0;
};

} // namespace battery
