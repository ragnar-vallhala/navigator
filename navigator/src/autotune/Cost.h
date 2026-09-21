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

#include <cstdint> // ensure int32_t is defined before <stdlib.h> (glibc quirk)
#include <optional>
#include <vector>

// Scalar cost for an autotune rollout — a faithful C++ port of the scoring in
// tools/autotune/autotune.py (_axis_cost / _yaw_rate_cost / throttle buzz).
// Pure: a window of telemetry samples in, a cost out, so it's unit-testable
// without the sim. The rollout (driving the SITL stack to produce the samples)
// is the separate, sim-coupled layer.
namespace autotune {

inline constexpr double kBig = 1.0e6; // divergence / failsafe penalty
inline constexpr double kChatterThresh = 0.04;
inline constexpr double kChatterScale = 25.0;
inline constexpr double kBuzzThresh = 0.02;
inline constexpr double kBuzzScale = 150.0;
inline constexpr double kBuzzDiverge = 100.0;

// One control-loop telemetry sample: per-axis (setpoint, measured, output).
// Angle loops for roll/pitch; the yaw RATE loop for yaw.
struct Sample {
  double rollAngleSp = 0, rollAngleCurr = 0, rollOut = 0;
  double pitchAngleSp = 0, pitchAngleCurr = 0, pitchOut = 0;
  double yawRateSp = 0, yawRateCurr = 0, yawOut = 0;
};

// Mean sample-to-sample swing of a signal (the "chatter" metric).
double chatter(const std::vector<double> &xs);

// Angle-loop cost for axis 0=roll / 1=pitch: tracking IAE + overshoot + a
// dead-zoned chatter penalty. Returns nullopt when there are too few samples
// to score (telemetry starved -> the caller retries, not a divergence), and
// kBig on genuine divergence (|angle| > 80°).
std::optional<double> axisCost(const std::vector<Sample> &samples, int axis);

// Yaw RATE-loop cost: rate-tracking error normalised to a fraction of the
// commanded rate and rescaled to the angle-cost magnitude, + overshoot +
// chatter. nullopt / kBig as above (gyro saturation at |rate| > 2000°/s).
std::optional<double> yawRateCost(const std::vector<Sample> &samples);

} // namespace autotune
