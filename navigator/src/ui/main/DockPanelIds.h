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

// Stable panel ids of the dock host, as stored in saved layouts: never rename
// one once shipped. (No docking-library dependency, so MainWindow can use them
// in every build.) `numbers` is part of view3d for now; see HomePanelIds.h.
namespace dockpanel {
inline constexpr const char *kView3d = "view3d"; // attitude (+ numbers)
inline constexpr const char *kImu = "imu";
inline constexpr const char *kLogs = "logs";
inline constexpr const char *kRc = "rc";
inline constexpr const char *kPackets = "packets";
inline constexpr const char *kCalibration = "calibration";
inline constexpr const char *kMotors = "motors";
inline constexpr const char *kControlLoop = "control_loop";
inline constexpr const char *kGyroNotch = "gyro_notch";
inline constexpr const char *kPerf = "perf";
inline constexpr const char *kSim = "sim"; // the whole simulator page
inline constexpr const char *kSettings = "settings";
} // namespace dockpanel
