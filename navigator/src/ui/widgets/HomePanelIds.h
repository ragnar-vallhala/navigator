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

// Stable panel ids for the home page pieces. These are the keys the dock
// PanelRegistry / saved layouts will use once the home page is docked; they
// must not change once shipped.
namespace homepanel {
inline constexpr const char *kView3d = "view3d";
inline constexpr const char *kImu = "imu";
inline constexpr const char *kNumbers = "numbers";
inline constexpr const char *kLogs = "logs";
} // namespace homepanel
