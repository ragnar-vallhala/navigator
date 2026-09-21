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

#include <QMetaType>

// The active telemetry source — the single authority over which feed drives the
// GCS (navigator/docs/roadmap/gcs-source-state-machine.md). Exactly one is active
// at a time (strict single source), plus Idle for "no source". The transition
// graph is fully connected: any state can move to any other, and each move tears
// down the current source before setting up the new one.
//
//   Idle     - app start / disconnected; no feed.
//   Fc       - live serial/UDP link to the flight controller.
//   Sim      - in-app SITL (interactive).
//   Autotune - isolated tuner run (its own SITL; does not feed the GCS engine).
//   Replay   - a recorded .bin played through the parser.
enum class SourceState { Idle, Fc, Sim, Autotune, Replay };

Q_DECLARE_METATYPE(SourceState)
