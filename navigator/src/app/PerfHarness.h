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

#include <QObject>

class MainWindow;

// Scripted, input-free profiling run. Off unless NAVIGATOR_PROFILE is set.
//
//   NAVIGATOR_PROFILE=<scenario>   home | sim | world | worldchange | long |
//                                  flight
//   NAVIGATOR_PROFILE_OUT=<file>   JSON lines (meta, phase, hitch, sample)
//   NAVIGATOR_PROFILE_TAG=<label>  copied into every line (e.g. gpu / sw)
//   NAVIGATOR_PROFILE_PRESET=<id>  world preset for world / flight
//   NAVIGATOR_PROFILE_GRASS=1      tick the Grass box (needs a grass build)
//   NAVIGATOR_PROFILE_SECS=<n>     measured seconds per phase (default per
//                                  scenario)
//   NAVIGATOR_PROFILE_SIZE=WxH     main window size (default 1600x900)
//
// It drives the real widgets (Start button, World preset combo, RC through the
// bridge) with no user input, times the paints / event loop / threads (see
// core/PerfProbe.h) and quits the app when done. Not for production use.
class PerfHarness : public QObject {
  Q_OBJECT
public:
  // Returns true when the harness took over (env set); the caller then just
  // runs the event loop, the harness quits it when finished.
  static bool maybeStart(MainWindow *w);

private:
  explicit PerfHarness(MainWindow *w);
  struct Impl;
  Impl *d;
};
