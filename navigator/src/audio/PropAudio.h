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

#include <array>
#include <atomic>
#include <thread>

// PropAudio — real-time propeller-noise synthesizer driven by the four motor
// angular speeds. Each motor contributes a blade-pass tone (pitch ∝ rpm) over
// a broadband whoosh, with overall loudness ∝ how hard the props are spinning,
// so the sim "spools up" as throttle rises. Runs on its own thread and streams
// S16 mono PCM to PulseAudio.
//
// PulseAudio is wired in only when libpulse-simple is found at configure time
// (HAVE_PULSE_SIMPLE); otherwise this is a silent stub so the build and the UI
// toggle still work. setMotors() is safe to call from the GUI thread.
class PropAudio {
public:
  PropAudio();
  ~PropAudio();

  PropAudio(const PropAudio &) = delete;
  PropAudio &operator=(const PropAudio &) = delete;

  void setEnabled(bool on);
  void setMotors(const std::array<float, 4> &omega_rads);

private:
  void run();

  std::atomic<bool> alive_{false};
  std::atomic<bool> enabled_{false};
  std::atomic<float> omega_[4];
  std::thread thread_;
};
