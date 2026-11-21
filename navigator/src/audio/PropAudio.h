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
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <thread>

// Decision logic of PropAudio, kept free of threads and PulseAudio so it can
// be unit tested. The synth must go quiet whenever the rpm feed stops, however
// it stops (user Stop, pause, reset, world change, engine death): a held last
// value would otherwise drone on forever.
namespace propaudio {
constexpr int kStaleMs = 300;      // rpm older than this counts as zero
constexpr int kIdleCloseMs = 1500; // silent this long => release the stream
constexpr float kFullOmega = 900.0f; // rad/s mapped to full loudness

// Loudness target in [0,1] from the newest rpm and how old it is.
inline float targetGain(const std::array<float, 4> &w, int64_t ageMs) {
  if (ageMs > kStaleMs)
    return 0.0f;
  float wmax = 0.0f;
  for (float v : w)
    wmax = std::max(wmax, v);
  return std::min(1.0f, wmax / kFullOmega);
}

// True while the output stream should stay open. silentMs is how long the
// target gain has been zero and the smoothed gain has had time to decay.
inline bool streamWanted(bool enabled, int64_t silentMs) {
  return enabled && silentMs < kIdleCloseMs;
}
} // namespace propaudio

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
  // Ramp to silence now and drop the rpm feed (Stop/pause/reset/world change).
  void silence();

private:
  void run();

  std::atomic<bool> alive_{false};
  std::atomic<bool> enabled_{false};
  std::atomic<float> omega_[4];
  std::atomic<int64_t> lastSetMs_{0}; // steady-clock ms of the last setMotors
  std::thread thread_;
};
