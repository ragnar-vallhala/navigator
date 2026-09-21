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

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

// ChimeAudio — one-shot alert chimes (arm / disarm / failsafe) synthesized and
// streamed to PulseAudio, mirroring PropAudio's S16-mono / pa_simple approach.
// Runs on its own worker thread; play() is safe to call from the GUI thread and
// returns immediately (the tone is rendered + written on the worker).
//
// Like PropAudio, the PulseAudio path is compiled in only when libpulse-simple
// is found (HAVE_PULSE_SIMPLE); otherwise this is a silent stub so the build and
// the Settings toggle still work.
class ChimeAudio {
public:
  enum class Kind { Arm, Disarm, Failsafe };

  ChimeAudio();
  ~ChimeAudio();

  ChimeAudio(const ChimeAudio &) = delete;
  ChimeAudio &operator=(const ChimeAudio &) = delete;

  void setEnabled(bool on);
  void play(Kind k); // no-op while disabled

private:
  void run();

  std::atomic<bool> alive_{false};
  std::atomic<bool> enabled_{false};
  std::mutex mtx_;
  std::condition_variable cv_;
  std::deque<Kind> queue_;
  std::thread thread_;
};
