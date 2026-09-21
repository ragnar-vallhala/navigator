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

#include <QtGlobal> // quint32

// Excitation/rollout parameters shared by the autotune backends (RtosEval).
// Historically this header also declared a SITL-stack-backed rollout
// (runRollout/applyGains); that path — and the vsim_d SitlStack it drove — has
// been removed in favour of the in-process vayu_sitl_rtos backend.
namespace autotune {

// Excitation waveform fired on each axis: a single doublet (step) or a swept
// sinusoid (chirp f0→f1 over `hold`), which probes a band of frequencies.
enum class Excitation { Step, Chirp };

struct RolloutParams {
  int stepUs = 1800;    // excitation amplitude (peak stick µs; 1500 = centre)
  double hold = 1.0;    // s at the step / chirp sweep duration
  double ret = 0.7;     // s settle window after release (chatter shows here)
  double settle = 0.6;  // s after spin-up before exciting
  int hover = 1500;     // hover throttle stick
  double tetherK = 0.0; // >0 = soft rig (estimator-aware)
  quint32 seed = 0;     // deterministic sensor-noise seed
  int maxRetries = 2;
  Excitation excite = Excitation::Step;
  double chirpF0 = 1.0;  // Hz, chirp sweep start
  double chirpF1 = 12.0; // Hz, chirp sweep end
};

} // namespace autotune
