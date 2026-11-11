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

// Pure logic for the "Custom vehicle, no tuned gains" notice shown at sim Start.
//
// "Custom" = no airframe preset is selected (SimulatorWidget::m_presetId is
// empty or names no shipped preset), so Start sends no gains and the firmware
// keeps its stock ones. "Tuned" = the user pressed Apply Gains with an
// autotuner result for this vehicle. Any change of the vehicle (preset chosen,
// geometry edited) makes that result stale, so the state resets.
// Header-only so it is testable without the simulator widget.

inline bool shouldWarnCustomUntuned(bool isCustomVehicle,
                                    bool autotuneAppliedForVehicle) {
  return isCustomVehicle && !autotuneAppliedForVehicle;
}

class CustomTuneState {
public:
  void onVehicleChanged() { applied_ = false; }
  void onGainsApplied() { applied_ = true; }
  bool applied() const { return applied_; }
  bool shouldWarn(bool isCustomVehicle) const {
    return shouldWarnCustomUntuned(isCustomVehicle, applied_);
  }

private:
  bool applied_ = false;
};
