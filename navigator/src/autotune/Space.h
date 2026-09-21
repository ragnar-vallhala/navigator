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

#include <cstdint> // int32_t before <stdlib.h> (glibc quirk)
#include <string>
#include <vector>

#include "Optimizer.h" // Vec, Bounds

// The autotune parameter space — a C++ port of tools/autotune/autotune.py's
// Space (_BASE [+ _YAW]). Maps the optimizer's vector to named gains; the
// bounds reflect the measured plant (rate loop capped below the buzz knee,
// angle_kp wide). Pure — no Qt/sim — so it's unit-testable.
namespace autotune {

struct Param {
  std::string name;
  double lo, hi, seed;
};

class Space {
public:
  // fastRtos: restrict the space to the gains the in-process vayu_sitl_rtos
  // backend can actually set (rate kp/ki/kd, angle_kp [, yaw_rate_kp]) — it has
  // no env hook for gyro_lpf or the yaw ki/kd/lpf, so tuning them there would
  // burn budget on dimensions its cost is blind to. See AutotuneWorker::runRtos.
  explicit Space(bool tuneYaw, bool fastRtos = false);

  bool tuneYaw() const { return m_tuneYaw; }
  int dim() const { return int(m_params.size()); }
  std::vector<std::string> names() const;
  Bounds bounds() const;
  Vec seed() const;

private:
  bool m_tuneYaw;
  std::vector<Param> m_params;
};

} // namespace autotune
