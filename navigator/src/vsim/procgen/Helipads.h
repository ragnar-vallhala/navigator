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
// Helipads.h — deterministic helipad placement (Qt-free).
//
// One jittered candidate set per coarse global cell, so the same pads appear
// regardless of view history. A pad is accepted on flat ground by the raw
// field's slope at its centre AND, on the meshed (slope-capped) surface, by the
// relief across its whole footprint: the deck is a rigid disc sitting
// kHelipadDeckM above the surface at the centre with a skirt kHelipadSkirtM
// below it, so a footprint that falls away leaves the pad floating and one that
// rises buries it.
#pragma once

#include "ProcgenTypes.h"

#include <functional>
#include <vector>

namespace vsim::procgen {

using HeightFn = std::function<float(float, float)>; // world (x, y) -> elevation [m]

struct HelipadParams {
  float cellM = 70.0f;       // candidate grid cell
  float maxSlope = 0.09f;    // raw-field slope (rise/run) accepted at the centre
  float radiusM = 3.0f;      // pad footprint radius (SimRendererWidget::kHelipadRadiusM)
  // Largest |h - h(centre)| tolerated on the footprint ring (deck 0.9 m above,
  // skirt 1.3 m below the centre surface). <= 0 disables the footprint screen.
  float maxReliefM = 0.6f;
  int ringPoints = 16;
};

// Largest |h(ring point) - h(centre)| over the footprint ring [m].
float helipadRelief(const HeightFn &surface, float x, float y, float radiusM,
                    int ringPoints = 16);

// Pads within "range" of (cx, cy): the home pad at the origin first, then the
// accepted grid pads. Each is (x, y, surface elevation at its centre). "raw" is
// the cheap raw field height (slope screening); `surface` the meshed surface.
std::vector<PgVec3> helipadsAround(const HeightFn &surface, const HeightFn &raw,
                                   float cx, float cy, float range,
                                   const HelipadParams &p = HelipadParams());

} // namespace vsim::procgen
