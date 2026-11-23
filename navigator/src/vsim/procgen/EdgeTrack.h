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
// EdgeTrack: the rule for the tracks that run along field-parcel borders
// (GroundCover.cpp). Qt-free, and the one place its numbers live: the terrain
// shader's GLSL is generated from these constants (TerrainMaterial.cpp), so the
// C++ weight below and the per-pixel one cannot drift apart.
//
// A border track is 1.5-3 m wide, narrower than the 1.92 m mesh cell, so
// baking it into the node colour aliased it into teeth. The mesh carries the
// SIGNED distance to the nearest tracked border line instead (a linear
// function, so a cell interpolates it exactly) and the fragment draws the track
// from that.
#pragma once

#include <algorithm>
#include <cmath>

namespace vsim::procgen {

// The track is full strength within kEdgeTrackD0 metres of the border line and
// gone beyond kEdgeTrackD1.
inline constexpr float kEdgeTrackD0 = 0.9f;
inline constexpr float kEdgeTrackD1 = 2.0f;
// The ground takes at most this much of the track colour: the full dirt colour
// reads as a dark mud ditch against the meadow, and the baked node colour never
// reached it (its peak was undersampled), so the tracks looked like worn grass.
inline constexpr float kEdgeTrackMax = 0.75f;
// Distances are stored clamped to +-kEdgeFar; "no tracked border near" is
// +kEdgeFar for both the signed and the unsigned value.
inline constexpr float kEdgeFar = 8.0f;
// A pixel's interpolated unsigned distance must be below this for the signed
// one to be trusted. The signed distance changes sign half way between two
// parcel borders (far from any track); a cell that straddles that jump would
// interpolate through zero, and the unsigned distance there stays large. A real
// crossing has both nodes within one cell (1.92 m) of the line.
inline constexpr float kEdgeValidMax = 3.0f;

inline float edgeSmooth(float e0, float e1, float x) {
  const float t = std::clamp((x - e0) / (e1 - e0), 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

// Track strength 0..1 from the signed distance `s` and the unsigned one `a`
// (as the shader sees them, interpolated across a cell).
inline float edgeTrackWeight(float s, float a) {
  if (a >= kEdgeValidMax)
    return 0.0f;
  return kEdgeTrackMax * (1.0f - edgeSmooth(kEdgeTrackD0, kEdgeTrackD1, std::fabs(s)));
}

} // namespace vsim::procgen
