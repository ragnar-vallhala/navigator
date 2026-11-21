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
// WaterRipples.h — Qt-free ripple simulation for a pond, driven by prop wash.
//
// A damped 2D wave equation on a square window of kN x kN texels (kCellM metres
// each) anchored to one pond. The CPU steps it at a fixed rate from a time
// accumulator, so the result depends on elapsed time and not on the render
// rate, and two views can call advanceTo() with the same clock without
// double-stepping. Heights are metres measured from the pond's water level.
//
//  * Forcing: each rotor close above the water presses a shallow dimple into
//    the surface and stirs it with deterministic turbulence; both fade with
//    height and vanish above kMaxHeightM.
//  * Absorbing edges: waves are damped hard outside the shore (so they do not
//    climb the bank or bounce back) and in a sponge strip at the window edge.
//  * The window origin is snapped to whole texels, and bindBody() resets the
//    state: a new pond starts flat.
#pragma once

#include "procgen/WaterField.h"

#include <cstdint>
#include <vector>

namespace vsim {

// One rotor's push on the water: world position (x, y), height of the rotor
// above the water level, prop radius, and strength 0..1 (about thrust / max).
struct RippleRotor {
  float x = 0, y = 0, heightM = 0, radiusM = 0.06f, strength = 0;
};

class WaterRipples {
public:
  static constexpr int kN = 256;           // texels per side
  static constexpr float kCellM = 0.35f;   // metres per texel (window ~ 90 m)
  static constexpr float kSpeedMps = 2.0f; // wave speed
  static constexpr float kStepS = 1.0f / 60.0f;
  static constexpr int kMaxSubsteps = 4;   // per advanceTo(), then the clock slips
  static constexpr float kMaxHeightM = 10.0f; // wash has no effect above this

  WaterRipples();

  // Anchor the window on a pond (centre snapped to whole texels), clear it, and
  // build the absorbing mask from the shoreline. A no-op for the same body.
  void bindBody(const procgen::WaterBody &b);
  bool bound() const { return bound_; }
  float centreX() const { return bx_; }
  float centreY() const { return by_; }
  float level() const { return level_; }

  // The rotors in force until the next call (empty clears the forcing).
  void setRotors(const RippleRotor *r, int n);
  // Run fixed steps up to wall/sim time tSec. The first call only sets the clock.
  void advanceTo(double tSec);
  // Run exactly n fixed steps (tests).
  void stepFixed(int n);

  // True while the surface still moves (or is being forced).
  bool active() const { return forced_ || energy_ > 1e-5f; }
  float maxAbsHeight() const { return maxAbs_; }
  uint64_t stepCount() const { return steps_; }

  // Window geometry in world metres, and the height field (row-major, +x along
  // the row, row = y). Heights are metres above the level (negative = dimple).
  float originX() const { return ox_; }
  float originY() const { return oy_; }
  float sizeM() const { return kN * kCellM; }
  const std::vector<float> &heights() const { return h_; }
  float heightAt(float wx, float wy) const; // bilinear, 0 outside the window
  // Half-float copy of the field for a texture upload (R16F).
  void toHalf(std::vector<uint16_t> &out) const;

  static uint16_t floatToHalf(float f);

private:
  void step();
  void applyForcing();
  std::vector<float> h_, v_, damp_;
  float ox_ = 0, oy_ = 0, bx_ = 0, by_ = 0, level_ = 0;
  bool bound_ = false, forced_ = false;
  std::vector<RippleRotor> rotors_;
  double clock_ = 0, acc_ = 0;
  bool clockSet_ = false;
  float energy_ = 0, maxAbs_ = 0;
  uint64_t steps_ = 0;
};

} // namespace vsim
