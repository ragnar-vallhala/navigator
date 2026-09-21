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
// Noise.h — deterministic, dependency-free 2D coherent noise.
//
// Everything seeds from one 32-bit integer, so a given (seed, params) pair
// reproduces the same field byte-for-byte — the same world must come out on the
// render side and the physics/BVH side. No global RNG, no wall-clock.
#pragma once

#include <cstdint>

namespace vsim::procgen {

// Gradient (Perlin-style) value noise. Cheap, branch-light, good enough for
// terrain. fbm() and ridged() are the two compositions terrain actually uses;
// gradient() is exposed for tests and for callers that want a single octave.
class Noise {
public:
  explicit Noise(uint32_t seed) : seed_(seed) {}

  // Single-octave gradient noise at continuous (x, y). Range ~[-1, 1], smooth
  // (C2) and zero at integer lattice points.
  float gradient(float x, float y) const;

  // Fractal Brownian motion: `octaves` summed gradient layers, each octave at
  // `lacunarity`x the previous frequency and `gain`x the amplitude. Normalised
  // back to ~[-1, 1] regardless of octave count.
  float fbm(float x, float y, int octaves, float lacunarity, float gain) const;

  // Ridged multifractal: per-octave (1 - |gradient|)^2, weighted by the running
  // amplitude. Produces sharp ridges and rounded valleys — the mountain look.
  // Range ~[0, 1].
  float ridged(float x, float y, int octaves, float lacunarity,
               float gain) const;

private:
  // 2D hash -> pseudo-random unit gradient at lattice corner (ix, iy).
  void cornerGradient(int ix, int iy, float &gx, float &gy) const;

  uint32_t seed_;
};

} // namespace vsim::procgen
