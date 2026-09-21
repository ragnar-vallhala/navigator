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
#ifndef GCS_MATH_UTILS_H
#define GCS_MATH_UTILS_H

#include <cstdint>

namespace MathUtils {

/**
 * @brief Converts a 16-bit half-precision float (IEEE 754) to a 32-bit float.
 *
 * @param h The 16-bit float to convert.
 * @return float The 32-bit float representation.
 */
inline float float16_to_float32(uint16_t h) {
  uint32_t sign = (h & 0x8000) << 16;
  uint32_t exp = (h & 0x7c00) >> 10;
  uint32_t mant = (h & 0x03ff) << 13;

  if (exp == 0) { // Zero or subnormal
    if (mant == 0) {
      union {
        uint32_t i;
        float f;
      } u;
      u.i = sign;
      return u.f;
    }
    // Subnormal to normal conversion (simplified for typical drone use)
    // This is a rare case for sensor differences
    while (!(mant & 0x00800000)) {
      mant <<= 1;
      exp--;
    }
    exp++;
    mant &= ~0x00800000;
  } else if (exp == 0x1f) { // Inf or NaN
    exp = 0xff;
  } else {
    exp = exp - 15 + 127;
  }

  union {
    uint32_t i;
    float f;
  } u;
  u.i = sign | (exp << 23) | mant;
  return u.f;
}

} // namespace MathUtils

#endif // GCS_MATH_UTILS_H
