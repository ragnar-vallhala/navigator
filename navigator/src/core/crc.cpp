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
#include "crc.h"

uint32_t CRC32::calculate(const uint8_t *data, uint32_t length) {
  uint32_t crc = 0xFFFFFFFF; // initial value

  // Byte-wise, MSB-first. Matches the STM32 HAL CRC block configured for
  // byte-aligned input on the firmware side.
  for (uint32_t i = 0; i < length; i++) {
    crc ^= (uint32_t)(data[i] << 24);
    for (int j = 0; j < 8; j++) {
      if (crc & 0x80000000) {
        crc = (crc << 1) ^ 0x04C11DB7; // Polynomial
      } else {
        crc = (crc << 1);
      }
    }
  }

  return crc;
}
