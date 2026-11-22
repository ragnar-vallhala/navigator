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
// TreeColliderWindow.cpp — see TreeColliderWindow.h.
#include "TreeColliderWindow.h"

#include <algorithm>
#include <cmath>
#include <tuple>

namespace vsim::procgen {

namespace {
auto key(const EngineCylinder &c) {
  return std::make_tuple(c.center.x, c.center.y, c.center.z, c.radius, c.height,
                         c.isRock);
}
} // namespace

bool operator==(const EngineCylinder &a, const EngineCylinder &b) {
  return key(a) == key(b) && a.restitution == b.restitution;
}

std::vector<EngineCylinder>
treeColliderWindow(const std::vector<VegCollider> &colliders, const PgVec3 &drone,
                   float speedMps, float chunkM, bool trees, bool rocks) {
  std::vector<EngineCylinder> out;
  if (!(chunkM > 0.0f))
    return out;
  const float margin = std::max(0.0f, speedMps) * kColliderLookaheadS;
  const float cx = std::floor(drone.x / chunkM);
  const float cy = std::floor(drone.y / chunkM);
  const float x0 = (cx - 1.0f) * chunkM - margin;
  const float x1 = (cx + 2.0f) * chunkM + margin;
  const float y0 = (cy - 1.0f) * chunkM - margin;
  const float y1 = (cy + 2.0f) * chunkM + margin;
  for (const VegCollider &c : colliders) {
    if (c.isRock ? !rocks : !trees)
      continue;
    if (!(c.radius > 0.0f) || !(c.height > 0.0f))
      continue;
    if (c.pos.x < x0 || c.pos.x >= x1 || c.pos.y < y0 || c.pos.y >= y1)
      continue;
    EngineCylinder e;
    e.center = PgVec3{c.pos.x, c.pos.y, c.pos.z - c.height * 0.5f};
    e.radius = c.radius;
    e.height = c.height;
    e.isRock = c.isRock;
    out.push_back(e);
  }
  std::sort(out.begin(), out.end(), [](const EngineCylinder &a, const EngineCylinder &b) {
    return key(a) < key(b);
  });
  return out;
}

} // namespace vsim::procgen
