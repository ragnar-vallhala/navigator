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
// TreeColliderWindow.cpp -- see TreeColliderWindow.h.
#include "TreeColliderWindow.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <tuple>

namespace vsim::procgen {

namespace {
// vsim_env_cyl_t and vsim_ctl_env_add_t / vsim_ctl_env_commit_t of vsim_proto.h,
// repeated locally (see the header).
struct EnvCylWire {
  float x, y, z, radius, height;
};
static_assert(sizeof(EnvCylWire) == 20, "vsim_env_cyl_t is 20 bytes");
static_assert(offsetof(EnvCylWire, z) == 8 && offsetof(EnvCylWire, height) == 16,
              "vsim_env_cyl_t field order is x, y, z, radius, height");
static_assert(sizeof(float) == 4 && sizeof(std::uint32_t) == 4, "wire widths");
static_assert(4 + kEnvAddMax * sizeof(EnvCylWire) == 244,
              "vsim_ctl_env_add_t is 244 bytes");
static_assert(4 + kEnvAddMax * sizeof(EnvCylWire) <= 256,
              "an ENV_ADD fits the 256-byte ctl body");
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__)
static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__,
              "the engine reads the body in host order; layouts pinned little-endian");
#endif

auto key(const EngineCylinder &c) {
  return std::make_tuple(c.center.x, c.center.y, c.center.z, c.radius, c.height,
                         c.isRock);
}
} // namespace

bool operator==(const EngineCylinder &a, const EngineCylinder &b) {
  return key(a) == key(b);
}

ColliderWindow treeColliderWindow(const std::vector<VegCollider> &colliders,
                                  const PgVec3 &drone, float speedMps,
                                  const ColliderOptions &opts) {
  ColliderWindow out;
  const float margin = std::isfinite(speedMps) ? std::max(0.0f, speedMps) * kColliderLookaheadS : 0.0f;
  const float reach = kColliderWindowRadiusM + margin;
  const float r2 = reach * reach;
  struct Cand {
    EngineCylinder e;
    float d2;
  };
  std::vector<Cand> cand;
  for (const VegCollider &c : colliders) {
    if (c.isRock ? !opts.rocks : !opts.trees)
      continue;
    if (!std::isfinite(c.pos.x) || !std::isfinite(c.pos.y) || !std::isfinite(c.pos.z) ||
        !std::isfinite(c.radius) || !std::isfinite(c.height))
      continue;
    if (!(c.radius > 0.0f) || !(c.height > 0.0f))
      continue;
    const float dx = c.pos.x - drone.x, dy = c.pos.y - drone.y;
    const float d2 = dx * dx + dy * dy;
    if (!(d2 <= r2))
      continue;
    EngineCylinder e;
    e.center = PgVec3{c.pos.x, c.pos.y, c.pos.z - c.height * 0.5f};
    e.radius = c.radius;
    e.height = c.height;
    e.isRock = c.isRock;
    cand.push_back({e, d2});
  }
  if (cand.size() > opts.cap) {
    // Nearest first; ties by the stable key so the cut is deterministic.
    std::sort(cand.begin(), cand.end(), [](const Cand &a, const Cand &b) {
      return a.d2 != b.d2 ? a.d2 < b.d2 : key(a.e) < key(b.e);
    });
    out.dropped = cand.size() - opts.cap;
    cand.resize(opts.cap);
  }
  out.cylinders.reserve(cand.size());
  for (const Cand &c : cand)
    out.cylinders.push_back(c.e);
  std::sort(out.cylinders.begin(), out.cylinders.end(),
            [](const EngineCylinder &a, const EngineCylinder &b) {
              return key(a) < key(b);
            });
  return out;
}

std::vector<std::vector<std::uint8_t>>
encodeEnvAdds(const std::vector<EngineCylinder> &cyl) {
  std::vector<std::vector<std::uint8_t>> out;
  for (std::size_t i = 0; i < cyl.size(); i += kEnvAddMax) {
    const std::uint32_t count =
        static_cast<std::uint32_t>(std::min(kEnvAddMax, cyl.size() - i));
    std::vector<std::uint8_t> m(sizeof count + sizeof(EnvCylWire) * count);
    std::memcpy(m.data(), &count, sizeof count);
    for (std::uint32_t k = 0; k < count; ++k) {
      const EngineCylinder &c = cyl[i + k];
      const EnvCylWire w{c.center.x, c.center.y, c.center.z, c.radius, c.height};
      std::memcpy(m.data() + sizeof count + sizeof w * k, &w, sizeof w);
    }
    out.push_back(std::move(m));
  }
  return out;
}

std::vector<std::uint8_t> encodeEnvCommit(float restitution) {
  if (!std::isfinite(restitution))
    restitution = kColliderRestitution;
  restitution = std::min(1.0f, std::max(0.0f, restitution));
  std::vector<std::uint8_t> m(sizeof restitution);
  std::memcpy(m.data(), &restitution, sizeof restitution);
  return m;
}

bool colliderWindowNeedsRebuild(const ColliderBuildKey &last,
                                const ColliderBuildKey &now) {
  if (!last.valid)
    return true;
  if (last.trees != now.trees || last.rocks != now.rocks || last.seed != now.seed ||
      last.palette != now.palette || last.streamGen != now.streamGen ||
      last.rev != now.rev)
    return true;
  const float dx = now.x - last.x, dy = now.y - last.y;
  if (!std::isfinite(dx) || !std::isfinite(dy))
    return true;
  return dx * dx + dy * dy > kColliderRebuildM * kColliderRebuildM;
}

} // namespace vsim::procgen
