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
// WorldMeshBuilder.h — GCS-side bridge to the shared trimesh BVH builder.
//
// trimesh_bvh.h pulls in vsim_math.h, whose `vsim::Vec3` is a plain struct,
// while the GCS's VsimTypes.h aliases `vsim::Vec3` to QVector3D. The two
// definitions can't coexist in one translation unit, so all trimesh_bvh.h use
// is quarantined in WorldMeshBuilder.cpp (which never includes VsimTypes.h or
// Qt). This header exposes only POD/std types so SimulatorWidget can build a
// serialized BVH blob for an imported world mesh without that clash.
#pragma once

#include <cstdint>
#include <vector>

namespace vsim {

// Build a serialized BVH blob (trimesh_bvh.h format) over a triangle SOUP:
// `verts` holds `nverts` vertices, 3 floats each, in NED world space (import
// transform already baked GCS-side); every consecutive triple of vertices is
// one triangle. Returns the blob bytes (empty on failure) and, via out-params,
// the node count for the ctl-frame cross-check.
std::vector<uint8_t> buildWorldBvh(const float *verts, uint32_t nverts,
                                   bool doubleSided, uint32_t &outNodeCount);

} // namespace vsim
