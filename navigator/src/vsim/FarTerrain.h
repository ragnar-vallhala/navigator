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
// FarTerrain.h — a coarse terrain ring beyond the streamed chunk window.
//
// The streamer keeps a square window of full-detail chunks around the vehicle;
// past its edge nothing was drawn, so from altitude the terrain ended in a cut
// silhouette in front of a featureless fog band. FarTerrain samples the same
// infinite TerrainField on a few concentric square rings whose cell size doubles
// with distance (48 / 96 / 192 / 384 m) out to ~6.5 km, so the terrain carries
// on into the fog and dissolves into the horizon.
//
// Qt- and GL-free: it yields interleaved vertices (pos3, normal3, colour3 — the
// layout of the chunk meshes) for the renderer to upload and draw with the same
// lit terrain program, after the near chunks. The ring is anchored to the loaded
// chunk window (rebuilt only when that window moves, i.e. once per chunk
// crossing), starts kOverlapM inside its edge but is clipped inside the window, and its heights
// follow the meshed (slope-capped) near surface at the edge; the fine rings conform to the coarser ones along
// their shared edge, so the ring has no cracks.
#pragma once

#include "procgen/TerrainField.h"

#include <cstdint>
#include <future>
#include <memory>
#include <vector>

namespace vsim {

// Fog, shared by the lit-terrain and flora fragment shaders. True distance (the
// old path weighted the vertical by 0.12, so a high camera saw no fog on the
// ground and a hard edge at the window), thinned exponentially with altitude:
// the optical depth along the ray is the distance times the mean of
// exp(-height / kFogHeightM) between the eye and the fragment, with heights
// measured above the ground under the camera (u_fogbase, NED z of that ground).
// At ground level the mean is 1, so the look there is the original (start 45 m,
// density 1/420). The fog colour stays the horizon colour (see skyColor), which
// is what the sky shows at the horizon, so a fully fogged ring has no seam.
inline constexpr float kFogHeightM = 150.0f;
inline constexpr const char *kFogGLSL = R"GLSL(
uniform float u_fogbase;    // NED z of the ground under the camera
uniform float u_fogheight;  // height-fog scale height [m]
float fogAmount(vec3 cam, vec3 frag, float start, float density) {
  float d = length(frag - cam);
  float k = 1.0 / u_fogheight;
  float h0 = max(-cam.z + u_fogbase, 0.0) * k;
  float h1 = max(-frag.z + u_fogbase, 0.0) * k;
  float avg = abs(h1 - h0) > 1e-3 ? (exp(-h0) - exp(-h1)) / (h1 - h0) : exp(-h0);
  float fd = max(d - start, 0.0) * density * avg;
  return clamp(1.0 - exp(-fd * fd), 0.0, 1.0);
}
)GLSL";

class FarTerrain {
public:
  // Chunk cells currently loaded: inclusive cell bounds, and their size [m].
  struct Window {
    int minCx = 0, maxCx = -1, minCy = 0, maxCy = -1;
    float chunkM = 192.0f;
    bool valid() const { return maxCx >= minCx && maxCy >= minCy; }
    bool operator==(const Window &o) const {
      return minCx == o.minCx && maxCx == o.maxCx && minCy == o.minCy &&
             maxCy == o.maxCy && chunkM == o.chunkM;
    }
  };

  static constexpr float kOverlapM = 48.0f; // ring starts this far inside the window
  static constexpr float kBlendM = 96.0f; // ring heights: meshed surface -> raw field

  // The field to sample (null disables the ring and drops the geometry).
  void setField(std::shared_ptr<const procgen::TerrainField> f);
  bool active() const { return field_ != nullptr; }

  // Starts a background rebuild when the window differs from the last build and
  // swaps in a finished one. True if vertices() changed (so the caller
  // re-uploads); until then the previous ring stays.
  bool update(const Window &w);

  // Interleaved pos(3) normal(3) colour(3), triangle soup, NED metres.
  const std::vector<float> &vertices() const { return verts_; }
  int vertexCount() const { return static_cast<int>(verts_.size() / 9); }

  // The loaded window [x0, y0, x1, y1] (NED xy): the renderer discards the ring
  // inside it, where the chunks are drawn. Empty (x1 <= x0) when unbuilt.
  const float *clipRect() const { return clip_; }

  // NED z of the terrain under (x, y), i.e. the fog reference height; `fallback`
  // when there is no field.
  float groundZ(float x, float y, float fallback) const;

private:
  struct Built {
    std::vector<float> verts;
    float clip[4] = {0, 0, 0, 0};
  };
  static Built build(const procgen::TerrainField &f, const Window &w);
  std::future<Built> pending_;
  Window pendingWin_;
  std::shared_ptr<const procgen::TerrainField> field_;
  Window built_;
  float clip_[4] = {0, 0, 0, 0};
  std::vector<float> verts_;
};

} // namespace vsim
