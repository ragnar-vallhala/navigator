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
// WaterRenderer.h — ponds and lakes as a lit, animated water surface.
//
// Draws the procgen::WaterBody list (WaterField) as one flat polar-grid mesh per
// body at the body's level. The ground under it is the real basin carved into
// the terrain, so the water depth at any vertex is known exactly (the bowl
// profile) and is passed as a vertex attribute; no depth-buffer read is needed.
//
//  * Surface height: a small sum of directional sine waves (a Gerstner-style
//    height field, analytic in time), displaced in the vertex shader and faded
//    out toward the shore so the waterline stays put. The fragment shader takes
//    the normal from the same field plus a finer ripple layer.
//  * Shading: Schlick Fresnel between the reflected sky (Atmosphere skyColor,
//    no cube map) and a depth-tinted body colour, a sun glint on the wave
//    normal, shore foam, soft alpha at the edge, and the renderer's own
//    aerial perspective. No texture files.
//  * Owned by the sim renderer, which has a current GL context whenever a
//    method here that needs one is called. Nothing is drawn (and nothing is
//    allocated on the GPU) until setBodies() gives it bodies and draw() runs.
#pragma once

#include "Atmosphere.h"
#include "procgen/WaterField.h"

#include <QByteArray>
#include <QMatrix4x4>
#include <QOpenGLExtraFunctions>
#include <QVector3D>

#include <cstdint>
#include <memory>
#include <vector>

class QOpenGLShaderProgram;

namespace vsim {

// ---- CPU mesh (Qt-free logic, unit tested) ------------------------------------

constexpr int kWaterRings = 22;
constexpr int kWaterSegments = 56;
constexpr float kWaterRimScale = 1.05f; // mesh reaches this far past the shore

// Vertex: world x, y, z (NED, z = -level), water depth [m] under it, q (distance
// over the local shore radius). Outside the shore depth is 0.
constexpr int kWaterVertexFloats = 5;

struct WaterMesh {
  std::vector<float> vertices; // kWaterVertexFloats per vertex
  std::vector<uint32_t> indices;
  int vertexCount() const { return int(vertices.size() / kWaterVertexFloats); }
};

// Appends one body's polar grid to `m` (indices offset to the existing vertices).
void appendWaterBody(WaterMesh &m, const procgen::WaterBody &b);

// Depth of the carved bowl at normalised distance q from the centre: depthM at
// the centre, 0 at the shore and beyond (the profile WaterField::carve cuts).
inline float waterDepthAtQ(const procgen::WaterBody &b, float q) {
  return q >= 1.0f ? 0.0f : b.depthM * (1.0f - q * q);
}

struct WaterFrame {
  QMatrix4x4 proj, view;
  Atmosphere atmo;
  QVector3D camEye, sunDir;
  float time = 0.0f;
  float sunIntensity = 1.0f, ambientStrength = 1.0f;
  float fogDensity = 1.0f / 420.0f, fogStart = 45.0f;
  float fogBase = 0.0f, fogHeight = 150.0f;
  float wind = 0.6f; // 0 glass .. 1 choppy
  QVector3D waterColor{0.14f, 0.27f, 0.31f}; // palette pond colour
};

// The renderer's shared GLSL, as for VegetationRenderer: atmosphere + fog, and
// the lighting helpers (u_campos, applyAerialPerspective).
struct WaterGlsl {
  QByteArray atmosphere, lighting;
};

class WaterRenderer : protected QOpenGLExtraFunctions {
public:
  WaterRenderer();
  ~WaterRenderer();

  // The bodies to draw (rebuilds the mesh on the next draw).
  void setBodies(std::vector<procgen::WaterBody> bodies);
  bool hasBodies() const { return !bodies_.empty(); }
  int bodyCount() const { return int(bodies_.size()); }

  bool glReady() const { return prog_ != nullptr; }
  bool initGL(const WaterGlsl &g);
  // Draw the surface. Blended, depth-tested, no depth write; call after the
  // opaque scene.
  void draw(const WaterFrame &f);
  // Frees every GL object (context current). Bodies stay; GL rebuilds on demand.
  void releaseGL();
  int liveObjects() const; // GL objects alive (program + VAO + 2 buffers)

  int drawnVertices() const { return lastVertices_; }

private:
  void uploadMesh();
  std::vector<procgen::WaterBody> bodies_;
  bool meshDirty_ = true;
  std::unique_ptr<QOpenGLShaderProgram> prog_;
  unsigned vao_ = 0, vbo_ = 0, ibo_ = 0;
  int indexCount_ = 0;
  int lastVertices_ = 0;
};

} // namespace vsim
