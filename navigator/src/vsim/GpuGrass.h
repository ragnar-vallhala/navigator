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
// GpuGrass.h — GPU-driven grass (Ghost-of-Tsushima style).
//
// A compute shader regenerates grass blade instances around the camera EVERY
// frame: it evaluates the terrain (the procgen noise ported to GLSL), applies
// the same density (slope/altitude) + a distance falloff + frustum culling,
// and appends surviving blades to an SSBO via an atomic counter. An indirect
// draw then renders exactly that many instances — the CPU never touches a blade,
// so there is no scatter cost, no instance storage, and no upload hitches.
//
// Ground cover: the terrain mesh keeps grass off tracks, ponds, rock and bare
// patches (procgen::GroundCover::grass, via TerrainField::grassFactor). The
// shader has no copy of that layer; instead the CPU evaluates the SAME
// grassFactor over a world-anchored tile around the camera (on a worker thread,
// rebuilt only when the camera leaves the tile's safe zone), uploads it as an R8
// texture per ring and the compute pass multiplies the blade density by it. So
// the CPU flora path and this one use one definition of where grass may grow.
//
// Requires GL 4.3+ (compute + SSBO + indirect). init() returns false otherwise
// and the renderer falls back to the CPU flora path.
#pragma once

#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QVector3D>

#include "procgen/Flora.h" // procgen::GrassLook

#include <cstdint>
#include <future>
#include <memory>
#include <vector>

class QOpenGLExtraFunctions;

namespace vsim {

class GpuGrass {
public:
  struct Params {
    // Terrain field — MUST mirror procgen::TerrainField so blades sit on the
    // same surface the mesh is built from.
    uint32_t seed = 1337u;
    float heightM = 70.0f, featureM = 220.0f, macroM = 1400.0f;
    int octaves = 6;
    float lacunarity = 2.0f, gain = 0.5f, mountainMix = 0.55f;
    // Grass placement (mirrors procgen::FloraParams).
    float grassMaxFrac = 0.5f, slopeLo = 0.80f, slopeHi = 0.93f;
    float heightMean = 1.0f, heightStdDev = 0.3f, flowerFrac = 0.02f;
    // Generation grid around the camera.
    float cell = 0.16f; // candidate spacing [m] (smaller = denser)
    int grid = 768;     // candidates per side
    float falloffStart = 55.0f, falloffEnd = 90.0f; // distance density fade [m]
    procgen::GrassLook look; // live shading knobs (drive draw/compute uniforms)
    // The world's full field (palette, shape, colour bands...) for the ground-
    // cover mask. Its seed/height/noise members are overridden by the mirrored
    // values above so the mask always sees the surface the blades sit on.
    procgen::FieldParams field;
    // false: the first mask of each ring is built on the calling thread (tests,
    // deterministic captures). true: built on a worker and that ring draws
    // nothing until it lands, so blades never show up on tracks meanwhile.
    bool asyncMask = true;
  };

  // One blade as the compute pass writes it (3 x vec4).
  struct Blade {
    float x, y, negH, yaw, height, flower, bend, pad0, r, g, b, pad1;
  };
  // Blades the last render() put in `ring`'s buffer (debug readback, stalls the
  // GPU: tests only). Empty if not ready.
  std::vector<Blade> readBlades(QOpenGLExtraFunctions *gl, int ring) const;

  // Where grass may grow at (wx, wy): grassFactor() on the terrain at that point,
  // with the slope taken over +-tm like the GPU's height texture. 0..1.
  static float maskAt(const procgen::TerrainField &f, float wx, float wy,
                      float tm);
  // n x n R8 tile of maskAt() over the square [minX, minX+size)^2, texel (i, j)
  // centred at min + (i+0.5)*size/n (row j = y), the layout the shader samples.
  static std::vector<uint8_t> buildMask(const procgen::FieldParams &field,
                                        float minX, float minY, float size,
                                        int n);

  ~GpuGrass(); // GL objects must have been release()d with a current context
  bool init(QOpenGLExtraFunctions *gl); // false if compute unsupported
  // Frees every GL object (context current). init() may be called again on a
  // new context afterwards; params are kept.
  void release(QOpenGLExtraFunctions *gl);
  // GL objects currently held (for leak checks).
  int liveObjects() const;
  bool ready() const { return ready_; }
  void setParams(const Params &p);

  // Regenerate blades around camPos and draw them. proj/view are the render
  // matrices; sunDir/time feed lighting + wind.
  void render(QOpenGLExtraFunctions *gl, const QMatrix4x4 &proj,
              const QMatrix4x4 &view, const QVector3D &camPos,
              const QVector3D &sunDir, float time,
              const QMatrix4x4 &lightVP = QMatrix4x4(),
              unsigned int shadowTex = 0, bool shadowOn = false);

public:
  // Concentric density rings (dense near .. coarse far) each get their own
  // instance buffer + indirect draw + a lower-LOD blade mesh (fewer segments
  // with distance), so far blades cost a fraction of the near ones.
  static constexpr int kRings = 4;

private:
  void buildBlade(QOpenGLExtraFunctions *gl, int ring, int segments);

  struct MaskTile { // one ring's ground-cover tile (world-anchored)
    unsigned int tex = 0;
    bool valid = false;
    float minX = 0, minY = 0, size = 0, cx = 0, cy = 0;
    unsigned int gen = 0;
    struct Built {
      std::vector<uint8_t> data;
      float minX, minY, size, cx, cy;
    };
    std::future<Built> pending;
    unsigned int pendingGen = 0;
  };
  // Make ring's mask current for camera (cx, cy); false while none exists yet.
  bool updateMask(QOpenGLExtraFunctions *gl, int ring, float cx, float cy,
                  float regionSize, float ringCell);
  procgen::FieldParams maskField() const;

  bool ready_ = false;
  MaskTile mask_[kRings];
  unsigned int maskGen_ = 1; // bumped by setParams: tiles older than it rebuild
  Params params_;
  // Heap-held so release() can drop them and init() build fresh ones in a new
  // context (a linked program cannot take shaders again, nor change context).
  std::unique_ptr<QOpenGLShaderProgram> fill_; // fill the height texture
  std::unique_ptr<QOpenGLShaderProgram> comp_; // generation (samples it)
  std::unique_ptr<QOpenGLShaderProgram> draw_; // render
  unsigned int heightTex_ = 0; // R32F terrain-height image around the camera
  int texSize_ = 512;
  int maskSize_ = 512; // texels per mask tile side
  unsigned int ssbo_[kRings] = {0};     // per-ring blade instances
  unsigned int indirect_[kRings] = {0}; // per-ring DrawArraysIndirectCommand
  unsigned int counter_[kRings] = {0};  // per-ring atomic counter (-> indirect)
  QOpenGLBuffer bladeVbo_[kRings];      // per-ring LOD blade geometry
  QOpenGLVertexArrayObject vao_[kRings];
  int bladeVerts_[kRings] = {0};
  int maxBladesPerRing_ = 0;
};

} // namespace vsim
