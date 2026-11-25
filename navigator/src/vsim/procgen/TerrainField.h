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
// TerrainField.h — an infinite, translation-invariant terrain function.
//
// Where TerrainGen builds one finite origin-centred arena, TerrainField is a
// pure function height(worldX, worldY) defined over ALL of space, with no
// centre. It's the basis for endless streaming terrain: a chunk is just this
// field sampled over a square world-coordinate window. Because every chunk
// samples the SAME global field (and normals are finite differences of it),
// adjacent chunks meet seamlessly with no stitching — shared edge coordinates
// produce identical positions, normals, and colors.
//
// Macro composition: a slow noise field decides where mountain ranges sit vs.
// open meadow basins, so terrain stays varied forever instead of repeating one
// bowl. Qt-free and deterministic from one seed.
#pragma once

#include "GroundCover.h"
#include "Noise.h"
#include "ProcgenTypes.h"
#include "WaterField.h"

#include <cstdint>

namespace vsim::procgen {

struct FieldParams {
  uint32_t seed = 1337u;
  float heightM = 70.0f;   // peak (mountain) elevation above ground [m]
  float featureM = 220.0f; // metres of the largest detail feature (broader)
  float macroM = 1400.0f;  // metres of the macro range/basin placement field
  int octaves = 6;         // detail layers (6 is plenty; 7 over-sharpened)
  float lacunarity = 2.0f;
  float gain = 0.5f;
  float mountainMix = 0.55f; // ridged-ridge weight inside mountainous regions
  // Hard slope cap applied when meshing (rise/run; 1.0 = 45 deg). Spikes steeper
  // than this are eroded down so the mesh has no near-vertical faces. >=8
  // disables it.
  float maxSlope = 0.8f;

  // Surface-colour thresholds, as fractions of heightM (0..1). Each is the
  // elevation where that band is reached; colour() blends with fixed widths.
  float colBrownT = 0.60f; // green -> brown by this elevation
  float colRockT = 0.82f;  // brown -> bare rock by this elevation
  float colSnowT = 0.68f;  // snow begins capping above this elevation
  float colSlopeT = 0.65f; // slope steepness (0..1) that exposes rock

  // World-preset extensions (all default to the original behaviour).
  int shape = 0;   // 0 = hills/mountains (macro field), 1 = wind-blown dunes
  int palette = 0; // colour table (GroundPalette): base in TerrainField.cpp,
                   // cover layer in GroundCover.cpp
  // Added to the macro [0,1] field before the mountain threshold: >0 forces
  // hills/mountains everywhere (so a small arena doesn't land in an empty basin).
  float mountainBias = 0.0f;
  // Radius [m] of a level pad blended in around the origin (the spawn) at the
  // elevation the raw field has there; 0 disables. Blends out over 1.5x radius.
  float flatRadiusM = 0.0f;
  // Peak height [m] of small-scale ground relief (hummocks and hollows with a
  // 15..30 m wavelength) added on top of the field; 0 = none (default). Kept at
  // centimetre to a few decimetres so the slope stays gentle; ignored on the
  // spawn pad and for dune worlds.
  float reliefM = 0.0f;
  // Deepest point of the pond basins carved into the ground (WaterField); 0
  // turns water off. Which palettes get ponds is a data table in GroundCover.cpp.
  float waterDepthM = 3.0f;
};

class TerrainField : private HeightSource {
public:
  explicit TerrainField(const FieldParams &p);

  // Elevation [m] above the ground plane at world (wx, wy). Always >= 0. Pond
  // basins are cut in here, so every consumer of the terrain sees them.
  float height(float wx, float wy) const;

  // The ponds and lakes (WaterField) cut into this terrain.
  const WaterField &water() const { return water_; }
  const HeightSource &heightSource() const { return *this; }

  // Outward unit surface normal (NED, up is -Z) from central differences of
  // height() with step `eps` (use the mesh vertex spacing so normals match the
  // facets). Sampling the global field means chunk edges agree automatically.
  PgVec3 normal(float wx, float wy, float eps) const;

  // Meadow color splat for elevation `h` (metres) and `flatness` in [0,1]
  // (1 = level ground, 0 = vertical). Green valley -> olive -> rock.
  PgVec3 color(float h, float flatness) const;

  // color() plus the ground-cover layer (tint, patches, tracks, rock, ponds,
  // fields) at world (wx, wy). Meshing uses this; `grass` in the result is the
  // fraction of the ground that may carry grass there.
  GroundCover::Sample colorAt(float wx, float wy, float h, float flatness) const;
  // As colorAt(), with the parcel-border tracks left out of `color` and given
  // as distances for the shader (GroundCover::sampleSplit, EdgeTrack.h). Used
  // for the streamed chunk mesh; the far ring keeps colorAt().
  GroundCover::Sample colorAtSplit(float wx, float wy, float h, float flatness) const;
  // Grass density multiplier at (wx, wy) (0 on tracks/water/rock/bare patches).
  float grassFactor(float wx, float wy, float h, float flatness) const {
    return colorAt(wx, wy, h, flatness).grass;
  }

  const FieldParams &params() const { return p_; }

private:
  FieldParams p_;
  Noise hills_;
  Noise mtn_;
  Noise macro_;
  Noise warp_;
  Noise relief_;
  GroundCover cover_;
  WaterField water_;
  // Ponds, shore and wet bank over a ground-cover sample (colorAt / colorAtSplit).
  GroundCover::Sample withWater(GroundCover::Sample s, float wx, float wy, float h) const;
  float baseHeight(float wx, float wy) const override; // before the basins
  float rawHeight(float wx, float wy) const; // before the spawn pad
  float duneHeight(float wx, float wy) const;
  float reliefHeight(float wx, float wy) const; // [0, reliefM]
  float withRelief(float h, float relief) const;
  float padHeight_ = 0.0f; // raw elevation at the origin (spawn pad level)
  float duneDir_ = 0.0f;   // dune crest-normal heading [rad], from the seed
  float baseFreq_;  // 1 / featureM
  float macroFreq_; // 1 / macroM
};

// Sample `f` over the square world window of chunk (cx, cy) and triangulate it
// into a triangle soup (NED metres, per-vertex normals + meadow colors).
// Chunk (cx, cy) spans [cx*chunkM, (cx+1)*chunkM] x [cy*chunkM, (cy+1)*chunkM].
// `res` is grid cells per side; vertices on shared edges land on identical world
// coordinates as the neighbour chunk, so meshes tile seamlessly.
ProcMesh meshFieldChunk(const TerrainField &f, int cx, int cy, float chunkM,
                        int res);

// Elevation [m] of the surface meshFieldChunk(f, cx, cy, chunkM, res) presents at
// world (wx, wy): slope cap included, interpolated on the same triangles. Use
// this (not f.height()) for anything that must rest on the mesh the pilot sees
// and the physics collides with: spawn, re-drop, helipad decks. It agrees with
// the mesh to float precision; cost is one small (~18x18) local patch, not a
// chunk. f.height() is the raw field and can sit >1 m from the mesh.
float surfaceHeight(const TerrainField &f, float wx, float wy, float chunkM,
                    int res);

// The surface of one chunk exactly as meshFieldChunk(f, cx, cy, chunkM, res)
// draws it (slope cap included, same triangles), held for a whole chunk so many
// points can rest on it for the price of one patch: vegetation uses it to stand
// instances on what the pilot sees instead of the raw field, which the slope cap
// can leave metres above the mesh. Valid for points within `margin` nodes of the
// chunk (the margin lets a footprint cross the chunk edge).
class SurfacePatch {
public:
  SurfacePatch(const TerrainField &f, int cx, int cy, float chunkM, int res,
               int margin);
  // Surface elevation [m] at world (wx, wy).
  float at(float wx, float wy) const;
  // Lowest surface elevation [m] over the disc of `radius` around (wx, wy):
  // every mesh node inside it and a ring of points on its edge.
  float lowestUnder(float wx, float wy, float radius) const;
  // Same, for a plane through the point tilted by gradient (gx, gy) [rise/run]:
  // the lowest of surface minus plane over the disc, i.e. the height at the
  // point where a plane of that tilt first touches the surface from below.
  float lowestUnderTilted(float wx, float wy, float radius, float gx,
                          float gy) const;
  // Mean surface gradient (rise/run, x then y) across the disc: central
  // differences over its diameter.
  void gradientAcross(float wx, float wy, float radius, float *gx,
                      float *gy) const;

private:
  float node(int i, int j) const;
  std::vector<float> h_;
  float x0_, y0_, step_;
  int res_, margin_, gn_, off_;
};

// A small window of the slope-capped surface around one point, for many samples
// close to it (a pad's footprint ring: 17 of them). surfaceHeight() builds its
// own window for every call; this builds one for all of them, on the same
// lattice (the chunk of the centre), so a sample inside the radius is the same
// number surfaceHeight() gives (to float precision across a chunk edge). A
// sample outside the window falls back to surfaceHeight(). Cheap to make
// (about the cost of two surfaceHeight() calls) and read-only afterwards.
class SurfaceWindow {
public:
  SurfaceWindow(const TerrainField &f, float cx, float cy, float radius,
                float chunkM, int res);
  float at(float wx, float wy) const;

private:
  const TerrainField *f_;
  float chunkM_;
  int res_;
  float x0_ = 0.0f, y0_ = 0.0f, step_ = 1.0f;
  int iLo_ = 0, jLo_ = 0, gn_ = 0, apron_ = 0;
  std::vector<float> h_;
};

// The one chunk lattice every procedural world is streamed on (ChunkStreamer's
// defaults). Anything that must rest on the streamed surface without a streamer
// at hand (preset obstacle layout) samples it via streamSurfaceHeight().
inline constexpr float kStreamChunkM = 192.0f;
inline constexpr int kStreamChunkRes = 100;
inline float streamSurfaceHeight(const TerrainField &f, float wx, float wy) {
  return surfaceHeight(f, wx, wy, kStreamChunkM, kStreamChunkRes);
}

} // namespace vsim::procgen
