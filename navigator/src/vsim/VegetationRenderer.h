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
// VegetationRenderer.h — instanced, textured trees, shrubs, grass and rocks.
//
// Draws the vegetation instances procgen::scatterVegetation places on each
// streamed terrain chunk, using the models of VegetationAssets. The owner (the
// sim renderer, which has a current GL context whenever a method here that
// needs one is called) feeds it per-chunk instance lists and calls draw() once
// per frame after the terrain.
//
//  * One shared GLSL program lights with the sim renderer's own sun, ambient
//    and aerial-perspective functions (the renderer hands over its GLSL
//    snippets and uniforms; nothing is re-invented here) and receives its
//    shadow map. Normal maps use tangents derived from screen-space
//    derivatives, because the meshes carry none.
//  * Per frame, every instance of every chunk that survives the frustum and
//    distance tests is placed in a (variant, LOD) bucket; each non-empty bucket
//    is one instanced draw call per submesh. LODs cross-fade by screen-door
//    dither.
//  * Trees beyond their last mesh LOD turn into camera-facing billboards taken
//    from an atlas baked once at start-up (8 azimuths per tree variant, one
//    FBO pass each). Until a variant is baked, or if the FBO is unusable, its
//    last mesh LOD is used up to a shorter range and then fades out.
//  * Textures upload lazily, per species, the first time a species is seen;
//    the images decode off the GL thread.
//
// Vegetation is independent of the procedural grass: it draws whether or not
// the sim is built with VAYU_SIM_GRASS.
#pragma once

#include "Atmosphere.h"
#include "ShadowCascades.h"
#include "VegetationAssets.h"
#include "VsimTypes.h"
#include "procgen/Vegetation.h"

#include <QByteArray>
#include <QMatrix4x4>
#include <QOpenGLExtraFunctions>
#include <QVector>
#include <QVector3D>

#include <algorithm>
#include <array>
#include <future>
#include <map>
#include <memory>
#include <vector>

class QOpenGLShaderProgram;

namespace vsim {

// The renderer's shared GLSL, appended in this order after the vegetation
// shaders' own declarations: atmosphere (skyColor, u_sundir), lighting
// (u_campos, sunLightColor, ambientLit, applyAerialPerspective), then the shadow
// receive: kCascadeShadowGLSL (csmVisibility) followed by kShadowGLSL.
struct VegGlsl {
  QByteArray atmosphere, shadow, lighting;
};

struct VegFrame {
  QMatrix4x4 proj, view;
  ShadowCascadeSet csm; // this frame's cascades (matrices, tiles, splits)
  Atmosphere atmo;      // sun, sky, ambient and haze colours (uniforms of kAtmosphereGLSL)
  QVector3D camEye, sunDir, dronePos;
  float time = 0.0f;
  float droneRadius = 0.0f;
  bool droneInMap = false; // the drone is a caster in the shadow map: no cylinder
  float sunIntensity = 1.0f, ambientStrength = 1.0f;
  float fogDensity = 1.0f / 420.0f, fogStart = 45.0f;
  float fogBase = 0.0f, fogHeight = 150.0f; // height fog (FarTerrain.h kFogGLSL)
  unsigned shadowTex = 0;
  bool shadowOn = false;
  int defaultFbo = 0;
  int viewportW = 0, viewportH = 0;
  // NED z of the terrain under the drone (the drone's contact blob sits there);
  // droneGroundKnown is false when there is no terrain field to ask.
  float droneGroundZ = 0.0f;
  bool droneGroundKnown = false;
};

// The soft contact blob under every tree trunk, rock and the drone: a darkened,
// feathered disc that grounds the object on the terrain (the sun shadow map is
// too coarse for a trunk's foot, and there is no ground occlusion). One
// instanced draw, multiplied into the framebuffer. Every number that shapes it
// is here.
struct VegContactBlob {
  // Size, as a multiple of the object's own base radius.
  static constexpr float kTrunkRadiusMult = 5.0f; // x the trunk radius (collider radius x scale)
  static constexpr float kRockRadiusMult = 2.0f;  // x the rock's half span
  static constexpr float kMinRadiusM = 0.35f;
  // Darkness at the centre: 0 = none, 1 = black.
  static constexpr float kTreeDarkness = 0.55f;
  static constexpr float kRockDarkness = 0.5f;
  // Falloff: full darkness out to this fraction of the radius, then a smooth
  // fade to nothing at the rim.
  static constexpr float kCoreFraction = 0.15f;
  // Distance fade [m]: full to kFadeStartM, gone at kFadeEndM; blobs beyond are
  // not even sent.
  static constexpr float kFadeStartM = 35.0f;
  static constexpr float kFadeEndM = 70.0f;
  // The disc is flat, kLiftM above the surface at the object's foot.
  static constexpr float kLiftM = 0.02f;
  // Its depth is taken this far [m] toward the eye, so ground that is a little
  // above the disc (a slope, a bump) does not hide it.
  static constexpr float kDepthNudgeM = 1.0f;
  // The drone: radius = drone shadow radius x (1 + height x kDroneSpread),
  // darkness kDroneDarkness / (1 + height / kDroneHalfHeightM), none above
  // kDroneMaxHeightM.
  static constexpr float kDroneDarkness = 0.35f;
  static constexpr float kDroneSpread = 0.12f;
  static constexpr float kDroneHalfHeightM = 6.0f;
  static constexpr float kDroneMaxHeightM = 40.0f;
  static constexpr int kMaxInstances = 6000;
};

// Shadow casters (see VegetationRenderer::drawShadowCasters). Trees and rocks go
// into the shadow cascades whose texel is at most kVegCasterMaxTexelM (a trunk
// is one texel at 0.3 m; coarser maps give noise, and Low is 0.35), as the
// coarsest mesh LOD, whatever the distance: a cascade's tile is reused while the
// camera moves, so what it holds must not depend on the camera.
constexpr float kVegCasterMaxTexelM = 0.3f;
constexpr int kVegCasterLod = 2;
// The casters are pulled this far [m] away from the sun in depth (the drone's
// caster is pulled toward it). A crown seen from the sun's side is drawn from a
// finer LOD than the coarse hull in the map, and the receiver's bias is only
// about 0.5 m in the finest cascade: without this the hull shades the leaves
// next to it (a 20-27% darker crown). The push also lifts the start of a shadow
// off the ground by that much along the sun ray, so with the sun low it opens a
// gap at the trunk's foot: measured on a lone jacaranda at 30 deg the shadow
// starts 0.55 m from the base with no push, 0.9 m at -1 m and 1.8 m at -2 m
// (none above 45 deg). -1 m leaves the crown within 13% (it varies with the
// tree's sway); -2 m would keep it within 7%.
constexpr float kVegCasterPushM = -1.0f;

// Which asset species stand in for each procgen species, and their sizes and
// ranges: the one table to tune the look. Distances are metres from the eye.
struct VegSpeciesInfo {
  const char *id;            // manifest species id
  procgen::VegSpecies cat;   // procgen species it serves
  float targetSize;          // world size at instance scale 1 [m]
  bool bySpan;               // size = largest footprint side (else height)
  float lodEnd[3];           // LOD0 / LOD1 / LOD2 end distances
  float maxDist;             // last distance drawn (impostor or fade end)
  float fallbackMax;         // same, when no impostor is available
  float sway;                // wind bend at the top, fraction of height
  bool impostor;             // gets a baked billboard beyond LOD2
  float barkGain = 1.0f;     // impostor trunk brightness against the mesh trunk (tests measure it)
  float cone = 0.0f;         // 1 for a conifer: the impostor's top tile stays inside a cone's outline
};
const VegSpeciesInfo *vegSpeciesTable(int *count);
// Model units to metres at instance scale 1 for one variant of a species.
float vegUnitScale(const VegSpeciesInfo &info, const VegVariant &v);
// Which of `poolSize` variants an instance draws: a hash of its own scatter
// values (position-derived), so it is the same on every chunking and run.
int vegPickVariant(const procgen::VegInstance &v, int poolSize);

// What to draw at distance d: up to two entries (a cross-fade), lod 0..2 = mesh
// LOD, 3 = impostor. fade encodes the dither test (see the shader): in (0,1]
// keep where fade > threshold; in [2,3] keep where fade-2 < threshold. The
// renderer turns a pair into overlapping layers (4 + s and 2 + s) when it draws.
struct VegLodChoice {
  int n = 0;
  int lod[2] = {0, 0};
  float fade[2] = {1.0f, 1.0f};
};
VegLodChoice vegChooseLod(const VegSpeciesInfo &s, float dist, bool impostorReady);

// Impostor atlas layout: kVegAtlasTiles x kVegAtlasTiles tiles, each tree
// variant baked from kVegAtlasAzimuths directions on the horizon plus one view
// straight down (the tile after the azimuths), kVegAtlasViews tiles in all.
constexpr int kVegAtlasTiles = 10;
constexpr int kVegAtlasAzimuths = 8;
constexpr int kVegAtlasViews = kVegAtlasAzimuths + 1;
// The weight of the top-down tile in the billboard rises smoothly with the
// camera's elevation about the crown: 0 on the horizon, 1 at kVegTopToDeg and
// above. It starts at 0 and finishes early because a real crown
// (horizontal needle cards, round broadleaf) already shows most of its top from
// 30 to 40 degrees up. The top tile keeps texels above kVegTopCoverage (the side
// tiles 0.35): from above a mip-blurred crown is a solid disc and would
// otherwise overshoot the mesh.
constexpr float kVegTopFromDeg = 0.0f;
constexpr float kVegTopToDeg = 55.0f;
constexpr float kVegTopCoverage = 0.6f;
// The top tile comes in from the crown's centre outward: inside a radius that
// grows with the elevation (quad half-widths: a crown fills about 0.7, the tile's
// corner is at 1.41, kVegTopSoft the soft band) it is drawn where the side tiles
// show no foliage, and the side tiles stay under it (so the top tile's holes show
// the crown behind them, not the sky). The radius grows in equal proportions per
// degree (kVegTopGrow shapes it) until the crown is filled at kVegTopFull of the
// ramp, then kVegTopRest more. Inside the disc the side tiles fade out over
// kVegTopKeep0..1 of the ramp (where the top tile covers), and outside it over
// kVegTopOut0..1, so from above no spire or trunk is left standing under the crown.
constexpr float kVegTopReach = 0.8f;
constexpr float kVegTopGrow = 1.8f;
constexpr float kVegTopFull = 0.95f;
constexpr float kVegTopRest = 0.8f;
constexpr float kVegTopSoft = 0.15f;
constexpr float kVegTopOut0 = 0.6f;
constexpr float kVegTopOut1 = 0.85f;
constexpr float kVegTopKeep0 = 0.5f;
constexpr float kVegTopKeep1 = 0.9f;
// From kVegTopDownFromDeg to kVegTopDownToDeg of elevation the top tile slides
// down by kVegTopDown (quad half-widths, times the cosine of the elevation): the
// footprint of a tall crown lies under its middle, and the cone's tip stays above.
constexpr float kVegTopDown = 0.3f;
constexpr float kVegTopDownFromDeg = 30.0f;
constexpr float kVegTopDownToDeg = 45.0f;
// The part of the side tile above kVegTopSpireY0..1 (the quad's own -1..1 up
// axis) is that tip: it keeps its weight, disc or not, until kVegTopSpireOut0..1.
constexpr float kVegTopSpireY0 = 0.0f;
constexpr float kVegTopSpireY1 = 0.35f;
constexpr float kVegTopSpireOut0 = 0.9f;
constexpr float kVegTopSpireOut1 = 1.0f;
// A conifer's outline in the quad's own coordinates (the top tile counts only inside
// it): half-width kVegConeHalfWidth at kVegConeBase of the up axis, narrowing in a
// straight line to nothing at kVegConeTip, softened over kVegConeSoft.
constexpr float kVegConeHalfWidth = 0.68f;
constexpr float kVegConeTip = 0.95f;
constexpr float kVegConeBase = -0.6f;
constexpr float kVegConeSoft = 0.12f;
// Seen from above the top tile is the whole crown, a disc, and the outline would cut
// it to a wedge: the clip's weight is 1 up to kVegConeFadeFromDeg of elevation and
// 0 from kVegConeFadeToDeg on, where the impostor is the unclipped one.
constexpr float kVegConeFadeFromDeg = 45.0f;
constexpr float kVegConeFadeToDeg = 70.0f;

// Vertex shader source of the billboard program, with the atlas layout filled
// in from the constants above (exposed so a test can check it).
QByteArray vegBillboardVertexSource();

// Keep-clear discs for the helipads (pad footprint plus a margin) and the
// world's obstacles (their footprint plus a margin), NED world metres.
std::vector<procgen::VegExclusion>
vegExclusionsFor(const std::vector<QVector3D> &helipads,
                 const QVector<vsim::Obstacle> &obstacles);

class VegetationRenderer : protected QOpenGLExtraFunctions {
public:
  VegetationRenderer();
  ~VegetationRenderer();

  // QSettings "sim/vegetation" (default true), overridden by the environment
  // variable NAV_VEGETATION=0/1.
  static bool enabledByConfig();
  // QSettings "sim/vegetationTrees" (default true), overridden by the
  // environment variable NAV_VEGETATION_TREES=0/1 ("0", "false" and "off"
  // disable them). Trees cost the most (impostor atlas, thousands of
  // instances); switch them off here on a slow GPU.
  static bool treesEnabledByConfig();
  void setTreesEnabled(bool on);
  bool treesEnabled() const { return treesOn_; }

  // Chunk instance lists. CPU only, no GL needed; call on the GUI thread.
  // (cx, cy) is the chunk centre and half its side, world metres.
  void setChunk(qint64 key, std::vector<procgen::VegInstance> instances,
                float cx, float cy, float half);
  void removeChunk(qint64 key);
  void clearChunks();
  // Drops the stored instances that fall inside any of `ex` (a pad or obstacle
  // that became known after a chunk was scattered).
  void exclude(const std::vector<procgen::VegExclusion> &ex);
  size_t instanceCount() const; // stored, over every chunk, trees only when on
  size_t instanceCount(int species) const; // one procgen::VegSpecies
  size_t chunkCount() const { return chunks_.size(); }

  // The GL context is current for all of these.
  bool initGL(const VegGlsl &glsl);
  void releaseGL(); // frees every GL object; CPU data stays
  void draw(const VegFrame &f);
  bool glReady() const { return glReady_; }
  // Shadow pass: draws the trees and rocks whose bounding sphere touches the
  // light window of `lightVP` (world -> light clip, orthographic) as depth,
  // coarsest LOD, with the cut-out alpha test; `push` pulls them toward the sun
  // in clip z. The shadow framebuffer is bound, its viewport and scissor set.
  // `cascade` only labels the statistics. Returns the instances drawn. Not for
  // frame-rate use: the caller redraws a cascade only when its tile is stale.
  int drawShadowCasters(const QMatrix4x4 &lightVP, int cascade, float push);
  // Changes whenever what drawShadowCasters would draw changes (chunks, trees
  // flag, assets or species turning ready, GL rebuilt): the renderer redraws the
  // shadow tiles on a new value.
  unsigned casterVersion() const { return casterVer_; }
  struct CasterStats {
    int instances[3] = {};          // last pass of each cascade
    long long triangles[3] = {};
    int drawCalls[3] = {};
    int passes = 0;                 // drawShadowCasters calls so far
  };
  const CasterStats &casterStats() const { return casterStats_; }
  // A cascade was redrawn without casters: its counts are zero.
  void clearCasterStats(int cascade) {
    const int ci = std::clamp(cascade, 0, 2);
    casterStats_.instances[ci] = 0;
    casterStats_.triangles[ci] = 0;
    casterStats_.drawCalls[ci] = 0;
  }
  // The drone's contact blob, independent of the vegetation flags and assets: needs
  // only a current GL context. Uses f.proj, view, camEye, dronePos, droneRadius
  // and the drone ground fields.
  void drawDroneBlob(const VegFrame &f);
  bool droneBlobDrawn() const { return droneBlobDrawn_; } // by the last drawDroneBlob()
  int liveObjects() const; // GL names currently held

  // True while species are still decoding/uploading or impostors still to
  // bake: keep rendering frames until it turns false.
  bool busy() const;

  struct Stats {
    int instancesDrawn = 0, drawCalls = 0, impostors = 0, chunksDrawn = 0;
    long long triangles = 0;
    int speciesReady = 0, impostorVariants = 0;
    int blobs = 0;         // contact blobs drawn (trees and rocks)
    // Trees drawn this frame per kind (procgen::kVegTreeKinds order), as meshes
    // and as impostors, and the triangles the meshes cost.
    int treeMeshes[procgen::kVegTreeKindCount] = {};
    int treeImpostors[procgen::kVegTreeKindCount] = {};
    long long lodTriangles[3] = {}; // all species, by mesh LOD
  };
  const Stats &stats() const { return stats_; }
  bool impostorsAvailable() const { return atlasOk_; }
  // Tests: read every baked impostor tile back and total its foliage and bark
  // texels (see BakeStats). Costs a readback per baked variant.
  struct BakeStats {
    long long foliagePx = 0, barkPx = 0;
    double foliageHeightSum = 0, barkHeightSum = 0; // 0 foot .. 1 top, summed over texels
    long long topPx = 0;      // covered texels in the top-down tiles
    unsigned long long hash = 0; // of every baked tile (colour and mask), order independent
  };
  // Tests: draw every tree at this LOD (0-2 mesh, 3 impostor), -1 = by distance.
  void setForcedLod(int lod) { forcedLod_ = lod; }
  void setBakeStats(bool on) { bakeStats_ = on; bakeStatsOut_ = BakeStats(); }
  const BakeStats &bakeStats() const { return bakeStatsOut_; }

private:
  struct Rec { // one resolved instance
    float x, y, z, yaw, scale, tint, paint;
    uint16_t variant; // flat variant id
    uint8_t info;     // species table row
  };
  struct Chunk {
    std::vector<procgen::VegInstance> src; // as scattered
    std::vector<Rec> recs;                 // resolved lazily (needs assets)
    bool resolved = false;
    float cx = 0, cy = 0, half = 0, zLo = 0, zHi = 0;
  };
  struct Mat {
    unsigned tex[3] = {0, 0, 0}; // diffuse(+alpha), normal, arm
  };
  enum class SpState { Idle, Decoding, Ready };
  struct SpeciesGpu {
    SpState state = SpState::Idle;
    std::future<void> decode;
    std::vector<Mat> mats;
  };
  struct VariantGpu {
    int species = 0, variant = 0, info = -1;
    float k = 1.0f;       // model unit -> metres at instance scale 1
    float height = 1.0f;  // model units
    float radius = 1.0f;  // bounding sphere about its centre, model units
    float centreY = 0.0f; // model units
    float billH = 1.0f;   // impostor half-extent, model units
    int tile = -1;        // first impostor tile, -1 = not a billboard variant
    bool baked = false;
    unsigned vao[3] = {0, 0, 0}, vbo[3] = {0, 0, 0}, ibo[3] = {0, 0, 0};
    unsigned inst[4] = {0, 0, 0, 0}; // per LOD, and the impostor
    unsigned billVao = 0;
  };

  void resolve(Chunk &c);
  int variantFor(const procgen::VegInstance &v, int *infoOut) const;
  bool ensureSpecies(int si);
  void uploadSpecies(int si);
  void ensureMesh(VariantGpu &v, int lod);
  void bakeImpostors(VariantGpu &v);
  void collectBakeStats(const VariantGpu &v);
  bool buildPrograms(const VegGlsl &glsl);
  bool buildAtlas();
  void drawBucket(VariantGpu &v, int lod, const std::vector<float> &data,
                  const VegFrame &f);
  void drawImpostors(VariantGpu &v, const std::vector<float> &data,
                     const VegFrame &f);
  void setCommonUniforms(QOpenGLShaderProgram &p, const VegFrame &f);
  CascadeUniforms csmLocs_; // shadow uniforms of prog_, resolved once per program
  void freeSpeciesTextures(SpeciesGpu &s);
  unsigned uploadTexture(const QImage &rgba, bool srgb, bool preserveCoverage,
                         float cutoff);

  std::map<qint64, Chunk> chunks_;
  VegAssets assets_;
  std::future<bool> assetsLoad_;
  bool assetsStarted_ = false, assetsReady_ = false, assetsFailed_ = false;
  std::vector<SpeciesGpu> sp_;
  std::vector<VariantGpu> var_;
  std::vector<int> infoOfSpecies_; // species table row per asset species, -1 none
  std::array<std::vector<int>, procgen::kVegSpeciesCount> pool_; // flat variants
  // Tree variants per tree kind (procgen::VegInstance::kind).
  std::array<std::vector<int>, procgen::kVegTreeKindCount> treePool_;
  bool resolvedAssets_ = false;

  std::unique_ptr<QOpenGLShaderProgram> prog_, progBillboard_, progBake_, progBlob_;
  unsigned blobVao_ = 0, blobDisc_ = 0, blobInst_ = 0;
  std::vector<float> blobData_; // per frame: x, y, z, radius, darkness
  void drawBlobs(const VegFrame &f);
  void drawBlobInstances(const VegFrame &f, const float *data, int n);
  bool ensureBlobGl();
  void releaseBlobGL();
  bool blobGlTried_ = false, droneBlobDrawn_ = false;
  // Per-draw uniform locations (the per-frame ones are set by name).
  struct Locs {
    int k = -1, height = -1, sway = -1, cutout = -1, cutoff = -1, a2c = -1;
    int cy = -1, half = -1, tile = -1;
    int bakeMvp = -1;
  } loc_;
  unsigned atlasTex_ = 0, atlasFbo_ = 0, atlasDepth_ = 0, atlasMask_ = 0;
  bool bakeStats_ = false;
  int forcedLod_ = -1;
  unsigned casterVer_ = 1;
  CasterStats casterStats_;
  std::unique_ptr<QOpenGLShaderProgram> progShadow_;
  std::vector<std::vector<float>> casterBuckets_; // per variant, 8 floats an instance
  BakeStats bakeStatsOut_;
  bool atlasOk_ = false, glReady_ = false, atlasTried_ = false, treesOn_ = false;
  bool a2c_ = false; // alpha-to-coverage this frame (multisampled target)
  float maxAniso_ = 1.0f;
  int speciesUploadBudget_ = 0;
  std::vector<std::vector<float>> buckets_; // variant*4 + lod
  Stats stats_;
};

} // namespace vsim
