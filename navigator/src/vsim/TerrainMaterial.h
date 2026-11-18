// TerrainMaterial: photo-textured, splat-blended terrain shading for the lit
// program. Qt-free in this header (<cmath> only) so the splat rule can be unit
// tested; the GL side (GLSL text, texture upload, uniform binding) is TerrainTextures.h /
// TerrainMaterial.cpp.
//
// The terrain mesh still carries one colour per cell from GroundCover. The
// material keeps that colour as the large-scale hue (green fields, brown
// tracks) and adds what the cell colour cannot: the fine brightness detail of
// real grass, rock and dirt photos, a normal-map perturbation of the light, and
// anti-tiling so the repeat is not visible. Which of the three photos shows is
// the splat rule below: rock on slopes, dirt where the cover turns brown,
// grass elsewhere. The GLSL is generated from the constants in this file, so
// the shader and splatWeights() cannot drift apart.
//
// Light and fog come from the shared Atmosphere / fog GLSL of the lit program
// (u_sundir, u_suncol, ambientLight(), fogColor()); the material only changes
// the surface albedo and normal, never the lighting.
#pragma once

#include <algorithm>
#include <cmath>
#include <cctype>
#include <string_view>

namespace vsim::terrain {

// Low is the plain shader of before this file existed; High is the textured
// one. Auto (the setting's default) resolves with defaultQuality().
enum class Quality { Low, High };

// True for a CPU rasteriser (Mesa llvmpipe / softpipe / swrast), given the
// GL_RENDERER string: the textured terrain costs about 4-5x a frame there.
inline bool isSoftwareRenderer(const char *glRenderer) {
  if (!glRenderer)
    return false;
  std::string_view r(glRenderer);
  auto has = [&](std::string_view needle) {
    return std::search(r.begin(), r.end(), needle.begin(), needle.end(),
                       [](char a, char b) {
                         return std::tolower(static_cast<unsigned char>(a)) ==
                                std::tolower(static_cast<unsigned char>(b));
                       }) != r.end();
  };
  return has("llvmpipe") || has("softpipe") || has("swrast");
}

inline Quality defaultQuality(const char *glRenderer) {
  return isSoftwareRenderer(glRenderer) ? Quality::Low : Quality::High;
}

// ---- Splat rule (the numbers the shader is generated from) -----------------
inline constexpr float kRockSlope0 = 0.18f;   // slope = 1 - up.z: rock starts here...
inline constexpr float kRockSlope1 = 0.42f;   // ...and is complete here
inline constexpr float kRockNoise = 1.0f;     // +-0.5 * this of slope noise, 31 m patches
inline constexpr float kDirtEdge0 = -0.02f;   // dirt where cover r - kDirtG*g passes this...
inline constexpr float kDirtEdge1 = 0.10f;    // ...and is complete here
inline constexpr float kDirtG = 0.92f;

struct Splat {
  float grass = 1.0f, rock = 0.0f, dirt = 0.0f; // sum to 1
};

inline float smooth01(float e0, float e1, float x) {
  const float t = std::clamp((x - e0) / (e1 - e0), 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

// slope: 1 - clamp(up.z, 0, 1) of the surface normal (0 flat, 1 vertical);
// coverR, coverG: the red and green of the mesh's cell colour; noise01: a
// smooth 0..1 noise (the shader's 31 m value noise) that wobbles the rock edge.
inline Splat splatWeights(float slope, float coverR, float coverG, float noise01) {
  Splat s;
  s.rock = smooth01(kRockSlope0, kRockSlope1, slope + kRockNoise * (noise01 - 0.5f));
  s.dirt = smooth01(kDirtEdge0, kDirtEdge1, coverR - coverG * kDirtG) * (1.0f - s.rock);
  s.grass = 1.0f - s.rock - s.dirt;
  return s;
}

} // namespace vsim::terrain
