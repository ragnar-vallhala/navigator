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
#include "VegetationRenderer.h"

#include <QByteArray>
#include <QOpenGLContext>
#include <QOpenGLShaderProgram>
#include <QSettings>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace vsim {

namespace {

constexpr int kAtlasTiles = kVegAtlasTiles;
constexpr int kTuftSize = 7;      // grass pieces per grass instance
constexpr int kAtlasAzimuths = kVegAtlasAzimuths;
constexpr int kAtlasViews = kVegAtlasViews;
constexpr int kTilePx = 128;
constexpr float kPi = 3.14159265358979f;
constexpr float kSinkM = 0.06f;    // instances stand this far into the surface
constexpr float kLodBand = 0.07f;  // cross-fade half width, fraction of the edge

// One row per asset species. lodEnd: LOD0 / LOD1 / LOD2 end; maxDist: the
// farthest the species is drawn (the billboard range for trees, the fade-out
// distance otherwise). Fog is nearly opaque by ~500 m, so nothing goes farther.
const VegSpeciesInfo kTable[] = {
    // id                       cat            size  span   lod ends         max   fb    sway  imp
    {"fir_tree_01",             procgen::kVegTree,  procgen::kVegTreeKinds[0].heightM, false, {30, 70, 130}, 560, 300, 0.012f, true, 0.6f},
    {"pine_sapling_medium",     procgen::kVegTree,  procgen::kVegTreeKinds[1].heightM, false, {30, 70, 130}, 560, 300, 0.012f, true, 0.75f},
    {"jacaranda_tree",          procgen::kVegTree,  procgen::kVegTreeKinds[2].heightM, false, {30, 70, 130}, 560, 300, 0.008f, true, 1.7f},
    {"island_tree_01",          procgen::kVegTree,  procgen::kVegTreeKinds[3].heightM, false, {30, 70, 130}, 560, 300, 0.008f, true, 1.95f},
    {"shrub_03",                procgen::kVegBush,  1.0f, false, {25, 60, 110}, 170, 170, 0.030f, false},
    {"shrub_04",                procgen::kVegBush,  1.3f, true,  {25, 60, 110}, 170, 170, 0.030f, false},
    {"weed_plant_02",           procgen::kVegBush,  0.8f, true,  {20, 45, 80},  110, 110, 0.050f, false},
    {"grass_bermuda_01",        procgen::kVegGrassClump, 0.40f, false, {15, 35, 60}, 85, 85, 0.120f, false},
    {"rock_09",                 procgen::kVegRock,  procgen::kVegRockSpanM, true,  {40, 90, 150}, 260, 260, 0.0f, false},
    {"rock_07",                 procgen::kVegRock,  procgen::kVegRockSpanM, true,  {40, 90, 150}, 260, 260, 0.0f, false},
    {"namaqualand_boulders_01", procgen::kVegRock,  procgen::kVegRockSpanM, true,  {40, 90, 150}, 260, 260, 0.0f, false},
};
constexpr int kTableN = int(sizeof(kTable) / sizeof(kTable[0]));

// ---------------------------------------------------------------------------
// GLSL. The sim renderer's atmosphere / shadow / lighting snippets are
// appended to the fragment shaders' heads (see VegGlsl).
// ---------------------------------------------------------------------------

// glTF (x right, y up, z toward viewer) -> NED, a proper rotation:
// n = -z, e = x, d = -y.
const char *kCommonGLSL = R"GLSL(
vec3 gltfToNed(vec3 g) { return vec3(-g.z, g.x, -g.y); }
// Per-instance look from the base position alone (no data to ship, same for
// every chunking): x = mirror, y = width, z = lean azimuth, w = lean amount.
vec4 instVary(vec2 p) {
  float a = fract(sin(dot(p, vec2(12.9898, 78.233))) * 43758.5453);
  float b = fract(a * 37.719 + 0.31);
  float c = fract(b * 53.17 + 0.7);
  float d = fract(c * 29.41 + 0.13);
  return vec4(a > 0.5 ? -1.0 : 1.0, mix(0.80, 1.22, b), c * 6.2831853, d * 0.07);
}
vec3 tintMul(float t) { return mix(vec3(0.86, 0.90, 0.82), vec3(1.10, 1.06, 0.96), t); }
// Crown colour per tree: value and hue vary independently (t is a hash of the
// tree's cell, so a tree keeps its colour). Hue runs from a yellower green to a
// bluer one; value from dim to bright.
vec3 foliageTint(float t) {
  float h = fract(t * 7.31 + 0.17);
  return mix(vec3(1.10, 1.04, 0.78), vec3(0.86, 1.00, 1.14), h) * mix(0.76, 1.20, t);
}
vec3 srgbEnc(vec3 c) {
  vec3 hi = 1.055 * pow(max(c, vec3(0.0)), vec3(1.0 / 2.4)) - 0.055;
  return mix(c * 12.92, hi, step(vec3(0.0031308), c));
}
)GLSL";

const char *kVS = R"GLSL(
#version 330 core
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec3 a_nrm;
layout(location = 2) in vec2 a_uv;
layout(location = 3) in vec4 i_posyaw;  // base (NED), yaw (+ 8 x tilt code, rocks)
layout(location = 4) in vec4 i_misc;    // scale, tint, fade, grass paint (packed RGB hue)
uniform mat4 u_vp;
uniform float u_k;       // model unit -> metres at instance scale 1
uniform float u_height;  // variant height, model units
uniform float u_sway;    // wind bend at the top, fraction of the height
uniform float u_time;
out vec3 v_world;
out vec3 v_nrm;
out vec2 v_uv;
out float v_tint;
out float v_fade;
out float v_hm;          // metres above the instance base
flat out float v_paint;
)GLSL";

const char *kVSMain = R"GLSL(
// Tilts a model-frame offset d so the up axis (0,0,-1) (NED) lands on u1.
vec3 tiltTo(vec3 d, vec3 u1) {
  vec3 v = cross(vec3(0.0, 0.0, -1.0), u1);
  return d + cross(v, d) + cross(v, cross(v, d)) / (1.0 - u1.z);
}

void main() {
  float s = i_misc.x * u_k;
  // Rocks carry their base tilt in the whole part of the yaw channel: yaw +
  // 8 * (codeX + 64 * codeY), 0 = level (see vegTiltCode in procgen/Vegetation.h).
  float tq = floor(i_posyaw.w * 0.125);
  float yaw = i_posyaw.w - 8.0 * tq;
  float cy = cos(yaw), sy = sin(yaw);
  mat2 R = mat2(cy, sy, -sy, cy);
  vec3 g = a_pos, gn = a_nrm;
  // Trees (the 0-0.02 sway band; shrubs, grass and rocks sway more or not at all)
  // vary per instance: mirror, width and a slight lean break up the rows.
  if (u_sway > 0.0 && u_sway < 0.02) {
    vec4 vv = instVary(i_posyaw.xy);
    g.x *= vv.x; gn.x *= vv.x;
    g.xz *= vv.y;
    g.xz += vec2(cos(vv.z), sin(vv.z)) * (g.y * vv.w);
  }
  vec3 p = gltfToNed(g) * s;
  vec3 n = gltfToNed(gn);
  p.xy = R * p.xy;
  n.xy = R * n.xy;
  if (tq > 0.5) {
    const float kTiltMax = 0.58; // = procgen::kVegTiltMax
    vec2 tg = (vec2(mod(tq, 64.0), floor(tq / 64.0)) - 32.0) * (kTiltMax / 31.0);
    vec3 u1 = normalize(vec3(-tg, -1.0));
    p = tiltTo(p, u1);
    n = tiltTo(n, u1);
  }
  float h = clamp(a_pos.y / max(u_height, 1e-4), 0.0, 1.0);
  float phase = i_posyaw.x * 0.37 + i_posyaw.y * 0.23;
  float gust = sin(u_time * 1.7 + phase) + 0.5 * sin(u_time * 3.1 + phase * 1.7);
  p.xy += vec2(0.8, 0.6) * (h * h * u_sway * u_height * s * gust);
  v_world = i_posyaw.xyz + p;
  v_nrm = n;
  v_uv = a_uv;
  v_tint = i_misc.y;
  v_fade = i_misc.z;
  v_paint = i_misc.w;
  v_hm = g.y * s;
  gl_Position = u_vp * vec4(v_world, 1.0);
}
)GLSL";

const char *kFSHead = R"GLSL(
#version 330 core
in vec3 v_world;
in vec3 v_nrm;
in vec2 v_uv;
in float v_tint;
in float v_fade;
in float v_hm;
flat in float v_paint;
out vec4 o_color;
uniform sampler2D u_diffuse;  // sRGB colour, linear alpha coverage
uniform sampler2D u_normal;   // linear, tangent space, +Y up
uniform sampler2D u_arm;      // linear: R = ambient occlusion, G = roughness
uniform float u_cutout;
uniform float u_cutoff;
uniform float u_a2c;      // 1: alpha-to-coverage (multisampled target)
uniform float u_sway;
)GLSL";

const char *kFSMain = R"GLSL(
float bayer4(vec2 p) {
  const float m[16] = float[16](0.0, 8.0, 2.0, 10.0, 12.0, 4.0, 14.0, 6.0,
                                3.0, 11.0, 1.0, 9.0, 15.0, 7.0, 13.0, 5.0);
  ivec2 i = ivec2(mod(p, 4.0));
  return (m[i.x + i.y * 4] + 0.5) / 16.0;
}
// Tangent frame from screen-space derivatives (the meshes carry no tangents).
// The images are unflipped (v grows downwards) while the normal map is +Y up,
// so the bitangent points against v.
mat3 cotangentFrame(vec3 N, vec3 p, vec2 uv) {
  vec3 dp1 = dFdx(p), dp2 = dFdy(p);
  vec2 duv1 = dFdx(uv), duv2 = dFdy(uv);
  vec3 dp2perp = cross(dp2, N), dp1perp = cross(N, dp1);
  vec3 T = dp2perp * duv1.x + dp1perp * duv2.x;
  vec3 B = -(dp2perp * duv1.y + dp1perp * duv2.y);
  float invmax = inversesqrt(max(max(dot(T, T), dot(B, B)), 1e-20));
  return mat3(T * invmax, B * invmax, N);
}
void main() {
  vec3 toFrag = v_world - u_campos;
  float dist = length(toFrag);
  vec3 sun = normalize(u_sundir);
  vec3 N = normalize(v_nrm);
  if (!gl_FrontFacing) N = -N;                 // leaves are two-sided
  // Terrain shadow first: it takes derivatives, which must not follow a discard.
  // Foliage is thin and two-sided: no receiver-plane bias (usePlane=false), the
  // lookup is offset along the shading normal.
  float sh = csmVisibility(v_world, N, sun, false) * droneShadow(v_world, sun);
  mat3 TBN = cotangentFrame(N, v_world, v_uv);
  vec4 d = texture(u_diffuse, v_uv);
  // Leaf edge: with a multisampled target the cut-out becomes coverage (no
  // aliased speckle on crowns); otherwise a plain alpha test.
  float cov = 1.0;
  if (u_cutout > 0.5) {
    if (u_a2c > 0.5) {
      cov = clamp((d.a - u_cutoff) / max(fwidth(d.a), 1e-3) + 0.5, 0.0, 1.0);
      if (cov <= 0.0) discard;
    } else if (d.a < u_cutoff) {
      discard;
    }
  }
  // Cross-fade between LODs by screen-door dither: fade in (0,1] keeps pixels
  // above the threshold, fade in [2,3] keeps those below it (the partner).
  float b = bayer4(gl_FragCoord.xy);
  bool keep = v_fade > 1.5 ? (v_fade - 2.0) < b : v_fade > b;
  if (!keep) discard;
  vec3 nm = texture(u_normal, v_uv).xyz * 2.0 - 1.0;
  vec3 Nm = normalize(TBN * nm);
  Nm = normalize(mix(N, Nm, 0.85));
  float ao = texture(u_arm, v_uv).r;
  // Albedo is stored sRGB and sampled linear (correct mip filtering); the sim's
  // lighting works on display-referred colour, so encode back before applying
  // the same sun + ambient terms the terrain uses.
  vec3 base = srgbEnc(d.rgb) * (u_cutout > 0.5 && v_paint < 0.5 ? foliageTint(v_tint) : tintMul(v_tint));
  if (v_paint > 0.5) {
    // Grass: keep the blade's own brightness, move its hue to the ground's.
    int pk = int(v_paint + 0.5);
    vec3 hue = vec3(float((pk >> 16) & 255), float((pk >> 8) & 255), float(pk & 255)) / 255.0;
    float bl = dot(base, vec3(0.299, 0.587, 0.114));
    float hl = max(dot(hue, vec3(0.299, 0.587, 0.114)), 0.05);
    base = mix(base, hue * (bl / hl), 0.75);
  }
  vec3 L;
  if (u_cutout > 0.5) {
    // Foliage: shaded as a rounded crown (normals leaned toward the sky), with
    // a floor of ambient light and the sun shining through thin leaves, so a
    // tree seen from its shaded side is dark green, not black.
    vec3 Nf = normalize(mix(Nm, vec3(0.0, 0.0, -1.0), 0.45));
    float thru = clamp(-dot(N, sun), 0.0, 1.0);
    L = ambientLit(Nf) * (0.55 + 0.45 * ao) * 1.5 +
        sunLightColor() * (wrapDiffuse(Nf, sun) * 0.95 + thru * 0.45) * sh;
  } else {
    L = ambientLit(Nm) * ao * 1.25 + sunLightColor() * wrapDiffuse(Nm, sun) * 0.95 * sh;
    // Poly Haven bark albedo averages 0.2-0.3 linear; with the AO term it read
    // as black. Trunks (the swaying species) get a lift and an AO floor.
    if (u_sway > 0.0) {
      // The island tree's bark has grey lichen patches; under the blue sky
      // ambient they went pale blue-white. Warm the bark and keep the AO floor.
      base *= vec3(1.0, 0.90, 0.78);
      // The root flare is textured with pale sand and lichen that does not
      // match the ground it sits in: blend it into soil over the first 40 cm.
      base = mix(vec3(0.10, 0.075, 0.05), base, smoothstep(0.0, 0.4, v_hm) * 0.6 + 0.4);
      L = ambientLit(Nm) * (0.6 + 0.4 * ao) * 1.3 + sunLightColor() * wrapDiffuse(Nm, sun) * 1.0 * sh;
    }
  }
  o_color = vec4(applyAerialPerspective(base * L, toFrag, dist), cov);
}
)GLSL";

// Billboards: one camera-facing quad per instance (spherical about the crown
// centre, so it is vertical from the side and tilts to face a camera above the
// tree), sampled from the baked atlas. From the side: the two nearest azimuth
// tiles. From above: the top-down tile, blended in by the camera elevation.
const char *kBillVS = R"GLSL(
#version 330 core
layout(location = 3) in vec4 i_posyaw;  // base (NED), plain yaw: only trees are billboarded, never a tilted rock
layout(location = 4) in vec4 i_misc;
uniform mat4 u_vp;
uniform vec3 u_eye;
uniform float u_k;
uniform float u_cy;      // quad centre height, model units
uniform float u_half;    // quad half size, model units
uniform float u_tile;    // first atlas tile of this variant
out vec2 v_uv0;
out vec2 v_uv1;
out vec2 v_uvT;          // the top-down tile
out vec2 v_tc;           // position in the top tile, -1..1 (outside the tile: nothing there)
out float v_w;
flat out float v_top;    // weight of the top-down tile
out vec2 v_el;           // sin, cos of the camera elevation about the crown centre
out float v_fade;
out float v_tint;
out vec3 v_world;
// Per-instance look from the base position alone (no data to ship, same for
// every chunking): x = mirror, y = width, z = lean azimuth, w = lean amount.
vec4 instVary(vec2 p) {
  float a = fract(sin(dot(p, vec2(12.9898, 78.233))) * 43758.5453);
  float b = fract(a * 37.719 + 0.31);
  float c = fract(b * 53.17 + 0.7);
  float d = fract(c * 29.41 + 0.13);
  return vec4(a > 0.5 ? -1.0 : 1.0, mix(0.80, 1.22, b), c * 6.2831853, d * 0.07);
}
vec2 tileUv(float tile, vec2 q) {
  float col = mod(tile, @ATLAS_TILES@), row = floor(tile / @ATLAS_TILES@);
  return (vec2(col, row) + q) / @ATLAS_TILES@;
}
void main() {
  vec2 c = vec2(float(gl_VertexID & 1), float(gl_VertexID >> 1)) * 2.0 - 1.0;
  float s = i_misc.x * u_k;
  vec3 centre = i_posyaw.xyz + vec3(0.0, 0.0, -u_cy * s);
  vec3 te = u_eye - centre;
  float hlen = length(te.xy);
  // Straight overhead has no heading: any fixed one keeps the quad and the top
  // tile's rotation finite (and they agree, both come from h).
  vec2 h = hlen > 1e-4 ? te.xy / hlen : vec2(1.0, 0.0);
  float sinE = clamp(-te.z / max(length(te), 1e-4), 0.0, 1.0); // an eye below the crown: the horizon view
  float cosE = sqrt(1.0 - sinE * sinE);
  vec3 dir = vec3(h * cosE, -sinE);          // to the eye, NED
  vec3 rgt3 = vec3(h.y, -h.x, 0.0);
  vec3 upq = cross(dir, rgt3);               // (0,0,-1) at the horizon, -h overhead
  vec4 vv = instVary(i_posyaw.xy);
  vec3 w = centre + rgt3 * (c.x * u_half * s * vv.y) + upq * (c.y * u_half * s);
  float cyw = cos(i_posyaw.w), syw = sin(i_posyaw.w);
  vec2 hl = vec2(cyw * h.x + syw * h.y, -syw * h.x + cyw * h.y); // into model frame
  hl.y *= vv.x; // a mirrored tree seen from here is the baked one seen from the mirror azimuth
  float f = atan(hl.y, -hl.x) / 6.28318530718 * @ATLAS_AZ@;
  f = f - @ATLAS_AZ@ * floor(f / @ATLAS_AZ@);
  float i0 = floor(f);
  v_w = f - i0;
  float i1 = mod(i0 + 1.0, @ATLAS_AZ@);
  vec2 q = (c * 0.5 + 0.5) * 0.96 + 0.02;
  if (vv.x < 0.0) q.x = 1.0 - q.x;
  v_uv0 = tileUv(u_tile + i0, q);
  v_uv1 = tileUv(u_tile + i1, q);
  // The top tile is the view from straight above with the image's up toward model
  // north (the side tile at azimuth 0 tilted over). Screen right/up of this quad
  // are the horizontal offsets (hl.y, -hl.x) and, away from the eye, -hl; the
  // ground offset along the up axis is stretched by 1/sqrt(sinE): a crown seen from
  // 45 degrees is only partly flattened, its height still shows.
  float e = asin(sinE);
  v_top = smoothstep(@TOP_FROM@, @TOP_TO@, e);
  float cxe = vv.x * c.x;
  float cyv = c.y / max(sqrt(sinE), 0.6);
  v_tc = vec2(-hl.x * cxe - hl.y * cyv, hl.y * cxe - hl.x * cyv);
  v_uvT = tileUv(u_tile + @ATLAS_AZ@, 0.5 + 0.48 * v_tc);
  v_el = vec2(sinE, cosE);
  v_fade = i_misc.z;
  v_tint = i_misc.y;
  v_world = w;
  gl_Position = u_vp * vec4(w, 1.0);
}
)GLSL";

const char *kBillFSHead = R"GLSL(
#version 330 core
in vec2 v_uv0;
in vec2 v_uv1;
in vec2 v_uvT;
in vec2 v_tc;
in float v_w;
flat in float v_top;
in vec2 v_el;
in float v_fade;
in float v_tint;
in vec3 v_world;
out vec4 o_color;
uniform sampler2D u_atlas;   // display-referred colour, coverage in alpha
uniform sampler2D u_atlasMask; // alpha: foliage share of the covered texels
uniform float u_barkGain;     // per-species trunk brightness, measured against the mesh
)GLSL";

const char *kBillFSMain = R"GLSL(
float bayer4(vec2 p) {
  const float m[16] = float[16](0.0, 8.0, 2.0, 10.0, 12.0, 4.0, 14.0, 6.0,
                                3.0, 11.0, 1.0, 9.0, 15.0, 7.0, 13.0, 5.0);
  ivec2 i = ivec2(mod(p, 4.0));
  return (m[i.x + i.y * 4] + 0.5) / 16.0;
}
void main() {
  vec4 c0 = texture(u_atlas, v_uv0), c1 = texture(u_atlas, v_uv1);
  vec4 m0 = texture(u_atlasMask, v_uv0), m1 = texture(u_atlasMask, v_uv1);
  // Coverage, colour, foliage share and bark occlusion of the side views (the
  // two nearest azimuths), then of the top-down view where it comes in. There is
  // no per-pixel pick between the two (the top tile's coverage is mostly holes
  // where the side tile's is solid, so a dither shows as a screen door): a pixel
  // is covered if either tile covers it, and takes the side tiles' colour where
  // they show foliage, the top tile's elsewhere. The top tile comes in inside a
  // radius that grows with the elevation, so the area the tree covers grows
  // steadily instead of the whole footprint appearing at once, and the side tiles
  // fade out last, which keeps the trunk and the crown's outline until the top
  // tile really covers them.
  float a = mix(c0.a, c1.a, v_w);
  vec3 rgbA = c0.rgb * c0.a * (1.0 - v_w) + c1.rgb * c1.a * v_w;
  float mfol = mix(m0.a, m1.a, v_w);
  float aoSum = mix(m0.g, m1.g, v_w);
  vec3 rgb = rgbA / max(a, 1e-3);
  float fol = clamp(mfol / max(a, 1e-3), 0.0, 1.0);
  float barkAo = clamp(aoSum / max(a - mfol, 0.02), 0.0, 1.0);
  float cov = a >= 0.35 ? 1.0 : 0.0; // mip levels thin the coverage, so keep a bit more than half
  if (v_top > 0.0) { // flat per instance, so the fetches stay in uniform control flow
    float in_t = max(abs(v_tc.x), abs(v_tc.y)) <= 1.0 ? 1.0 : 0.0; // the rotated quad's corners
    vec4 ct = texture(u_atlas, v_uvT) * in_t;
    vec4 mt = texture(u_atlasMask, v_uvT) * in_t;
    float lin = 0.5 - sin(asin(clamp(1.0 - 2.0 * v_top, -1.0, 1.0)) / 3.0);
    float tn = clamp(lin / @FULL@, 0.0, 1.0);
    float grow = (exp(@GROW@ * tn) - 1.0) / (exp(@GROW@) - 1.0);
    float reach = @REACH@ * sqrt(grow) + @REST@ * smoothstep(@FULL@ - 0.1, 1.0, lin);
    float mT = 1.0 - smoothstep(reach - @SOFT@, reach, length(v_tc));
    // The side tiles stay inside the disc (so the top tile's holes show the crown
    // behind them instead of the sky) and fade out as the elevation climbs.
    float keep = 1.0 - smoothstep(@KEEP0@, @KEEP1@, lin);
    float gOut = 1.0 - smoothstep(@OUT0@, @OUT1@, lin);
    float sCov = a * mix(gOut, keep, mT) >= 0.35 ? 1.0 : 0.0;
    float sFol = (sCov > 0.5 && fol > 0.5) ? 1.0 : 0.0; // bark under the crown from above is hidden by it
    float tCov = (mT >= 0.5 && ct.a >= @TOP_THR@) ? 1.0 : 0.0;
    float wT = tCov * (1.0 - sFol); // the side tiles win where they still show foliage
    cov = max(sCov, tCov);
    vec3 rgbT = ct.rgb;
    float folT = clamp(mt.a / max(ct.a, 1e-3), 0.0, 1.0);
    float aoT = clamp(mt.g / max(ct.a - mt.a, 0.02), 0.0, 1.0);
    rgb = mix(rgb, rgbT, wT);
    fol = mix(fol, folT, wT);
    barkAo = mix(barkAo, aoT, wT);
  }
  if (cov < 0.5) discard;
  float b = bayer4(gl_FragCoord.xy);
  bool keep = v_fade > 1.5 ? (v_fade - 2.0) < b : v_fade > b;
  if (!keep) discard;
  vec3 toFrag = v_world - u_campos;
  float dist = length(toFrag);
  vec3 sun = normalize(u_sundir);
  // A crown has no single normal: lean the upward one toward the viewer, in 3D
  // (the horizontal lean shrinks and the upward part grows as the camera climbs).
  vec2 hz = toFrag.xy / max(length(toFrag.xy), 1e-3);
  vec3 N = normalize(vec3(-hz * 0.45 * v_el.y, -(0.85 + 0.45 * v_el.x)));
  // Same foliage terms as the mesh shader (ambient floor, light through leaves).
  vec3 L = ambientLit(N) * 1.5 + sunLightColor() * (wrapDiffuse(N, sun) * 0.95 + 0.25);
  // Bark is lit like the mesh trunk (no crown rounding, no light through leaves):
  // the mean over the visible half of a vertical cylinder, with the mesh's warm
  // bark grade and lighting weights. Foliage keeps the crown terms above.
  vec2 pp = vec2(-hz.y, hz.x);
  vec3 nb0 = vec3(-hz, 0.0);
  vec3 nb1 = normalize(vec3(-0.5 * hz + 0.866 * pp, 0.0));
  vec3 nb2 = normalize(vec3(-0.5 * hz - 0.866 * pp, 0.0));
  vec3 Lb = vec3(0.0);
  for (int i = 0; i < 3; ++i) {
    vec3 nb = i == 0 ? nb0 : (i == 1 ? nb1 : nb2);
    Lb += (i == 0 ? 0.4 : 0.3) * (ambientLit(nb) * 1.3 * (0.6 + 0.4 * barkAo) + sunLightColor() * wrapDiffuse(nb, sun));
  }
  vec3 barkTint = tintMul(v_tint);
  vec3 barkLit = rgb * barkTint * vec3(1.0, 0.90, 0.78) * Lb * u_barkGain;
  vec3 lit = mix(barkLit, rgb * foliageTint(v_tint) * L, fol);
  o_color = vec4(applyAerialPerspective(lit, toFrag, dist), 1.0);
}
)GLSL";

// Atlas bake: albedo (as the lit shader would show an unlit surface) + coverage.
const char *kBakeVS = R"GLSL(
#version 330 core
layout(location = 0) in vec3 a_pos;
layout(location = 2) in vec2 a_uv;
uniform mat4 u_mvp;
out vec2 v_uv;
void main() { v_uv = a_uv; gl_Position = u_mvp * vec4(a_pos, 1.0); }
)GLSL";

const char *kBakeFS = R"GLSL(
#version 330 core
in vec2 v_uv;
layout(location = 0) out vec4 o_color;
layout(location = 1) out vec4 o_mask; // alpha 1 on foliage (cut-out) texels, 0 on bark
uniform sampler2D u_diffuse;
uniform sampler2D u_arm;
uniform float u_cutout;
uniform float u_cutoff;
uniform vec3 u_albedoMul; // colour multiplier (identity for trees)
vec3 srgbEnc(vec3 c) {
  vec3 hi = 1.055 * pow(max(c, vec3(0.0)), vec3(1.0 / 2.4)) - 0.055;
  return mix(c * 12.92, hi, step(vec3(0.0031308), c));
}
void main() {
  vec4 d = texture(u_diffuse, v_uv);
  if (u_cutout > 0.5 && d.a < u_cutoff) discard;
  float ao = texture(u_arm, v_uv).r;
  // Foliage bakes its occlusion into the albedo; bark keeps its albedo plain and
  // stores the occlusion in the mask's green for the billboard's trunk lighting.
  bool fol = u_cutout > 0.5;
  o_color = vec4(srgbEnc(d.rgb) * u_albedoMul * (fol ? mix(0.55, 1.0, ao) : 1.0), 1.0);
  o_mask = vec4(0.0, fol ? 0.0 : ao, 0.0, fol ? 1.0 : 0.0);
}
)GLSL";

bool linkProgram(QOpenGLShaderProgram &p, const char *name, const QByteArray &vs,
                 const QByteArray &fs) {
  if (!p.addShaderFromSourceCode(QOpenGLShader::Vertex, vs) ||
      !p.addShaderFromSourceCode(QOpenGLShader::Fragment, fs) || !p.link()) {
    qWarning("[Vegetation] %s program failed:\n%s", name,
             p.log().toLocal8Bit().constData());
    return false;
  }
  return true;
}

// Splits "head" (the #version and declarations) from the snippets so the shared
// GLSL lands before main.
QByteArray joinFs(const char *head, const VegGlsl &g, const char *common,
                  const char *main) {
  // Lighting declares u_campos, which the cascade lookup needs, so it goes first.
  return QByteArray(head) + g.atmosphere + g.lighting + g.shadow + common + main;
}

QImage downsample2x(const QImage &src) {
  const int w = std::max(1, src.width() / 2), h = std::max(1, src.height() / 2);
  QImage dst(w, h, QImage::Format_RGBA8888);
  for (int y = 0; y < h; ++y) {
    const uchar *r0 = src.constScanLine(std::min(2 * y, src.height() - 1));
    const uchar *r1 = src.constScanLine(std::min(2 * y + 1, src.height() - 1));
    uchar *o = dst.scanLine(y);
    for (int x = 0; x < w; ++x) {
      const int x0 = std::min(2 * x, src.width() - 1) * 4;
      const int x1 = std::min(2 * x + 1, src.width() - 1) * 4;
      for (int c = 0; c < 4; ++c)
        o[x * 4 + c] = uchar((r0[x0 + c] + r0[x1 + c] + r1[x0 + c] + r1[x1 + c] + 2) / 4);
    }
  }
  return dst;
}

// Scales alpha so that the share of texels at or above the cutoff matches
// `target`: without it a mip-mapped cut-out thins out and vanishes with distance.
void restoreCoverage(QImage &img, int cutoff255, double target) {
  long hist[256] = {};
  const int w = img.width(), h = img.height();
  for (int y = 0; y < h; ++y) {
    const uchar *r = img.constScanLine(y);
    for (int x = 0; x < w; ++x)
      ++hist[r[x * 4 + 3]];
  }
  const double total = double(w) * h;
  auto cover = [&](double s) {
    long n = 0;
    for (int a = 0; a < 256; ++a)
      if (a * s >= cutoff255)
        n += hist[a];
    return double(n) / total;
  };
  double lo = 1.0, hi = 16.0;
  if (cover(1.0) >= target)
    return;
  for (int i = 0; i < 14; ++i) {
    const double mid = 0.5 * (lo + hi);
    (cover(mid) < target ? lo : hi) = mid;
  }
  const double s = hi;
  for (int y = 0; y < h; ++y) {
    uchar *r = img.scanLine(y);
    for (int x = 0; x < w; ++x)
      r[x * 4 + 3] = uchar(std::min(255.0, r[x * 4 + 3] * s));
  }
}

} // namespace

// The tile and azimuth counts reach the shader from the same constants the
// bake uses, so the two cannot drift apart.
// The billboard fragment shader's main, with the top tile's coverage threshold in.
static QByteArray billboardFragmentMain() {
  QByteArray src(kBillFSMain);
  src.replace("@TOP_THR@", QByteArray::number(double(kVegTopCoverage), 'f', 3));
  src.replace("@REACH@", QByteArray::number(double(kVegTopReach), 'f', 3));
  src.replace("@FULL@", QByteArray::number(double(kVegTopFull), 'f', 3));
  src.replace("@GROW@", QByteArray::number(double(kVegTopGrow), 'f', 3));
  src.replace("@REST@", QByteArray::number(double(kVegTopRest), 'f', 3));
  src.replace("@SOFT@", QByteArray::number(double(kVegTopSoft), 'f', 3));
  src.replace("@OUT0@", QByteArray::number(double(kVegTopOut0), 'f', 3));
  src.replace("@OUT1@", QByteArray::number(double(kVegTopOut1), 'f', 3));
  src.replace("@KEEP0@", QByteArray::number(double(kVegTopKeep0), 'f', 3));
  src.replace("@KEEP1@", QByteArray::number(double(kVegTopKeep1), 'f', 3));
  return src;
}

QByteArray vegBillboardVertexSource() {
  QByteArray src(kBillVS);
  src.replace("@ATLAS_TILES@", QByteArray::number(double(kAtlasTiles), 'f', 1));
  src.replace("@ATLAS_AZ@", QByteArray::number(double(kAtlasAzimuths), 'f', 1));
  src.replace("@TOP_FROM@", QByteArray::number(double(kVegTopFromDeg * kPi / 180.0f), 'f', 5));
  src.replace("@TOP_TO@", QByteArray::number(double(kVegTopToDeg * kPi / 180.0f), 'f', 5));
  return src;
}

const VegSpeciesInfo *vegSpeciesTable(int *count) {
  if (count)
    *count = kTableN;
  return kTable;
}

float vegUnitScale(const VegSpeciesInfo &info, const VegVariant &v) {
  const float basis = info.bySpan ? std::max(v.footprintX, v.footprintZ) : v.nominalHeight;
  return basis > 1e-5f ? info.targetSize / basis : 1.0f;
}

int vegPickVariant(const procgen::VegInstance &v, int poolSize) {
  if (poolSize <= 1)
    return 0;
  uint32_t h = uint32_t(v.variant) * 2654435761u ^
               uint32_t(std::lround(v.tint * 65535.0f)) * 40503u;
  h ^= h >> 15;
  h *= 2246822519u;
  h ^= h >> 13;
  return int(h % uint32_t(poolSize));
}

VegLodChoice vegChooseLod(const VegSpeciesInfo &s, float d, bool impostorReady) {
  VegLodChoice c;
  const bool bill = s.impostor && impostorReady;
  const float maxD = bill ? s.maxDist : (s.impostor ? s.fallbackMax : s.maxDist);
  // Segment edges; the last segment (impostor or LOD2) ends at maxD.
  const float e[4] = {s.lodEnd[0], s.lodEnd[1], s.lodEnd[2], maxD};
  if (d >= maxD)
    return c;
  int seg = 0;
  while (seg < 3 && d >= e[seg])
    ++seg;
  // The segment index is the LOD; a billboard replaces LOD2's tail past lodEnd[2].
  const int lodOf[4] = {0, 1, 2, bill ? 3 : 2};
  const int hereLod = lodOf[seg];
  // Cross-fade zones: [edge - w, edge + w] around an interior edge.
  // seg 3 exists only with a billboard; without one LOD2 simply runs on.
  float fadeHere = 1.0f;
  auto edgeZone = [&](int edge, float *t) {
    const float w = kLodBand * e[edge];
    if (d > e[edge] - w && d < e[edge] + w) {
      *t = (d - (e[edge] - w)) / (2.0f * w);
      return true;
    }
    return false;
  };
  float t = 0.0f;
  if (seg > 0 && edgeZone(seg - 1, &t) && (bill || seg - 1 < 2)) {
    // Past the edge: fading in from the previous LOD.
    c.lod[c.n] = hereLod;
    c.fade[c.n++] = t;
    c.lod[c.n] = lodOf[seg - 1];
    c.fade[c.n++] = 2.0f + t; // keeps exactly the pixels the first one drops
    return c;
  }
  if (seg < 3 && edgeZone(seg, &t) && (bill || seg < 2)) {
    // Before the edge: fading out into the next LOD.
    c.lod[c.n] = hereLod;
    c.fade[c.n++] = 1.0f - t;
    c.lod[c.n] = lodOf[seg + 1];
    c.fade[c.n++] = 2.0f + (1.0f - t);
    return c;
  }
  // Far fade-out at maxD.
  const float w = kLodBand * maxD * 2.0f;
  if (d > maxD - w)
    fadeHere = (maxD - d) / w;
  c.lod[0] = hereLod;
  c.fade[0] = std::clamp(fadeHere, 0.001f, 1.0f);
  c.n = 1;
  return c;
}

bool VegetationRenderer::enabledByConfig() {
  const QByteArray env = qgetenv("NAV_VEGETATION");
  if (!env.isEmpty())
    return env != "0" && env.toLower() != "false" && env.toLower() != "off";
  QSettings s;
  return s.value("sim/vegetation", true).toBool();
}

bool VegetationRenderer::treesEnabledByConfig() {
  const QByteArray env = qgetenv("NAV_VEGETATION_TREES");
  if (!env.isEmpty())
    return env != "0" && env.toLower() != "false" && env.toLower() != "off";
  QSettings s;
  return s.value("sim/vegetationTrees", true).toBool();
}

VegetationRenderer::VegetationRenderer() : treesOn_(treesEnabledByConfig()) {}

void VegetationRenderer::setTreesEnabled(bool on) {
  if (on == treesOn_)
    return;
  treesOn_ = on;
  for (auto &kv : chunks_) // re-resolve with / without the trees
    kv.second.resolved = false;
}

size_t VegetationRenderer::instanceCount(int species) const {
  size_t n = 0;
  if (species == procgen::kVegTree && !treesOn_)
    return 0;
  for (const auto &kv : chunks_)
    for (const procgen::VegInstance &v : kv.second.src)
      n += v.species == species ? 1 : 0;
  return n;
}

VegetationRenderer::~VegetationRenderer() {
  if (assetsLoad_.valid())
    assetsLoad_.wait();
  for (SpeciesGpu &s : sp_)
    if (s.decode.valid())
      s.decode.wait();
}

// ---- chunk data --------------------------------------------------------------

void VegetationRenderer::setChunk(qint64 key, std::vector<procgen::VegInstance> inst,
                                  float cx, float cy, float half) {
  if (!assetsStarted_) { // decode the manifest and meshes off the GUI thread
    assetsStarted_ = true;
    assetsLoad_ = std::async(std::launch::async, [this] {
      QString err;
      const bool ok = loadVegetationAssets(assets_, &err,
                                           QStringLiteral(":/vegetation"), false);
      if (!ok)
        qWarning("[Vegetation] assets: %s", qPrintable(err));
      return ok;
    });
  }
  Chunk &c = chunks_[key];
  c.src = std::move(inst);
  c.recs.clear();
  c.resolved = false;
  c.cx = cx;
  c.cy = cy;
  c.half = half;
  float lo = 1e9f, hi = -1e9f;
  for (const procgen::VegInstance &v : c.src) {
    lo = std::min(lo, v.pos.z);
    hi = std::max(hi, v.pos.z);
  }
  if (c.src.empty())
    lo = hi = 0.0f;
  c.zLo = lo - 20.0f; // NED: trees reach up (more negative)
  c.zHi = hi + 3.0f;
}

void VegetationRenderer::removeChunk(qint64 key) { chunks_.erase(key); }
void VegetationRenderer::clearChunks() { chunks_.clear(); }

std::vector<procgen::VegExclusion>
vegExclusionsFor(const std::vector<QVector3D> &pads,
                 const QVector<vsim::Obstacle> &obstacles) {
  std::vector<procgen::VegExclusion> ex;
  constexpr float kPadRadiusM = 3.0f; // SimRendererWidget::kHelipadRadiusM
  constexpr float kMarginM = 2.0f;
  for (const QVector3D &p : pads)
    ex.push_back(procgen::VegExclusion::disc(p.x(), p.y(), kPadRadiusM + kMarginM));
  for (const vsim::Obstacle &o : obstacles) {
    float r = o.size.x();
    if (o.type == vsim::Obstacle::Box)
      r = 0.5f * std::hypot(o.size.x(), o.size.y());
    ex.push_back(procgen::VegExclusion::disc(o.pos.x(), o.pos.y(), r + kMarginM));
  }
  return ex;
}

void VegetationRenderer::exclude(const std::vector<procgen::VegExclusion> &ex) {
  if (ex.empty())
    return;
  for (auto &kv : chunks_) {
    Chunk &c = kv.second;
    const size_t before = c.src.size();
    c.src.erase(std::remove_if(c.src.begin(), c.src.end(),
                               [&](const procgen::VegInstance &v) {
                                 for (const procgen::VegExclusion &e : ex)
                                   if (e.contains(v.pos.x, v.pos.y))
                                     return true;
                                 return false;
                               }),
                c.src.end());
    if (c.src.size() != before)
      c.resolved = false;
  }
}

size_t VegetationRenderer::instanceCount() const {
  size_t n = 0;
  for (const auto &kv : chunks_)
    for (const procgen::VegInstance &v : kv.second.src)
      n += (treesOn_ || v.species != procgen::kVegTree) ? 1 : 0;
  return n;
}

int VegetationRenderer::variantFor(const procgen::VegInstance &v, int *infoOut) const {
  if (v.species >= procgen::kVegSpeciesCount)
    return -1;
  const std::vector<int> &pool =
      v.species == procgen::kVegTree
          ? treePool_[std::min<int>(v.kind, procgen::kVegTreeKindCount - 1)]
          : pool_[v.species];
  if (pool.empty())
    return -1;
  const int flat = pool[size_t(vegPickVariant(v, int(pool.size())))];
  *infoOut = var_[size_t(flat)].info;
  return flat;
}

void VegetationRenderer::resolve(Chunk &c) {
  c.recs.clear();
  c.recs.reserve(c.src.size());
  for (const procgen::VegInstance &v : c.src) {
    if (v.species == procgen::kVegTree && !treesOn_)
      continue; // trees are off: no instance, impostor or atlas bake for them
    int info = -1;
    const int flat = variantFor(v, &info);
    if (flat < 0)
      continue;
    Rec r;
    r.x = v.pos.x;
    r.y = v.pos.y;
    r.z = v.pos.z + kSinkM;
    r.yaw = v.yaw;
    if (v.tiltX != 0.0f || v.tiltY != 0.0f)
      r.yaw += 8.0f * float(procgen::vegTiltCode(v.tiltX) +
                            64 * procgen::vegTiltCode(v.tiltY));
    r.scale = v.scale;
    r.tint = std::clamp(v.tint, 0.0f, 1.0f);
    r.paint = float(v.paint);
    r.variant = uint16_t(flat);
    r.info = uint8_t(info);
    c.recs.push_back(r);
    // One grass instance stands for a tuft: the asset pieces are single blades
    // and seedlings a few centimetres across, which vanish alone, so a tuft is
    // several of them a hand's breadth apart (deterministic from the instance).
    if (kTable[info].cat == procgen::kVegGrassClump) {
      uint32_t h = uint32_t(std::lround(v.pos.x * 64.0f)) * 73856093u ^
                   uint32_t(std::lround(v.pos.y * 64.0f)) * 19349663u;
      auto rnd = [&h]() {
        h ^= h << 13; h ^= h >> 17; h ^= h << 5;
        return float(h & 0xffffffu) / 16777216.0f;
      };
      for (int k = 1; k < kTuftSize; ++k) {
        procgen::VegInstance w = v;
        w.variant = uint8_t(v.variant + 31 * k);
        w.tint = rnd();
        int info2 = -1;
        const int f2 = variantFor(w, &info2);
        if (f2 < 0)
          continue;
        const float a = rnd() * 6.2831853f, d = 0.08f + 0.40f * std::sqrt(rnd());
        Rec q = r;
        q.x += std::cos(a) * d;
        q.y += std::sin(a) * d;
        q.yaw = rnd() * 6.2831853f;
        q.scale = v.scale * (0.65f + 0.7f * rnd());
        q.tint = w.tint;
        q.variant = uint16_t(f2);
        q.info = uint8_t(info2);
        c.recs.push_back(q);
      }
    }
  }
  c.resolved = true;
}

// ---- GL setup ---------------------------------------------------------------

bool VegetationRenderer::buildPrograms(const VegGlsl &g) {
  csmLocs_ = CascadeUniforms();
  prog_ = std::make_unique<QOpenGLShaderProgram>();
  if (!linkProgram(*prog_, "vegetation", QByteArray(kVS) + kCommonGLSL + kVSMain,
                   joinFs(kFSHead, g, kCommonGLSL, kFSMain)))
    return false;
  progBillboard_ = std::make_unique<QOpenGLShaderProgram>();
  const bool billOk = linkProgram(
      *progBillboard_, "vegetation billboard", vegBillboardVertexSource(),
      QByteArray(kBillFSHead) + g.atmosphere + g.lighting + kCommonGLSL + billboardFragmentMain());
  progBake_ = std::make_unique<QOpenGLShaderProgram>();
  const bool bakeOk = linkProgram(*progBake_, "vegetation bake", kBakeVS, kBakeFS);
  if (!billOk || !bakeOk) {
    progBillboard_.reset();
    progBake_.reset();
  }
  loc_.k = prog_->uniformLocation("u_k");
  loc_.height = prog_->uniformLocation("u_height");
  loc_.sway = prog_->uniformLocation("u_sway");
  loc_.cutout = prog_->uniformLocation("u_cutout");
  loc_.cutoff = prog_->uniformLocation("u_cutoff");
  loc_.a2c = prog_->uniformLocation("u_a2c");
  if (progBillboard_) {
    loc_.cy = progBillboard_->uniformLocation("u_cy");
    loc_.half = progBillboard_->uniformLocation("u_half");
    loc_.tile = progBillboard_->uniformLocation("u_tile");
  }
  if (progBake_)
    loc_.bakeMvp = progBake_->uniformLocation("u_mvp");
  return true;
}

bool VegetationRenderer::buildAtlas() {
  if (!progBillboard_ || !progBake_)
    return false;
  const int px = kAtlasTiles * kTilePx;
  glGenTextures(1, &atlasTex_);
  glBindTexture(GL_TEXTURE_2D, atlasTex_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, px, px, 0, GL_RGBA, GL_UNSIGNED_BYTE,
               nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 3); // tiles stay >= 16 px
  // Second target of the bake: which covered texels are foliage (see kBakeFS).
  glGenTextures(1, &atlasMask_);
  glBindTexture(GL_TEXTURE_2D, atlasMask_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, px, px, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 3);
  glGenRenderbuffers(1, &atlasDepth_);
  glBindRenderbuffer(GL_RENDERBUFFER, atlasDepth_);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, px, px);
  glGenFramebuffers(1, &atlasFbo_);
  GLint prevFbo = 0;
  glGetIntegerv(GL_FRAMEBUFFER_BINDING, &prevFbo);
  glBindFramebuffer(GL_FRAMEBUFFER, atlasFbo_);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                         atlasTex_, 0);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D,
                         atlasMask_, 0);
  const GLenum bufs[2] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
  glDrawBuffers(2, bufs);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER,
                            atlasDepth_);
  const bool ok = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
  glBindFramebuffer(GL_FRAMEBUFFER, GLuint(prevFbo));
  glBindTexture(GL_TEXTURE_2D, 0);
  if (!ok)
    qInfo("[Vegetation] atlas FBO incomplete: trees use mesh LODs only.");
  return ok;
}

bool VegetationRenderer::initGL(const VegGlsl &glsl) {
  if (glReady_)
    releaseGL();
  initializeOpenGLFunctions();
  if (!buildPrograms(glsl)) {
    prog_.reset();
    return false;
  }
  maxAniso_ = 1.0f;
  if (QOpenGLContext::currentContext()->hasExtension("GL_EXT_texture_filter_anisotropic"))
    glGetFloatv(0x84FF /* GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT */, &maxAniso_);
  atlasTried_ = false; // the impostor atlas is built when trees are first drawn
  glReady_ = true;
  return true;
}

void VegetationRenderer::freeSpeciesTextures(SpeciesGpu &s) {
  for (Mat &m : s.mats)
    for (unsigned &t : m.tex)
      if (t) {
        glDeleteTextures(1, &t);
        t = 0;
      }
  s.mats.clear();
}

void VegetationRenderer::releaseGL() {
  if (!glReady_)
    return;
  glReady_ = false;
  for (size_t i = 0; i < sp_.size(); ++i) {
    SpeciesGpu &s = sp_[i];
    if (s.decode.valid())
      s.decode.wait();
    freeSpeciesTextures(s);
    s.state = SpState::Idle;
    // assets_ is written by the loader thread until assetsReady_ is set (on
    // this thread, after the future is collected); never touch it before.
    if (assetsReady_ && i < assets_.species.size())
      for (VegMaterial &m : assets_.species[i].materials) {
        m.diffuse = m.normal = m.arm = m.alpha = QImage();
      }
  }
  for (VariantGpu &v : var_) {
    for (int l = 0; l < 3; ++l) {
      if (v.vao[l])
        glDeleteVertexArrays(1, &v.vao[l]);
      if (v.vbo[l])
        glDeleteBuffers(1, &v.vbo[l]);
      if (v.ibo[l])
        glDeleteBuffers(1, &v.ibo[l]);
      v.vao[l] = v.vbo[l] = v.ibo[l] = 0;
    }
    for (int l = 0; l < 4; ++l) {
      if (v.inst[l])
        glDeleteBuffers(1, &v.inst[l]);
      v.inst[l] = 0;
    }
    if (v.billVao)
      glDeleteVertexArrays(1, &v.billVao);
    v.billVao = 0;
    v.baked = false;
  }
  if (atlasFbo_)
    glDeleteFramebuffers(1, &atlasFbo_);
  if (atlasDepth_)
    glDeleteRenderbuffers(1, &atlasDepth_);
  if (atlasTex_)
    glDeleteTextures(1, &atlasTex_);
  if (atlasMask_)
    glDeleteTextures(1, &atlasMask_);
  atlasFbo_ = atlasDepth_ = atlasTex_ = atlasMask_ = 0;
  atlasOk_ = false;
  atlasTried_ = false;
  prog_.reset();
  progBillboard_.reset();
  progBake_.reset();
}

int VegetationRenderer::liveObjects() const {
  int n = (prog_ ? 1 : 0) + (progBillboard_ ? 1 : 0) + (progBake_ ? 1 : 0);
  n += (atlasFbo_ ? 1 : 0) + (atlasDepth_ ? 1 : 0) + (atlasTex_ ? 1 : 0) + (atlasMask_ ? 1 : 0);
  for (const SpeciesGpu &s : sp_)
    for (const Mat &m : s.mats)
      for (unsigned t : m.tex)
        n += t ? 1 : 0;
  for (const VariantGpu &v : var_) {
    for (int l = 0; l < 3; ++l)
      n += (v.vao[l] ? 1 : 0) + (v.vbo[l] ? 1 : 0) + (v.ibo[l] ? 1 : 0);
    for (int l = 0; l < 4; ++l)
      n += v.inst[l] ? 1 : 0;
    n += v.billVao ? 1 : 0;
  }
  return n;
}

bool VegetationRenderer::busy() const {
  if (assetsStarted_ && !assetsReady_ && !assetsFailed_)
    return true;
  for (const SpeciesGpu &s : sp_)
    if (s.state == SpState::Decoding)
      return true;
  return false;
}

// ---- textures and meshes -----------------------------------------------------

unsigned VegetationRenderer::uploadTexture(const QImage &rgba, bool srgb,
                                           bool preserveCoverage, float cutoff) {
  if (rgba.isNull())
    return 0;
  GLuint t = 0;
  glGenTextures(1, &t);
  glBindTexture(GL_TEXTURE_2D, t);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
  const GLint ifmt = srgb ? GL_SRGB8_ALPHA8 : GL_RGBA8;
  QImage level = rgba.convertToFormat(QImage::Format_RGBA8888);
  glTexImage2D(GL_TEXTURE_2D, 0, ifmt, level.width(), level.height(), 0, GL_RGBA,
               GL_UNSIGNED_BYTE, level.constBits());
  if (preserveCoverage) {
    const int cut = int(std::lround(cutoff * 255.0f));
    long n = 0;
    for (int y = 0; y < level.height(); ++y) {
      const uchar *r = level.constScanLine(y);
      for (int x = 0; x < level.width(); ++x)
        n += r[x * 4 + 3] >= cut;
    }
    const double target = double(n) / (double(level.width()) * level.height());
    int lv = 1;
    while (level.width() > 1 || level.height() > 1) {
      level = downsample2x(level);
      restoreCoverage(level, cut, target);
      glTexImage2D(GL_TEXTURE_2D, lv++, ifmt, level.width(), level.height(), 0,
                   GL_RGBA, GL_UNSIGNED_BYTE, level.constBits());
    }
  } else {
    glGenerateMipmap(GL_TEXTURE_2D);
  }
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT); // bark tiles
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  if (maxAniso_ > 1.0f)
    glTexParameterf(GL_TEXTURE_2D, 0x84FE /* GL_TEXTURE_MAX_ANISOTROPY_EXT */,
                    std::min(8.0f, maxAniso_));
  return t;
}

bool VegetationRenderer::ensureSpecies(int si) {
  SpeciesGpu &s = sp_[size_t(si)];
  if (s.state == SpState::Ready)
    return true;
  if (s.state == SpState::Idle) {
    s.state = SpState::Decoding;
    auto *mats = &assets_.species[size_t(si)].materials;
    s.decode = std::async(std::launch::async, [mats] {
      for (VegMaterial &m : *mats) {
        QString err;
        if (!loadVegetationMaterialImages(m, &err))
          qWarning("[Vegetation] %s", qPrintable(err));
      }
    });
    return false;
  }
  // Decoding: finished?
  if (s.decode.wait_for(std::chrono::seconds(0)) == std::future_status::ready &&
      speciesUploadBudget_ > 0) {
    --speciesUploadBudget_;
    s.decode.get();
    uploadSpecies(si);
    return s.state == SpState::Ready;
  }
  return false;
}

void VegetationRenderer::uploadSpecies(int si) {
  SpeciesGpu &s = sp_[size_t(si)];
  VegSpeciesAssets &a = assets_.species[size_t(si)];
  s.mats.assign(a.materials.size(), Mat());
  for (size_t i = 0; i < a.materials.size(); ++i) {
    VegMaterial &m = a.materials[i];
    QImage colour = m.diffuse.convertToFormat(QImage::Format_RGBA8888);
    if (m.cutout() && !m.alpha.isNull()) { // coverage rides in the colour's alpha
      const QImage al = m.alpha.size() == colour.size()
                            ? m.alpha
                            : m.alpha.scaled(colour.size(), Qt::IgnoreAspectRatio,
                                             Qt::SmoothTransformation);
      for (int y = 0; y < colour.height(); ++y) {
        uchar *r = colour.scanLine(y);
        const uchar *ar = al.constScanLine(y);
        for (int x = 0; x < colour.width(); ++x)
          r[x * 4 + 3] = ar[x];
      }
    }
    s.mats[i].tex[0] = uploadTexture(colour, true, m.cutout(), m.alphaCutoff);
    s.mats[i].tex[1] = uploadTexture(m.normal, false, false, 0.0f);
    s.mats[i].tex[2] = uploadTexture(m.arm, false, false, 0.0f);
    m.diffuse = m.normal = m.arm = m.alpha = QImage(); // GPU holds them now
  }
  glBindTexture(GL_TEXTURE_2D, 0);
  s.state = SpState::Ready;
}

void VegetationRenderer::ensureMesh(VariantGpu &v, int lod) {
  if (v.vao[lod])
    return;
  const VegMesh &m = assets_.species[size_t(v.species)].variants[size_t(v.variant)].lods[lod];
  glGenVertexArrays(1, &v.vao[lod]);
  glBindVertexArray(v.vao[lod]);
  glGenBuffers(1, &v.vbo[lod]);
  glBindBuffer(GL_ARRAY_BUFFER, v.vbo[lod]);
  glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(m.vertices.size() * sizeof(float)),
               m.vertices.data(), GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 32, nullptr);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 32, reinterpret_cast<void *>(12));
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 32, reinterpret_cast<void *>(24));
  glGenBuffers(1, &v.ibo[lod]);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, v.ibo[lod]);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(m.indices.size() * sizeof(uint32_t)),
               m.indices.data(), GL_STATIC_DRAW);
  if (!v.inst[lod])
    glGenBuffers(1, &v.inst[lod]);
  glBindBuffer(GL_ARRAY_BUFFER, v.inst[lod]);
  glEnableVertexAttribArray(3);
  glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, 32, nullptr);
  glVertexAttribDivisor(3, 1);
  glEnableVertexAttribArray(4);
  glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, 32, reinterpret_cast<void *>(16));
  glVertexAttribDivisor(4, 1);
  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

// Reads this variant's baked tiles back (tests only): how many covered texels
// are foliage and how many are bark, and where they sit vertically.
void VegetationRenderer::collectBakeStats(const VariantGpu &v) {
  GLint prevFbo = 0, prevRead = 0;
  glGetIntegerv(GL_FRAMEBUFFER_BINDING, &prevFbo);
  glGetIntegerv(GL_READ_BUFFER, &prevRead);
  glBindFramebuffer(GL_FRAMEBUFFER, atlasFbo_);
  std::vector<unsigned char> cov(size_t(kTilePx) * kTilePx * 4), msk(cov.size());
  for (int a = 0; a < kAtlasViews; ++a) {
    const int tile = v.tile + a;
    const int tx = (tile % kAtlasTiles) * kTilePx, ty = (tile / kAtlasTiles) * kTilePx;
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glReadPixels(tx, ty, kTilePx, kTilePx, GL_RGBA, GL_UNSIGNED_BYTE, cov.data());
    glReadBuffer(GL_COLOR_ATTACHMENT1);
    glReadPixels(tx, ty, kTilePx, kTilePx, GL_RGBA, GL_UNSIGNED_BYTE, msk.data());
    unsigned long long hsh = 1469598103934665603ull; // FNV-1a over the tile
    for (unsigned char byte : cov)
      hsh = (hsh ^ byte) * 1099511628211ull;
    for (unsigned char byte : msk)
      hsh = (hsh ^ byte) * 1099511628211ull;
    bakeStatsOut_.hash += hsh * (2ull * (unsigned long long)tile + 1ull);
    if (a >= kAtlasAzimuths) { // the top tile only counts its covered texels
      for (size_t i = 3; i < cov.size(); i += 4)
        bakeStatsOut_.topPx += cov[i] >= 128 ? 1 : 0;
      continue;
    }
    for (int y = 0; y < kTilePx; ++y)
      for (int x = 0; x < kTilePx; ++x) {
        const size_t i = (size_t(y) * kTilePx + size_t(x)) * 4 + 3;
        if (cov[i] < 128)
          continue;
        const double up = double(y) / double(kTilePx - 1); // 0 at the foot, 1 at the top
        if (msk[i] >= 128) {
          ++bakeStatsOut_.foliagePx;
          bakeStatsOut_.foliageHeightSum += up;
        } else {
          ++bakeStatsOut_.barkPx;
          bakeStatsOut_.barkHeightSum += up;
        }
      }
  }
  glBindFramebuffer(GL_FRAMEBUFFER, GLuint(prevFbo));
  glReadBuffer(GLenum(prevRead)); // belongs to the framebuffer bound when it was read
}

// Renders `v` from kAtlasAzimuths directions and from above into its atlas tiles.
void VegetationRenderer::bakeImpostors(VariantGpu &v) {
  ensureMesh(v, 1);
  const VegMesh &m =
      assets_.species[size_t(v.species)].variants[size_t(v.variant)].lods[1];
  const SpeciesGpu &sg = sp_[size_t(v.species)];
  GLint prevFbo = 0;
  GLint prevVp[4];
  glGetIntegerv(GL_FRAMEBUFFER_BINDING, &prevFbo);
  glGetIntegerv(GL_VIEWPORT, prevVp);
  glBindFramebuffer(GL_FRAMEBUFFER, atlasFbo_);
  glEnable(GL_SCISSOR_TEST);
  glEnable(GL_DEPTH_TEST);
  glDepthMask(GL_TRUE);
  glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
  glDisable(GL_CULL_FACE);
  glDisable(GL_BLEND);
  progBake_->bind();
  progBake_->setUniformValue("u_diffuse", 2);
  progBake_->setUniformValue("u_arm", 4);
  const float H = v.billH, D = 4.0f * H + 2.0f;
  QMatrix4x4 proj;
  proj.ortho(-H, H, -H, H, 0.1f, 2.0f * D);
  glBindVertexArray(v.vao[1]);
  for (int a = 0; a < kAtlasViews; ++a) {
    const int tile = v.tile + a;
    const int tx = (tile % kAtlasTiles) * kTilePx, ty = (tile / kAtlasTiles) * kTilePx;
    glViewport(tx, ty, kTilePx, kTilePx);
    glScissor(tx, ty, kTilePx, kTilePx);
    glClearColor(0.30f, 0.36f, 0.24f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    QMatrix4x4 view;
    if (a < kAtlasAzimuths) {
      const float al = 2.0f * kPi * float(a) / float(kAtlasAzimuths);
      view.lookAt(QVector3D(std::sin(al) * D, v.centreY, std::cos(al) * D),
                  QVector3D(0, v.centreY, 0), QVector3D(0, 1, 0));
    } else {
      // From straight above. The usual up vector (0,1,0) is parallel to the view
      // direction here (lookAt would give a degenerate basis), so say which way
      // the image goes: toward -z (model north), the azimuth-0 view tilted over.
      view.lookAt(QVector3D(0, v.centreY + D, 0), QVector3D(0, v.centreY, 0),
                  QVector3D(0, 0, -1));
    }
    progBake_->setUniformValue(loc_.bakeMvp, proj * view);
    for (const VegSubmesh &sm : m.submeshes) {
      const Mat &mat = sg.mats[size_t(sm.material)];
      const VegMaterial &vm =
          assets_.species[size_t(v.species)].materials[size_t(sm.material)];
      glActiveTexture(GL_TEXTURE2);
      glBindTexture(GL_TEXTURE_2D, mat.tex[0]);
      glActiveTexture(GL_TEXTURE4);
      glBindTexture(GL_TEXTURE_2D, mat.tex[2]);
      progBake_->setUniformValue("u_cutout", vm.cutout() ? 1.0f : 0.0f);
      progBake_->setUniformValue("u_albedoMul", QVector3D(1.0f, 1.0f, 1.0f));
      progBake_->setUniformValue("u_cutoff", vm.alphaCutoff);
      glDrawElements(GL_TRIANGLES, GLsizei(sm.indexCount), GL_UNSIGNED_INT,
                     reinterpret_cast<void *>(uintptr_t(sm.firstIndex) * 4));
    }
  }
  glBindVertexArray(0);
  progBake_->release();
  glDisable(GL_SCISSOR_TEST);
  glBindFramebuffer(GL_FRAMEBUFFER, GLuint(prevFbo));
  glViewport(prevVp[0], prevVp[1], prevVp[2], prevVp[3]);
  if (bakeStats_)
    collectBakeStats(v);
  glBindTexture(GL_TEXTURE_2D, atlasTex_);
  glGenerateMipmap(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, atlasMask_);
  glGenerateMipmap(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, 0);
  glActiveTexture(GL_TEXTURE0);
  v.baked = true;
}

// ---- drawing -----------------------------------------------------------------

void VegetationRenderer::setCommonUniforms(QOpenGLShaderProgram &p, const VegFrame &f) {
  // The Atmosphere.h uniforms (sun colour, sky, ambient) first: the explicit
  // sets below, such as the caller's sun direction, win over them.
  f.atmo.forEachUniform([&p](const char *name, auto... v) {
    const float a[] = {static_cast<float>(v)...};
    if constexpr (sizeof...(v) == 3)
      p.setUniformValue(name, QVector3D(a[0], a[1], a[2]));
    else
      p.setUniformValue(name, a[0]);
  });
  p.setUniformValue("u_vp", f.proj * f.view);
  p.setUniformValue("u_campos", f.camEye);
  p.setUniformValue("u_eye", f.camEye);
  p.setUniformValue("u_sundir", f.sunDir);
  p.setUniformValue("u_sunint", f.sunIntensity);
  p.setUniformValue("u_ambstr", f.ambientStrength);
  p.setUniformValue("u_fogdensity", f.fogDensity);
  p.setUniformValue("u_fogstart", f.fogStart);
  p.setUniformValue("u_fogbase", f.fogBase);
  p.setUniformValue("u_fogheight", f.fogHeight);
  p.setUniformValue("u_time", f.time);
  p.setUniformValue("u_drone", f.dronePos);
  p.setUniformValue("u_droner", f.droneRadius);
  if (&p == prog_.get()) { // only the lit program receives shadows
    if (csmLocs_.vp < 0)
      csmLocs_.resolve(&p);
    csmLocs_.upload(&p, f.csm, f.shadowOn);
  }
}

void VegetationRenderer::drawBucket(VariantGpu &v, int lod,
                                    const std::vector<float> &data,
                                    const VegFrame &) {
  ensureMesh(v, lod);
  const GLsizei n = GLsizei(data.size() / 8);
  const VegMesh &m =
      assets_.species[size_t(v.species)].variants[size_t(v.variant)].lods[lod];
  glBindVertexArray(v.vao[lod]);
  glBindBuffer(GL_ARRAY_BUFFER, v.inst[lod]);
  glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(data.size() * sizeof(float)), data.data(),
               GL_STREAM_DRAW);
  prog_->setUniformValue(loc_.k, v.k);
  prog_->setUniformValue(loc_.height, v.height);
  prog_->setUniformValue(loc_.sway, kTable[v.info].sway);
  const SpeciesGpu &sg = sp_[size_t(v.species)];
  const VegSpeciesAssets &sa = assets_.species[size_t(v.species)];
  for (const VegSubmesh &sm : m.submeshes) {
    const Mat &mat = sg.mats[size_t(sm.material)];
    const VegMaterial &vm = sa.materials[size_t(sm.material)];
    for (int u = 0; u < 3; ++u) {
      glActiveTexture(GLenum(GL_TEXTURE2 + u));
      glBindTexture(GL_TEXTURE_2D, mat.tex[u]);
    }
    prog_->setUniformValue(loc_.cutout, vm.cutout() ? 1.0f : 0.0f);
    prog_->setUniformValue(loc_.cutoff, vm.alphaCutoff);
    prog_->setUniformValue(loc_.a2c, a2c_ ? 1.0f : 0.0f);
    glDrawElementsInstanced(GL_TRIANGLES, GLsizei(sm.indexCount), GL_UNSIGNED_INT,
                            reinterpret_cast<void *>(uintptr_t(sm.firstIndex) * 4), n);
    ++stats_.drawCalls;
    stats_.triangles += (long long)(sm.indexCount / 3) * n;
    stats_.lodTriangles[lod] += (long long)(sm.indexCount / 3) * n;
  }
  stats_.instancesDrawn += int(n);
  if (kTable[v.info].cat == procgen::kVegTree) {
    const int kind = procgen::vegTreeKindIndex(kTable[v.info].id);
    if (kind >= 0)
      stats_.treeMeshes[kind] += int(n);
  }
}

void VegetationRenderer::drawImpostors(VariantGpu &v, const std::vector<float> &data,
                                       const VegFrame &) {
  const GLsizei n = GLsizei(data.size() / 8);
  if (!v.billVao) {
    glGenVertexArrays(1, &v.billVao);
    if (!v.inst[3])
      glGenBuffers(1, &v.inst[3]);
    glBindVertexArray(v.billVao);
    glBindBuffer(GL_ARRAY_BUFFER, v.inst[3]);
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, 32, nullptr);
    glVertexAttribDivisor(3, 1);
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, 32, reinterpret_cast<void *>(16));
    glVertexAttribDivisor(4, 1);
  }
  glBindVertexArray(v.billVao);
  glBindBuffer(GL_ARRAY_BUFFER, v.inst[3]);
  glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(data.size() * sizeof(float)), data.data(),
               GL_STREAM_DRAW);
  progBillboard_->setUniformValue(loc_.cy, v.centreY);
  progBillboard_->setUniformValue(loc_.half, v.billH);
  progBillboard_->setUniformValue(loc_.tile, float(v.tile));
  progBillboard_->setUniformValue("u_k", v.k);
  progBillboard_->setUniformValue("u_barkGain", kTable[v.info].barkGain);
  glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, n);
  ++stats_.drawCalls;
  stats_.impostors += int(n);
  stats_.instancesDrawn += int(n);
  stats_.triangles += 2LL * n;
  if (kTable[v.info].cat == procgen::kVegTree) {
    const int kind = procgen::vegTreeKindIndex(kTable[v.info].id);
    if (kind >= 0)
      stats_.treeImpostors[kind] += int(n);
  }
}

void VegetationRenderer::draw(const VegFrame &f) {
  stats_ = Stats();
  if (!glReady_ || !prog_ || chunks_.empty())
    return;
  // Manifest + meshes decoded off-thread: build the variant tables once.
  if (!assetsReady_ && !assetsFailed_ && assetsLoad_.valid() &&
      assetsLoad_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    if (!assetsLoad_.get()) {
      assetsFailed_ = true;
    } else {
      assetsReady_ = true;
      sp_.clear();
      sp_.resize(assets_.species.size());
      var_.clear();
      infoOfSpecies_.assign(assets_.species.size(), -1);
      int nextTile = 0;
      for (size_t si = 0; si < assets_.species.size(); ++si) {
        int row = -1;
        for (int r = 0; r < kTableN; ++r)
          if (assets_.species[si].id == kTable[r].id)
            row = r;
        infoOfSpecies_[si] = row;
        if (row < 0)
          continue;
        const VegSpeciesInfo &info = kTable[row];
        for (size_t vi = 0; vi < assets_.species[si].variants.size(); ++vi) {
          const VegVariant &av = assets_.species[si].variants[vi];
          VariantGpu v;
          v.species = int(si);
          v.variant = int(vi);
          v.info = row;
          v.k = vegUnitScale(info, av);
          v.height = std::max(av.nominalHeight, 1e-4f);
          const QVector3D ext = av.bboxMax - av.bboxMin;
          v.radius = 0.5f * ext.length();
          v.centreY = 0.5f * (av.bboxMin.y() + av.bboxMax.y());
          const float rh = std::hypot(std::max(std::fabs(av.bboxMin.x()), std::fabs(av.bboxMax.x())),
                                      std::max(std::fabs(av.bboxMin.z()), std::fabs(av.bboxMax.z())));
          // A square that holds the crown from any direction: a crown fits an
          // ellipsoid, whose projection never exceeds its longest semi-axis (so
          // the side view's size, and so its resolution, are unchanged).
          v.billH = 1.06f * std::max(0.5f * ext.y(), rh);
          if (info.impostor && nextTile + kAtlasViews <= kAtlasTiles * kAtlasTiles) {
            v.tile = nextTile;
            nextTile += kAtlasViews;
          }
          // Grass tufts outnumber the single blades; every tree variant of a
          // kind is equally likely.
          const int copies = av.name.contains("seedling") ? 3 : 1;
          for (int c = 0; c < copies; ++c) {
            if (info.cat == procgen::kVegTree) {
              const int kind = procgen::vegTreeKindIndex(info.id);
              if (kind >= 0)
                treePool_[size_t(kind)].push_back(int(var_.size()));
            } else {
              pool_[size_t(info.cat)].push_back(int(var_.size()));
            }
          }
          var_.push_back(v);
        }
      }
      buckets_.assign(var_.size() * 4, std::vector<float>());
    }
  }
  if (!assetsReady_)
    return;
  speciesUploadBudget_ = 1;

  // Frustum planes (world space) from the clip matrix.
  const QMatrix4x4 vp = f.proj * f.view;
  float pl[6][4];
  {
    const QVector4D r0 = vp.row(0), r1 = vp.row(1), r2 = vp.row(2), r3 = vp.row(3);
    const QVector4D p[6] = {r3 + r0, r3 - r0, r3 + r1, r3 - r1, r3 + r2, r3 - r2};
    for (int i = 0; i < 6; ++i) {
      const float len = std::sqrt(p[i].x() * p[i].x() + p[i].y() * p[i].y() + p[i].z() * p[i].z());
      const float inv = len > 1e-9f ? 1.0f / len : 1.0f;
      pl[i][0] = p[i].x() * inv;
      pl[i][1] = p[i].y() * inv;
      pl[i][2] = p[i].z() * inv;
      pl[i][3] = p[i].w() * inv;
    }
  }
  const QVector3D eye = f.camEye;
  for (std::vector<float> &b : buckets_)
    b.clear();
  float farthest = 0.0f;
  for (const VegSpeciesInfo &s : kTable)
    farthest = std::max(farthest, s.maxDist);

  for (auto &kv : chunks_) {
    Chunk &c = kv.second;
    // Distance from the eye to the chunk's footprint, then its box vs frustum.
    const float dx = std::max(std::fabs(eye.x() - c.cx) - c.half, 0.0f);
    const float dy = std::max(std::fabs(eye.y() - c.cy) - c.half, 0.0f);
    if (dx * dx + dy * dy > farthest * farthest)
      continue;
    const float lo[3] = {c.cx - c.half, c.cy - c.half, c.zLo};
    const float hi[3] = {c.cx + c.half, c.cy + c.half, c.zHi};
    bool outside = false;
    for (int i = 0; i < 6 && !outside; ++i) {
      const float px = pl[i][0] >= 0 ? hi[0] : lo[0];
      const float py = pl[i][1] >= 0 ? hi[1] : lo[1];
      const float pz = pl[i][2] >= 0 ? hi[2] : lo[2];
      outside = pl[i][0] * px + pl[i][1] * py + pl[i][2] * pz + pl[i][3] < 0.0f;
    }
    if (outside)
      continue;
    ++stats_.chunksDrawn;
    if (!c.resolved)
      resolve(c);
    for (const Rec &r : c.recs) {
      const VegSpeciesInfo &info = kTable[r.info];
      const float ddx = r.x - eye.x(), ddy = r.y - eye.y();
      const float dzz = r.z - eye.z();
      const float d2 = ddx * ddx + ddy * ddy + dzz * dzz;
      if (d2 >= info.maxDist * info.maxDist)
        continue;
      VariantGpu &v = var_[r.variant];
      if (sp_[size_t(v.species)].state != SpState::Ready) {
        ensureSpecies(v.species);
        continue;
      }
      const float sc = v.k * r.scale;
      const float cz = r.z - v.centreY * sc, rad = v.radius * sc + 0.5f;
      bool vis = true;
      for (int i = 0; i < 6 && vis; ++i)
        vis = pl[i][0] * r.x + pl[i][1] * r.y + pl[i][2] * cz + pl[i][3] >= -rad;
      if (!vis)
        continue;
      const float d = std::sqrt(d2);
      const bool billReady = atlasOk_ && v.baked;
      VegLodChoice ch = vegChooseLod(info, d, billReady);
      if (forcedLod_ >= 0 && d < info.maxDist) { // tests: one LOD at every distance
        ch = VegLodChoice();
        ch.n = 1;
        ch.lod[0] = forcedLod_ == 3 && !billReady ? 2 : forcedLod_;
        ch.fade[0] = 1.0f;
      }
      for (int k = 0; k < ch.n; ++k) {
        std::vector<float> &b = buckets_[size_t(r.variant) * 4 + size_t(ch.lod[k])];
        b.insert(b.end(), {r.x, r.y, r.z, r.yaw, r.scale, r.tint, ch.fade[k], r.paint});
      }
    }
  }

  if (treesOn_ && !atlasTried_) {
    atlasTried_ = true;
    atlasOk_ = buildAtlas();
    if (!atlasOk_) {
      if (atlasFbo_)
        glDeleteFramebuffers(1, &atlasFbo_);
      if (atlasDepth_)
        glDeleteRenderbuffers(1, &atlasDepth_);
      if (atlasTex_)
        glDeleteTextures(1, &atlasTex_);
      if (atlasMask_)
        glDeleteTextures(1, &atlasMask_);
      atlasFbo_ = atlasDepth_ = atlasTex_ = atlasMask_ = 0;
    }
  }
  // Bake what is needed (a few per frame): variants seen at LOD2 range.
  int bakes = 2;
  if (atlasOk_)
    for (size_t vi = 0; vi < var_.size() && bakes > 0; ++vi) {
      VariantGpu &v = var_[vi];
      if (v.tile < 0 || v.baked || sp_[size_t(v.species)].state != SpState::Ready)
        continue;
      if (buckets_[vi * 4 + 2].empty())
        continue; // nothing in billboard range yet
      bakeImpostors(v);
      --bakes;
    }

  // Draw. Common GL state first.
  glEnable(GL_DEPTH_TEST);
  glDepthMask(GL_TRUE);
  glDisable(GL_CULL_FACE);
  glDisable(GL_BLEND);
  // Alpha-to-coverage turns leaf edges into MSAA coverage. It only helps on a
  // multisampled target, and it reads the fragment alpha, so keep that out of
  // the framebuffer's own alpha channel.
  GLint samples = 0;
  glGetIntegerv(GL_SAMPLES, &samples);
  a2c_ = samples > 1;
  if (a2c_) {
    glEnable(GL_SAMPLE_ALPHA_TO_COVERAGE);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_FALSE);
  }
  glActiveTexture(GL_TEXTURE1);
  glBindTexture(GL_TEXTURE_2D, f.shadowTex);
  prog_->bind();
  setCommonUniforms(*prog_, f);
  prog_->setUniformValue("u_diffuse", 2);
  prog_->setUniformValue("u_normal", 3);
  prog_->setUniformValue("u_arm", 4);
  for (size_t vi = 0; vi < var_.size(); ++vi)
    for (int l = 0; l < 3; ++l)
      if (!buckets_[vi * 4 + size_t(l)].empty())
        drawBucket(var_[vi], l, buckets_[vi * 4 + size_t(l)], f);
  prog_->release();
  if (a2c_) {
    glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
  }

  if (progBillboard_ && atlasOk_) {
    progBillboard_->bind();
    setCommonUniforms(*progBillboard_, f);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, atlasTex_);
    progBillboard_->setUniformValue("u_atlas", 2);
    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_2D, atlasMask_);
    progBillboard_->setUniformValue("u_atlasMask", 3);
    for (size_t vi = 0; vi < var_.size(); ++vi)
      if (!buckets_[vi * 4 + 3].empty())
        drawImpostors(var_[vi], buckets_[vi * 4 + 3], f);
    progBillboard_->release();
  }
  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  for (int u = 4; u >= 1; --u) {
    glActiveTexture(GLenum(GL_TEXTURE0 + u));
    glBindTexture(GL_TEXTURE_2D, 0);
  }
  glActiveTexture(GL_TEXTURE0);
  for (const SpeciesGpu &s : sp_)
    stats_.speciesReady += s.state == SpState::Ready;
  for (const VariantGpu &v : var_)
    stats_.impostorVariants += v.baked;
}

} // namespace vsim
