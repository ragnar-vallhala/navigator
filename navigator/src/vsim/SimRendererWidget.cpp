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
#include "SimRendererWidget.h"

#include "procgen/EdgeTrack.h"
#include "procgen/PropStyle.h"

#include <algorithm>
#include <QSettings>
#include <QCursor>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QTimer>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QOpenGLContext>
#include <QPainter>
#include <QSurfaceFormat>
#include <QWidget>
#include <QWheelEvent>
#include <QtMath>

#include <array>
#include <atomic>
#include <climits>
#include <cmath>
#include <vector>

namespace vsim {

// All geometry lives in NED. The camera "up" is (0,0,-1) so the
// rendered "up" on screen is NED -Z = above ground. World axes look
// like: red=N (+X), green=E (+Y), blue=down (+Z).
namespace {

// Model transform for a helipad at pad=(x,y,terrainHeight): the cylinder mesh
// (local z in [-1 top .. 0 base]) is scaled to the full height and shifted so
// the base buries below the surface and the deck sits kHelipadDeckM above it.
QMatrix4x4 helipadModel(const QVector3D &pad) {
  const float bury =
      SimRendererWidget::kHelipadHeightM -
      SimRendererWidget::kHelipadDeckM; // base depth below surface
  QMatrix4x4 m;
  m.translate(pad.x(), pad.y(),
              -pad.z() + bury); // surface z=-h, base sunk by bury
  m.scale(SimRendererWidget::kHelipadRadiusM,
          SimRendererWidget::kHelipadRadiusM,
          SimRendererWidget::kHelipadHeightM);
  return m;
}

constexpr float kDroneMapMaxTexelM = 0.05f; // finest cascade texel that resolves the drone
// Fraction of a rotor disc the blades cover (solidity = blades x chord / (pi R)),
// used as the shadow coverage of a spinning prop. Basis: the procedural drone's
// own numbers in DroneModel.cpp styleOf(), with the chord profile's mean of 0.79:
// 5in freestyle 3 x 14.5 mm / (pi x 62 mm) x 0.79 = 0.18; 3in 0.19; 7in
// long-range (2 blades) 0.11; 10in 0.10. 0.15 is the middle of that range and
// coord's figure.
constexpr float kDronePushM = 0.4f; // drone caster depth pull toward the sun [m]
constexpr float kRotorCoverage = 0.15f;
constexpr float kPropDiscThickM = 0.01f; // drone prop disc caster (shadow pass)
constexpr float kFovYDeg = 60.0f; // vertical field of view (also fits the shadow cascades)
constexpr int kGridHalf = 10; // m, ground plane extends +/- this
constexpr float kGridStep = 1.0f;
constexpr float kBodyL = 0.40f; // body extents in X
constexpr float kBodyW = 0.30f; // body extents in Y
constexpr float kBodyH = 0.06f; // body extents in Z
constexpr float kRotorR = 0.08f;
constexpr int kRotorSeg = 24;

const char *kVertexShader = R"GLSL(
#version 330 core
layout(location=0) in vec3 a_pos;
uniform mat4 u_mvp;
void main() {
  gl_Position = u_mvp * vec4(a_pos, 1.0);
}
)GLSL";

const char *kFragmentShader = R"GLSL(
#version 330 core
out vec4 o_color;
uniform vec3 u_color;
void main() {
  o_color = vec4(u_color, 1.0);
}
)GLSL";

// The shared sky/sun GLSL (skyColor, fogColor, sunDisc, ambientLight) lives in
// Atmosphere.h, appended to the sky, lit and flora fragment programs so distant
// terrain fog fades into exactly the sky behind it.

// Lit program for solids (imported airframe + world mesh / terrain, obstacles):
// world-space directional Lambert + sky/ground hemispheric ambient, then aerial
// perspective (distance fog into the sky color) and the shared tonemap. Surface
// color is u_color * a_color.
const char *kLitVertexShader = R"GLSL(
#version 330 core
layout(location=0) in vec3 a_pos;
layout(location=1) in vec3 a_normal;
layout(location=2) in vec3 a_color;
#ifdef TERRAIN_EDGE
layout(location=3) in vec2 a_trk;   // terrain: signed, unsigned distance to a parcel-border track
layout(location=4) in vec3 a_trkc;  // terrain: the ground's colour on that track
layout(location=5) in vec3 a_split; // terrain: a_color without the border track
#endif
uniform mat4 u_mvp;
uniform mat4 u_model;
uniform mat3 u_nmat;
out vec3 v_normal;
out vec3 v_color;
#ifdef TERRAIN_EDGE
out vec2 v_trk;
out vec3 v_trkc;
out vec3 v_split;
#endif
out vec3 v_world;
out vec3 v_local;
void main() {
  v_world = (u_model * vec4(a_pos, 1.0)).xyz;
  v_local = a_pos;
  gl_Position = u_mvp * vec4(a_pos, 1.0);
  v_normal = u_nmat * a_normal;
  v_color = a_color;
#ifdef TERRAIN_EDGE
  v_trk = a_trk;
  v_trkc = a_trkc;
  v_split = a_split;
#endif
}
)GLSL";

const char *kLitFragmentHead = R"GLSL(
#version 330 core
in vec3 v_normal;
in vec3 v_color;
#ifdef TERRAIN_EDGE
in vec2 v_trk;
in vec3 v_trkc;
in vec3 v_split;
#endif
in vec3 v_world;
in vec3 v_local;             // object-space position (unit solids)
out vec4 o_color;
uniform vec3 u_color;
uniform float u_terrain;     // 1 = ground: procedural albedo detail + grid
uniform float u_grid;        // 1 = draw the survey grid on that ground
uniform float u_bands;       // >0: pole painted in this many 1 m red/white bands
)GLSL";

// Sun colour, ambient and aerial perspective, shared by the lit/terrain shader
// and the vegetation shader so both are lit and fogged by the same functions.
// Declares their uniforms; appended after kAtmosphereGLSL (skyColor).
const char *kLightingGLSL = R"GLSL(
uniform vec3 u_campos;       // camera world position (NED)
uniform float u_fogdensity;  // aerial-perspective strength
uniform float u_fogstart;    // metres before fog begins
uniform vec4 u_ringclip;     // far ring: discard inside this xy rect (x0,y0,x1,y1)
uniform float u_sunint;      // sun-intensity look knob (matches grass)
uniform float u_ambstr;      // ambient-strength look knob (matches grass)
vec3 sunLightColor() { return u_suncol * u_sunint; }  // Atmosphere.h sun, look knob
// Wrapped diffuse for a soft terminator that still shows light direction.
float wrapDiffuse(vec3 n, vec3 sun) {
  float w = clamp(dot(n, sun) * 0.45 + 0.55, 0.0, 1.0);
  return w * w;
}
// Atmosphere.h hemispheric ambient (ambientLight) scaled by the look knob.
vec3 ambientLit(vec3 n) { return ambientLight(n) * u_ambstr; }
// Aerial perspective: fade toward the horizon colour with distance, so the
// streamed-terrain edge dissolves into haze. Needs kFogGLSL (fogAmount).
vec3 applyAerialPerspective(vec3 lit, vec3 toFrag, float dist) {
  vec3 vdir = dist > 1e-4 ? toFrag / dist : vec3(0.0, 0.0, 1.0);
  float fog = fogAmount(u_campos, u_campos + toFrag, u_fogstart, u_fogdensity);
  return mix(lit, fogColor(vdir), clamp(fog, 0.0, 1.0));
}
)GLSL";

const char *kLitFragmentMain = R"GLSL(
// Ground detail. The terrain is one flat vertex colour per ~1.5 m cell, which
// gives the eye nothing to track: no parallax, no speed or height cue. Three
// noise octaves (patches of ~0.4 / 2 / 9 / 47 m, each fading out once it is
// smaller than a few pixels) break the colour up, and a 1 / 10 / 100 m
// survey grid gives a metric scale. All procedural, keyed on world position so
// it sticks to the ground as the camera moves.
float gh(vec2 p) { p = fract(p * vec2(123.34, 345.45)); p += dot(p, p + 34.345); return fract(p.x * p.y); }
float gn(vec2 p) {
  vec2 i = floor(p), f = fract(p); f = f * f * (3.0 - 2.0 * f);
  return mix(mix(gh(i), gh(i + vec2(1, 0)), f.x), mix(gh(i + vec2(0, 1)), gh(i + vec2(1, 1)), f.x), f.y);
}
// Anti-aliased grid line mask for a cell size [m]; vanishes once cells get
// down to a few pixels so distant ground never shimmers.
float gridLine(vec2 p, float cell, float w) {
  vec2 c = p / cell;
  vec2 fw = fwidth(c);
  vec2 d = abs(fract(c - 0.5) - 0.5);
  vec2 half_ = max(vec2(w), fw * 0.75);
  vec2 l = 1.0 - smoothstep(half_ - fw, half_ + fw, d);
  return max(l.x, l.y) * (1.0 - smoothstep(0.12, 0.35, max(fw.x, fw.y)));
}
void main() {
  if (u_ringclip.z > u_ringclip.x && v_world.x > u_ringclip.x &&
      v_world.x < u_ringclip.z && v_world.y > u_ringclip.y &&
      v_world.y < u_ringclip.w)
    discard;
  vec3 n = normalize(v_normal);
  vec3 nSmooth = n;  // shadow gating wants the mesh normal, not the material's detail normal
  vec3 toFrag = v_world - u_campos;
  float dist = length(toFrag);
  vec3 sun = normalize(u_sundir);
  vec3 sunCol = u_suncol * u_sunint;  // warm golden key, tinted at low sun
  // Wrapped diffuse for a soft terminator that still shows light direction.
  float wrap = clamp(dot(n, sun) * 0.45 + 0.55, 0.0, 1.0); wrap *= wrap;
  // Hemispheric ambient: NED up is -Z, so up-facing (n.z<0) catches sky light.
  // Cool-but-warmed overcast ambient (shadows read soft teal, not cold).
  vec3 ambient = ambientLight(n) * u_ambstr;
  // Parcel-border tracks are drawn per pixel from the interpolated distance, not
  // baked into the node colour (EdgeTrack.h).
  vec3 vcol = v_color;
#ifdef TERRAIN_EDGE
  // High draws the parcel-border track per pixel over the track-free colour;
  // Low (and any other mesh) is the other program and keeps it baked in v_color;
  // a chunk sent without a track-free colour (negative) does too.
  if (u_terrain > 0.5 && v_split.x >= 0.0)
    vcol = mix(v_split, v_trkc, edgeTrackWeight(v_trk));
#endif
  vec3 base = u_color * vcol;
  if (u_terrain > 0.5) {
    vec2 p = v_world.xy;
    if (u_matq > 0.5) {
      // Textured ground (TerrainMaterial.cpp): albedo and normal from photos.
      terrainMaterial(vcol, p, dist, base, n);
      wrap = clamp(dot(n, sun) * 0.45 + 0.55, 0.0, 1.0); wrap *= wrap;
      ambient = ambientLight(n) * u_ambstr;
    } else {
    float f1 = 1.0 - smoothstep(4.0, 24.0, dist);
    float f2 = 1.0 - smoothstep(25.0, 140.0, dist);
    float f3 = 1.0 - smoothstep(120.0, 800.0, dist);
    float v = (gn(p / 0.4) - 0.5) * 0.20 * f1 + (gn(p / 2.0) - 0.5) * 0.24 * f2 +
              (gn(p / 9.0) - 0.5) * 0.28 * f3 + (gn(p / 47.0) - 0.5) * 0.22;
    // Macro tint: hundreds-of-metres warm/cool drift, no distance fade, so the
    // far ground and the horizon plane vary too instead of reading as one colour.
    float m1 = gn(p / 310.0 + 3.7) - 0.5, m2 = gn(p / 95.0 + 9.1) - 0.5;
    vec3 macroTint = vec3(1.0 + 0.30 * m1 + 0.10 * m2, 1.0 + 0.10 * m1, 1.0 - 0.26 * m1 - 0.08 * m2);
    base *= (1.0 + v) * macroTint;
    }
    // Survey grid: a calm hint for the close ground, not a net. The fine lines
    // fade out within ~30 m and every line is gone by 60 m: from the air the
    // long straight lines read as seams in the ground, not as a scale.
    if (u_grid > 0.5) {
      float up = smoothstep(0.55, 0.85, -n.z);       // grid only on level ground
      float g = 0.030 * gridLine(p, 1.0, 0.012) * (1.0 - smoothstep(6.0, 30.0, dist)) +
                0.060 * gridLine(p, 10.0, 0.004) * (1.0 - smoothstep(30.0, 60.0, dist)) +
                0.100 * gridLine(p, 100.0, 0.0015) * (1.0 - smoothstep(30.0, 60.0, dist));
      base = mix(base, base * 0.55 + vec3(0.40, 0.41, 0.38), clamp(g * up, 0.0, 0.2));
    }
  }
  if (u_bands > 0.5) {                              // survey pole: 1 m bands
    float k = floor((v_local.z + 0.5) * u_bands);
    base = (mod(k, 2.0) < 0.5) ? vec3(0.80, 0.14, 0.08) : vec3(0.92, 0.92, 0.88);
  }
  float sunTerm = wrap * shadowFactor(v_world, nSmooth, sun);
  // Shaded faces and cast shadows get a sky-tinted fill (never near-black).
  vec3 lit  = base * (ambient + shadowFill() * u_ambstr * (1.0 - clamp(sunTerm, 0.0, 1.0)) +
                      sunCol * 0.95 * sunTerm);
  if (u_bands > 0.5) {
    // A survey pole is a thin vertical marker seen against bright snow and
    // rock, often from its sun-shaded side or in a terrain shadow; with only
    // ambient light its red/white bands both read as black. Light it from a
    // floor so the bands stay legible, and let the terrain shadow darken it
    // only partly.
    float sf = mix(1.0, shadowFactor(v_world, nSmooth, sun), 0.3);
    lit = base * (max(ambient, vec3(0.45)) + sunCol * wrap * 0.55 * sf);
  }
  // Aerial perspective: fade toward the sky behind the surface with distance,
  // so the streamed-terrain edge dissolves into haze.
  vec3 vdir = dist > 1e-4 ? toFrag / dist : vec3(0.0, 0.0, 1.0);
  // Haze: true distance, thinned with altitude (see kFogGLSL). Fog colour is
  // the horizon's, never the dark below-horizon sky.
  float fog = fogAmount(u_campos, v_world, u_fogstart, u_fogdensity);
  o_color = vec4(mix(lit, fogColor(vdir), clamp(fog, 0.0, 1.0)), 1.0);
}
)GLSL";

// Sky background: a fullscreen triangle (generated from gl_VertexID, no VBO)
// shaded by the shared skyColor() along the per-pixel world view ray, tonemapped
// to match the fogged terrain. Drawn first, depth test off.
const char *kSkyVertexShader = R"GLSL(
#version 330 core
out vec2 v_ndc;
void main() {
  const vec2 verts[3] = vec2[3](vec2(-1.0, -1.0), vec2(3.0, -1.0), vec2(-1.0, 3.0));
  v_ndc = verts[gl_VertexID];
  gl_Position = vec4(v_ndc, 1.0, 1.0);   // far plane
}
)GLSL";

const char *kSkyFragmentHead = R"GLSL(
#version 330 core
in vec2 v_ndc;
out vec4 o_color;
uniform mat4 u_invvp;   // inverse(proj * view)
uniform float u_time;   // cloud drift clock
)GLSL";

// Procedural overcast cloud deck layered on the sky gradient. Value-noise fBm is
// sampled on a parallax-projected dome plane (clouds spread toward the horizon)
// and drifts slowly with u_time. Kept ONLY in the sky shader so per-pixel fog on
// terrain/grass stays a cheap gradient (it still fades into the overcast base).
const char *kSkyFragmentMain = R"GLSL(
float h21(vec2 p){ p=fract(p*vec2(123.34,345.45)); p+=dot(p,p+34.345); return fract(p.x*p.y); }
float vnoise(vec2 p){ vec2 i=floor(p),f=fract(p); f=f*f*(3.0-2.0*f);
  float a=h21(i),b=h21(i+vec2(1,0)),c=h21(i+vec2(0,1)),d=h21(i+vec2(1,1));
  return mix(mix(a,b,f.x),mix(c,d,f.x),f.y); }
float cfbm(vec2 p){ float s=0.0,a=0.55; for(int i=0;i<5;++i){ s+=a*vnoise(p); p=p*2.03+11.7; a*=0.5; } return s; }
void main() {
  vec4 wn = u_invvp * vec4(v_ndc, -1.0, 1.0);
  vec4 wf = u_invvp * vec4(v_ndc,  1.0, 1.0);
  vec3 dir = normalize(wf.xyz / wf.w - wn.xyz / wn.w);
  float up = clamp(-dir.z, 0.0, 1.0);
  vec3 sky = skyColor(dir);

  // Parallax-project the ray onto a cloud plane and sample drifting fBm.
  vec2 cuv = dir.xy / max(-dir.z * 0.65 + 0.12, 0.10);
  cuv = cuv * 0.9 + u_time * vec2(0.012, 0.005);
  float cov = cfbm(cuv);
  // Coverage 1 = the original dense overcast; lower values open gaps.
  cov = smoothstep(mix(0.95, 0.30, u_cloudcover), mix(1.15, 0.92, u_cloudcover), cov);
  cov *= smoothstep(0.02, 0.22, up);          // no clouds right at the horizon
  // Cloud shading: dark teal-grey masses, lighter toward the diffused sun.
  float sunAmt = pow(max(dot(normalize(dir), normalize(u_sundir)), 0.0), 3.0);
  vec3 darkCloud = vec3(0.15, 0.19, 0.21);
  vec3 litCloud  = vec3(0.55, 0.59, 0.60);
  vec3 cloudCol = mix(darkCloud, litCloud, clamp(sunAmt * 0.8 + 0.18, 0.0, 1.0));
  sky = mix(sky, cloudCol, cov * 0.88);
  // Sun disc last, dimmed by the cloud in front of it.
  sky += sunDisc(dir) * (1.0 - 0.92 * cov);
  o_color = vec4(sky, 1.0);
}
)GLSL";

// Depth-only program for the shadow pass: terrain transformed into the sun's
// light-space clip; the rasteriser writes depth, no colour.
const char *kDepthVertexShader = R"GLSL(
#version 330 core
layout(location = 0) in vec3 a_pos;
uniform mat4 u_lightmvp;   // lightVP * model
uniform float u_push;      // pull the depth toward the sun by this much (clip z)
void main() {
  gl_Position = u_lightmvp * vec4(a_pos, 1.0);
  gl_Position.z -= u_push * gl_Position.w;
}
)GLSL";
const char *kDepthFragmentShader = R"GLSL(
#version 330 core
uniform float u_cover;   // 1 = solid; below 1 only that fraction of the texels is written
void main() {
  if (u_cover < 1.0) {
    // Ordered 4x4 dither fixed to the shadow texels (no flicker): the PCF of the
    // receiver averages it back to a partial shadow.
    ivec2 p = ivec2(gl_FragCoord.xy) & 3;
    int b = (((p.x ^ p.y) & 1) << 3) | ((p.y & 1) << 2) | ((((p.x ^ p.y) >> 1) & 1) << 1) |
            ((p.y >> 1) & 1);
    if ((float(b) + 0.5) / 16.0 >= u_cover)
      discard;
  }
}
)GLSL";

// Shadow-receive helper for the lit program: the cascade lookup (see
// ShadowCascades.h, spliced in before this) plus the facet-normal bias, the
// low-sun fade and the analytic drone shadow. 1 = lit, 0 = shadow; guarded by
// u_shadowon so the scene is unchanged when the map is unavailable.
const char *kShadowGLSL = R"GLSL(
uniform vec3 u_drone;     // drone position (world, NED)
uniform float u_droner;   // drone shadow radius [m]; 0 = off
uniform float u_dronemap; // 1 = the drone is drawn into the shadow map: no cylinder
uniform float u_dronebody; // 1 only while the drone's own meshes are drawn
// Analytic soft shadow of the drone: a cylinder along the sun ray from the
// drone, with the penumbra growing with distance. The shadow map cannot
// resolve a 0.25 m quad, and this needs no ground height, so it lands on any
// terrain and doubles as a height cue.
float droneShadow(vec3 wpos, vec3 lightDir) {
  if (u_droner <= 0.0 || u_dronemap > 0.5) return 1.0;
  vec3 L = normalize(lightDir);
  vec3 v = wpos - u_drone;
  float a = dot(v, -L);               // distance past the drone, away from the sun
  if (a < u_droner) return 1.0;       // not below the drone (also spares the body)
  float perp = length(v + L * a);
  float pen = u_droner * (1.0 + a * 0.03);
  float cover = 1.0 - smoothstep(pen * 0.45, pen * 1.35, perp);
  return 1.0 - 0.6 * cover / (1.0 + a / 150.0);
}
float shadowMapFactor(vec3 wpos, vec3 nrm, vec3 lightDir) {
  // The drone is its own caster: its own meshes (and only those) skip the map,
  // which would otherwise acne them. Everything else, the ground under a landed
  // drone included, gets the full shadow.
  if (u_dronebody > 0.5 && u_dronemap > 0.5) return 1.0;
  vec3 n = normalize(nrm);
  vec3 L = normalize(lightDir);
  // The interpolated normal is smooth but the mesh is faceted: on a steep face
  // they differ enough that a smooth-normal offset can push the lookup INTO the
  // facet. Offset along the true facet normal (screen derivatives) instead.
  vec3 ng = normalize(cross(dFdx(wpos), dFdy(wpos)));
  if (dot(ng, n) < 0.0) ng = -ng;
  float s = csmVisibility(wpos, ng, L, true);
  float ndg = dot(ng, L);   // facet N.L
  float ndl = dot(n, L);    // smooth N.L
  // A surface turned away from the sun is self-shadowed by definition; whether
  // the map calls it lit or dark flips triangle to triangle on a steep face. Fade
  // the map in over the first few percent of N.L, with the SMOOTH normal: the
  // facet normal flips between adjacent triangles at low sun, which paints a
  // per-triangle mosaic of lit and shadowed blocks on lee slopes. At a high sun
  // the smooth normal alone leaves saw-tooth hatching on steep sun-facing
  // slopes (facets turned away from the sun read dark between lit ones), so
  // there the facet N.L joins the fade, blended in with the sun's height.
  float ws = smoothstep(0.0, 0.10, ndl);
  float wf = min(ws, smoothstep(0.0, 0.10, ndg));
  float fade = mix(1.0, s, mix(ws, wf, smoothstep(0.55, 0.72, -L.z)));
  // Terminator gate: below N.L ~ 0.15 the wrapped diffuse would still light a
  // slope that faces away from or grazes the sun, brighter than the shadow fill
  // of the cast shadows beside it. Let the shadow factor take over there (smooth
  // normal, so it is uniform across a slope; 1 above 0.15). Only with a live map.
  float gate = mix(1.0, smoothstep(0.0, 0.15, ndl), step(0.5, u_shadowon));
  return fade * gate;
}
float shadowFactor(vec3 wpos, vec3 nrm, vec3 lightDir) {
  return shadowMapFactor(wpos, nrm, lightDir) * droneShadow(wpos, lightDir);
}
)GLSL";

#ifdef VAYU_SIM_GRASS
// Instanced grass/flora. A shared unit-blade mesh (crossed tapered quads, local
// z in [-1,0] = up) is drawn once per scattered instance: the vertex shader
// rotates it by the instance yaw, scales to its height, bends the tip with a
// world-space wind wave, and shrinks blades to nothing past a fade distance so
// the flora edge dissolves (the terrain fog finishes the job). Fragment shades a
// base->tip AO gradient over the instance tint, then the same distance fog into
// the sky as the terrain — so grass and ground share one atmosphere.
const char *kFloraVertexShader = R"GLSL(
#version 330 core
layout(location=0) in vec3 a_local;   // unit blade (z in [-1,0])
layout(location=1) in vec3 i_pos;     // world base (NED)
layout(location=2) in vec3 i_yhf;     // yaw, height, flower
layout(location=3) in vec3 i_tint;
layout(location=4) in vec3 a_normal;  // ribbon surface normal (local)
uniform mat4 u_vp;
uniform vec3 u_campos;
uniform float u_time;
uniform float u_fadestart;
uniform float u_fadeend;
out vec3 v_color;
out vec3 v_world;
out vec3 v_normal;
out float v_hf;
out float v_flower;
void main() {
  float yaw = i_yhf.x, height = i_yhf.y, flower = i_yhf.z;
  float hf = -a_local.z;                       // 0 base .. 1 tip
  // Distance fade: shrink height to 0 between fadestart..fadeend.
  float d = length(i_pos - u_campos);
  float fade = 1.0 - clamp((d - u_fadestart) / max(u_fadeend - u_fadestart, 1.0),
                           0.0, 1.0);
  height *= fade;
  // Scale the whole curved blade (arc + width + length) by its height.
  vec3 L = a_local * height;
  L.xy *= (1.0 + flower * hf * hf * 2.0);        // flowers fan a small bloom
  float s = sin(yaw), c = cos(yaw);
  vec3 r = vec3(c * L.x - s * L.y, s * L.x + c * L.y, L.z);
  // Wind: extra bend on top of the baked arc, strongest near the tip.
  float w = sin(u_time * 1.6 + i_pos.x * 0.22 + i_pos.y * 0.18);
  vec2 bend = vec2(0.80, 0.55) * (w * 0.14 * height * hf * hf);
  vec3 world = i_pos + vec3(r.xy + bend, r.z);
  v_world = world;
  v_color = i_tint;
  v_hf = hf;
  v_flower = flower;
  // Rotate the (uniform-scaled) ribbon normal by the same yaw.
  v_normal = vec3(c * a_normal.x - s * a_normal.y,
                  s * a_normal.x + c * a_normal.y, a_normal.z);
  gl_Position = u_vp * vec4(world, 1.0);
}
)GLSL";

const char *kFloraFragmentHead = R"GLSL(
#version 330 core
in vec3 v_color;
in vec3 v_world;
in vec3 v_normal;
in float v_hf;
in float v_flower;
out vec4 o_color;
uniform vec3 u_campos;
uniform float u_fogdensity;
uniform float u_fogstart;
)GLSL";

const char *kFloraFragmentMain = R"GLSL(
void main() {
  // Flowers keep a green stem and only bloom their colour near the tip.
  vec3 stem = vec3(0.28, 0.46, 0.18);
  vec3 albedo = mix(v_color, mix(stem, v_color, smoothstep(0.6, 0.95, v_hf)),
                    v_flower);
  // Per-blade lighting: directional sun + sky/ground hemispheric ambient off the
  // (up-biased) ribbon normal, with a base->tip ambient-occlusion gradient.
  vec3 n = normalize(v_normal);
  float ndl = max(dot(n, normalize(u_sundir)), 0.0);
  float hemi = 0.5 + 0.5 * (-n.z);             // up-facing catches sky
  vec3 ambient = mix(vec3(0.22, 0.24, 0.28),
                     vec3(0.50, 0.53, 0.58), clamp(hemi, 0.0, 1.0));
  float ao = mix(0.55, 1.0, v_hf);             // darker at the base
  vec3 col = albedo * (ambient + vec3(0.85) * ndl) * ao;
  vec3 toFrag = v_world - u_campos;
  float dist = length(toFrag);
  vec3 vdir = dist > 1e-4 ? toFrag / dist : vec3(0.0, 0.0, 1.0);
  // Subsurface translucency: blades glow when backlit (looking toward the sun
  // through them), strongest near the thin tip — the soft GoT meadow look.
  float trans = pow(max(dot(vdir, normalize(u_sundir)), 0.0), 4.0);
  col += albedo * trans * (0.25 + 0.75 * v_hf) * 0.8;
  float fog = fogAmount(u_campos, v_world, u_fogstart, u_fogdensity);
  o_color = vec4(mix(col, fogColor(vdir), clamp(fog, 0.0, 1.0)), 1.0);
}
)GLSL";
#endif // VAYU_SIM_GRASS

} // namespace

SimRendererWidget::SimRendererWidget(QWindow *parent)
    : QOpenGLWindow(NoPartialUpdate, parent) {
  // The widget used to get depth/stencil from its FBO; a window must ask for
  // them (QPainter needs the stencil for the HUD's clipping).
  // (Mouse tracking needs no switch here: a QWindow always receives moves with
  // no button held, which the G/R/S gizmo tools rely on.)
  QSurfaceFormat f = QSurfaceFormat::defaultFormat();
  f.setDepthBufferSize(24);
  f.setStencilBufferSize(8);
  setFormat(f);
  veg_ = std::make_unique<VegetationRenderer>();
  water_ = std::make_unique<WaterRenderer>();
  vegEnabled_ = VegetationRenderer::enabledByConfig();
}

void SimRendererWidget::setHudWidget(QWidget *w) {
  if (hud_)
    disconnect(hud_, nullptr, this, nullptr);
  hud_ = w;
  hudDirty_ = true;
  hudHasSignal_ = w && w->metaObject()->indexOfSignal("changed()") >= 0;
  if (hudHasSignal_)
    connect(w, SIGNAL(changed()), this, SLOT(invalidateHud()));
  update();
}

void SimRendererWidget::setHudVisible(bool on) {
  if (on && !hudVisible_)
    hudDirty_ = true; // content may have changed while it was hidden
  hudVisible_ = on;
  update();
}

void SimRendererWidget::invalidateHud() {
  hudDirty_ = true;
  if (hudVisible_)
    update();
}

QImage SimRendererWidget::grabFrame(int timeoutMs) {
  if (!isExposed())
    return QImage();
  QEventLoop loop;
  QTimer guard;
  guard.setSingleShot(true);
  connect(&guard, &QTimer::timeout, &loop, &QEventLoop::quit);
  int swaps = 0; // the first swap may be a frame that started before this call
  const QMetaObject::Connection c =
      connect(this, &QOpenGLWindow::frameSwapped, &loop, [&] {
        if (++swaps >= 2)
          loop.quit();
        else
          update();
      });
  update();
  guard.start(timeoutMs);
  loop.exec();
  disconnect(c);
  return grabFramebuffer();
}

// QPainter leaves blending, depth, stencil, scissor, the bound program and VAO
// in its own state; every frame starts from the one the scene expects.
void SimRendererWidget::resetGlState() {
  glBindFramebuffer(GL_FRAMEBUFFER, context()->defaultFramebufferObject());
  glViewport(0, 0, fbW_, fbH_);
  glUseProgram(0);
  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, 0);
  glDisable(GL_BLEND);
  glDisable(GL_SCISSOR_TEST);
  glDisable(GL_STENCIL_TEST);
  glDisable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glDepthMask(GL_TRUE);
  glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
  glEnable(GL_LINE_SMOOTH);
  glClearColor(0.08f, 0.09f, 0.11f, 1.0f);
}

// Composites the HUD widget over the finished frame. The widget is never on
// screen: it is rendered into an ARGB image at the window's size (device pixels
// honoured) and drawn with QPainter on the GL window.
void SimRendererWidget::paintHud() {
  if (!hud_ || !hudVisible_ || width() <= 0 || height() <= 0)
    return;
  QElapsedTimer t;
  t.start();
  const qreal dpr = devicePixelRatio();
  const QSize sz(width(), height());
  // Re-render only when the HUD changed, the size / pixel ratio changed, or
  // the HUD cannot tell us (no changed() signal) or caching is off. An image
  // that is unchanged keeps its cacheKey, so QPainter reuses the texture it
  // already uploaded instead of sending the pixels again.
  if (!hudCache_ || !hudHasSignal_ || hudDirty_ || sz != hudSize_ ||
      dpr != hudDpr_ || hudImg_.isNull()) {
    hudImg_ = QImage(sz * dpr, QImage::Format_ARGB32_Premultiplied);
    hudImg_.setDevicePixelRatio(dpr);
    hudImg_.fill(Qt::transparent);
    hud_->resize(sz);
    hud_->render(&hudImg_);
    hudSize_ = sz;
    hudDpr_ = dpr;
    hudDirty_ = false;
    ++hudRenders_;
  }
  glDisable(GL_DEPTH_TEST);
  QPainter p(this);
  p.drawImage(0, 0, hudImg_);
  p.end();
  resetGlState();
  hudNs_ += t.nsecsElapsed();
}

void SimRendererWidget::setMotorsEditable(bool on) {
  editable_ = on;
  if (!on) { // leaving edit mode: drop any selection / live tool
    if (tool_ != Tool::None)
      commitTool(false);
    selected_ = -1;
  }
  update();
}

void SimRendererWidget::setObstacleEditMode(bool on) {
  obsMode_ = on;
  if (!on) {
    if (obsTool_ != Tool::None)
      commitObsTool(false);
    selObs_ = -1;
  }
  update();
}

void SimRendererWidget::selectObstacle(int index) {
  if (index < 0 || index >= obstacles_.size())
    index = -1;
  if (index == selObs_)
    return;
  if (obsTool_ != Tool::None)
    commitObsTool(false);
  selObs_ = index;
  update();
}

namespace {
std::atomic<int> g_chunkReadBacks{0};
}
int SimRendererWidget::chunkReadBacks() { return g_chunkReadBacks; }

SimRendererWidget::~SimRendererWidget() {
  // Free the GL side while a context is still current. Nothing will use the
  // streamed data again, so no read-back (readBack=false). Disconnect first so
  // the context, which QOpenGLWindow deletes after this body has run, cannot
  // call back into a half-destroyed object.
  if (glCtx_)
    disconnect(glCtx_, nullptr, this, nullptr);
  if (glLive_) {
    makeCurrent();
    releaseGLResources(false);
    doneCurrent();
  }
}

// Chunk bookkeeping that frees the GL objects: a Mesh's destructor does not
// delete its QOpenGLBuffer, so every drop goes through release() first.
bool SimRendererWidget::chunkCpuMode() const {
  return forceChunkCpu_ || (context() && context()->isOpenGLES());
}

void SimRendererWidget::eraseChunk(qint64 key) {
  const auto it = worldChunks_.find(key);
  if (it == worldChunks_.end())
    return;
  it->second->release();
  worldChunks_.erase(it);
  ++shadowGeomVer_;
  chunkCpu_.erase(key);
}

void SimRendererWidget::eraseAllChunks() {
  for (auto &kv : worldChunks_)
    kv.second->release();
  ++shadowGeomVer_;
  worldChunks_.clear();
  chunkCpu_.clear();
}

template <class F> void SimRendererWidget::forEachMesh(F &&f) {
  for (Mesh *m :
       {&ground_,    &unitBox_,     &unitSphere_,  &horizonQuad_, &unitCyl_,
        &helipadDisk_, &helipadMark_, &axes_,       &body_,        &modelBody_,
        &rotor_,     &thrustLine_,  &comMarker_,   &gizmoArrow_,  &gizmoRing_,
        &locator_,   &northArrow_,  &glyphN_,      &ring_,        &guideArrow_,
        &droneMesh_, &worldMesh_, &farMesh_})
    f(*m);
  for (Mesh &m : modelProp_)
    f(m);
  for (Mesh &m : digitMesh_)
    f(m);
}

int SimRendererWidget::liveGlObjects() const {
  int n = 0;
  const_cast<SimRendererWidget *>(this)->forEachMesh(
      [&](Mesh &m) { n += m.liveObjects(); });
  for (const auto &kv : worldChunks_)
    n += kv.second->liveObjects();
  n += skyVao_.isCreated() ? 1 : 0;
  n += (prog_ ? 1 : 0) + (progLit_ ? 1 : 0) + (progLitEdge_ ? 1 : 0) + (progDepth_ ? 1 : 0) +
       (progSky_ ? 1 : 0);
  n += (shadowFbo_ ? 1 : 0) + (shadowTex_ ? 1 : 0);
  n += terrainTex_.loaded() ? 6 : 0;
  if (veg_)
    n += veg_->liveObjects();
  if (water_)
    n += water_->liveObjects();
#ifdef VAYU_SIM_GRASS
  n += progFlora_ ? 1 : 0;
  for (const QOpenGLBuffer &b : grassVbo_)
    n += b.isCreated() ? 1 : 0;
  n += floraVao_.isCreated() ? 1 : 0;
  for (const auto &kv : floraChunks_)
    n += kv.second->inst.isCreated() ? 1 : 0;
  n += gpuGrass_.liveObjects();
#endif
  return n;
}

// Frees every GL object (the context must be current) and puts back into the
// pending queues whatever the next context has to re-upload. Nothing here may
// assume the objects exist: it runs from the destructor and from the context's
// aboutToBeDestroyed, possibly both.
void SimRendererWidget::releaseGLResources(bool readBack) {
  if (!glLive_)
    return;
  glLive_ = false;

  // Streamed terrain chunks normally have no CPU copy (a copy would double the
  // memory of the whole loaded terrain), so when a following context needs
  // them each VBO is read back into the upload queue before the buffer goes.
  // glGetBufferSubData does not exist on GLES, so there (or when forced) the
  // copy made at upload time is used instead. Teardown (destructor, context
  // going away) reads nothing: the data is not wanted and the context may be
  // on its way out. Re-queued chunks go ahead of anything still pending, which
  // is newer and overrides them by key.
  if (clearAllChunks_) { // a queued clear: nothing of the old set survives
    eraseAllChunks();
    pendingChunkRemovals_.clear();
    clearAllChunks_ = false;
  }
  for (const qint64 key : pendingChunkRemovals_)
    eraseChunk(key);
  pendingChunkRemovals_.clear();
  if (readBack) {
    std::vector<PendingChunk> back;
    for (auto &kv : worldChunks_) {
      PendingChunk pc;
      pc.key = kv.first;
      Mesh &m = *kv.second;
      const auto cit = chunkCpu_.find(kv.first);
      if (cit != chunkCpu_.end()) {
        pc.data = std::move(cit->second);
        back.push_back(std::move(pc));
      } else if (!chunkCpuMode() && m.vbo.isCreated() && m.vbo.bind()) {
        const int bytes = m.vbo.size();
        pc.data.resize(size_t(std::max(bytes, 0)) / sizeof(float));
        ++g_chunkReadBacks;
        if (bytes > 0 && m.vbo.read(0, pc.data.data(), bytes))
          back.push_back(std::move(pc));
        else
          qWarning("[SimRenderer] chunk %lld could not be read back; dropped",
                   static_cast<long long>(kv.first));
        m.vbo.release();
      }
    }
    for (PendingChunk &pc : pendingChunkUploads_)
      back.push_back(std::move(pc));
    pendingChunkUploads_ = std::move(back);
    chunksDirty_ = !pendingChunkUploads_.empty();
  }
  eraseAllChunks();

#ifdef VAYU_SIM_GRASS
  if (clearAllFlora_) {
    eraseAllFlora();
    pendingFloraRemovals_.clear();
    clearAllFlora_ = false;
  }
  for (const qint64 key : pendingFloraRemovals_)
    eraseFlora(key);
  pendingFloraRemovals_.clear();
  if (readBack) {
    std::vector<PendingFlora> floraBack;
    for (auto &kv : floraChunks_) {
      FloraChunk &fc = *kv.second;
      const auto cit = floraCpu_.find(kv.first);
      if (cit != floraCpu_.end()) {
        floraBack.push_back(std::move(cit->second));
      } else if (!chunkCpuMode() && fc.inst.isCreated() && fc.inst.bind()) {
        PendingFlora pf;
        pf.key = kv.first;
        pf.count = fc.count;
        pf.cx = fc.cx;
        pf.cy = fc.cy;
        pf.half = fc.half;
        const int bytes = fc.inst.size();
        pf.data.resize(size_t(std::max(bytes, 0)) / sizeof(float));
        ++g_chunkReadBacks;
        if (bytes > 0 && fc.inst.read(0, pf.data.data(), bytes))
          floraBack.push_back(std::move(pf));
        fc.inst.release();
      }
    }
    for (PendingFlora &pf : pendingFloraUploads_)
      floraBack.push_back(std::move(pf));
    pendingFloraUploads_ = std::move(floraBack);
    floraDirty_ = !pendingFloraUploads_.empty();
  }
  eraseAllFlora();
  gpuGrass_.release(this);
  for (QOpenGLBuffer &b : grassVbo_)
    b.destroy();
  for (int &v : grassVerts_)
    v = 0;
  floraVao_.destroy();
  progFlora_.reset();
#endif

  if (veg_)
    veg_->releaseGL(); // instance lists stay, GL objects rebuild on demand
  if (water_)
    water_->releaseGL(); // bodies stay, GL objects rebuild on demand
  forEachMesh([](Mesh &m) { m.release(); });
  skyVao_.destroy();
  prog_.reset();
  progLit_.reset();
  progLitEdge_.reset();
  progDepth_.reset();
  progSky_.reset();
  if (shadowFbo_)
    glDeleteFramebuffers(1, &shadowFbo_);
  if (shadowTex_)
    glDeleteTextures(1, &shadowTex_);
  shadowFbo_ = shadowTex_ = 0;
  terrainTex_.release(this, true);
  shadowReady_ = shadowOn_ = false;

  // Everything CPU-side that the GL objects were built from is still held:
  // the drone mesh and the world mesh (pendingPos_/pendingWorldPos_), the
  // layout the procedural airframe is rebuilt from, and the static meshes are
  // regenerated by initResources. Flag the lazy uploads to run again.
  meshDirty_ = true;
  worldMeshDirty_ = true;
  modelDirty_ = true;
}

void SimRendererWidget::resetGlResources() {
  makeCurrent();
  releaseGLResources(true); // same context stays; later frames need the data
  initResources();
  doneCurrent();
  update();
}

// aboutToBeDestroyed slot: the window's native context is being destroyed
// (undock / re-create) but THIS renderer lives on and will get a new context
// from initializeGL. Qt emits the signal with the context still current-able,
// so the VBOs can be read: readBack=true re-queues the streamed terrain and
// flora, whose only copy is on the GPU, for the next context. Safe to call
// when nothing is live.
void SimRendererWidget::onContextGone() {
  if (!glLive_)
    return;
  ++goneCount_;
  makeCurrent();
  goneCurrent_ = QOpenGLContext::currentContext() == glCtx_;
  releaseGLResources(true);
  doneCurrent();
}

void SimRendererWidget::setSnapshot(const vsim::SimSnapshot &s) {
  snap_ = s;
  update(); // schedules paintGL on the GUI thread
}

void SimRendererWidget::setDroneMesh(const std::vector<QVector3D> &positions,
                                     const std::vector<QVector3D> &normals) {
  pendingPos_ = positions;
  pendingNrm_ = normals;
  meshDirty_ = true; // uploaded lazily in paintGL (needs GL context)
  update();
}

void SimRendererWidget::setMotorLayout(const std::array<QVector3D, 4> &pos,
                                       const std::array<QVector3D, 4> &axis,
                                       const std::array<int, 4> &spin) {
  motorPos_ = pos;
  motorAxis_ = axis;
  motorSpin_ = spin;
  modelDirty_ = true;
  update();
}

void SimRendererWidget::setAirframeStyle(const QString &presetId,
                                         float propRadius) {
  if (presetId == presetId_ && propRadius == propRadius_)
    return;
  presetId_ = presetId;
  propRadius_ = propRadius;
  modelDirty_ = true;
  update();
}

void SimRendererWidget::rebuildDroneModel() {
  modelDirty_ = false;
  vsim::DroneModelInput in;
  in.propRadius = propRadius_;
  for (int i = 0; i < 4; ++i) {
    in.motorPos[i] = {motorPos_[i].x(), motorPos_[i].y(), motorPos_[i].z()};
    in.spin[i] = motorSpin_[i];
  }
  in.cls = vsim::droneClassFor(presetId_.toStdString(), propRadius_);
  model_ = vsim::buildDroneModel(in);
  modelLift_ = model_.lift;
  const vsim::OrbitRange orbit = vsim::orbitRangeFor(model_);
  camMinRadius_ = orbit.min;
  camMaxRadius_ = orbit.max;
  cam_radius_ = userZoomed_ ? std::clamp(cam_radius_, camMinRadius_, camMaxRadius_)
                            : orbit.def;
  uploadColoredMesh(modelBody_, model_.body.data);
  for (int i = 0; i < 4; ++i)
    uploadColoredMesh(modelProp_[i], model_.prop[i].data);
}

// Advance each prop by the real motor speed. The drawn rate is soft-capped
// (tanh) below the blade-passing alias limit (~0.4 of the blade spacing per
// frame) so it never strobes or appears to run backwards; the motion-blur
// disc alpha carries "how fast" instead.
void SimRendererWidget::updateRotors(float dt) {
  for (int i = 0; i < 4; ++i) {
    const float rps = std::fabs(snap_.motor_omega[i]) / (2.0f * float(M_PI));
    rotorAngle_[i] = vsim::advanceRotorAngle(
        rotorAngle_[i], motorSpin_[i], snap_.motor_omega[i], model_.bladeCount,
        dt);
    const float t = std::clamp((rps - 25.0f) / 55.0f, 0.0f, 1.0f);
    rotorBlur_[i] = t * t * (3.0f - 2.0f * t);
  }
}

// When the model covers fewer than ~12 px, draw a fixed-size heading chevron
// at its screen position so the drone can always be found. Pixel size is
// estimated from the model span and the camera distance.
void SimRendererWidget::drawDroneLocator(const QMatrix4x4 &view,
                                         const QMatrix4x4 &model) {
  float span = model_.visualPropRadius * 2.0f;
  for (int i = 0; i < 4; ++i)
    span = std::max(span, 2.0f * (std::hypot(model_.hub[i][0], model_.hub[i][1]) +
                                  model_.visualPropRadius));
  const float d = (camEye_ - snap_.pos_w).length();
  const float pxSize = span * 1.5f / std::max(0.01f, d * 2.0f * std::tan(
                                       qDegreesToRadians(30.0f))) * float(fbH_);
  if (pxSize >= 12.0f || !std::isfinite(pxSize))
    return;
  const QMatrix4x4 vp = proj_ * view;
  const QVector4D c = vp * (model * QVector4D(0, 0, 0, 1));
  const QVector4D f = vp * (model * QVector4D(1, 0, 0, 1));
  if (c.w() <= 0.0f || f.w() <= 0.0f)
    return;
  const QVector2D pc(c.x() / c.w() * 0.5f * fbW_, c.y() / c.w() * 0.5f * fbH_);
  const QVector2D pf(f.x() / f.w() * 0.5f * fbW_, f.y() / f.w() * 0.5f * fbH_);
  QVector2D dir = pf - pc;
  if (dir.lengthSquared() < 1e-6f)
    dir = QVector2D(0, 1);
  const float ang = qRadiansToDegrees(std::atan2(dir.y(), dir.x())) - 90.0f;
  const float r = 9.0f; // chevron half-size [px]
  QMatrix4x4 m;
  m.translate(c.x() / c.w(), c.y() / c.w(), 0.0f);
  m.scale(r / (0.5f * fbW_), r / (0.5f * fbH_), 1.0f);
  m.rotate(ang, 0.0f, 0.0f, 1.0f);
  const QMatrix4x4 savedProj = proj_;
  proj_.setToIdentity();
  glDisable(GL_DEPTH_TEST);
  drawMesh(locator_, m, QVector3D(1.0f, 0.9f, 0.1f));
  glEnable(GL_DEPTH_TEST);
  proj_ = savedProj;
}

float SimRendererWidget::droneVisualScale() const {
  const float d = (camEye_ - snap_.pos_w).length();
  return std::clamp(1.0f + (d - 12.0f) / 40.0f, 1.0f, 1.5f);
}

void SimRendererWidget::drawDroneModel(const QMatrix4x4 &view,
                                       const QMatrix4x4 &modelIn) {
  // Far-visibility boost: past ~12 m the model grows gently about its origin
  // (x1.2 at 20 m, x1.5 from ~32 m), capped so size cues stay honest; the
  // screen-space locator covers the distance where it is still tiny. Not in
  // the editor, where the geometry must read true to scale.
  QMatrix4x4 model = modelIn;
  if (!editable_)
    model.scale(droneVisualScale());
  drawingDroneBody_ = true;
  drawLit(modelBody_, view, model, QVector3D(1, 1, 1));
  for (int i = 0; i < 4; ++i) {
    QMatrix4x4 m = model;
    m.translate(model_.hub[i][0], model_.hub[i][1], model_.hub[i][2]);
    // +angle about body -Z (up) = counter-clockwise seen from above.
    m.rotate(qRadiansToDegrees(rotorAngle_[i]), 0.0f, 0.0f, -1.0f);
    drawLit(modelProp_[i], view, m, QVector3D(1, 1, 1));
  }
  drawingDroneBody_ = false;
  // Motion-blur discs, after the opaque solids: constant-alpha blend so the
  // flat-colour program needs no alpha channel.
  bool any = false;
  for (float a : rotorBlur_)
    any = any || a > 0.01f;
  if (!any)
    return;
  glEnable(GL_BLEND);
  glDepthMask(GL_FALSE);
  const float s = model_.visualPropRadius / kRotorR;
  for (int i = 0; i < 4; ++i) {
    if (rotorBlur_[i] <= 0.01f)
      continue;
    glBlendColor(0, 0, 0, 0.36f * rotorBlur_[i]);
    glBlendFunc(GL_CONSTANT_ALPHA, GL_ONE_MINUS_CONSTANT_ALPHA);
    QMatrix4x4 m = model;
    m.translate(model_.hub[i][0], model_.hub[i][1], model_.hub[i][2] - 0.0006f);
    m.scale(s, s, 1.0f);
    const auto tip = vsim::propTipColor(motorSpin_[i]);
    drawMesh(rotor_, view * m,
             QVector3D(0.55f + 0.25f * tip[0], 0.58f + 0.25f * tip[1],
                       0.62f + 0.25f * tip[2]));
  }
  glDepthMask(GL_TRUE);
  glDisable(GL_BLEND);
}

void SimRendererWidget::setComMarker(const QVector3D &com) {
  comOffset_ = com;
  update();
}

void SimRendererWidget::initializeGL() {
  // Qt calls this even when the window could not get a context (no GL on this
  // platform); every GL call below would crash.
  if (!context() || !context()->isValid())
    return;
  initResources();
}

namespace {
std::unique_ptr<QOpenGLShaderProgram>
buildProgram(const char *name, const QByteArray &vs, const QByteArray &fs) {
  auto p = std::make_unique<QOpenGLShaderProgram>();
  if (!p->addShaderFromSourceCode(QOpenGLShader::Vertex, vs) ||
      !p->addShaderFromSourceCode(QOpenGLShader::Fragment, fs) || !p->link())
    qWarning("[SimRenderer] %s program failed:\n%s", name,
             p->log().toLocal8Bit().constData());
  return p;
}
} // namespace

void SimRendererWidget::LitLocs::resolve(QOpenGLShaderProgram *p) {
  mvp = p->uniformLocation("u_mvp");
  model = p->uniformLocation("u_model");
  nmat = p->uniformLocation("u_nmat");
  color = p->uniformLocation("u_color");
  campos = p->uniformLocation("u_campos");
  fogdensity = p->uniformLocation("u_fogdensity");
  fogstart = p->uniformLocation("u_fogstart");
  csm.resolve(p);
  drone = p->uniformLocation("u_drone");
  droner = p->uniformLocation("u_droner");
  sunint = p->uniformLocation("u_sunint");
  ambstr = p->uniformLocation("u_ambstr");
  terrain = p->uniformLocation("u_terrain");
  grid = p->uniformLocation("u_grid");
  bands = p->uniformLocation("u_bands");
  matq = p->uniformLocation("u_matq");
}

void SimRendererWidget::initResources() {
  // A new context: anything still held belongs to the previous one and is
  // released first, so init can run any number of times without leaking or
  // adding shaders to an already-linked program.
  // Handles of a context that is already gone: the objects cannot be read
  // (and onContextGone, which does read, has already run when it could).
  if (glLive_)
    releaseGLResources(false);
  initializeOpenGLFunctions();
  if (context() != glCtx_) {
    glCtx_ = context();
    connect(glCtx_, &QOpenGLContext::aboutToBeDestroyed, this,
            &SimRendererWidget::onContextGone, Qt::DirectConnection);
  }
  glLive_ = true;
  ++glInitCount_;
  glClearColor(0.08f, 0.09f, 0.11f, 1.0f);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LINE_SMOOTH);

  prog_ = buildProgram("flat", kVertexShader, kFragmentShader);
  u_mvp_ = prog_->uniformLocation("u_mvp");
  u_color_ = prog_->uniformLocation("u_color");

  {
    const QByteArray fsBody = QByteArray(kLitFragmentHead) + kAtmosphereGLSL + kFogGLSL +
                              kLightingGLSL + kCascadeShadowGLSL + kShadowGLSL +
                              vsim::terrain::terrainMaterialGLSL() + kLitFragmentMain;
    // The edge build is the same source with TERRAIN_EDGE defined after #version.
    auto withEdge = [](QByteArray src) {
      return src.replace("#version 330 core\n", "#version 330 core\n#define TERRAIN_EDGE 1\n");
    };
    progLit_ = buildProgram("lit", kLitVertexShader, fsBody);
    progLitEdge_ = buildProgram("lit+edge", withEdge(kLitVertexShader), withEdge(fsBody));
    litLoc_.resolve(progLit_.get());
    litLocEdge_.resolve(progLitEdge_.get());
  }
  terrainDefault_ = vsim::terrain::defaultQuality(
      reinterpret_cast<const char *>(glGetString(GL_RENDERER)));

  progDepth_ = buildProgram("depth", kDepthVertexShader, kDepthFragmentShader);
  ud_lightmvp_ = progDepth_->uniformLocation("u_lightmvp");
  ud_cover_ = progDepth_->uniformLocation("u_cover");
  ud_push_ = progDepth_->uniformLocation("u_push");
  buildShadowMap();

  progSky_ = buildProgram("sky", kSkyVertexShader,
                          QByteArray(kSkyFragmentHead) + kAtmosphereGLSL +
                              kSkyFragmentMain);
  us_invvp_ = progSky_->uniformLocation("u_invvp");
  us_time_ = progSky_->uniformLocation("u_time");
  skyVao_.create(); // core profile needs a bound VAO even with no attributes

#ifdef VAYU_SIM_GRASS
  progFlora_ = buildProgram("flora", kFloraVertexShader,
                            QByteArray(kFloraFragmentHead) + kAtmosphereGLSL + kFogGLSL +
                                kFloraFragmentMain);
  uf_vp_ = progFlora_->uniformLocation("u_vp");
  uf_campos_ = progFlora_->uniformLocation("u_campos");
  uf_time_ = progFlora_->uniformLocation("u_time");
  uf_fadestart_ = progFlora_->uniformLocation("u_fadestart");
  uf_fadeend_ = progFlora_->uniformLocation("u_fadeend");
  uf_fogdensity_ = progFlora_->uniformLocation("u_fogdensity");
  uf_fogstart_ = progFlora_->uniformLocation("u_fogstart");
  buildGrassBlade();
  gpuGrass_.init(this); // GPU grass if the context supports compute (4.3+)
#endif

  buildGroundGrid();
  {
    // Unit quad in the XY plane, normal up (NED -Z): the horizon ground.
    std::vector<float> v;
    const float c[4][2] = {{-0.5f, -0.5f}, {0.5f, -0.5f}, {0.5f, 0.5f}, {-0.5f, 0.5f}};
    for (int i : {0, 1, 2, 0, 2, 3})
      v.insert(v.end(), {c[i][0], c[i][1], 0.0f, 0.0f, 0.0f, -1.0f});
    uploadLitMesh(horizonQuad_, v);
  }
  buildObstacleMeshes();
  buildHelipadMeshes();
  buildAxes();
  buildDroneBody();
  buildRotorDisk();
  buildThrustLine();
  buildComMarker();
  buildGizmo();
  buildDigits();
  buildNorth();
  buildRing();
  buildGuideArrow();
}

void SimRendererWidget::resizeGL(int, int) {
  // Device pixels, whatever the framework passes: the GL viewport and the
  // projection aspect need them; input and picking stay in logical pixels.
  fbW_ = std::max(1, int(std::lround(width() * devicePixelRatio())));
  fbH_ = std::max(1, int(std::lround(height() * devicePixelRatio())));
  if (!context() || !context()->isValid())
    return; // no GL here: Qt still calls this
  glViewport(0, 0, fbW_, fbH_);
  nearCur_ = 0.0f; // force a rebuild with the new aspect
  updateProjection();
}

void SimRendererWidget::updateProjection() {
  const float nearM = (fpv_ || downCam_)
                          ? kNearLensM
                          : vsim::chaseNearPlane(cam_radius_, kNearChaseM);
  if (nearM == nearCur_ || fbH_ <= 0)
    return;
  nearCur_ = nearM;
  proj_.setToIdentity();
  proj_.perspective(kFovYDeg, float(fbW_) / std::max(1, fbH_), nearM, kFarPlaneM);
}

QVector3D SimRendererWidget::freeForward() const {
  // Same yaw/pitch convention as the orbit offset, but pointing FROM the
  // camera (= -offset direction), so swapping orbit<->free keeps the view
  // pointing the same way.
  return QVector3D(-std::cos(cam_pitch_) * std::cos(cam_yaw_),
                   -std::cos(cam_pitch_) * std::sin(cam_yaw_),
                   std::sin(cam_pitch_))
      .normalized();
}

float SimRendererWidget::viewHeadingRad() const {
  if (freeFly_) {
    const QVector3D f = freeForward();
    return std::atan2(f.y(), f.x()); // NED: x=north, y=east
  }
  return bodyYawRad();
}

float SimRendererWidget::bodyYawRad() const {
  // NED yaw (heading about world +Z) from the body->world quaternion.
  const float w = snap_.att.scalar(), x = snap_.att.x(), y = snap_.att.y(),
              z = snap_.att.z();
  return std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

QVector3D SimRendererWidget::orbitOffset() const {
  // Azimuth = relative drag offset + the drone's heading, so the camera
  // rotates WITH the vehicle. NED up is -Z (pitch>0 lifts the eye above).
  const float az = cam_yaw_ + bodyYawRad();
  return QVector3D(cam_radius_ * std::cos(cam_pitch_) * std::cos(az),
                   cam_radius_ * std::cos(cam_pitch_) * std::sin(az),
                   -cam_radius_ * std::sin(cam_pitch_));
}

void SimRendererWidget::setFreeFly(bool on) {
  if (on && !freeFly_) {
    // Seed the free camera at the current orbit eye for a seamless handoff.
    camPos_ = snap_.pos_w + orbitOffset();
  }
  freeFly_ = on;
  update();
}

void SimRendererWidget::setWorldVisible(bool on) {
  worldVisible_ = on;
  update();
}

QMatrix4x4 SimRendererWidget::cameraView() const {
  if (downCam_) {
    // Underbelly camera: rigidly mounted just beneath the airframe, looking
    // straight down the body's +Z (down) axis. It rides the drone's attitude,
    // so it banks with the vehicle, and renders the world below (imported mesh,
    // ground, obstacles). Body +X (forward/north) maps to screen-up.
    const QVector3D down = snap_.att.rotatedVector(QVector3D(0, 0, 1));
    const QVector3D fwd = snap_.att.rotatedVector(QVector3D(1, 0, 0));
    const QVector3D eye =
        snap_.pos_w +
        snap_.att.rotatedVector(QVector3D(0.0f, 0.0f, downCamOffset_));
    QMatrix4x4 view;
    view.lookAt(eye, eye + down, fwd);
    return view;
  }
  if (freeFly_ && !fpv_) {
    // Free-roam: look along freeForward() from camPos_, not locked to drone.
    const QVector3D fwd = freeForward();
    QMatrix4x4 view;
    // Straight down (or up) the world up vector is parallel to the view and the
    // basis degenerates (a blank frame): take the heading as the image's up.
    QVector3D up(0.0f, 0.0f, -1.0f);
    if (std::fabs(fwd.z()) > 0.9999f) {
      const float sgn = fwd.z() > 0.0f ? 1.0f : -1.0f;
      up = QVector3D(-std::cos(cam_yaw_) * sgn, -std::sin(cam_yaw_) * sgn, 0.0f);
    }
    view.lookAt(camPos_, camPos_ + fwd, up);
    return view;
  }
  if (fpv_) {
    // Onboard camera: sit just ahead of + above the CoM, look along body +X
    // (forward), with the body's up (-Z) as the view up. Rides the airframe.
    const QVector3D fwd = snap_.att.rotatedVector(QVector3D(1, 0, 0));
    const QVector3D up = snap_.att.rotatedVector(QVector3D(0, 0, -1));
    const QVector3D eye =
        snap_.pos_w + snap_.att.rotatedVector(QVector3D(0.12f, 0.0f, -0.03f));
    QMatrix4x4 view;
    view.lookAt(eye, eye + fwd, up);
    return view;
  }
  // Third-person orbit, locked to the drone's heading: the eye yaws with the
  // vehicle so it never spins out of frame. The world up (-Z) stays fixed so
  // the horizon doesn't tilt; only a mouse drag re-aims the relative offset.
  const auto &tgt = snap_.pos_w;
  QVector3D eye = tgt + orbitOffset();
  QMatrix4x4 view;
  view.lookAt(eye, tgt, QVector3D(0.0f, 0.0f, -1.0f)); // NED up = -Z
  return view;
}

void SimRendererWidget::paintGL() {
  if (!context() || !context()->isValid())
    return;
  // After a context-gone the resources were released (and the streamed data
  // queued); come back up on whatever context is current. initializeGL does
  // this when Qt makes a new context; this covers a drop that Qt does not
  // follow with initializeGL.
  if (!glLive_)
    initResources();
  // Do not trust that resizeGL ran for every size the window has had: a window
  // resized while hidden, masked or re-parented by a dock switch can reach a
  // frame with the old cached size, and the scene then renders at that size
  // in a corner of the (differently sized) drawable. Re-derive it per frame
  // (two multiplies) before the viewport is set from it.
  {
    const int w = std::max(1, int(std::lround(width() * devicePixelRatio())));
    const int h = std::max(1, int(std::lround(height() * devicePixelRatio())));
    if (w != fbW_ || h != fbH_)
      resizeGL(w, h);
  }
  resetGlState();
  if (glErrorCheck_)
    while (glGetError() != GL_NO_ERROR) {
    } // not ours: whatever the framework left
  updateProjection();
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  if (meshDirty_)
    uploadDroneMesh();
  if (worldMeshDirty_)
    uploadWorldMesh();
  if (chunksDirty_)
    flushChunkUpdates();
#ifdef VAYU_SIM_GRASS
  if (floraDirty_)
    flushFloraUpdates();
#endif
  if (!animFrozenForTest_)
    floraTime_ += 0.016f; // ~60 Hz wind + sky clock

  QMatrix4x4 view = cameraView();
  camEye_ =
      view.inverted().map(QVector3D(0.0f, 0.0f, 0.0f)); // world eye for fog

  // Training mode hides the imported world + obstacles (computed up here so the
  // shadow pass knows what to render as casters).
  const bool training = !gates_.isEmpty();
  const bool showWorld = worldVisible_ && hasWorldMesh_ && !training;
  const bool showChunks = worldVisible_ && !training && !worldChunks_.empty();
  updateFarTerrain(showChunks); // far ring + fog reference height

  // Shadow map: render terrain depth from the sun's view into the FBO first, so
  // the lit terrain + grass can sample it. Restores the default FBO + viewport.
  // Drone state first: the shadow pass draws the drone as a caster.
  // Radius follows the drawn model (same distance boost as drawDroneModel); off
  // in the editor, where the drone is a fixed marker.
  droneShadowPos_ = snap_.pos_w;
  if (editable_) {
    droneShadowR_ = 0.0f;
  } else {
    float half = model_.visualPropRadius;
    for (int i = 0; i < 4; ++i)
      half = std::max(half, std::hypot(model_.hub[i][0], model_.hub[i][1]) +
                                model_.visualPropRadius);
    droneShadowR_ = std::clamp(half * droneVisualScale(), 0.08f, 1.5f);
  }
  renderShadowPass(showWorld, showChunks, view);


  // Sky-dome background (gradient + sun glow along the per-pixel view ray).
  // Drawn first with depth test off so the scene paints over it.
  applyAtmosphere(progLitEdge_.get());
  applyAtmosphere(progLit_.get()); // once per frame; uniforms persist per program
#ifdef VAYU_SIM_GRASS
  applyAtmosphere(progFlora_.get());
#endif
  applyAtmosphere(progSky_.get());
  glDisable(GL_DEPTH_TEST);
  progSky_->bind();
  progSky_->setUniformValue(us_invvp_, (proj_ * view).inverted());
  progSky_->setUniformValue(us_time_, floraTime_);
  skyVao_.bind();
  glDrawArrays(GL_TRIANGLES, 0, 3);
  skyVao_.release();
  progSky_->release();
  glEnable(GL_DEPTH_TEST);

  // Reference ground grid at z=0 — skipped when an imported world mesh is
  // shown (it carries its own ground plane at z=0; the two would z-fight).
  // training/showWorld/showChunks were computed above for the shadow pass.
  const bool horizonPlane =
      worldVisible_ && !showChunks && (!showWorld || horizonOn_);
  if (!horizonPlane && !showWorld && !showChunks)
    drawMesh(ground_, view, QVector3D(0.25f, 0.27f, 0.32f)); // Vehicle mode
  drawMesh(axes_, view, QVector3D(1, 1, 1));

  // World geometry (imported mesh + obstacles) only in World mode; Vehicle
  // mode shows just the airframe.
  // Imported world mesh (lit solid, world frame).
  if (showWorld)
    drawLit(worldMesh_, view, QMatrix4x4(), QVector3D(1.0f, 1.0f, 1.0f),
            horizonOn_); // procedural terrain gets the ground detail

  // Streaming terrain chunks (lit, world frame). Same lit shader / vertex-color
  // path as the imported mesh, one draw per loaded chunk.
  if (showChunks)
    for (auto &kv : worldChunks_)
      if (kv.second->vertex_count)
        drawLit(*kv.second, view, QMatrix4x4(), QVector3D(1.0f, 1.0f, 1.0f),
                true);
  if (showChunks && farMesh_.vertex_count) { // coarse ring beyond the window
    drawingFarRing_ = true;
    drawLit(farMesh_, view, QMatrix4x4(), QVector3D(1.0f, 1.0f, 1.0f), true);
    drawingFarRing_ = false;
  }

  // With no terrain mesh in World mode, and around a finite procedural arena,
  // draw a detailed ground out to the horizon instead of the old +/-10 m line
  // grid that hung in a dark void. Drawn after the terrain so early-z skips the
  // fragments the arena already covers. Vehicle mode keeps the plain grid.
  if (horizonPlane) {
    QMatrix4x4 m;
    const bool atMesh = showWorld && horizonOn_;
    m.translate(camEye_.x(), camEye_.y(), atMesh ? horizonZ_ : 0.0f);
    m.scale(2.0f * kFarPlaneM, 2.0f * kFarPlaneM, 1.0f);
    drawLit(horizonQuad_, view, m,
            atMesh ? horizonCol_ : QVector3D(0.33f, 0.45f, 0.24f), true);
  }

#ifdef VAYU_SIM_GRASS
  // Grass over the terrain (after the ground so depth works). GPU-generated when
  // available + selected; otherwise the CPU chunk-instanced path.
  if (worldVisible_ && !training && floraVisible_) {
    if (gpuGrassActive_ && gpuGrass_.ready()) {
      gpuGrass_.setAtmosphere(atmo_);
      gpuGrass_.render(this, proj_, view, camEye_, sunDir_, floraTime_,
                       csm_, shadowTex_, shadowOn_);
    } else
      drawFlora(view);
  }
#endif

  if (vegEnabled_ && worldVisible_ && !training && veg_)
    drawVegetation(view);
  if (worldVisible_ && !training)
    drawDroneBlob(view);

  // Water over the carved ponds, after the opaque scene (it blends).
  if (worldVisible_ && !training && water_ && waterField_ &&
      waterQuality() == vsim::terrain::Quality::High)
    drawWater(view);

  // Static world obstacles (lit solids), each scaled/rotated/placed.
  for (int oi = 0; worldVisible_ && !training && oi < obstacles_.size(); ++oi) {
    const vsim::Obstacle &o = obstacles_[oi];
    QMatrix4x4 m;
    m.translate(o.pos);
    m.rotate(o.rotate.x(), 1, 0, 0);
    m.rotate(o.rotate.y(), 0, 1, 0);
    m.rotate(o.rotate.z(), 0, 0, 1);
    const Mesh *mesh = &unitBox_;
    if (o.type == vsim::Obstacle::Sphere) {
      mesh = &unitSphere_;
      m.scale(o.size.x(), o.size.x(), o.size.x()); // radius = size.x
    } else if (o.type == vsim::Obstacle::Cylinder) {
      mesh = &unitCyl_;
      m.scale(o.size.x(), o.size.x(), o.size.z()); // radius, height
    } else {
      m.scale(o.size.x(), o.size.y(), o.size.z()); // box full extents
    }
    const procgen::PgVec3 pc =
        procgen::propColor(o.style, o.type, o.pos.x(), o.pos.y());
    QVector3D col(pc.x, pc.y, pc.z);
    // A tree crown is a collision sphere, but drawn as lumpy blobs (leafy) or
    // stacked tiers (conifer) inside it, so the silhouette is not a lollipop.
    if (o.style == procgen::kPropCanopy && !(obsMode_ && oi == selObs_)) {
      drawCrown(view, o, col);
      continue;
    }
    if (obsMode_ && oi == selObs_)
      col = QVector3D(0.95f, 0.80f, 0.30f); // selected
    // Slim posts (gate posts, survey markers) get 1 m red/white bands so their
    // height can be counted.
    const float bands = (o.type == vsim::Obstacle::Cylinder &&
                         o.size.x() <= kPoleMaxR && o.size.z() >= 1.0f)
                            ? std::round(o.size.z())
                            : 0.0f;
    drawLit(*mesh, view, m, col, false, bands);
  }

  // Helipad landing platforms (scattered on flat ground; the drone spawns on one).
  if (worldVisible_ && !training)
    for (const QVector3D &pad : helipads_) {
      QMatrix4x4 m = helipadModel(pad);
      drawLit(helipadDisk_, view, m,
              QVector3D(0.22f, 0.23f, 0.26f)); // grey deck
      drawLit(helipadMark_, view, m,
              QVector3D(0.93f, 0.93f, 0.90f)); // white H+ring
    }

  // Training course halo gates + guidance arrow (World mode / down-cam only).
  if (worldVisible_)
    drawTraining(view);

  // Drone body: apply pos+orientation. Quaternion is normalized by the
  // sim after every step. In FPV / belly-cam the camera is inside the airframe,
  // so the body/markers are skipped — you're looking *out* at the world.
  const QMatrix4x4 model = modelMatrix();
  const bool hideBody = fpv_ || downCam_;

  if (!hideBody) {
    if (hasMesh_) {
      drawingDroneBody_ = true;
      drawLit(droneMesh_, view, model, QVector3D(0.80f, 0.81f, 0.85f));
      drawingDroneBody_ = false;
    } else {
      if (modelDirty_)
        rebuildDroneModel();
      const float dt = rotorClock_.isValid()
                           ? float(rotorClock_.restart()) * 1e-3f
                           : 0.0f;
      if (!rotorClock_.isValid())
        rotorClock_.start();
      updateRotors(dt);
      drawDroneModel(view, model);
      if (!editable_)
        drawDroneLocator(view, model);
    }
  }
  if (!hideBody) {

    // Motor markers (disk colored by spin + activity) and a thrust-axis
    // line, at the editable body-frame positions/axes.
    // The procedural airframe has its own rotors; the flat disks + thrust
    // lines remain for the editor (where they are the editable handles) and
    // over a user-imported mesh (which has no rotors of its own).
    const bool markers = editable_ || hasMesh_;
    for (int i = 0; markers && i < 4; ++i) {
      QVector3D axisN = motorAxis_[i];
      if (axisN.lengthSquared() < 1e-12f)
        axisN = QVector3D(0, 0, -1);
      axisN.normalize();
      const QQuaternion ori =
          QQuaternion::rotationTo(QVector3D(0, 0, 1), axisN);

      QMatrix4x4 mr = markerModel();
      mr.translate(motorPos_[i]);
      mr.rotate(ori);
      const QVector3D base = (motorSpin_[i] >= 0)
                                 ? QVector3D(0.30f, 0.85f, 0.30f)
                                 : QVector3D(0.85f, 0.30f, 0.30f);
      const float duty = snap_.motor_duty[i];
      const QVector3D c =
          (editable_ && i == selected_)
              ? QVector3D(1.0f, 0.9f, 0.3f) // selection highlight
              : base * (0.4f + 0.6f * duty);
      drawMesh(rotor_, view * mr, c);

      // Thrust direction: unit +Z segment rotated onto the axis, shortened.
      QMatrix4x4 ml = markerModel();
      ml.translate(motorPos_[i]);
      ml.rotate(ori);
      ml.scale(1.0f, 1.0f, 0.18f);
      drawMesh(thrustLine_, view * ml, QVector3D(0.95f, 0.95f, 0.40f));
    }

    // Center-of-mass crosshair (body frame, diagnostic): editor / imported
    // mesh only, it would clutter the flying model.
    if (markers) {
      QMatrix4x4 mc = model;
      mc.translate(comOffset_);
      drawMesh(comMarker_, view * mc, QVector3D(0.95f, 0.35f, 0.95f));
    }

    // Motor gizmo (only when editing and a motor is selected).
    if (editable_ && selected_ >= 0)
      drawGizmo(view);
    // Obstacle gizmo (World mode, an obstacle selected).
    if (obsMode_ && selObs_ >= 0)
      drawObsGizmo(view);

    // Motor numbers (1..4): only in Vehicle mode (editable_).
    if (editable_)
      drawMotorLabels(view);

    // Body +X / north arrow: shown in both Vehicle and World modes.
    drawNorthIndicator(view);
  } // end if (!hideBody)
  paintHud();
  if (glErrorCheck_)
    while (glGetError() != GL_NO_ERROR)
      ++glErrors_;
}

void SimRendererWidget::drawMesh(const Mesh &m, const QMatrix4x4 &mvp,
                                 const QVector3D &color) {
  prog_->bind();
  prog_->setUniformValue(u_mvp_, proj_ * mvp);
  prog_->setUniformValue(u_color_, color);
  QOpenGLVertexArrayObject::Binder b(
      const_cast<QOpenGLVertexArrayObject *>(&m.vao));
  glDrawArrays(m.primitive, 0, m.vertex_count);
  prog_->release();
}

#ifndef GL_DEPTH_COMPONENT32F
#define GL_DEPTH_COMPONENT32F 0x8CAC
#endif
#ifndef GL_TEXTURE_COMPARE_MODE
#define GL_TEXTURE_COMPARE_MODE 0x884C
#define GL_TEXTURE_COMPARE_FUNC 0x884D
#define GL_COMPARE_REF_TO_TEXTURE 0x884E
#endif
#ifndef GL_LEQUAL
#define GL_LEQUAL 0x0203
#endif
#ifndef GL_CLAMP_TO_EDGE
#define GL_CLAMP_TO_EDGE 0x812F
#endif

void SimRendererWidget::buildShadowMap() {
  // Depth clamp is core in desktop GL 3.2+, an extension elsewhere (not in GLES).
  const QOpenGLContext *ctx = context();
  {
    const QSurfaceFormat fmt = ctx->format();
    depthClamp_ =
        (ctx->isOpenGLES() == false &&
         (fmt.majorVersion() > 3 ||
          (fmt.majorVersion() == 3 && fmt.minorVersion() >= 2))) ||
        ctx->hasExtension("GL_ARB_depth_clamp") ||
        ctx->hasExtension("GL_EXT_depth_clamp");
  }
  if (shadowFbo_)
    glDeleteFramebuffers(1, &shadowFbo_);
  if (shadowTex_)
    glDeleteTextures(1, &shadowTex_);
  shadowFbo_ = shadowTex_ = 0;
  shadowCache_ = ShadowCascadeSet(); // the new atlas holds nothing yet

  // High needs a 3-tile atlas; fall back to Low on GLES (no cascades there:
  // the shaders and the caster range assume the cheap path) or when the atlas
  // would exceed the texture size limit.
  GLint maxTex = 0;
  glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maxTex);
  shadowBuilt_ = shadowQuality_;
  if (shadowBuilt_ == ShadowQuality::High &&
      (ctx->isOpenGLES() || maxTex < kMaxCascades * kShadowSizeHigh))
    shadowBuilt_ = ShadowQuality::Low;
  const bool high = shadowBuilt_ == ShadowQuality::High;
  const int tile = high ? kShadowSizeHigh : kShadowSizeLow;
  const int w = tile * (high ? kMaxCascades : 1);

  // Depth-only FBO: a single 32F depth atlas, no colour buffer. Hardware compare
  // + linear filtering makes every shader tap a bilinear 2x2 PCF.
  glGenFramebuffers(1, &shadowFbo_);
  glGenTextures(1, &shadowTex_);
  glBindTexture(GL_TEXTURE_2D, shadowTex_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, w, tile, 0,
               GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE,
                  GL_COMPARE_REF_TO_TEXTURE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
  glBindFramebuffer(GL_FRAMEBUFFER, shadowFbo_);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D,
                         shadowTex_, 0);
  GLenum none = GL_NONE;
  glDrawBuffers(1, &none); // depth-only: no colour draw/read targets
  glReadBuffer(GL_NONE);
  shadowReady_ =
      glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
  glBindTexture(GL_TEXTURE_2D, 0);
  glBindFramebuffer(GL_FRAMEBUFFER, context()->defaultFramebufferObject());
  shadowRebuild_ = false;
  if (!shadowReady_)
    qInfo("[SimRenderer] shadow FBO incomplete -> shadows disabled.");
}

void SimRendererWidget::renderShadowPass(bool showWorld, bool showChunks,
                                         const QMatrix4x4 &view) {
  shadowOn_ = false;
  droneInMap_ = false;
  csm_.count = 0;
  if (shadowRebuild_)
    buildShadowMap();
  if (!shadowReady_ || (!showWorld && !showChunks))
    return;

  const bool high = shadowBuilt_ == ShadowQuality::High;
  const int tile = high ? kShadowSizeHigh : kShadowSizeLow;
  // Cascades fitted to the camera frustum (ShadowCascades.h). The ground is the
  // far-plane-independent slice of the view, so high terrain needs no special
  // centring; casters toward the sun come from the caster margin.
  csm_ = fitShadowCascades(sunDir_, view.inverted(), kFovYDeg,
                           float(fbW_) / std::max(1, fbH_),
                           high ? kMaxCascades : 1, tile, shadowSnap_);

  // Trees and rocks cast into the cascades whose texel is fine enough for them.
  // What they put in the tiles is the vegetation's own state, so its version and
  // the cascade set are part of the geometry version: a change redraws the tiles.
  bool vegCast[kMaxCascades] = {false, false, false};
  bool anyVegCast = false;
  if (veg_ && vegEnabled_ && worldVisible_ && !vegCastersOffForTest_ && veg_->glReady())
    for (int ci = vegCasterFirst_; ci < csm_.count; ++ci) {
      vegCast[ci] = csm_.info[ci].x() <= kVegCasterMaxTexelM;
      anyVegCast = anyVegCast || vegCast[ci];
    }
  {
    const quint64 key = !anyVegCast ? 0
                                    : (quint64(veg_->casterVersion()) << 3 |
                                       quint64((vegCast[0] ? 1 : 0) | (vegCast[1] ? 2 : 0) |
                                               (vegCast[2] ? 4 : 0))) + 1;
    if (key != vegCasterKey_) {
      vegCasterKey_ = key;
      ++shadowGeomVer_;
    }
  }

  // Which cascades need drawing: a cascade whose casters, sun, size and
  // visibility are unchanged and whose window moved by < kReuseTexels keeps its
  // tile (and the matrices that tile was rendered with).
  const int show = (showWorld ? 1 : 0) | (showChunks ? 2 : 0);
  const bool tilesValid = shadowReuse_ && shadowCache_.count == csm_.count &&
                          shadowCache_.tileSize == tile && shadowCacheShow_ == show &&
                          (shadowCacheSun_ - sunDir_).lengthSquared() < 1e-12f;
  const QVector3D eye = view.inverted().map(QVector3D(0, 0, 0));
  bool redraw[kMaxCascades] = {true, true, true};
  constexpr float kReuseTexels = 3.0f;

  // The drone is a caster too. It moves, so it has its own pose key: a change
  // re-renders only the cascades whose window holds the drone, never the far
  // ones (their tiles keep the terrain), and a still drone costs nothing.
  const bool casterOn = !editable_ && droneCasterReady();
  {
    const float sc = droneVisualScale();
    const bool moved =
        casterOn != droneKeyReady_ ||
        (casterOn && ((snap_.pos_w - droneKeyPos_).lengthSquared() > 1e-4f ||
                      1.0f - std::abs(QQuaternion::dotProduct(snap_.att, droneKeyAtt_)) > 1e-5f ||
                      std::abs(sc - droneKeyScale_) > 1e-3f));
    if (moved) {
      ++droneShadowVer_;
      droneKeyReady_ = casterOn;
      droneKeyPos_ = snap_.pos_w;
      droneKeyAtt_ = snap_.att;
      droneKeyScale_ = sc;
    }
  }
  bool droneIn[kMaxCascades] = {false, false, false};
  // Only the finest cascade: a 0.3 m drone is ~10 texels there, a sliver in the
  // others. Where that cascade is coarser than kDroneMapMaxTexelM (Low quality)
  // or the drone is outside it, the cylinder stays.
  for (int ci = 0; ci < std::min(csm_.count, 1) && casterOn &&
                   csm_.info[0].x() <= kDroneMapMaxTexelM;
       ++ci) {
    // Sideways in light space: the drone's bounding sphere against the window.
    const float r = droneShadowR_ + 0.05f;
    int out[4] = {0, 0, 0, 0};
    for (int c = 0; c < 8; ++c) {
      const QVector3D q = csm_.vp[ci].map(
          snap_.pos_w + QVector3D((c & 1) ? r : -r, (c & 2) ? r : -r, (c & 4) ? r : -r));
      out[0] += q.x() < -1.0f;
      out[1] += q.x() > 1.0f;
      out[2] += q.y() < -1.0f;
      out[3] += q.y() > 1.0f;
    }
    droneIn[ci] = !(out[0] == 8 || out[1] == 8 || out[2] == 8 || out[3] == 8);
    droneInMap_ = droneInMap_ || droneIn[ci];
  }
  for (int ci = 0; ci < csm_.count; ++ci) {
    const bool droneSame =
        droneIn[ci] == shadowCacheDroneIn_[ci] &&
        (!droneIn[ci] || shadowCacheDroneVer_[ci] == droneShadowVer_);
    if (tilesValid && droneSame && shadowCacheVer_[ci] == shadowGeomVer_ &&
        std::abs(shadowCache_.info[ci].x() - csm_.info[ci].x()) < 1e-6f) {
      const QVector4D a = csm_.vp[ci] * QVector4D(eye, 1.0f);
      const QVector4D b = shadowCache_.vp[ci] * QVector4D(eye, 1.0f);
      const float texel = csm_.info[ci].x();
      // xy in tile texels (clip spans 2 over the tile), z in metres.
      const float dx = (a.x() - b.x()) * 0.5f * float(tile);
      const float dy = (a.y() - b.y()) * 0.5f * float(tile);
      const float dz = (a.z() - b.z()) * 0.5f * csm_.info[ci].y() / texel;
      if (std::abs(dx) < kReuseTexels && std::abs(dy) < kReuseTexels &&
          std::abs(dz) < kReuseTexels) {
        redraw[ci] = false;
        csm_.vp[ci] = shadowCache_.vp[ci];
        csm_.info[ci] = shadowCache_.info[ci];
      }
    }
    if (redraw[ci]) {
      shadowCache_.vp[ci] = csm_.vp[ci];
      shadowCache_.info[ci] = csm_.info[ci];
      shadowCacheVer_[ci] = shadowGeomVer_;
      shadowCacheDroneIn_[ci] = droneIn[ci];
      shadowCacheDroneVer_[ci] = droneShadowVer_;
    }
  }
  shadowCache_.count = csm_.count;
  shadowCache_.tileSize = tile;
  shadowCacheShow_ = show;
  shadowCacheSun_ = sunDir_;

  glBindFramebuffer(GL_FRAMEBUFFER, shadowFbo_);
  glViewport(0, 0, csm_.atlasW(), csm_.atlasH());
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_SCISSOR_TEST);
  glDepthMask(GL_TRUE);
  // Casters nearer the sun than the near plane (a ridge far up-sun at a low
  // sun) are clamped to depth 0 instead of clipped away, so they still shadow.
#ifdef GL_DEPTH_CLAMP
  if (depthClamp_)
    glEnable(GL_DEPTH_CLAMP);
#endif
  progDepth_->bind();
  progDepth_->setUniformValue(ud_cover_, 1.0f);
  progDepth_->setUniformValue(ud_push_, 0.0f);
  for (int ci = 0; ci < csm_.count; ++ci) {
    if (!redraw[ci])
      continue;
    glViewport(ci * tile, 0, tile, tile);
    glScissor(ci * tile, 0, tile, tile);
    glClear(GL_DEPTH_BUFFER_BIT);
    const QMatrix4x4 &lightVP = csm_.vp[ci];
    // A mesh whose bounding box lies wholly outside this cascade's window
    // (sideways in light space) cannot cast into it.
    auto outside = [&](const Mesh &m) {
      int out[4] = {0, 0, 0, 0};
      for (int c = 0; c < 8; ++c) {
        const QVector3D q = lightVP.map(QVector3D(
            (c & 1) ? m.bmax.x() : m.bmin.x(), (c & 2) ? m.bmax.y() : m.bmin.y(),
            (c & 4) ? m.bmax.z() : m.bmin.z()));
        out[0] += q.x() < -1.0f;
        out[1] += q.x() > 1.0f;
        out[2] += q.y() < -1.0f;
        out[3] += q.y() > 1.0f;
      }
      return out[0] == 8 || out[1] == 8 || out[2] == 8 || out[3] == 8;
    };
    auto drawDepth = [&](const Mesh &m) {
      if (m.vertex_count == 0 || outside(m))
        return;
      progDepth_->setUniformValue(ud_lightmvp_,
                                 lightVP); // model = identity (world)
      QOpenGLVertexArrayObject::Binder b(
          const_cast<QOpenGLVertexArrayObject *>(&m.vao));
      glDrawArrays(GL_TRIANGLES, 0, m.vertex_count);
    };
    if (showWorld)
      drawDepth(worldMesh_);
    if (showChunks)
      for (auto &kv : worldChunks_)
        if (kv.second->vertex_count)
          drawDepth(*kv.second);
    // Helipad platforms cast shadows too (each has its own model transform).
    if (helipadDisk_.vertex_count)
      for (const QVector3D &pad : helipads_) {
        progDepth_->setUniformValue(ud_lightmvp_, lightVP * helipadModel(pad));
        QOpenGLVertexArrayObject::Binder b(&helipadDisk_.vao);
        glDrawArrays(GL_TRIANGLES, 0, helipadDisk_.vertex_count);
      }
    if (veg_ && !vegCast[ci])
      veg_->clearCasterStats(ci);
    if (vegCast[ci]) {
      veg_->drawShadowCasters(lightVP, ci, 2.0f * vegCasterPushM_ / csm_.info[ci].y());
      progDepth_->bind(); // the vegetation drew with its own program
    }
    if (droneIn[ci] && !noCasterForTest_) {
      // The receiver's depth bias (about 12 texels) would swallow the shadow of
      // anything that low, a landed drone. Pulling the drone toward the sun in
      // depth only (not sideways: the projection is orthographic) lets it shadow
      // the ground under it without changing where the shadow falls.
      progDepth_->setUniformValue(ud_push_, 2.0f * kDronePushM / csm_.info[ci].y());
      drawDroneCaster(lightVP);
      progDepth_->setUniformValue(ud_push_, 0.0f);
    }
  }
  progDepth_->release();
#ifdef GL_DEPTH_CLAMP
  if (depthClamp_)
    glDisable(GL_DEPTH_CLAMP);
#endif
  glDisable(GL_SCISSOR_TEST);

  glBindFramebuffer(GL_FRAMEBUFFER, context()->defaultFramebufferObject());
  glViewport(0, 0, fbW_, fbH_);
  shadowOn_ = true;
}

bool SimRendererWidget::droneCasterReady() const {
  return hasMesh_ ? droneMesh_.vertex_count > 0 : modelBody_.vertex_count > 0;
}

// The drone into the depth pass: the hull is solid; each spinning prop is a thin
// disc of the full rotor radius at its hub that writes only kRotorCoverage of its
// texels (a stable dither the receiver's PCF averages), because a spinning rotor
// shades like a disc of that solidity, not a solid one. An imported mesh is drawn
// as it is.
void SimRendererWidget::drawDroneCaster(const QMatrix4x4 &lightVP) {
  auto draw = [&](const Mesh &m, const QMatrix4x4 &model) {
    if (m.vertex_count == 0)
      return;
    progDepth_->setUniformValue(ud_lightmvp_, lightVP * model);
    QOpenGLVertexArrayObject::Binder b(const_cast<QOpenGLVertexArrayObject *>(&m.vao));
    glDrawArrays(GL_TRIANGLES, 0, m.vertex_count);
  };
  QMatrix4x4 model = modelMatrix();
  if (hasMesh_) {
    draw(droneMesh_, model);
    return;
  }
  model.scale(droneVisualScale());
  draw(modelBody_, model);
  float hubR = 1e9f;
  for (int i = 0; i < 4; ++i)
    hubR = std::min(hubR, std::hypot(model_.hub[i][0], model_.hub[i][1]));
  const float pr = model_.visualPropRadius > 0.0f ? model_.visualPropRadius : 0.4f * hubR;
  for (int i = 0; i < 4; ++i) {
    QMatrix4x4 m = model;
    m.translate(model_.hub[i][0], model_.hub[i][1], model_.hub[i][2]);
    m.scale(pr, pr, kPropDiscThickM);
    progDepth_->setUniformValue(ud_cover_, rotorCoverageForTest_ >= 0.0f ? rotorCoverageForTest_ : kRotorCoverage);
    draw(helipadDisk_, m); // unit cylinder, local z in [-1, 0]
  }
  progDepth_->setUniformValue(ud_cover_, 1.0f);
}

void SimRendererWidget::applyAtmosphere(QOpenGLShaderProgram *prog) const {
  if (!prog)
    return;
  prog->bind();
  atmo_.forEachUniform([prog](const char *name, auto... v) {
    const float a[] = {static_cast<float>(v)...};
    if constexpr (sizeof...(v) == 3)
      prog->setUniformValue(name, QVector3D(a[0], a[1], a[2]));
    else
      prog->setUniformValue(name, a[0]);
  });
  prog->release();
}

void SimRendererWidget::drawLit(const Mesh &m, const QMatrix4x4 &view,
                                const QMatrix4x4 &model, const QVector3D &color,
                                bool terrain, float bands) {
  if (m.vertex_count == 0)
    return;
  // The textures upload on first use; a missing resource leaves the plain look.
  const bool mat = terrain && terrainQuality() == vsim::terrain::Quality::High &&
                   (terrainTex_.loaded() || terrainTex_.load(this));
  // High ground that carries the border-track data takes the per-pixel build;
  // everything else (Low, other meshes) the one without those interpolants.
  const bool edge = mat && m.trackData && progLitEdge_ && progLitEdge_->isLinked();
  QOpenGLShaderProgram *prog = edge ? progLitEdge_.get() : progLit_.get();
  const LitLocs &L = edge ? litLocEdge_ : litLoc_;
  prog->bind();
  prog->setUniformValue(L.mvp, proj_ * view * model);
  prog->setUniformValue(L.model, model);
  prog->setUniformValue(L.nmat, model.normalMatrix());
  prog->setUniformValue(L.color, color);
  prog->setUniformValue(L.campos, camEye_);
  // Aerial perspective: gentle haze that fully veils the streamed-terrain edge.
  prog->setUniformValue(L.fogdensity, atmo_.fogDensity);
  prog->setUniformValue(L.fogstart, atmo_.fogStartM);
  prog->setUniformValue("u_fogbase", fogBaseZ_);
  prog->setUniformValue("u_fogheight", atmo_.fogHeightM);
  prog->setUniformValue("u_ringclip", drawingFarRing_ ? QVector4D(farTerrain_.clipRect()[0], farTerrain_.clipRect()[1], farTerrain_.clipRect()[2], farTerrain_.clipRect()[3]) : QVector4D());
  L.csm.upload(prog, csm_, shadowOn_);
  prog->setUniformValue(L.drone, droneShadowPos_);
  prog->setUniformValue(L.droner, droneShadowR_);
  prog->setUniformValue("u_dronemap", droneInMap_ ? 1.0f : 0.0f);
  prog->setUniformValue("u_dronebody", drawingDroneBody_ ? 1.0f : 0.0f);
  prog->setUniformValue(L.sunint, litSunInt_);
  prog->setUniformValue(L.ambstr, litAmbStr_);
  prog->setUniformValue(L.terrain, terrain ? 1.0f : 0.0f);
  prog->setUniformValue(L.grid, gridOn_ ? 1.0f : 0.0f);
  prog->setUniformValue(L.matq, mat ? 1.0f : 0.0f);
  if (mat)
    terrainTex_.bind(this, prog);
  prog->setUniformValue(L.bands, bands);
  glActiveTexture(GL_TEXTURE1);
  glBindTexture(GL_TEXTURE_2D, shadowTex_);
  glActiveTexture(GL_TEXTURE0);
  QOpenGLVertexArrayObject::Binder b(
      const_cast<QOpenGLVertexArrayObject *>(&m.vao));
  // Flat solids leave attribute 2 disabled; feed white as the generic value so
  // u_color*a_color == u_color. Meshes with a real color array (world mesh)
  // enable attribute 2 and override this.
  glVertexAttrib3f(2, 1.0f, 1.0f, 1.0f);
  glDrawArrays(GL_TRIANGLES, 0, m.vertex_count);
  prog->release();
}

// Tree crown: blobs of the crown colour packed inside the obstacle's sphere
// (radius = size.x), so the collision shape bounds what is drawn.
void SimRendererWidget::drawCrown(const QMatrix4x4 &view,
                                  const vsim::Obstacle &o, const QVector3D &col) {
  const float R = o.size.x();
  const float px = o.pos.x(), py = o.pos.y();
  auto blob = [&](float ox, float oy, float oz, float rh, float rv, float shade) {
    QMatrix4x4 m;
    m.translate(o.pos + QVector3D(ox, oy, oz)); // NED: up is -z
    m.scale(rh, rh, rv);
    drawLit(unitSphere_, view, m, col * shade, false, 0.0f);
  };
  if (procgen::propIsConifer(px, py)) {
    blob(0, 0, 0.42f * R, 0.88f * R, 0.5f * R, 0.85f);
    blob(0, 0, -0.04f * R, 0.64f * R, 0.5f * R, 1.0f);
    blob(0, 0, -0.50f * R, 0.40f * R, 0.5f * R, 1.12f);
    return;
  }
  blob(0, 0, -0.10f * R, 0.72f * R, 0.70f * R, 1.0f);
  const float a0 = 6.2831853f * procgen::propRand(px, py, 7u);
  for (int k = 0; k < 4; ++k) {
    const float a = a0 + 1.5707963f * static_cast<float>(k) +
                    0.5f * procgen::propRand(px, py, 20u + k);
    const float rr = 0.40f + 0.12f * procgen::propRand(px, py, 30u + k);
    const float lift = (procgen::propRand(px, py, 40u + k) - 0.5f) * 0.4f;
    blob(std::cos(a) * (1.0f - rr) * R, std::sin(a) * (1.0f - rr) * R,
         (0.10f - lift) * R, rr * R, rr * R * 0.9f,
         0.88f + 0.2f * procgen::propRand(px, py, 50u + k));
  }
  blob(0, 0, -0.50f * R, 0.50f * R, 0.46f * R, 1.14f); // sunlit top
}

void SimRendererWidget::uploadDroneMesh() {
  meshDirty_ = false;
  hasMesh_ = !pendingPos_.empty();
  if (!hasMesh_)
    return;

  // Interleave [px,py,pz, nx,ny,nz] per vertex (triangle soup).
  std::vector<float> data;
  data.reserve(pendingPos_.size() * 6);
  for (size_t i = 0; i < pendingPos_.size(); ++i) {
    const QVector3D &p = pendingPos_[i];
    const QVector3D n =
        (i < pendingNrm_.size()) ? pendingNrm_[i] : QVector3D(0, 0, 1);
    data.insert(data.end(), {p.x(), p.y(), p.z(), n.x(), n.y(), n.z()});
  }

  if (!droneMesh_.vao.isCreated())
    droneMesh_.vao.create();
  droneMesh_.vao.bind();
  if (!droneMesh_.vbo.isCreated())
    droneMesh_.vbo.create();
  droneMesh_.vbo.bind();
  droneMesh_.vbo.allocate(data.data(), int(data.size() * sizeof(float)));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                        reinterpret_cast<void *>(3 * sizeof(float)));
  droneMesh_.vbo.release();
  droneMesh_.vao.release();
  droneMesh_.vertex_count = int(pendingPos_.size());
  droneMesh_.primitive = GL_TRIANGLES;
  // pendingPos_/pendingNrm_ stay: they are the CPU copy a new context
  // re-uploads from.
}

void SimRendererWidget::uploadWorldMesh() {
  worldMeshDirty_ = false;
  hasWorldMesh_ = !pendingWorldPos_.empty();
  if (!hasWorldMesh_) {
    worldMesh_.vertex_count = 0;
    return;
  }
  // Interleave [px,py,pz, nx,ny,nz, r,g,b] per vertex so the world mesh keeps
  // its baked material/vertex colors. Missing colors default to neutral grey.
  std::vector<float> data;
  data.reserve(pendingWorldPos_.size() * 9);
  float lowestZ = -1e9f; // NED: largest z = lowest surface
  QVector3D colSum(0, 0, 0);
  for (size_t i = 0; i < pendingWorldPos_.size(); ++i) {
    const QVector3D &p = pendingWorldPos_[i];
    const QVector3D n = (i < pendingWorldNrm_.size()) ? pendingWorldNrm_[i]
                                                      : QVector3D(0, 0, 1);
    const QVector3D c = (i < pendingWorldCol_.size())
                            ? pendingWorldCol_[i]
                            : QVector3D(0.72f, 0.73f, 0.76f);
    data.insert(data.end(), {p.x(), p.y(), p.z(), n.x(), n.y(), n.z(), c.x(),
                             c.y(), c.z()});
    lowestZ = std::max(lowestZ, p.z());
    colSum += c;
  }
  // Horizon ground for a finite procedural arena: just under its lowest point,
  // in the arena's average surface colour so the join reads as continuous land.
  horizonZ_ = lowestZ + 0.5f; // well clear of depth resolution at distance
  horizonCol_ = colSum / float(pendingWorldPos_.size());
  uploadColoredMesh(worldMesh_, data);
  ++shadowGeomVer_;
  // pendingWorld* stay as the CPU copy for re-upload into a new context.
}

// ---------- geometry generators ----------

void SimRendererWidget::uploadLitMesh(Mesh &m, const std::vector<float> &data) {
  if (!m.vao.isCreated())
    m.vao.create();
  m.vao.bind();
  if (!m.vbo.isCreated())
    m.vbo.create();
  m.vbo.bind();
  m.vbo.allocate(data.data(), int(data.size() * sizeof(float)));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                        reinterpret_cast<void *>(3 * sizeof(float)));
  m.vbo.release();
  m.vao.release();
  m.vertex_count = int(data.size() / 6);
  m.primitive = GL_TRIANGLES;
}

void SimRendererWidget::uploadColoredMesh(Mesh &m,
                                          const std::vector<float> &data,
                                          int fpv) {
  if (!m.vao.isCreated())
    m.vao.create();
  m.vao.bind();
  if (!m.vbo.isCreated())
    m.vbo.create();
  m.vbo.bind();
  m.vbo.allocate(data.data(), int(data.size() * sizeof(float)));
  const int stride = fpv * int(sizeof(float));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, nullptr);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride,
                        reinterpret_cast<void *>(3 * sizeof(float)));
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, stride,
                        reinterpret_cast<void *>(6 * sizeof(float)));
  if (fpv >= 17) { // terrain: border-track distances, colour, track-free colour
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, stride,
                          reinterpret_cast<void *>(9 * sizeof(float)));
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, stride,
                          reinterpret_cast<void *>(11 * sizeof(float)));
    glEnableVertexAttribArray(5);
    glVertexAttribPointer(5, 3, GL_FLOAT, GL_FALSE, stride,
                          reinterpret_cast<void *>(14 * sizeof(float)));
    m.trackData = true;
  }
  m.vbo.release();
  m.vao.release();
  m.vertex_count = int(data.size() / size_t(fpv));
  m.primitive = GL_TRIANGLES;
  m.bmin = m.bmax = QVector3D();
  for (size_t i = 0; i + size_t(fpv) - 1 < data.size(); i += size_t(fpv)) {
    const QVector3D p(data[i], data[i + 1], data[i + 2]);
    if (i == 0)
      m.bmin = m.bmax = p;
    for (int k = 0; k < 3; ++k) {
      m.bmin[k] = std::min(m.bmin[k], p[k]);
      m.bmax[k] = std::max(m.bmax[k], p[k]);
    }
  }
}

void SimRendererWidget::buildObstacleMeshes() {
  auto tri = [](std::vector<float> &v, const QVector3D &a, const QVector3D &b,
                const QVector3D &c) {
    // Outward normal: the unit solids are centred on the origin, so the face
    // normal must point away from it. (The box's winding is inward, which
    // lit every face as facing away from the sun -- near-black boxes.)
    QVector3D n = QVector3D::crossProduct(b - a, c - a).normalized();
    if (QVector3D::dotProduct(n, a + b + c) < 0.0f)
      n = -n;
    for (const QVector3D &p : {a, b, c})
      v.insert(v.end(), {p.x(), p.y(), p.z(), n.x(), n.y(), n.z()});
  };

  // --- unit box [-0.5,0.5]^3 (full-extent unit cube) ---
  {
    std::vector<float> v;
    const float h = 0.5f;
    const QVector3D c[8] = {{-h, -h, -h}, {h, -h, -h}, {h, h, -h}, {-h, h, -h},
                            {-h, -h, h},  {h, -h, h},  {h, h, h},  {-h, h, h}};
    const int f[6][4] = {{0, 1, 2, 3}, {5, 4, 7, 6}, {4, 0, 3, 7},
                         {1, 5, 6, 2}, {4, 5, 1, 0}, {3, 2, 6, 7}};
    for (auto &q : f) {
      tri(v, c[q[0]], c[q[1]], c[q[2]]);
      tri(v, c[q[0]], c[q[2]], c[q[3]]);
    }
    uploadLitMesh(unitBox_, v);
  }

  // --- unit sphere (radius 1, UV tessellation) ---
  {
    std::vector<float> v;
    const int LA = 12, LO = 18;
    auto sph = [](double la, double lo) {
      return QVector3D(std::sin(la) * std::cos(lo), std::sin(la) * std::sin(lo),
                       std::cos(la));
    };
    for (int i = 0; i < LA; ++i)
      for (int j = 0; j < LO; ++j) {
        const double la0 = M_PI * i / LA, la1 = M_PI * (i + 1) / LA;
        const double lo0 = 2 * M_PI * j / LO, lo1 = 2 * M_PI * (j + 1) / LO;
        const QVector3D a = sph(la0, lo0), b = sph(la1, lo0),
                        c2 = sph(la1, lo1), d = sph(la0, lo1);
        // normals == positions for a unit sphere.
        for (const QVector3D &p : {a, b, c2})
          v.insert(v.end(), {p.x(), p.y(), p.z(), p.x(), p.y(), p.z()});
        for (const QVector3D &p : {a, c2, d})
          v.insert(v.end(), {p.x(), p.y(), p.z(), p.x(), p.y(), p.z()});
      }
    uploadLitMesh(unitSphere_, v);
  }

  // --- unit cylinder (radius 1, height 1 in z, +caps) ---
  {
    std::vector<float> v;
    const int N = 24;
    const float zt = 0.5f, zb = -0.5f;
    for (int j = 0; j < N; ++j) {
      const double a0 = 2 * M_PI * j / N, a1 = 2 * M_PI * (j + 1) / N;
      const QVector3D n0(std::cos(a0), std::sin(a0), 0),
          n1(std::cos(a1), std::sin(a1), 0);
      const QVector3D bt0(n0.x(), n0.y(), zt), bb0(n0.x(), n0.y(), zb),
          bt1(n1.x(), n1.y(), zt), bb1(n1.x(), n1.y(), zb);
      auto side = [&](const QVector3D &p, const QVector3D &n) {
        v.insert(v.end(), {p.x(), p.y(), p.z(), n.x(), n.y(), n.z()});
      };
      side(bb0, n0);
      side(bb1, n1);
      side(bt1, n1);
      side(bb0, n0);
      side(bt1, n1);
      side(bt0, n0);
      // caps (fan from axis point)
      tri(v, QVector3D(0, 0, zt), bt0, bt1);
      tri(v, QVector3D(0, 0, zb), bb1, bb0);
    }
    uploadLitMesh(unitCyl_, v);
  }
}

void SimRendererWidget::buildHelipadMeshes() {
  const QVector3D up(0, 0, -1); // NED up
  auto pushUp = [](std::vector<float> &v, const QVector3D &p,
                   const QVector3D &n) {
    v.insert(v.end(), {p.x(), p.y(), p.z(), n.x(), n.y(), n.z()});
  };
  // --- platform: radius 1, local z in [-1 (top/landing) .. 0 (base/terrain)] ---
  {
    std::vector<float> v;
    const int N = 32;
    const float zt = -1.0f, zb = 0.0f;
    for (int j = 0; j < N; ++j) {
      const double a0 = 2 * M_PI * j / N, a1 = 2 * M_PI * (j + 1) / N;
      const QVector3D n0(std::cos(a0), std::sin(a0), 0),
          n1(std::cos(a1), std::sin(a1), 0);
      const QVector3D bt0(n0.x(), n0.y(), zt), bb0(n0.x(), n0.y(), zb),
          bt1(n1.x(), n1.y(), zt), bb1(n1.x(), n1.y(), zb);
      // side wall (outward normals)
      pushUp(v, bb0, n0);
      pushUp(v, bb1, n1);
      pushUp(v, bt1, n1);
      pushUp(v, bb0, n0);
      pushUp(v, bt1, n1);
      pushUp(v, bt0, n0);
      // top cap (up-facing)
      pushUp(v, QVector3D(0, 0, zt), up);
      pushUp(v, bt0, up);
      pushUp(v, bt1, up);
    }
    uploadLitMesh(helipadDisk_, v);
  }
  // --- mark: "H" + ring border, flat just above the top, up-facing (white) ---
  {
    std::vector<float> v;
    // Sit a FIXED ~2 cm above the deck regardless of pad height: the local z is
    // scaled by kHelipadHeightM at draw time, so divide the world clearance out
    // here (otherwise a taller pad floats the "H" up off the deck).
    const float z = -1.0f - 0.02f / kHelipadHeightM;
    auto quad = [&](float x0, float x1, float y0, float y1) {
      const QVector3D a(x0, y0, z), b(x1, y0, z), c(x1, y1, z), d(x0, y1, z);
      for (const QVector3D &p : {a, b, c})
        pushUp(v, p, up);
      for (const QVector3D &p : {a, c, d})
        pushUp(v, p, up);
    };
    quad(-0.40f, -0.20f, -0.55f, 0.55f); // H left bar
    quad(0.20f, 0.40f, -0.55f, 0.55f);   // H right bar
    quad(-0.20f, 0.20f, -0.12f, 0.12f);  // H crossbar
    const int N = 48;
    const float ri = 0.80f, ro = 0.92f; // ring border annulus
    auto rv = [&](float r, double a) {
      return QVector3D(r * std::cos(a), r * std::sin(a), z);
    };
    for (int j = 0; j < N; ++j) {
      const double a0 = 2 * M_PI * j / N, a1 = 2 * M_PI * (j + 1) / N;
      const QVector3D i0 = rv(ri, a0), o0 = rv(ro, a0), i1 = rv(ri, a1),
                      o1 = rv(ro, a1);
      for (const QVector3D &p : {i0, o0, o1})
        pushUp(v, p, up);
      for (const QVector3D &p : {i0, o1, i1})
        pushUp(v, p, up);
    }
    uploadLitMesh(helipadMark_, v);
  }
}

void SimRendererWidget::setHelipads(const std::vector<QVector3D> &pads) {
  helipads_ = pads;
  ++shadowGeomVer_;
  if (veg_)
    veg_->exclude(vegExclusionsFor(pads, {}));
  update();
}

void SimRendererWidget::setObstacles(const QVector<vsim::Obstacle> &obs) {
  obstacles_ = obs;
  if (veg_)
    veg_->exclude(vegExclusionsFor({}, obs));
  update();
}

void SimRendererWidget::setTrainingGates(const QVector<vsim::RingGate> &gates) {
  gates_ = gates;
  update();
}

void SimRendererWidget::setTrainingActive(int activeIndex, bool showArrow) {
  trainActive_ = activeIndex;
  trainArrow_ = showArrow;
  update();
}

void SimRendererWidget::setWorldMesh(const std::vector<QVector3D> &positions,
                                     const std::vector<QVector3D> &normals,
                                     const std::vector<QVector3D> &colors) {
  pendingWorldPos_ = positions;
  pendingWorldNrm_ = normals;
  pendingWorldCol_ = colors;
  worldMeshDirty_ = true; // uploaded in paintGL (needs GL context)
  update();
}

void SimRendererWidget::setWorldChunk(qint64 key,
                                      const std::vector<QVector3D> &positions,
                                      const std::vector<QVector3D> &normals,
                                      const std::vector<QVector3D> &colors,
                                      const std::vector<QVector3D> &splitColors,
                                      const std::vector<QVector3D> &edge,
                                      const std::vector<QVector3D> &edgeColors) {
  // Interleave [px,py,pz, nx,ny,nz, r,g,b, signed, unsigned, er,eg,eb, sr,sg,sb]
  // now (UI thread, no GL) and queue the VBO upload for paintGL. The colour is
  // the baked one (track included); High takes the track-free one (sr..sb) and
  // draws the track per pixel. A chunk without the data gets "no track near"
  // (EdgeTrack.h kEdgeFar) and "no track-free colour" (negative).
  PendingChunk pc;
  pc.key = key;
  pc.data.reserve(positions.size() * 17);
  const QVector3D far(procgen::kEdgeFar, procgen::kEdgeFar, 0.0f);
  for (size_t i = 0; i < positions.size(); ++i) {
    const QVector3D &p = positions[i];
    const QVector3D n = (i < normals.size()) ? normals[i] : QVector3D(0, 0, -1);
    const QVector3D c =
        (i < colors.size()) ? colors[i] : QVector3D(0.4f, 0.5f, 0.3f);
    const QVector3D e = (i < edge.size()) ? edge[i] : far;
    const QVector3D ec = (i < edgeColors.size()) ? edgeColors[i] : c;
    const QVector3D sc = (i < splitColors.size()) ? splitColors[i] : QVector3D(-1, -1, -1);
    pc.data.insert(pc.data.end(), {p.x(), p.y(), p.z(), n.x(), n.y(), n.z(),
                                   c.x(), c.y(), c.z(), e.x(), e.y(), ec.x(),
                                   ec.y(), ec.z(), sc.x(), sc.y(), sc.z()});
  }
  pendingChunkUploads_.push_back(std::move(pc));
  chunksDirty_ = true;
  update();
}

void SimRendererWidget::removeWorldChunk(qint64 key) {
  removeChunkVegetation(key); // vegetation loads and unloads with its chunk
  // A queued upload of this chunk must not outlive the removal: the flush
  // erases first and uploads after, so it would bring the chunk back, and on a
  // view that is never painted it would sit there for good.
  pendingChunkUploads_.erase(
      std::remove_if(pendingChunkUploads_.begin(), pendingChunkUploads_.end(),
                     [key](const PendingChunk &pc) { return pc.key == key; }),
      pendingChunkUploads_.end());
  pendingChunkRemovals_.push_back(key);
  chunksDirty_ = true;
  update();
}

void SimRendererWidget::clearWorldChunks() {
  clearChunkVegetation();
  // Uploads queued so far belong to the set being cleared. Drop them now: a
  // widget that is not painted never reaches flushChunkUpdates(), so they would
  // wait there for good, one old world's meshes per world change. Uploads
  // queued after this call are the new set and stay.
  pendingChunkUploads_.clear();
  clearAllChunks_ = true;
  chunksDirty_ = true;
  update();
}

void SimRendererWidget::setTerrainField(
    std::shared_ptr<const procgen::TerrainField> f) {
  waterField_ = f;
  waterCentre_ = QVector2D(1e9f, 1e9f); // rebuild the body list
  if (water_)
    water_->setBodies({});
  farTerrain_.setField(std::move(f));
  farMesh_.vertex_count = 0; // re-upload (the GL objects are reused)
  update();
}

// Keeps the far ring on the loaded chunk window and sets the fog's ground
// reference. Called from paintGL (GL context current).
void SimRendererWidget::updateFarTerrain(bool showChunks) {
  fogBaseZ_ = farTerrain_.groundZ(camEye_.x(), camEye_.y(),
                                  hasWorldMesh_ ? horizonZ_ : 0.0f);
  if (!showChunks || !farTerrain_.active())
    return;
  FarTerrain::Window w;
  w.chunkM = procgen::kStreamChunkM;
  w.minCx = w.minCy = INT_MAX;
  w.maxCx = w.maxCy = INT_MIN;
  for (const auto &kv : worldChunks_) {
    const int cx = int(kv.first >> 32), cy = int(uint32_t(kv.first));
    w.minCx = std::min(w.minCx, cx);
    w.maxCx = std::max(w.maxCx, cx);
    w.minCy = std::min(w.minCy, cy);
    w.maxCy = std::max(w.maxCy, cy);
  }
  if (farTerrain_.update(w) || (!farMesh_.vertex_count && farTerrain_.vertexCount())) {
    uploadColoredMesh(farMesh_, farTerrain_.vertices());
  }
}

// ---- vegetation (logic lives in VegetationRenderer) -------------------------

void SimRendererWidget::setChunkVegetation(
    qint64 key, std::vector<vsim::procgen::VegInstance> instances, float cx,
    float cy, float half) {
  if (veg_)
    veg_->setChunk(key, std::move(instances), cx, cy, half);
  update();
}
void SimRendererWidget::removeChunkVegetation(qint64 key) {
  if (veg_)
    veg_->removeChunk(key);
}
void SimRendererWidget::clearChunkVegetation() {
  if (veg_)
    veg_->clearChunks();
}
bool SimRendererWidget::surveyGridByConfig() {
  return QSettings().value(QStringLiteral("sim/surveyGrid"), false).toBool();
}

void SimRendererWidget::setVegetationEnabled(bool on) {
  vegEnabled_ = on;
  update();
}
void SimRendererWidget::setVegetationTreesEnabled(bool on) {
  if (veg_)
    veg_->setTreesEnabled(on);
  update();
}
const VegetationRenderer::Stats &SimRendererWidget::vegetationStats() const {
  static const VegetationRenderer::Stats none;
  return veg_ && vegEnabled_ ? veg_->stats() : none;
}

// The drone's contact blob on the ground under it: its own pass, whatever the
// vegetation flags say (the height cue stays with vegetation off).
void SimRendererWidget::drawDroneBlob(const QMatrix4x4 &view) {
  if (!veg_)
    return;
  lastVegVp_ = proj_ * view; // for worldToPixelForTest
  VegFrame f;
  f.proj = proj_;
  f.view = view;
  f.camEye = camEye_;
  f.dronePos = droneShadowPos_;
  f.droneRadius = droneShadowR_;
  if (farTerrain_.active()) {
    f.droneGroundZ = farTerrain_.surfaceZ(droneShadowPos_.x(), droneShadowPos_.y(), 1e9f);
    f.droneGroundKnown = f.droneGroundZ < 1e8f;
  }
  veg_->drawDroneBlob(f);
}

void SimRendererWidget::drawVegetation(const QMatrix4x4 &view) {
  if (!veg_->glReady() &&
      !veg_->initGL({QByteArray(kAtmosphereGLSL) + kFogGLSL,
                     QByteArray(kCascadeShadowGLSL) + kShadowGLSL, kLightingGLSL}))
    return;
  VegFrame f;
  f.proj = proj_;
  f.view = view;
  f.csm = csm_;
  f.atmo = atmo_;
  f.camEye = camEye_;
  f.sunDir = sunDir_;
  f.dronePos = droneShadowPos_;
  f.droneRadius = droneShadowR_;
  f.droneInMap = droneInMap_;
  f.time = floraTime_;
  f.sunIntensity = litSunInt_;
  f.ambientStrength = litAmbStr_;
  f.fogDensity = atmo_.fogDensity; // the lit shader's aerial perspective
  f.fogStart = atmo_.fogStartM;
  f.fogBase = fogBaseZ_;
  f.fogHeight = atmo_.fogHeightM;
  f.shadowTex = shadowTex_;
  f.shadowOn = shadowOn_;
  f.defaultFbo = int(context()->defaultFramebufferObject());
  f.viewportW = fbW_;
  f.viewportH = fbH_;
  veg_->draw(f);
  if (veg_->busy())
    update(); // textures / impostors still arriving: keep painting
}

// Feeds the ripple sim: binds it to the pond the vehicle is over (or nearest
// to, within reach), hands it each rotor's spot and height above the water, and
// runs it up to the wall clock. A second view calling this at the same moment
// finds the clock already there and steps nothing.
void SimRendererWidget::updateRipples() {
  ripples();
  const QVector3D pos = snap_.pos_w; // NED: up is -z
  const procgen::WaterBody *nearest = nullptr;
  float nearD = 1e9f;
  for (const procgen::WaterBody &b : water_->bodies()) {
    const float dx = pos.x() - b.cx, dy = pos.y() - b.cy;
    const float d = std::sqrt(dx * dx + dy * dy) - b.shoreRadiusToward(dx, dy);
    if (d < nearD) {
      nearD = d;
      nearest = &b;
    }
  }
  // Only a vehicle close enough to matter (the wash reaches ~10 m, plus margin).
  constexpr float kReachM = WaterRipples::kMaxHeightM + 6.0f;
  std::array<RippleRotor, 4> rotors;
  int n = 0;
  if (nearest && nearD < kReachM) {
    ripples_->bindBody(*nearest);
    for (int i = 0; i < 4; ++i) {
      const QVector3D w = pos + snap_.att.rotatedVector(motorPos_[i]);
      RippleRotor &r = rotors[size_t(n)];
      r.x = w.x();
      r.y = w.y();
      r.heightM = -w.z() - nearest->level;
      r.radiusM = propRadius_;
      // Wash speed goes with rotor speed, i.e. with the duty.
      r.strength = std::clamp(snap_.motor_duty[size_t(i)] * 1.2f, 0.0f, 1.0f);
      ++n;
    }
  }
  ripples_->setRotors(rotors.data(), n);
  // Sim time (1 ms per engine tick): a stopped sim freezes the water, and both
  // views read the same tick, so the second one steps nothing.
  ripples_->advanceTo(double(snap_.tick_count) * 1e-3);
}

void SimRendererWidget::drawWater(const QMatrix4x4 &view) {
  // The bodies near the camera, rebuilt when it has moved a fair way: cells are
  // 170 m, the view ends in haze well inside 650 m.
  constexpr float kRangeM = 650.0f, kRebuildM = 120.0f;
  const QVector2D c(camEye_.x(), camEye_.y());
  if ((c - waterCentre_).length() > kRebuildM) {
    waterCentre_ = c;
    water_->setBodies(waterField_->water().bodiesIn(
        waterField_->heightSource(), c.x() - kRangeM, c.y() - kRangeM,
        c.x() + kRangeM, c.y() + kRangeM));
  }
  if (!water_->hasBodies())
    return;
  if (!water_->glReady() &&
      !water_->initGL({QByteArray(kAtmosphereGLSL) + kFogGLSL, kLightingGLSL}))
    return;
  updateRipples();
  WaterFrame f;
  f.proj = proj_;
  f.view = view;
  f.atmo = atmo_;
  f.camEye = camEye_;
  f.sunDir = sunDir_;
  f.time = floraTime_;
  f.ripples = ripples_.get();
  f.ssr = waterSsr_;
  f.sceneFbo = context()->defaultFramebufferObject();
  f.viewW = fbW_;
  f.viewH = fbH_;
  f.sunIntensity = litSunInt_;
  f.ambientStrength = litAmbStr_;
  f.fogDensity = atmo_.fogDensity;
  f.fogStart = atmo_.fogStartM;
  f.fogBase = fogBaseZ_;
  f.fogHeight = atmo_.fogHeightM;
  const auto pond =
      procgen::GroundCover::pondStyle(waterField_->params().palette).water;
  f.waterColor = QVector3D(pond.x, pond.y, pond.z);
  water_->draw(f);
}

void SimRendererWidget::flushChunkUpdates() {
  chunksDirty_ = false;
  if (clearAllChunks_) {
    eraseAllChunks(); // GL context current here -> safe to destroy
    pendingChunkRemovals_.clear();
    clearAllChunks_ = false;
    // NOTE: pendingChunkUploads_ is intentionally NOT cleared here —
    // clearWorldChunks() already dropped the older ones; uploads queued
    // after a clear request (e.g. reconfiguring the streamer: clear old set,
    // then stream the new one) are the fresh set and must still apply.
  }
  for (qint64 key : pendingChunkRemovals_)
    eraseChunk(key);
  pendingChunkRemovals_.clear();
  for (PendingChunk &pc : pendingChunkUploads_) {
    std::unique_ptr<Mesh> &mesh = worldChunks_[pc.key];
    if (!mesh)
      mesh = std::make_unique<Mesh>();
    uploadColoredMesh(*mesh, pc.data, 17);
    if (chunkCpuMode())
      chunkCpu_[pc.key] = std::move(pc.data);
  }
  pendingChunkUploads_.clear();
  ++shadowGeomVer_;
}

#ifdef VAYU_SIM_GRASS
void SimRendererWidget::buildGrassBlade() {
  // A single curved blade (UE5-style): a quadratic Bezier spine swept into a
  // tapered strip with a real surface (ribbon) normal per cross-section. Built
  // at 3 LODs (4/2/1 segments) so distant chunks draw far fewer verts/blade.
  // Local space: base at origin, up = -Z, arc leans toward +X.
  const float wb = 0.05f;
  const float P0x = 0.0f, P0z = 0.0f;
  const float P1x = 0.14f, P1z = -0.55f;
  const float P2x = 0.42f, P2z = -1.0f;
  auto bez = [&](float t, float &x, float &z) {
    const float u = 1.0f - t;
    x = u * u * P0x + 2.0f * u * t * P1x + t * t * P2x;
    z = u * u * P0z + 2.0f * u * t * P1z + t * t * P2z;
  };
  // Spine tangent (derivative); the ribbon normal is perpendicular to it and Y,
  // biased toward up (-Z) so blades catch overhead sun/sky (softer, UE5-like).
  auto normal = [&](float t, float &nx, float &nz) {
    const float tx = 2.0f * (1.0f - t) * (P1x - P0x) + 2.0f * t * (P2x - P1x);
    const float tz = 2.0f * (1.0f - t) * (P1z - P0z) + 2.0f * t * (P2z - P1z);
    float rx = -tz, rz = tx; // cross(T, +Y) in the X-Z plane
    rz -= 0.9f;              // up-bias (NED up is -Z)
    const float l = std::sqrt(rx * rx + rz * rz);
    nx = rx / l;
    nz = rz / l;
  };
  auto width = [&](float t) {
    return wb * (0.06f + 0.94f * std::pow(1.0f - t, 0.7f));
  };

  const int segCounts[3] = {4, 2, 1};
  for (int lod = 0; lod < 3; ++lod) {
    const int kSeg = segCounts[lod];
    std::vector<float> v; // [px,py,pz, nx,ny,nz] per vertex
    auto vert = [&](float x, float y, float z, float nx, float nz) {
      v.insert(v.end(), {x, y, z, nx, 0.0f, nz});
    };
    for (int s = 0; s < kSeg; ++s) {
      const float t0 = float(s) / kSeg, t1 = float(s + 1) / kSeg;
      float x0, z0, x1, z1, n0x, n0z, n1x, n1z;
      bez(t0, x0, z0);
      bez(t1, x1, z1);
      normal(t0, n0x, n0z);
      normal(t1, n1x, n1z);
      const float w0 = width(t0), w1 = width(t1);
      vert(x0, +w0, z0, n0x, n0z);
      vert(x0, -w0, z0, n0x, n0z);
      vert(x1, -w1, z1, n1x, n1z);
      vert(x0, +w0, z0, n0x, n0z);
      vert(x1, -w1, z1, n1x, n1z);
      vert(x1, +w1, z1, n1x, n1z);
    }
    grassVbo_[lod].create();
    grassVbo_[lod].bind();
    grassVbo_[lod].allocate(v.data(), int(v.size() * sizeof(float)));
    grassVbo_[lod].release();
    grassVerts_[lod] = int(v.size() / 6);
  }
  floraVao_.create();
}

void SimRendererWidget::setChunkFlora(qint64 key,
                                      const std::vector<float> &interleaved,
                                      int count, float centerX, float centerY,
                                      float halfExtent) {
  pendingFloraUploads_.push_back(
      {key, interleaved, count, centerX, centerY, halfExtent});
  floraDirty_ = true;
  update();
}

void SimRendererWidget::removeChunkFlora(qint64 key) {
  pendingFloraRemovals_.push_back(key);
  floraDirty_ = true;
  update();
}

void SimRendererWidget::clearChunkFlora() {
  clearAllFlora_ = true;
  floraDirty_ = true;
  update();
}

void SimRendererWidget::eraseFlora(qint64 key) {
  const auto it = floraChunks_.find(key);
  if (it == floraChunks_.end())
    return;
  it->second->inst.destroy();
  floraChunks_.erase(it);
  floraCpu_.erase(key);
}

void SimRendererWidget::eraseAllFlora() {
  for (auto &kv : floraChunks_)
    kv.second->inst.destroy();
  floraChunks_.clear();
  floraCpu_.clear();
}

void SimRendererWidget::flushFloraUpdates() {
  floraDirty_ = false;
  if (clearAllFlora_) {
    eraseAllFlora();
    pendingFloraRemovals_.clear();
    clearAllFlora_ = false;
    // (queued uploads are kept — fresh set after a reconfigure)
  }
  for (qint64 key : pendingFloraRemovals_)
    eraseFlora(key);
  pendingFloraRemovals_.clear();
  for (PendingFlora &pf : pendingFloraUploads_) {
    if (pf.count <= 0) {
      eraseFlora(pf.key);
      continue;
    }
    std::unique_ptr<FloraChunk> &fc = floraChunks_[pf.key];
    if (!fc)
      fc = std::make_unique<FloraChunk>();
    if (!fc->inst.isCreated())
      fc->inst.create();
    fc->inst.bind();
    fc->inst.allocate(pf.data.data(), int(pf.data.size() * sizeof(float)));
    fc->inst.release();
    fc->count = pf.count;
    fc->cx = pf.cx;
    fc->cy = pf.cy;
    fc->half = pf.half;
    if (chunkCpuMode())
      floraCpu_[pf.key] = pf;
  }
  pendingFloraUploads_.clear();
}

void SimRendererWidget::drawFlora(const QMatrix4x4 &view) {
  if (floraChunks_.empty() || grassVerts_[0] == 0)
    return;
  progFlora_->bind();
  progFlora_->setUniformValue(uf_vp_, proj_ * view);
  progFlora_->setUniformValue(uf_campos_, camEye_);
  progFlora_->setUniformValue(uf_time_, floraTime_);
  progFlora_->setUniformValue(uf_fadestart_, 90.0f);
  progFlora_->setUniformValue(uf_fadeend_, 150.0f);
  progFlora_->setUniformValue(uf_fogdensity_, atmo_.fogDensity);
  progFlora_->setUniformValue(uf_fogstart_, atmo_.fogStartM);
  progFlora_->setUniformValue("u_fogbase", fogBaseZ_);
  progFlora_->setUniformValue("u_fogheight", atmo_.fogHeightM);
  floraVao_.bind();
  for (auto &kv : floraChunks_) {
    FloraChunk *fc = kv.second.get();
    if (fc->count == 0)
      continue;
    // Per-chunk LOD by distance to the NEAREST point of the chunk (so the chunk
    // you're standing in stays full detail): fewer verts/blade the farther it is.
    const float nx =
        std::max(fc->cx - fc->half, std::min(camEye_.x(), fc->cx + fc->half));
    const float ny =
        std::max(fc->cy - fc->half, std::min(camEye_.y(), fc->cy + fc->half));
    const float dx = nx - camEye_.x(), dy = ny - camEye_.y();
    const float dist = std::sqrt(dx * dx + dy * dy);
    const int lod = dist < 40.0f ? 0 : (dist < 90.0f ? 1 : 2);

    // Bind the LOD geometry (pos + normal, divisor 0).
    grassVbo_[lod].bind();
    const int gstride = 6 * sizeof(float);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, gstride, nullptr);
    glVertexAttribDivisor(0, 0);
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, gstride,
                          reinterpret_cast<void *>(3 * sizeof(float)));
    glVertexAttribDivisor(4, 0);
    // Bind this chunk's instance data (9 floats/instance, divisor 1).
    fc->inst.bind();
    const int istride = 9 * sizeof(float);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, istride, nullptr);
    glVertexAttribDivisor(1, 1);
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, istride,
                          reinterpret_cast<void *>(3 * sizeof(float)));
    glVertexAttribDivisor(2, 1);
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, istride,
                          reinterpret_cast<void *>(6 * sizeof(float)));
    glVertexAttribDivisor(3, 1);
    glDrawArraysInstanced(GL_TRIANGLES, 0, grassVerts_[lod], fc->count);
  }
  floraVao_.release();
  progFlora_->release();
}
#endif // VAYU_SIM_GRASS

void SimRendererWidget::buildGroundGrid() {
  std::vector<float> verts;
  for (int i = -kGridHalf; i <= kGridHalf; ++i) {
    float f = i * kGridStep;
    // Line along +X at y=f, z=0
    verts.insert(verts.end(), {-(float)kGridHalf, f, 0.0f});
    verts.insert(verts.end(), {+(float)kGridHalf, f, 0.0f});
    // Line along +Y at x=f, z=0
    verts.insert(verts.end(), {f, -(float)kGridHalf, 0.0f});
    verts.insert(verts.end(), {f, +(float)kGridHalf, 0.0f});
  }
  ground_.vao.create();
  ground_.vao.bind();
  ground_.vbo.create();
  ground_.vbo.bind();
  ground_.vbo.allocate(verts.data(), int(verts.size() * sizeof(float)));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  ground_.vbo.release();
  ground_.vao.release();
  ground_.vertex_count = int(verts.size() / 3);
  ground_.primitive = GL_LINES;
}

void SimRendererWidget::buildAxes() {
  const float L = 1.0f;
  // X=red, Y=green, Z=blue per draw call - here just three line
  // segments and we'll re-tint per-axis later. For simplicity ship a
  // single white set; per-axis color would need separate draws.
  float verts[] = {
      0, 0, 0, L, 0, 0, // +X (north)
      0, 0, 0, 0, L, 0, // +Y (east)
      0, 0, 0, 0, 0, L, // +Z (down)
  };
  axes_.vao.create();
  axes_.vao.bind();
  axes_.vbo.create();
  axes_.vbo.bind();
  axes_.vbo.allocate(verts, sizeof(verts));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  axes_.vbo.release();
  axes_.vao.release();
  axes_.vertex_count = 6;
  axes_.primitive = GL_LINES;
}

void SimRendererWidget::buildDroneBody() {
  // Simple box from triangles, centered at body origin, extents kBodyL,
  // kBodyW, kBodyH. Body Z is "down" in NED; for the rendered body we
  // draw a thin plate so the orientation reads cleanly from above.
  const float x = kBodyL * 0.5f, y = kBodyW * 0.5f, z = kBodyH * 0.5f;
  float v[] = {
      // top
      -x,
      -y,
      -z,
      x,
      -y,
      -z,
      x,
      y,
      -z,
      -x,
      -y,
      -z,
      x,
      y,
      -z,
      -x,
      y,
      -z,
      // bottom
      -x,
      -y,
      z,
      x,
      y,
      z,
      x,
      -y,
      z,
      -x,
      -y,
      z,
      -x,
      y,
      z,
      x,
      y,
      z,
      // sides
      -x,
      -y,
      -z,
      -x,
      y,
      -z,
      -x,
      y,
      z,
      -x,
      -y,
      -z,
      -x,
      y,
      z,
      -x,
      -y,
      z,
      x,
      -y,
      -z,
      x,
      -y,
      z,
      x,
      y,
      z,
      x,
      -y,
      -z,
      x,
      y,
      z,
      x,
      y,
      -z,
      -x,
      -y,
      -z,
      -x,
      -y,
      z,
      x,
      -y,
      z,
      -x,
      -y,
      -z,
      x,
      -y,
      z,
      x,
      -y,
      -z,
      -x,
      y,
      -z,
      x,
      y,
      -z,
      x,
      y,
      z,
      -x,
      y,
      -z,
      x,
      y,
      z,
      -x,
      y,
      z,
  };
  body_.vao.create();
  body_.vao.bind();
  body_.vbo.create();
  body_.vbo.bind();
  body_.vbo.allocate(v, sizeof(v));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  body_.vbo.release();
  body_.vao.release();
  body_.vertex_count = sizeof(v) / (3 * sizeof(float));
  body_.primitive = GL_TRIANGLES;
}

void SimRendererWidget::buildRotorDisk() {
  // Disk as triangle fan around body Z = 0 (flat on the body plane).
  std::vector<float> v;
  v.insert(v.end(), {0.0f, 0.0f, 0.0f});
  for (int i = 0; i <= kRotorSeg; ++i) {
    float a = float(i) / kRotorSeg * 2.0f * float(M_PI);
    v.insert(v.end(), {kRotorR * std::cos(a), kRotorR * std::sin(a), 0.0f});
  }
  rotor_.vao.create();
  rotor_.vao.bind();
  rotor_.vbo.create();
  rotor_.vbo.bind();
  rotor_.vbo.allocate(v.data(), int(v.size() * sizeof(float)));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  rotor_.vbo.release();
  rotor_.vao.release();
  rotor_.vertex_count = int(v.size() / 3);
  rotor_.primitive = GL_TRIANGLE_FAN;
}

void SimRendererWidget::buildThrustLine() {
  // Unit segment along +Z; paintGL rotates it onto each motor's axis.
  float v[] = {0, 0, 0, 0, 0, 1};
  thrustLine_.vao.create();
  thrustLine_.vao.bind();
  thrustLine_.vbo.create();
  thrustLine_.vbo.bind();
  thrustLine_.vbo.allocate(v, sizeof(v));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  thrustLine_.vbo.release();
  thrustLine_.vao.release();
  thrustLine_.vertex_count = 2;
  thrustLine_.primitive = GL_LINES;
}

void SimRendererWidget::buildComMarker() {
  // Small 3-axis crosshair centered at origin.
  const float s = 0.06f;
  float v[] = {
      -s, 0, 0, s, 0, 0, 0, -s, 0, 0, s, 0, 0, 0, -s, 0, 0, s,
  };
  comMarker_.vao.create();
  comMarker_.vao.bind();
  comMarker_.vbo.create();
  comMarker_.vbo.bind();
  comMarker_.vbo.allocate(v, sizeof(v));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  comMarker_.vbo.release();
  comMarker_.vao.release();
  comMarker_.vertex_count = 6;
  comMarker_.primitive = GL_LINES;
}

void SimRendererWidget::buildGizmo() {
  // Unit +X arrow shaft (rotated per axis in drawGizmo).
  {
    float v[] = {0, 0, 0, 1, 0, 0};
    gizmoArrow_.vao.create();
    gizmoArrow_.vao.bind();
    gizmoArrow_.vbo.create();
    gizmoArrow_.vbo.bind();
    gizmoArrow_.vbo.allocate(v, sizeof(v));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    gizmoArrow_.vbo.release();
    gizmoArrow_.vao.release();
    gizmoArrow_.vertex_count = 2;
    gizmoArrow_.primitive = GL_LINES;
  }
  // Unit circle in the XY plane (line loop), rotated per axis for rings.
  {
    std::vector<float> v;
    const int seg = 48;
    for (int i = 0; i < seg; ++i) {
      float a = float(i) / seg * 2.0f * float(M_PI);
      v.insert(v.end(), {std::cos(a), std::sin(a), 0.0f});
    }
    gizmoRing_.vao.create();
    gizmoRing_.vao.bind();
    gizmoRing_.vbo.create();
    gizmoRing_.vbo.bind();
    gizmoRing_.vbo.allocate(v.data(), int(v.size() * sizeof(float)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    gizmoRing_.vbo.release();
    gizmoRing_.vao.release();
    gizmoRing_.vertex_count = seg;
    gizmoRing_.primitive = GL_LINE_LOOP;
  }
}

void SimRendererWidget::buildDigits() {
  // Each glyph is a set of 2D line segments (x0,y0,x1,y1,...) in a unit
  // cell, expanded to (x,y,0) vertices and drawn billboarded as GL_LINES.
  const std::vector<std::vector<float>> strokes = {
      // "1"
      {0, -0.5f, 0, 0.5f, -0.18f, 0.32f, 0, 0.5f, -0.2f, -0.5f, 0.2f, -0.5f},
      // "2"
      {-0.3f, 0.5f, 0.3f,  0.5f, 0.3f,  0.5f,  0.3f,  0.0f,  0.3f, 0.0f,
       -0.3f, 0.0f, -0.3f, 0.0f, -0.3f, -0.5f, -0.3f, -0.5f, 0.3f, -0.5f},
      // "3"
      {-0.3f, 0.5f, 0.3f, 0.5f, 0.3f, 0.5f, 0.3f, -0.5f, -0.25f, 0.0f, 0.3f,
       0.0f, -0.3f, -0.5f, 0.3f, -0.5f},
      // "4"
      {-0.3f, 0.5f, -0.3f, 0.0f, -0.3f, 0.0f, 0.3f, 0.0f, 0.3f, 0.5f, 0.3f,
       -0.5f},
  };
  for (int d = 0; d < 4; ++d) {
    std::vector<float> v;
    const auto &s = strokes[d];
    for (size_t k = 0; k + 1 < s.size(); k += 2) {
      v.insert(v.end(), {s[k], s[k + 1], 0.0f});
    }
    Mesh &m = digitMesh_[d];
    m.vao.create();
    m.vao.bind();
    m.vbo.create();
    m.vbo.bind();
    m.vbo.allocate(v.data(), int(v.size() * sizeof(float)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    m.vbo.release();
    m.vao.release();
    m.vertex_count = int(v.size() / 3);
    m.primitive = GL_LINES;
  }
}

void SimRendererWidget::drawMotorLabels(const QMatrix4x4 &view) {
  QVector3D eye, tgt;
  cameraEyeTarget(&eye, &tgt);
  const QVector3D fwd = (tgt - eye).normalized();
  const QVector3D worldUp(0, 0, -1); // NED up
  QVector3D right = QVector3D::crossProduct(fwd, worldUp);
  if (right.lengthSquared() < 1e-6f)
    right = QVector3D(1, 0, 0);
  right.normalize();
  const QVector3D up = QVector3D::crossProduct(right, fwd).normalized();

  const QMatrix4x4 model = modelMatrix();
  const float s = 0.07f; // glyph size [m]
  glDisable(GL_DEPTH_TEST);
  for (int i = 0; i < 4; ++i) {
    const QVector3D pos = markerModel().map(motorPos_[i]) + up * 0.12f; // above marker
    QMatrix4x4 m;
    m.setColumn(0, QVector4D(right * s, 0));
    m.setColumn(1, QVector4D(up * s, 0));
    m.setColumn(2, QVector4D(-fwd * s, 0));
    m.setColumn(3, QVector4D(pos, 1));
    const QVector3D col = (editable_ && i == selected_)
                              ? QVector3D(1.0f, 0.95f, 0.4f) // selected
                              : QVector3D(0.92f, 0.92f, 0.96f);
    drawMesh(digitMesh_[i], view * m, col);
  }
  glEnable(GL_DEPTH_TEST);
}

void SimRendererWidget::buildNorth() {
  // Screen-space locator: chevron pointing +Y (rotated to the heading).
  {
    float v[] = {
        0.0f, 1.0f, 0.0f, -0.7f, -0.7f, 0.0f, //
        0.0f, 1.0f, 0.0f, 0.7f,  -0.7f, 0.0f, //
        -0.7f, -0.7f, 0.0f, 0.0f, -0.25f, 0.0f, //
        0.7f, -0.7f, 0.0f, 0.0f, -0.25f, 0.0f,
    };
    locator_.vao.create();
    locator_.vao.bind();
    locator_.vbo.create();
    locator_.vbo.bind();
    locator_.vbo.allocate(v, sizeof(v));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    locator_.vbo.release();
    locator_.vao.release();
    locator_.vertex_count = 8;
    locator_.primitive = GL_LINES;
  }
  // Arrow along body +X (north), in the body XY plane: shaft + two head
  // segments. Drawn through the model matrix so it rotates with the body.
  {
    float v[] = {
        0.0f,  0.0f, 0.0f, 0.35f, 0.0f,   0.0f, // shaft
        0.35f, 0.0f, 0.0f, 0.27f, 0.05f,  0.0f, // head
        0.35f, 0.0f, 0.0f, 0.27f, -0.05f, 0.0f,
    };
    northArrow_.vao.create();
    northArrow_.vao.bind();
    northArrow_.vbo.create();
    northArrow_.vbo.bind();
    northArrow_.vbo.allocate(v, sizeof(v));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    northArrow_.vbo.release();
    northArrow_.vao.release();
    northArrow_.vertex_count = 6;
    northArrow_.primitive = GL_LINES;
  }
  // "N" glyph (billboarded at the tip).
  {
    float v[] = {
        -0.3f, -0.5f, 0.0f, -0.3f, 0.5f,  0.0f, // left vertical
        -0.3f, 0.5f,  0.0f, 0.3f,  -0.5f, 0.0f, // diagonal
        0.3f,  -0.5f, 0.0f, 0.3f,  0.5f,  0.0f, // right vertical
    };
    glyphN_.vao.create();
    glyphN_.vao.bind();
    glyphN_.vbo.create();
    glyphN_.vbo.bind();
    glyphN_.vbo.allocate(v, sizeof(v));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    glyphN_.vbo.release();
    glyphN_.vao.release();
    glyphN_.vertex_count = 6;
    glyphN_.primitive = GL_LINES;
  }
}

void SimRendererWidget::drawNorthIndicator(const QMatrix4x4 &view) {
  const QMatrix4x4 model = modelMatrix();
  const QVector3D north(1.0f, 0.45f, 0.45f); // matches +X axis colouring
  glDisable(GL_DEPTH_TEST);

  // Arrow points along body +X in 3D (not billboarded — it shows heading).
  drawMesh(northArrow_, view * model, north);

  // "N" at the tip, billboarded so it's always readable.
  QVector3D eye, tgt;
  cameraEyeTarget(&eye, &tgt);
  const QVector3D fwd = (tgt - eye).normalized();
  QVector3D right = QVector3D::crossProduct(fwd, QVector3D(0, 0, -1));
  if (right.lengthSquared() < 1e-6f)
    right = QVector3D(1, 0, 0);
  right.normalize();
  const QVector3D up = QVector3D::crossProduct(right, fwd).normalized();
  const float s = 0.055f;
  const QVector3D pos = model.map(QVector3D(0.44f, 0.0f, 0.0f)) + up * 0.05f;
  QMatrix4x4 m;
  m.setColumn(0, QVector4D(right * s, 0));
  m.setColumn(1, QVector4D(up * s, 0));
  m.setColumn(2, QVector4D(-fwd * s, 0));
  m.setColumn(3, QVector4D(pos, 1));
  drawMesh(glyphN_, view * m, north);

  glEnable(GL_DEPTH_TEST);
}

void SimRendererWidget::buildRing() {
  // Unit torus: major radius 1 in the local XY plane (axis = local +Z), minor
  // (tube) radius small so it reads as a thin glowing halo. Per-gate scale sets
  // the real radius. Interleaved pos+normal for the lit shader.
  const int kMajor = 40, kMinor = 12;
  const float kTube = 0.09f; // tube radius relative to the unit major radius
  std::vector<float> v;
  v.reserve(static_cast<size_t>(kMajor) * kMinor * 6 * 6);
  auto vert = [&](int i, int j) {
    const float u = float(i % kMajor) / kMajor * 2.0f * float(M_PI);
    const float vv = float(j % kMinor) / kMinor * 2.0f * float(M_PI);
    const float cu = std::cos(u), su = std::sin(u);
    const float cv = std::cos(vv), sv = std::sin(vv);
    // Position on the torus ring of major radius 1.
    const QVector3D p((1.0f + kTube * cv) * cu, (1.0f + kTube * cv) * su,
                      kTube * sv);
    // Outward normal points away from the ring centreline.
    const QVector3D n(cv * cu, cv * su, sv);
    v.insert(v.end(), {p.x(), p.y(), p.z(), n.x(), n.y(), n.z()});
  };
  for (int i = 0; i < kMajor; ++i)
    for (int j = 0; j < kMinor; ++j) {
      vert(i, j);
      vert(i + 1, j);
      vert(i + 1, j + 1); // tri 1
      vert(i, j);
      vert(i + 1, j + 1);
      vert(i, j + 1); // tri 2
    }
  uploadLitMesh(ring_, v);
}

void SimRendererWidget::buildGuideArrow() {
  // A solid arrow along +X: a thin square shaft (0..0.7) plus a cone head
  // (0.7..1.0). Built as flat-shaded triangles (pos+normal) for the lit shader.
  std::vector<float> v;
  auto tri = [&](const QVector3D &a, const QVector3D &b, const QVector3D &c) {
    QVector3D n = QVector3D::crossProduct(b - a, c - a);
    if (n.lengthSquared() > 1e-12f)
      n.normalize();
    for (const QVector3D &p : {a, b, c})
      v.insert(v.end(), {p.x(), p.y(), p.z(), n.x(), n.y(), n.z()});
  };
  const float sh = 0.05f; // shaft half-width
  const float L = 0.7f;   // shaft length (head from L..1.0)
  const float hr = 0.14f; // head base radius
  // Shaft: a thin box from x=0 to x=L, square cross-section in y/z.
  const QVector3D s0(0, -sh, -sh), s1(0, sh, -sh), s2(0, sh, sh),
      s3(0, -sh, sh);
  const QVector3D e0(L, -sh, -sh), e1(L, sh, -sh), e2(L, sh, sh),
      e3(L, -sh, sh);
  auto quad = [&](const QVector3D &a, const QVector3D &b, const QVector3D &c,
                  const QVector3D &d) {
    tri(a, b, c);
    tri(a, c, d);
  };
  quad(s0, s1, s2, s3); // back cap
  quad(e3, e2, e1, e0); // front cap (shaft/head junction filled by cone base)
  quad(s0, s3, e3, e0); // -y
  quad(s1, s0, e0, e1); // -z
  quad(s2, s1, e1, e2); // +y
  quad(s3, s2, e2, e3); // +z
  // Cone head: apex at x=1, base ring of radius hr at x=L.
  const QVector3D apex(1.0f, 0, 0);
  const int seg = 16;
  for (int i = 0; i < seg; ++i) {
    const float a0 = float(i) / seg * 2.0f * float(M_PI);
    const float a1 = float(i + 1) / seg * 2.0f * float(M_PI);
    const QVector3D b0(L, hr * std::cos(a0), hr * std::sin(a0));
    const QVector3D b1(L, hr * std::cos(a1), hr * std::sin(a1));
    tri(b0, b1, apex);                                        // side
    tri(L * QVector3D(1, 0, 0) + QVector3D(0, 0, 0), b1, b0); // base
  }
  uploadLitMesh(guideArrow_, v);
}

void SimRendererWidget::drawTraining(const QMatrix4x4 &view) {
  if (gates_.isEmpty())
    return;
  glowPhase_ += 0.08f;
  const float pulse = 0.5f + 0.5f * std::sin(glowPhase_);

  // Reveal gates one at a time: only the active target glows bright (cyan
  // pulse); the very next gate shows as a faint ghost so the pilot can read
  // ahead. Cleared and far-future gates are hidden so the arena stays clean.
  for (int i = trainActive_; i <= trainActive_ + 1 && i < gates_.size(); ++i) {
    const vsim::RingGate &g = gates_[i];
    QMatrix4x4 m;
    m.translate(g.center);
    // Orient the torus axis (local +Z) onto the gate normal.
    m.rotate(QQuaternion::rotationTo(QVector3D(0, 0, 1), g.normal));
    m.scale(g.radius);
    const QVector3D col =
        (i == trainActive_)
            ? QVector3D(0.20f + 0.6f * pulse, 0.85f, 0.95f) // active target
            : QVector3D(0.18f, 0.24f, 0.42f); // faint next-gate ghost
    drawLit(ring_, view, m, col);
  }

  // Guidance arrow: floats just above the drone, points at the active gate.
  if (trainArrow_ && trainActive_ >= 0 && trainActive_ < gates_.size()) {
    const QVector3D from = snap_.pos_w + QVector3D(0, 0, -0.35f); // above drone
    QVector3D dir = gates_[trainActive_].center - snap_.pos_w;
    if (dir.lengthSquared() > 1e-6f) {
      dir.normalize();
      QMatrix4x4 m;
      m.translate(from);
      m.rotate(QQuaternion::rotationTo(QVector3D(1, 0, 0), dir));
      m.scale(0.8f);
      glDisable(GL_DEPTH_TEST); // always visible, never buried in geometry
      drawLit(guideArrow_, view, m,
              QVector3D(1.0f, 0.85f, 0.15f + 0.3f * pulse));
      glEnable(GL_DEPTH_TEST);
    }
  }
}

// ---------- camera controls ----------

void SimRendererWidget::mousePressEvent(QMouseEvent *e) {
  requestActivate(); // key focus for G/R/X/Y/Z/Esc/WASD (the widget used setFocus)
  last_mouse_ = e->pos();

  // Obstacle editing (World mode) takes precedence over camera orbit.
  if (obsMode_) {
    if (obsTool_ != Tool::None) {
      if (e->button() == Qt::LeftButton)
        commitObsTool(true);
      else if (e->button() == Qt::RightButton)
        commitObsTool(false);
      return;
    }
    if (e->button() == Qt::LeftButton) {
      int idx = -1;
      if (pickObstacle(e->pos(), &idx)) {
        selObs_ = idx;
        emit obstacleSelected(idx);
        update();
        return; // consumed — don't orbit
      }
      if (selObs_ != -1) {
        selObs_ = -1;
        emit obstacleSelected(-1);
        update();
      }
    }
    return; // empty click: mouseMove (button held) orbits the camera
  }

  // A live tool: left-click confirms the transform, right-click cancels.
  if (editable_ && tool_ != Tool::None) {
    if (e->button() == Qt::LeftButton)
      commitTool(true);
    else if (e->button() == Qt::RightButton)
      commitTool(false);
    return;
  }

  // Otherwise left-click tries to select a motor; an empty click
  // deselects and falls through to camera orbit.
  if (editable_ && e->button() == Qt::LeftButton) {
    int idx = -1;
    if (pickMotor(e->pos(), &idx)) {
      selected_ = idx;
      emit motorSelected(idx);
      update();
      return; // consumed — don't orbit
    }
    if (selected_ != -1) {
      selected_ = -1;
      update();
    }
  }
}

void SimRendererWidget::mouseMoveEvent(QMouseEvent *e) {
  // While a tool is live the mouse drives it (no button held, Blender-style).
  if (obsMode_ && obsTool_ != Tool::None) {
    updateObsToolFromMouse(e->pos());
    return;
  }
  if (editable_ && tool_ != Tool::None) {
    updateToolFromMouse(e->pos());
    return;
  }
  QPoint d = e->pos() - last_mouse_;
  last_mouse_ = e->pos();
  if (e->buttons() & Qt::LeftButton) {
    cam_yaw_ -= d.x() * 0.01f;
    cam_pitch_ += d.y() * 0.01f;
    const float lim = 1.4f;
    if (cam_pitch_ > lim)
      cam_pitch_ = lim;
    if (cam_pitch_ < -lim)
      cam_pitch_ = -lim;
    update();
  }
}

bool SimRendererWidget::freeFlyMove(int key, bool fast) {
  const float step = (fast ? 2.0f : 0.5f);
  const QVector3D fwd = freeForward();
  const QVector3D worldUp(0.0f, 0.0f, -1.0f); // NED up
  QVector3D right = QVector3D::crossProduct(fwd, worldUp);
  if (right.lengthSquared() < 1e-9f)
    right = QVector3D(0, 1, 0);
  right.normalize();
  switch (key) {
  case Qt::Key_W:
    camPos_ += fwd * step;
    break;
  case Qt::Key_S:
    camPos_ -= fwd * step;
    break;
  case Qt::Key_D:
    camPos_ += right * step;
    break;
  case Qt::Key_A:
    camPos_ -= right * step;
    break;
  case Qt::Key_E:
    camPos_ += worldUp * step;
    break; // up
  case Qt::Key_Q:
    camPos_ -= worldUp * step;
    break; // down
  default:
    return false;
  }
  update();
  return true;
}

void SimRendererWidget::keyPressEvent(QKeyEvent *e) {
  // Free-roam navigation (World mode, sim stopped). WASD/QE fly the camera;
  // obstacle gizmo keys still work so you can edit while roaming: select an
  // obstacle then G/R/S; with nothing selected every movement key flies.
  if (freeFly_) {
    if (obsMode_ && obsTool_ != Tool::None) {
      switch (e->key()) {
      case Qt::Key_X:
        obsAxis_ = 0;
        updateObsToolFromMouse(last_mouse_);
        break;
      case Qt::Key_Y:
        obsAxis_ = 1;
        updateObsToolFromMouse(last_mouse_);
        break;
      case Qt::Key_Z:
        obsAxis_ = 2;
        updateObsToolFromMouse(last_mouse_);
        break;
      case Qt::Key_Escape:
        commitObsTool(false);
        break;
      case Qt::Key_Return:
      case Qt::Key_Enter:
        commitObsTool(true);
        break;
      default:
        QOpenGLWindow::keyPressEvent(e);
        return;
      }
      return;
    }
    if (obsMode_ && selObs_ >= 0) {
      switch (e->key()) {
      case Qt::Key_G:
        beginObsTool(Tool::Move);
        return;
      case Qt::Key_R:
        beginObsTool(Tool::Rotate);
        return;
      case Qt::Key_S:
        beginObsTool(Tool::Scale);
        return; // scale selected
      case Qt::Key_Escape:
        selObs_ = -1;
        emit obstacleSelected(-1);
        update();
        return;
      default:
        break; // fall through to movement (W/A/D/Q/E)
      }
    }
    if (freeFlyMove(e->key(), e->modifiers() & Qt::ShiftModifier))
      return;
    QOpenGLWindow::keyPressEvent(e);
    return;
  }
  if (obsMode_) {
    switch (e->key()) {
    case Qt::Key_G:
      if (selObs_ >= 0)
        beginObsTool(Tool::Move);
      break;
    case Qt::Key_R:
      if (selObs_ >= 0)
        beginObsTool(Tool::Rotate);
      break;
    case Qt::Key_S:
      if (selObs_ >= 0)
        beginObsTool(Tool::Scale);
      break;
    case Qt::Key_X:
      if (obsTool_ != Tool::None) {
        obsAxis_ = 0;
        updateObsToolFromMouse(last_mouse_);
      }
      break;
    case Qt::Key_Y:
      if (obsTool_ != Tool::None) {
        obsAxis_ = 1;
        updateObsToolFromMouse(last_mouse_);
      }
      break;
    case Qt::Key_Z:
      if (obsTool_ != Tool::None) {
        obsAxis_ = 2;
        updateObsToolFromMouse(last_mouse_);
      }
      break;
    case Qt::Key_Escape:
      if (obsTool_ != Tool::None)
        commitObsTool(false);
      else if (selObs_ != -1) {
        selObs_ = -1;
        emit obstacleSelected(-1);
        update();
      }
      break;
    case Qt::Key_Return:
    case Qt::Key_Enter:
      if (obsTool_ != Tool::None)
        commitObsTool(true);
      break;
    default:
      QOpenGLWindow::keyPressEvent(e);
      return;
    }
    return;
  }
  if (!editable_) {
    QOpenGLWindow::keyPressEvent(e);
    return;
  }
  switch (e->key()) {
  case Qt::Key_G:
    if (selected_ >= 0)
      beginTool(Tool::Move);
    break;
  case Qt::Key_R:
    if (selected_ >= 0)
      beginTool(Tool::Rotate);
    break;
  case Qt::Key_X:
    if (tool_ != Tool::None) {
      axisLock_ = 0;
      updateToolFromMouse(last_mouse_);
    }
    break;
  case Qt::Key_Y:
    if (tool_ != Tool::None) {
      axisLock_ = 1;
      updateToolFromMouse(last_mouse_);
    }
    break;
  case Qt::Key_Z:
    if (tool_ != Tool::None) {
      axisLock_ = 2;
      updateToolFromMouse(last_mouse_);
    }
    break;
  case Qt::Key_Escape:
    if (tool_ != Tool::None)
      commitTool(false);
    else if (selected_ != -1) {
      selected_ = -1;
      update();
    }
    break;
  case Qt::Key_Return:
  case Qt::Key_Enter:
    if (tool_ != Tool::None)
      commitTool(true);
    break;
  default:
    QOpenGLWindow::keyPressEvent(e);
    return;
  }
}

// ---------- gizmo editing ----------

// Body -> world for the editor markers (motor discs, thrust lines, numbers,
// gizmo). The procedural model is lifted by modelLift_ so its lowest point rests
// on the ground; the markers sit on the drawn motors, which moved with it.
// motorPos_ itself is the true layout and is never shifted.
QMatrix4x4 SimRendererWidget::markerModel() const {
  QMatrix4x4 m = modelMatrix();
  if (!hasMesh_)
    m.translate(0.0f, 0.0f, -modelLift_);
  return m;
}

QMatrix4x4 SimRendererWidget::modelMatrix() const {
  QMatrix4x4 m;
  m.translate(snap_.pos_w);
  m.rotate(snap_.att);
  return m;
}

void SimRendererWidget::cameraEyeTarget(QVector3D *eye,
                                        QVector3D *target) const {
  const QVector3D tgt = snap_.pos_w;
  if (target)
    *target = tgt;
  if (eye)
    *eye = tgt + orbitOffset();
}

void SimRendererWidget::rayThroughPixel(const QPoint &px, QVector3D *o,
                                        QVector3D *d) const {
  const float w = std::max(1, width()), h = std::max(1, height());
  const float ndcx = 2.0f * px.x() / w - 1.0f;
  const float ndcy = 1.0f - 2.0f * px.y() / h;
  bool ok = false;
  const QMatrix4x4 inv = (proj_ * cameraView()).inverted(&ok);
  const QVector3D np =
      (inv * QVector4D(ndcx, ndcy, -1.0f, 1.0f)).toVector3DAffine();
  const QVector3D fp =
      (inv * QVector4D(ndcx, ndcy, 1.0f, 1.0f)).toVector3DAffine();
  if (o)
    *o = np;
  if (d)
    *d = (fp - np).normalized();
}

bool SimRendererWidget::pickMotor(const QPoint &px, int *outIndex) const {
  const QMatrix4x4 vp = proj_ * cameraView();
  const QMatrix4x4 model = modelMatrix();
  const float w = std::max(1, width()), h = std::max(1, height());
  float best = 22.0f; // pixel radius
  int bestI = -1;
  for (int i = 0; i < 4; ++i) {
    const QVector3D wp = markerModel().map(motorPos_[i]);
    const QVector4D clip = vp * QVector4D(wp, 1.0f);
    if (clip.w() <= 0.0f)
      continue;
    const float sx = (clip.x() / clip.w() * 0.5f + 0.5f) * w;
    const float sy = (1.0f - (clip.y() / clip.w() * 0.5f + 0.5f)) * h;
    const float dpix = std::hypot(sx - px.x(), sy - px.y());
    if (dpix < best) {
      best = dpix;
      bestI = i;
    }
  }
  if (bestI >= 0 && outIndex)
    *outIndex = bestI;
  return bestI >= 0;
}

void SimRendererWidget::beginTool(Tool t) {
  if (selected_ < 0)
    return;
  tool_ = t;
  axisLock_ = -1;
  startPos_ = motorPos_[selected_];
  startAxis_ = motorAxis_[selected_];
  toolStartMouse_ = last_mouse_ = mapFromGlobal(QCursor::pos());
  startWorld_ = markerModel().map(startPos_);
  // Drag plane: through the motor, facing the camera.
  QVector3D eye, tgt;
  cameraEyeTarget(&eye, &tgt);
  const QVector3D fwd = (tgt - eye).normalized();
  QVector3D o, d;
  rayThroughPixel(toolStartMouse_, &o, &d);
  const float denom = QVector3D::dotProduct(d, fwd);
  const float tt = (std::fabs(denom) > 1e-6f)
                       ? QVector3D::dotProduct(startWorld_ - o, fwd) / denom
                       : 0.0f;
  planeHit0_ = o + d * tt;
  update();
}

void SimRendererWidget::updateToolFromMouse(const QPoint &px) {
  if (selected_ < 0 || tool_ == Tool::None)
    return;
  last_mouse_ = px;
  const QMatrix4x4 model = modelMatrix();

  if (tool_ == Tool::Move) {
    QVector3D eye, tgt;
    cameraEyeTarget(&eye, &tgt);
    const QVector3D fwd = (tgt - eye).normalized();
    QVector3D o, d;
    rayThroughPixel(px, &o, &d);
    const float denom = QVector3D::dotProduct(d, fwd);
    const float tt = (std::fabs(denom) > 1e-6f)
                         ? QVector3D::dotProduct(startWorld_ - o, fwd) / denom
                         : 0.0f;
    QVector3D delta = (o + d * tt) - planeHit0_;
    if (axisLock_ >= 0) {
      const QVector3D axisB(axisLock_ == 0 ? 1.f : 0.f,
                            axisLock_ == 1 ? 1.f : 0.f,
                            axisLock_ == 2 ? 1.f : 0.f);
      const QVector3D axisW = model.mapVector(axisB).normalized();
      delta = axisW * QVector3D::dotProduct(delta, axisW);
    }
    motorPos_[selected_] = markerModel().inverted().map(startWorld_ + delta);
  } else { // Rotate the thrust axis about a body axis (default X).
    const int k = (axisLock_ >= 0) ? axisLock_ : 0;
    const QVector3D kk(k == 0 ? 1.f : 0.f, k == 1 ? 1.f : 0.f,
                       k == 2 ? 1.f : 0.f);
    const float ang = (px.x() - toolStartMouse_.x()) * 0.01f; // rad
    const QVector3D a = startAxis_;
    const QVector3D r =
        a * std::cos(ang) + QVector3D::crossProduct(kk, a) * std::sin(ang) +
        kk * QVector3D::dotProduct(kk, a) * (1.0f - std::cos(ang));
    motorAxis_[selected_] = r.normalized();
  }
  update();
}

void SimRendererWidget::commitTool(bool confirm) {
  if (tool_ == Tool::None)
    return;
  tool_ = Tool::None;
  axisLock_ = -1;
  if (confirm && selected_ >= 0) {
    emit motorEdited(selected_, motorPos_[selected_], motorAxis_[selected_]);
  } else if (selected_ >= 0) { // cancelled: restore the working copy
    motorPos_[selected_] = startPos_;
    motorAxis_[selected_] = startAxis_;
  }
  update();
}

void SimRendererWidget::drawGizmo(const QMatrix4x4 &view) {
  if (selected_ < 0)
    return;
  QMatrix4x4 base = markerModel();
  base.translate(motorPos_[selected_]);
  const float L = 0.18f;
  const QVector3D cX(0.90f, 0.25f, 0.25f), cY(0.25f, 0.85f, 0.25f),
      cZ(0.30f, 0.55f, 1.0f), hi(1.0f, 0.95f, 0.40f);

  // Gizmo draws on top of the body for grabbability.
  glDisable(GL_DEPTH_TEST);
  if (tool_ == Tool::Rotate) {
    {
      QMatrix4x4 m = base;
      m.scale(L);
      drawMesh(gizmoRing_, view * m, axisLock_ == 2 ? hi : cZ);
    }
    {
      QMatrix4x4 m = base;
      m.rotate(90, 1, 0, 0);
      m.scale(L);
      drawMesh(gizmoRing_, view * m, axisLock_ == 1 ? hi : cY);
    }
    {
      QMatrix4x4 m = base;
      m.rotate(90, 0, 1, 0);
      m.scale(L);
      drawMesh(gizmoRing_, view * m, axisLock_ == 0 ? hi : cX);
    }
  } else { // Move (or just-selected): three axis arrows.
    {
      QMatrix4x4 m = base;
      m.scale(L);
      drawMesh(gizmoArrow_, view * m, axisLock_ == 0 ? hi : cX);
    }
    {
      QMatrix4x4 m = base;
      m.rotate(90, 0, 0, 1);
      m.scale(L);
      drawMesh(gizmoArrow_, view * m, axisLock_ == 1 ? hi : cY);
    }
    {
      QMatrix4x4 m = base;
      m.rotate(-90, 0, 1, 0);
      m.scale(L);
      drawMesh(gizmoArrow_, view * m, axisLock_ == 2 ? hi : cZ);
    }
  }
  glEnable(GL_DEPTH_TEST);
}

// ---------- obstacle gizmo editing ----------

bool SimRendererWidget::pickObstacle(const QPoint &px, int *outIndex) const {
  const QMatrix4x4 vp = proj_ * cameraView();
  const float w = std::max(1, width()), h = std::max(1, height());
  float best = 40.0f; // pixel radius (obstacles are larger than motors)
  int bestI = -1;
  for (int i = 0; i < obstacles_.size(); ++i) {
    const QVector4D clip = vp * QVector4D(obstacles_[i].pos, 1.0f);
    if (clip.w() <= 0.0f)
      continue;
    const float sx = (clip.x() / clip.w() * 0.5f + 0.5f) * w;
    const float sy = (1.0f - (clip.y() / clip.w() * 0.5f + 0.5f)) * h;
    const float dpix = std::hypot(sx - px.x(), sy - px.y());
    if (dpix < best) {
      best = dpix;
      bestI = i;
    }
  }
  if (bestI >= 0 && outIndex)
    *outIndex = bestI;
  return bestI >= 0;
}

void SimRendererWidget::beginObsTool(Tool t) {
  if (selObs_ < 0 || selObs_ >= obstacles_.size())
    return;
  obsTool_ = t;
  obsAxis_ = -1;
  obsStartPos_ = obstacles_[selObs_].pos;
  obsStartSize_ = obstacles_[selObs_].size;
  obsStartRot_ = obstacles_[selObs_].rotate;
  obsStartMouse_ = last_mouse_ = mapFromGlobal(QCursor::pos());
  // Drag plane: through the obstacle, facing the camera (world frame).
  QVector3D eye, tgt;
  cameraEyeTarget(&eye, &tgt);
  const QVector3D fwd = (tgt - eye).normalized();
  QVector3D o, d;
  rayThroughPixel(obsStartMouse_, &o, &d);
  const float denom = QVector3D::dotProduct(d, fwd);
  const float tt = (std::fabs(denom) > 1e-6f)
                       ? QVector3D::dotProduct(obsStartPos_ - o, fwd) / denom
                       : 0.0f;
  obsPlaneHit0_ = o + d * tt;
  update();
}

void SimRendererWidget::updateObsToolFromMouse(const QPoint &px) {
  if (selObs_ < 0 || obsTool_ == Tool::None)
    return;
  last_mouse_ = px;
  vsim::Obstacle &ob = obstacles_[selObs_];

  if (obsTool_ == Tool::Move) {
    QVector3D eye, tgt;
    cameraEyeTarget(&eye, &tgt);
    const QVector3D fwd = (tgt - eye).normalized();
    QVector3D o, d;
    rayThroughPixel(px, &o, &d);
    const float denom = QVector3D::dotProduct(d, fwd);
    const float tt = (std::fabs(denom) > 1e-6f)
                         ? QVector3D::dotProduct(obsStartPos_ - o, fwd) / denom
                         : 0.0f;
    QVector3D delta = (o + d * tt) - obsPlaneHit0_;
    if (obsAxis_ >= 0) {
      QVector3D ax(obsAxis_ == 0 ? 1.f : 0.f, obsAxis_ == 1 ? 1.f : 0.f,
                   obsAxis_ == 2 ? 1.f : 0.f);
      delta = ax * QVector3D::dotProduct(delta, ax);
    }
    ob.pos = obsStartPos_ + delta;
  } else if (obsTool_ == Tool::Rotate) {
    const int k = (obsAxis_ >= 0) ? obsAxis_ : 2; // default about Z
    const float deg = (px.x() - obsStartMouse_.x()) * 0.5f;
    ob.rotate = obsStartRot_;
    ob.rotate[k] = obsStartRot_[k] + deg;
  } else { // Scale
    const float f =
        std::clamp(1.0f + (px.x() - obsStartMouse_.x()) * 0.01f, 0.05f, 20.0f);
    if (obsAxis_ >= 0) {
      ob.size = obsStartSize_;
      ob.size[obsAxis_] = std::max(0.05f, obsStartSize_[obsAxis_] * f);
    } else {
      ob.size = obsStartSize_ * f;
      for (int i = 0; i < 3; ++i)
        ob.size[i] = std::max(0.05f, ob.size[i]);
    }
  }
  update();
}

void SimRendererWidget::commitObsTool(bool confirm) {
  if (obsTool_ == Tool::None)
    return;
  obsTool_ = Tool::None;
  obsAxis_ = -1;
  if (selObs_ >= 0 && selObs_ < obstacles_.size()) {
    if (confirm) {
      const vsim::Obstacle &o = obstacles_[selObs_];
      emit obstacleEdited(selObs_, o.pos, o.size, o.rotate);
    } else {
      obstacles_[selObs_].pos = obsStartPos_;
      obstacles_[selObs_].size = obsStartSize_;
      obstacles_[selObs_].rotate = obsStartRot_;
    }
  }
  update();
}

void SimRendererWidget::drawObsGizmo(const QMatrix4x4 &view) {
  if (selObs_ < 0 || selObs_ >= obstacles_.size())
    return;
  QMatrix4x4 base;
  base.translate(obstacles_[selObs_].pos);
  const float L = 0.7f;
  const QVector3D cX(0.90f, 0.25f, 0.25f), cY(0.25f, 0.85f, 0.25f),
      cZ(0.30f, 0.55f, 1.0f), hi(1.0f, 0.95f, 0.40f);
  glDisable(GL_DEPTH_TEST);
  if (obsTool_ == Tool::Rotate) {
    {
      QMatrix4x4 m = base;
      m.scale(L);
      drawMesh(gizmoRing_, view * m, obsAxis_ == 2 ? hi : cZ);
    }
    {
      QMatrix4x4 m = base;
      m.rotate(90, 1, 0, 0);
      m.scale(L);
      drawMesh(gizmoRing_, view * m, obsAxis_ == 1 ? hi : cY);
    }
    {
      QMatrix4x4 m = base;
      m.rotate(90, 0, 1, 0);
      m.scale(L);
      drawMesh(gizmoRing_, view * m, obsAxis_ == 0 ? hi : cX);
    }
  } else { // Move / Scale / just-selected: three axis arrows.
    {
      QMatrix4x4 m = base;
      m.scale(L);
      drawMesh(gizmoArrow_, view * m, obsAxis_ == 0 ? hi : cX);
    }
    {
      QMatrix4x4 m = base;
      m.rotate(90, 0, 0, 1);
      m.scale(L);
      drawMesh(gizmoArrow_, view * m, obsAxis_ == 1 ? hi : cY);
    }
    {
      QMatrix4x4 m = base;
      m.rotate(-90, 0, 1, 0);
      m.scale(L);
      drawMesh(gizmoArrow_, view * m, obsAxis_ == 2 ? hi : cZ);
    }
  }
  glEnable(GL_DEPTH_TEST);
}

void SimRendererWidget::wheelEvent(QWheelEvent *e) {
  if (freeFly_ && !fpv_) {
    // Free-roam: scroll dollies the camera along its view direction.
    const float d = (e->angleDelta().y() > 0) ? 1.0f : -1.0f;
    camPos_ += freeForward() * d;
    update();
    return;
  }
  const float k = (e->angleDelta().y() > 0) ? 0.9f : 1.1f;
  userZoomed_ = true;
  cam_radius_ = std::clamp(cam_radius_ * k, camMinRadius_, camMaxRadius_);
  update();
}

} // namespace vsim
