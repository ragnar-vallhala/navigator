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
#include "WaterRenderer.h"

#include <QOpenGLContext>
#include <QOpenGLShaderProgram>

#include <algorithm>
#include <cmath>

namespace vsim {

void appendWaterBody(WaterMesh &m, const procgen::WaterBody &b) {
  const uint32_t base = uint32_t(m.vertexCount());
  // The rim (q > 1) sits a hair BELOW the level so the ground, which is at or
  // above the level out there, always covers it: the shoreline is where the
  // ground crosses the water, with no z-fighting along the waterline.
  const float z = -b.level;
  const float zRim = z + 0.06f;
  // Centre vertex, then kWaterRings rings of kWaterSegments vertices. Ring r is
  // at q = r / (kWaterRings - 1) * kWaterRimScale so the last ring sits just past
  // the shore, where the depth is 0 and the ground is above the level.
  m.vertices.insert(m.vertices.end(), {b.cx, b.cy, z, b.depthM, 0.0f, b.radiusM});
  for (int r = 1; r <= kWaterRings; ++r) {
    const float q = kWaterRimScale * float(r) / float(kWaterRings);
    for (int s = 0; s < kWaterSegments; ++s) {
      const float a = 6.2831853f * float(s) / float(kWaterSegments);
      const float R = b.shoreRadius(a);
      m.vertices.insert(m.vertices.end(),
                        {b.cx + std::cos(a) * R * q, b.cy + std::sin(a) * R * q,
                         q > 1.0f ? zRim : z, waterDepthAtQ(b, q), q,
                         (1.0f - q) * R});
    }
  }
  auto idx = [&](int ring, int seg) {
    return base + 1u + uint32_t((ring - 1) * kWaterSegments + (seg % kWaterSegments));
  };
  for (int s = 0; s < kWaterSegments; ++s)
    m.indices.insert(m.indices.end(), {base, idx(1, s), idx(1, s + 1)});
  for (int r = 1; r < kWaterRings; ++r)
    for (int s = 0; s < kWaterSegments; ++s) {
      const uint32_t a = idx(r, s), b2 = idx(r, s + 1), c = idx(r + 1, s),
                     d = idx(r + 1, s + 1);
      m.indices.insert(m.indices.end(), {a, c, b2, b2, c, d});
    }
}

namespace {

const char *kWaterVS = R"GLSL(#version 330 core
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec3 a_dq; // depth, q, metres to the shore
uniform mat4 u_vp;
uniform float u_time;
uniform float u_wind;
out vec3 v_pos;
out vec3 v_dq;
// Directional wave set shared with the fragment shader's normal: five sines,
// wavelength [m], heading offset from the wind [rad], amplitude per metre of
// wavelength. Up is -Z (NED), so the displacement is subtracted from z.
const int NW = 5;
const float WL[NW] = float[](7.0, 3.6, 2.1, 1.25, 0.75);
const float WH[NW] = float[](0.0, 0.55, -0.7, 1.3, -1.9);
const float WA[NW] = float[](0.013, 0.011, 0.009, 0.007, 0.005);
float waveH(vec2 p, float t, float wind) {
  float h = 0.0;
  for (int i = 0; i < NW; ++i) {
    float k = 6.2831853 / WL[i];
    vec2 d = vec2(cos(WH[i]), sin(WH[i]));
    h += WA[i] * WL[i] * wind * sin(k * dot(d, p) - sqrt(9.81 * k) * t);
  }
  return h;
}
void main() {
  // Waves die toward the shore so the waterline stays where the ground crosses
  // the level, and in shallow water.
  float calm = smoothstep(0.0, 0.7, a_dq.x);
  float h = waveH(a_pos.xy, u_time, u_wind) * calm;
  v_pos = vec3(a_pos.xy, a_pos.z - h);
  v_dq = a_dq;
  gl_Position = u_vp * vec4(v_pos, 1.0);
}
)GLSL";

const char *kWaterFSHead = R"GLSL(#version 330 core
in vec3 v_pos;
in vec3 v_dq;
out vec4 o_color;
uniform float u_time;
uniform float u_wind;
uniform vec3 u_wcol;
uniform sampler2D u_rip;   // prop-wash ripple heights [m], R16F
uniform vec4 u_ripwin;     // window origin xy, size [m]; w = 1 when live
uniform mat4 u_vp;
uniform mat4 u_invvp;
uniform sampler2D u_scene;  // opaque scene so far (resolved copy)
uniform sampler2D u_sdepth; // its depth
uniform vec4 u_ssr;         // x = on, y = near, z = far, w = max march [m]
uniform vec2 u_vsize;       // copy size [px]
)GLSL";

const char *kWaterFSMain = R"GLSL(
const int NW = 5;
const float WL[NW] = float[](7.0, 3.6, 2.1, 1.25, 0.75);
const float WH[NW] = float[](0.0, 0.55, -0.7, 1.3, -1.9);
const float WA[NW] = float[](0.013, 0.011, 0.009, 0.007, 0.005);
// A wave this short against the pixel footprint (metres per pixel) would alias
// into rings, so it fades out as its wavelength nears a few pixels.
float waveFade(float wavelength, float footprint) {
  return 1.0 - smoothstep(0.12, 0.34, footprint / wavelength);
}
// Gradient of the height field (d/dx, d/dy) of the vertex shader's waves, plus a
// finer ripple layer that only the shading sees.
vec2 waveGrad(vec2 p, float t, float wind, float fp) {
  vec2 g = vec2(0.0);
  for (int i = 0; i < NW; ++i) {
    float k = 6.2831853 / WL[i];
    vec2 d = vec2(cos(WH[i]), sin(WH[i]));
    g += d * (WA[i] * WL[i] * wind * k * waveFade(WL[i], fp)) *
         cos(k * dot(d, p) - sqrt(9.81 * k) * t);
  }
  for (int i = 0; i < 3; ++i) {
    float a = 1.7 + 2.1 * float(i), wl = 0.32 + 0.11 * float(i), k = 6.2831853 / wl;
    vec2 d = vec2(cos(a), sin(a));
    g += d * (0.0035 * wind * k * waveFade(wl, fp)) *
         cos(k * dot(d, p) - sqrt(9.81 * k) * t * 1.3 + 3.0 * float(i));
  }
  return g;
}
// Ray-marches the reflected ray through the resolved scene depth. Returns the
// scene colour it meets in rgb and a confidence in a (0 = nothing found: the
// caller keeps the sky). Ground under the water is ignored (a ray above the
// surface cannot hit it), hits must pass a thickness test that grows with the
// step, and the result fades out toward the screen edge and with distance.
float ssrLinear(float d) {
  float n = u_ssr.y, f = u_ssr.z;
  return 2.0 * n * f / (f + n - (2.0 * d - 1.0) * (f - n));
}
vec4 ssrTrace(vec3 P, vec3 R, float jitter, vec2 slope) {
  float t = 0.35 + 0.5 * jitter, dt = 0.5 + 0.4 * jitter, prevT = 0.0;
  for (int i = 0; i < 22; ++i) {
    if (t > u_ssr.w) break;
    vec4 c = u_vp * vec4(P + R * t, 1.0);
    if (c.w < u_ssr.y) break;
    vec3 nd = c.xyz / c.w;
    if (abs(nd.x) > 1.0 || abs(nd.y) > 1.0 || nd.z > 1.0) break;
    ivec2 ip = clamp(ivec2((nd.xy * 0.5 + 0.5) * u_vsize), ivec2(0), ivec2(u_vsize) - 1);
    float d = texelFetch(u_sdepth, ip, 0).r;
    if (d < 0.99999) {
      float diff = c.w - ssrLinear(d);
      if (diff > 0.0 && diff < 0.3 + 1.4 * dt) {
        // Crossed the surface between prevT and t: bisect to the hit.
        float lo = prevT, hi = t;
        for (int k = 0; k < 4; ++k) {
          float mid = 0.5 * (lo + hi);
          vec4 cm = u_vp * vec4(P + R * mid, 1.0);
          ivec2 im = clamp(ivec2((cm.xy / cm.w * 0.5 + 0.5) * u_vsize), ivec2(0), ivec2(u_vsize) - 1);
          if (cm.w > ssrLinear(texelFetch(u_sdepth, im, 0).r)) hi = mid; else lo = mid;
        }
        vec4 ch = u_vp * vec4(P + R * hi, 1.0);
        vec2 uvh = ch.xy / ch.w * 0.5 + 0.5;
        ivec2 ih = clamp(ivec2(uvh * u_vsize), ivec2(0), ivec2(u_vsize) - 1);
        float dh = texelFetch(u_sdepth, ih, 0).r;
        vec4 wp = u_invvp * vec4(uvh * 2.0 - 1.0, dh * 2.0 - 1.0, 1.0);
        wp /= wp.w;
        // World is NED: a smaller z is higher. The lake bed (below the surface)
        // must not reflect, but the bank at the waterline sits within a few cm of
        // the level and the waves move P by about as much, so allow 12 cm of slack.
        if (dh < 0.99999 && wp.z < P.z + 0.12) {
          vec2 e = abs(uvh * 2.0 - 1.0);
          float conf = (1.0 - smoothstep(0.80, 1.0, max(e.x, e.y))) *
                       (1.0 - smoothstep(0.5 * u_ssr.w, u_ssr.w, hi));
          // The ray was traced on a flat mirror (a coherent edge); the waves
          // only bend where the colour is taken from: the hit point moves with
          // the surface slope times its distance, as a rippled mirror smears.
          vec4 cd = u_vp * vec4(P + R * hi + vec3(slope * (0.9 * min(hi, 40.0)), 0.0), 1.0);
          if (cd.w > u_ssr.y) {
            vec2 ud = cd.xy / cd.w * 0.5 + 0.5;
            ih = clamp(ivec2(ud * u_vsize), ivec2(0), ivec2(u_vsize) - 1);
          }
          return vec4(texelFetch(u_scene, ih, 0).rgb, conf);
        }
      }
    }
    prevT = t;
    t += dt;
    dt *= 1.27;
  }
  return vec4(0.0);
}
float wHash(vec2 p) { p = fract(p * vec2(123.34, 345.45)); p += dot(p, p + 34.345); return fract(p.x * p.y); }
float wNoise(vec2 p) {
  vec2 i = floor(p), f = fract(p); f = f * f * (3.0 - 2.0 * f);
  return mix(mix(wHash(i), wHash(i + vec2(1, 0)), f.x), mix(wHash(i + vec2(0, 1)), wHash(i + vec2(1, 1)), f.x), f.y);
}
void main() {
  vec3 toFrag = v_pos - u_campos;
  float dist = length(toFrag);
  vec3 V = -toFrag / max(dist, 1e-4);          // toward the eye
  float depth = max(v_dq.x, 0.0);
  float calm = smoothstep(0.0, 0.7, depth);
  // World metres per pixel, from screen derivatives (taken outside any branch).
  float fp = max(length(dFdx(v_pos.xy)), length(dFdy(v_pos.xy)));
  vec2 g = waveGrad(v_pos.xy, u_time, u_wind, fp) * calm;
  // Surface up is -Z: n = (-h_x, -h_y, -1).
  float ripFoam = 0.0;
  if (u_ripwin.w > 0.5) {
    vec2 uv = (v_pos.xy - u_ripwin.xy) / u_ripwin.z;
    float n = float(textureSize(u_rip, 0).x);
    float e = 1.5 / n, cell = 1.5 * u_ripwin.z / n; // 3-texel stencil: smoother than C0 bilinear
    // Slope of the ripple field by central differences, fading out toward the
    // window edge and as a texel shrinks below a pixel (no shimmer far away).
    float inWin = (1.0 - smoothstep(0.42, 0.5, abs(uv.x - 0.5))) *
                  (1.0 - smoothstep(0.42, 0.5, abs(uv.y - 0.5)));
    float lod = 1.0 - smoothstep(0.5, 1.2, fp / cell);
    vec2 rg = vec2(textureLod(u_rip, uv + vec2(e, 0.0), 0.0).r - textureLod(u_rip, uv - vec2(e, 0.0), 0.0).r,
                   textureLod(u_rip, uv + vec2(0.0, e), 0.0).r - textureLod(u_rip, uv - vec2(0.0, e), 0.0).r)
             / (2.0 * cell);
    rg *= 4.0 * inWin * lod * calm;      // shading gain: centimetre waves must read
    g += rg;
    ripFoam = smoothstep(0.12, 0.30, length(rg)) * 0.35;
  }
  vec3 N = normalize(vec3(-g.x, -g.y, -1.0));
  if (dot(N, V) < 0.0) N = reflect(N, V);       // never face away from the eye

  // Fresnel (Schlick, water F0 = 0.02) between the reflected sky and the body.
  float cosv = clamp(dot(N, V), 0.0, 1.0);
  float F = 0.02 + 0.98 * pow(1.0 - cosv, 5.0);
  vec3 R = reflect(-V, N);
  R.z = min(R.z, -0.03);                        // the sky, never the ground
  vec3 refl = skyColor(R);
  // Screen-space reflection of the terrain and trees. Skipped where the Fresnel
  // weight is negligible (looking steeply down), faded in smoothly above that.
  if (u_ssr.x > 0.5 && F > 0.035) {
    float jit = fract(52.9829189 * fract(dot(gl_FragCoord.xy, vec2(0.06711056, 0.00583715))));
    // Trace the flat mirror; the wave slope (g) distorts the lookup afterwards.
    // (Tracing the wavy normal bends a grazing ray by about as much as the
    // strip a bank occupies, so hits would come and go as ragged islands.)
    vec3 Rt = reflect(-V, vec3(0.0, 0.0, -1.0));
    Rt.z = min(Rt.z, -0.03);
    vec4 s = ssrTrace(v_pos, Rt, jit, g);
    refl = mix(refl, s.rgb * 0.92, s.a * smoothstep(0.035, 0.07, F));
  }

  // Body: absorption tints with depth from a lit shallow colour to a dark deep
  // one (the palette's pond colour), lit by sky ambient and a little sun scatter.
  vec3 deepCol = u_wcol * 0.62;
  vec3 shallowCol = mix(u_wcol, vec3(0.32, 0.44, 0.30), 0.5) * 1.45;
  float absorb = 1.0 - exp(-depth * 0.7);
  vec3 body = mix(shallowCol, deepCol, absorb);
  float sunUp = clamp(-u_sundir.z, 0.0, 1.0);
  body *= ambientLight(vec3(0.0, 0.0, -1.0)) * u_ambstr * 2.4 + sunLightColor() * (0.28 * sunUp);

  // Sun glint on the wave normal.
  vec3 H = normalize(normalize(u_sundir) + V);
  float spec = pow(max(dot(N, H), 0.0), 220.0);
  vec3 glint = sunLightColor() * spec * 1.4 * smoothstep(0.0, 0.08, sunUp);

  // Shore foam: flecks along the waterline where the water gets shallow,
  // broken up by two layers of drifting noise (none on the wave crests).
  // Width in metres (not depth), so a shallow pond gets the same thin band as a
  // deep one: foam only within ~1.6 m of the waterline.
  float edge = 1.0 - smoothstep(0.0, 1.6, v_dq.z);
  // Each noise octave fades to its mean once its cells are about a pixel wide.
  float n1 = mix(wNoise(v_pos.xy * 5.2 + vec2(u_time * 0.35, -u_time * 0.22)), 0.5,
                 smoothstep(0.25, 0.7, fp * 5.2));
  float n2 = mix(wNoise(v_pos.xy * 13.0 - vec2(u_time * 0.5, u_time * 0.3)), 0.5,
                 smoothstep(0.25, 0.7, fp * 13.0));
  // Sparse patches (not a continuous ring): threshold the noise so the foam
  // breaks into drifting flecks along the waterline.
  // Wide threshold ramp: flecks feather out instead of ending on a hard edge;
  // they thin toward the bank (the water is a film there) and at grazing
  // angles, where a flat white sheet would sit on top of the surface.
  float foam = edge * smoothstep(0.46, 0.95, n1 * 0.6 + n2 * 0.5);
  foam *= smoothstep(0.0, 0.35, v_dq.z) * 0.5 + 0.5;
  foam *= mix(0.6, 1.0, smoothstep(0.06, 0.45, cosv));
  foam = clamp(foam + ripFoam, 0.0, 1.0);
  vec3 foamCol = (ambientLight(vec3(0.0, 0.0, -1.0)) * u_ambstr + sunLightColor() * (0.55 * sunUp)) * 0.95 + 0.12;

  vec3 col = mix(body, refl, F) + glint;
  col = mix(col, foamCol, foam * 0.75);

  // Soft edge: clear at the waterline, fully there a short way in.
  // The shallows are see-through: the bed shows for the first metre of depth
  // (not just the first half metre), so the bank reads as water thinning over
  // ground, not a painted edge. The body colour fades in with the same ramp.
  float veil = smoothstep(0.0, 1.1, depth);
  float alpha = clamp(max(veil * 0.92, F) + foam * 0.8, 0.0, 1.0);
  col = applyAerialPerspective(col, toFrag, dist);
  o_color = vec4(col, alpha);
}
)GLSL";

} // namespace

WaterRenderer::WaterRenderer() = default;

WaterRenderer::~WaterRenderer() {
  // GL objects must be released with the context current (releaseGL()); the
  // owner does that before destruction.
}

void WaterRenderer::setBodies(std::vector<procgen::WaterBody> bodies) {
  bodies_ = std::move(bodies);
  meshDirty_ = true;
}

bool WaterRenderer::initGL(const WaterGlsl &g) {
  if (prog_)
    return true;
  initializeOpenGLFunctions();
  auto p = std::make_unique<QOpenGLShaderProgram>();
  const QByteArray fs = QByteArray(kWaterFSHead) + g.atmosphere + g.lighting + kWaterFSMain;
  if (!p->addShaderFromSourceCode(QOpenGLShader::Vertex, kWaterVS) ||
      !p->addShaderFromSourceCode(QOpenGLShader::Fragment, fs) || !p->link()) {
    qWarning("WaterRenderer: shader build failed: %s", qPrintable(p->log()));
    return false;
  }
  prog_ = std::move(p);
  return true;
}

void WaterRenderer::uploadMesh() {
  WaterMesh m;
  for (const procgen::WaterBody &b : bodies_)
    appendWaterBody(m, b);
  if (!vao_) {
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glGenBuffers(1, &ibo_);
  }
  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(m.vertices.size() * sizeof(float)),
               m.vertices.data(), GL_STATIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(m.indices.size() * sizeof(uint32_t)),
               m.indices.data(), GL_STATIC_DRAW);
  const GLsizei stride = kWaterVertexFloats * sizeof(float);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, nullptr);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride,
                        reinterpret_cast<const void *>(3 * sizeof(float)));
  glBindVertexArray(0);
  indexCount_ = int(m.indices.size());
  lastVertices_ = m.vertexCount();
  meshDirty_ = false;
}

// Uploads the ripple window while it is moving (plus the one frame after it
// settles, so it ends flat) and points the shader at it; otherwise the shader
// branch is off and nothing is bound or uploaded.
void WaterRenderer::bindRipples(const WaterRipples *r) {
  constexpr int kUnit = 6;
  const bool on = r && r->bound() && (r->active() || ripWasOn_);
  ripWasOn_ = r && r->bound() && r->active();
  QOpenGLShaderProgram &p = *prog_;
  if (!on) {
    p.setUniformValue("u_ripwin", QVector4D(0, 0, 1, 0));
    p.setUniformValue("u_rip", kUnit);
    return;
  }
  GLint prevUnit = GL_TEXTURE0;
  glGetIntegerv(GL_ACTIVE_TEXTURE, &prevUnit);
  glActiveTexture(GL_TEXTURE0 + kUnit);
  const int n = WaterRipples::kN;
  r->toHalf(ripHalf_);
  if (!ripTex_) {
    glGenTextures(1, &ripTex_);
    glBindTexture(GL_TEXTURE_2D, ripTex_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R16F, n, n, 0, GL_RED, GL_HALF_FLOAT, ripHalf_.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  } else {
    glBindTexture(GL_TEXTURE_2D, ripTex_);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, n, n, GL_RED, GL_HALF_FLOAT, ripHalf_.data());
  }
  ++rippleUploads_;
  glActiveTexture(GLenum(prevUnit));
  p.setUniformValue("u_rip", kUnit);
  p.setUniformValue("u_ripwin", QVector4D(r->originX(), r->originY(), r->sizeM(), 1.0f));
}

// Copies the opaque scene drawn so far (colour + depth, resolving MSAA) into
// single-sample textures the shader can read. The destination depth format has
// to match the source's, so it is chosen from the source framebuffer's own depth
// and stencil sizes. Returns false (and disables the feature) on any GL error.
bool WaterRenderer::captureScene(unsigned srcFbo, int w, int h) {
  if (ssrBroken_ || w <= 0 || h <= 0)
    return false;
  glBindFramebuffer(GL_READ_FRAMEBUFFER, srcFbo);
  if (!sceneFbo_ || w != sceneW_ || h != sceneH_) {
    // Match the source's depth format (the default framebuffer names its
    // attachments GL_DEPTH / GL_STENCIL, an FBO GL_DEPTH_ATTACHMENT / ...).
    GLint ds = 0, ss = 0;
    glGetFramebufferAttachmentParameteriv(GL_READ_FRAMEBUFFER, srcFbo ? GL_DEPTH_ATTACHMENT : GL_DEPTH,
                                          GL_FRAMEBUFFER_ATTACHMENT_DEPTH_SIZE, &ds);
    glGetFramebufferAttachmentParameteriv(GL_READ_FRAMEBUFFER, srcFbo ? GL_STENCIL_ATTACHMENT : GL_STENCIL,
                                          GL_FRAMEBUFFER_ATTACHMENT_STENCIL_SIZE, &ss);
    GLenum fmt = ds >= 32 ? (ss > 0 ? GL_DEPTH32F_STENCIL8 : GL_DEPTH_COMPONENT32F)
                 : ds >= 24 ? (ss > 0 ? GL_DEPTH24_STENCIL8 : GL_DEPTH_COMPONENT24)
                            : GL_DEPTH_COMPONENT16;
    if (glGetError() != GL_NO_ERROR || ds <= 0) {
      glBindFramebuffer(GL_READ_FRAMEBUFFER, srcFbo);
      ssrBroken_ = true;
      return false;
    }
    freeScene();
    GLint prevTex = 0; // the allocation below must not leave a texture bound on the active unit
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &prevTex);
    sceneDepthFmt_ = fmt;
    const bool packed = fmt == GL_DEPTH24_STENCIL8 || fmt == GL_DEPTH32F_STENCIL8;
    glGenTextures(1, &sceneColor_);
    glBindTexture(GL_TEXTURE_2D, sceneColor_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glGenTextures(1, &sceneDepth_);
    glBindTexture(GL_TEXTURE_2D, sceneDepth_);
    glTexImage2D(GL_TEXTURE_2D, 0, GLint(fmt), w, h, 0, packed ? GL_DEPTH_STENCIL : GL_DEPTH_COMPONENT,
                 fmt == GL_DEPTH24_STENCIL8 ? GL_UNSIGNED_INT_24_8
                 : fmt == GL_DEPTH32F_STENCIL8 ? GL_FLOAT_32_UNSIGNED_INT_24_8_REV
                 : fmt == GL_DEPTH_COMPONENT32F ? GL_FLOAT
                 : fmt == GL_DEPTH_COMPONENT16 ? GL_UNSIGNED_SHORT : GL_UNSIGNED_INT,
                 nullptr);
    for (const unsigned t : {sceneColor_, sceneDepth_}) {
      glBindTexture(GL_TEXTURE_2D, t);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }
    glBindTexture(GL_TEXTURE_2D, GLuint(prevTex));
    glGenFramebuffers(1, &sceneFbo_);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, sceneFbo_);
    glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, sceneColor_, 0);
    glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, packed ? GL_DEPTH_STENCIL_ATTACHMENT : GL_DEPTH_ATTACHMENT,
                           GL_TEXTURE_2D, sceneDepth_, 0);
    sceneW_ = w;
    sceneH_ = h;
    if (glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE ||
        glGetError() != GL_NO_ERROR) {
      glBindFramebuffer(GL_FRAMEBUFFER, srcFbo);
      freeScene();
      ssrBroken_ = true;
      return false;
    }
  }
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, sceneFbo_);
  glBlitFramebuffer(0, 0, w, h, 0, 0, w, h, GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT, GL_NEAREST);
  const bool ok = glGetError() == GL_NO_ERROR;
  glBindFramebuffer(GL_FRAMEBUFFER, srcFbo); // back to where the frame is drawn
  if (!ok) {
    freeScene();
    ssrBroken_ = true;
    return false;
  }
  ++sceneCaptures_;
  return true;
}

void WaterRenderer::freeScene() {
  if (sceneFbo_)
    glDeleteFramebuffers(1, &sceneFbo_);
  if (sceneColor_)
    glDeleteTextures(1, &sceneColor_);
  if (sceneDepth_)
    glDeleteTextures(1, &sceneDepth_);
  sceneFbo_ = sceneColor_ = sceneDepth_ = 0;
  sceneW_ = sceneH_ = 0;
}

void WaterRenderer::draw(const WaterFrame &f) {
  if (bodies_.empty() || !prog_)
    return;
  if (meshDirty_)
    uploadMesh();
  if (indexCount_ <= 0)
    return;
  ssrActive_ = f.ssr && captureScene(f.sceneFbo, f.viewW, f.viewH);
  prog_->bind();
  QOpenGLShaderProgram &p = *prog_;
  f.atmo.forEachUniform([&p](const char *name, auto... v) {
    const float a[] = {static_cast<float>(v)...};
    if constexpr (sizeof...(v) == 3)
      p.setUniformValue(name, QVector3D(a[0], a[1], a[2]));
    else
      p.setUniformValue(name, a[0]);
  });
  p.setUniformValue("u_vp", f.proj * f.view);
  p.setUniformValue("u_campos", f.camEye);
  p.setUniformValue("u_sundir", f.sunDir);
  p.setUniformValue("u_sunint", f.sunIntensity);
  p.setUniformValue("u_ambstr", f.ambientStrength);
  p.setUniformValue("u_fogdensity", f.fogDensity);
  p.setUniformValue("u_fogstart", f.fogStart);
  p.setUniformValue("u_fogbase", f.fogBase);
  p.setUniformValue("u_fogheight", f.fogHeight);
  p.setUniformValue("u_time", f.time);
  p.setUniformValue("u_wind", f.wind);
  p.setUniformValue("u_wcol", f.waterColor);
  bindRipples(f.ripples);
  {
    constexpr int kSceneUnit = 5, kDepthUnit = 4;
    // Near / far from the projection: m22 = -(f+n)/(f-n), m23 = -2fn/(f-n).
    const float *m = f.proj.constData();
    const float n = m[14] / (m[10] - 1.0f), fr = m[14] / (m[10] + 1.0f);
    p.setUniformValue("u_ssr", QVector4D(ssrActive_ ? 1.0f : 0.0f, n, fr, 140.0f));
    p.setUniformValue("u_scene", kSceneUnit);
    p.setUniformValue("u_sdepth", kDepthUnit);
    p.setUniformValue("u_vsize", QVector2D(float(f.viewW), float(f.viewH)));
    p.setUniformValue("u_invvp", (f.proj * f.view).inverted());
    if (ssrActive_) {
      GLint prevUnit = GL_TEXTURE0;
      glGetIntegerv(GL_ACTIVE_TEXTURE, &prevUnit);
      glActiveTexture(GL_TEXTURE0 + kSceneUnit);
      glBindTexture(GL_TEXTURE_2D, sceneColor_);
      glActiveTexture(GL_TEXTURE0 + kDepthUnit);
      glBindTexture(GL_TEXTURE_2D, sceneDepth_);
      glActiveTexture(GLenum(prevUnit));
    }
  }

  GLboolean blend = glIsEnabled(GL_BLEND), cull = glIsEnabled(GL_CULL_FACE);
  GLboolean depthMask = GL_TRUE;
  glGetBooleanv(GL_DEPTH_WRITEMASK, &depthMask);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDisable(GL_CULL_FACE); // seen from below too, e.g. a low drone over a pond
  glDepthMask(GL_FALSE);
  glBindVertexArray(vao_);
  glDrawElements(GL_TRIANGLES, indexCount_, GL_UNSIGNED_INT, nullptr);
  glBindVertexArray(0);
  glDepthMask(depthMask);
  if (!blend)
    glDisable(GL_BLEND);
  if (cull)
    glEnable(GL_CULL_FACE);
  prog_->release();
}

void WaterRenderer::releaseGL() {
  if (vao_) {
    glDeleteVertexArrays(1, &vao_);
    glDeleteBuffers(1, &vbo_);
    glDeleteBuffers(1, &ibo_);
    vao_ = vbo_ = ibo_ = 0;
  }
  if (ripTex_) {
    glDeleteTextures(1, &ripTex_);
    ripTex_ = 0;
  }
  freeScene();
  ssrActive_ = false;
  ssrBroken_ = false; // a new context gets a fresh try
  ripWasOn_ = false;
  prog_.reset();
  indexCount_ = 0;
  meshDirty_ = true;
}

int WaterRenderer::liveObjects() const {
  return (prog_ ? 1 : 0) + (vao_ ? 3 : 0);
}

} // namespace vsim
