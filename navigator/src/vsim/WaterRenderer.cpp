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
  vec3 N = normalize(vec3(-g.x, -g.y, -1.0));
  if (dot(N, V) < 0.0) N = reflect(N, V);       // never face away from the eye

  // Fresnel (Schlick, water F0 = 0.02) between the reflected sky and the body.
  float cosv = clamp(dot(N, V), 0.0, 1.0);
  float F = 0.02 + 0.98 * pow(1.0 - cosv, 5.0);
  vec3 R = reflect(-V, N);
  R.z = min(R.z, -0.03);                        // the sky, never the ground
  vec3 refl = skyColor(R);

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
  float foam = edge * smoothstep(0.50, 0.82, n1 * 0.6 + n2 * 0.5);
  vec3 foamCol = (ambientLight(vec3(0.0, 0.0, -1.0)) * u_ambstr + sunLightColor() * (0.55 * sunUp)) * 0.95 + 0.12;

  vec3 col = mix(body, refl, F) + glint;
  col = mix(col, foamCol, foam * 0.75);

  // Soft edge: clear at the waterline, fully there a short way in.
  float alpha = clamp(max(smoothstep(0.0, 0.5, depth) * 0.9, F) + foam * 0.8, 0.0, 1.0);
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

void WaterRenderer::draw(const WaterFrame &f) {
  if (bodies_.empty() || !prog_)
    return;
  if (meshDirty_)
    uploadMesh();
  if (indexCount_ <= 0)
    return;
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
  prog_.reset();
  indexCount_ = 0;
  meshDirty_ = true;
}

int WaterRenderer::liveObjects() const {
  return (prog_ ? 1 : 0) + (vao_ ? 3 : 0);
}

} // namespace vsim
