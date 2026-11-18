// Cascaded shadow maps for the simulator's sun.
//
// Layout: one depth ATLAS, the cascades side by side (count * size wide, size
// tall). An atlas keeps a single sampler2DShadow for the terrain and the grass
// and needs nothing newer than GL 3.3 / GLES 3.0 (a texture array would also
// work there, but the shared sampler and the single FBO attachment are simpler
// as an atlas). Memory: High 3 x 2048 x 2048 x 4 B = 48 MiB, Low 1 x 1024 x 1024
// x 4 B = 4 MiB (32-bit float depth).
//
// Fitting: each cascade is the bounding SPHERE of its slice of the camera
// frustum, so its size depends only on the split distances, field of view and
// aspect, never on where the camera points. The light-space window is then
// snapped to whole shadow texels, so the shadow map samples the world on the
// same grid whatever the camera does: no shimmer when it moves or turns.
//
// Pure maths (no GL) in fitShadowCascades(); the shader side is kCascadeShadowGLSL.
#pragma once

#include <QMatrix4x4>
#include <QVector2D>
#include <QVector3D>
#include <QVector4D>

#include <algorithm>
#include <cmath>

namespace vsim {

enum class ShadowQuality {
  Low,  // one 1024 map over kLowShadowRangeM, same filter
  High, // three 2048 cascades to kCascadeSplitsM[2], blended seams
};

inline constexpr int kMaxCascades = 3;
// Far edge (view depth, m) of each cascade. Roughly a log/uniform blend: the
// first cascade resolves the surroundings at ~5 cm/texel, the last still gives
// distant slopes a shadow at ~1 m/texel.
inline constexpr float kCascadeSplitsM[kMaxCascades] = {40.0f, 200.0f, 800.0f};
inline constexpr float kLowShadowRangeM = 150.0f;
inline constexpr int kShadowSizeHigh = 2048;
inline constexpr int kShadowSizeLow = 1024;
// Seam blend: the last fraction of a cascade's range cross-fades into the next.
inline constexpr float kCascadeBlend = 0.12f;
// Border kept free of data inside every tile so the PCF taps never read a
// neighbouring tile.
inline constexpr float kShadowMarginTexels = 8.0f;
// Casters this far toward the sun beyond the fitted sphere are still drawn (a
// ridge up-sun of the view at a low sun). With depth clamp they are pinned to
// the near plane, without it (GLES) they are clipped past this distance.
inline constexpr float kShadowCasterM = 600.0f;

struct ShadowCascadeSet {
  int count = 0;                    // 0 = no shadow map this frame
  int tileSize = 0;                 // texels per cascade side
  QMatrix4x4 vp[kMaxCascades];      // world -> light clip (orthographic, w = 1)
  QVector4D tile[kMaxCascades];     // atlas (u0, v0, du, dv) of each tile
  QVector4D info[kMaxCascades];     // (texel size m, light depth range m, 0, 0)
  QVector4D splits;                 // far view depth of each cascade; w = count
  QVector3D camFwd{0, 0, 1};        // camera forward (view depth = dot(p-eye, fwd))
  int atlasW() const { return tileSize * std::max(count, 1); }
  int atlasH() const { return tileSize; }
};

// Fit `count` cascades (1 = the Low path) for a camera with the given
// camera-to-world matrix (view space looks down -Z), vertical fov in degrees and
// aspect, lit from sunDir (unit vector toward the sun, world). `snap` = false
// only exists so a test can show what the snapping removes.
inline ShadowCascadeSet fitShadowCascades(const QVector3D &sunDir,
                                          const QMatrix4x4 &camToWorld,
                                          float fovYDeg, float aspect, int count,
                                          int tileSize, bool snap = true) {
  ShadowCascadeSet s;
  s.count = std::clamp(count, 1, kMaxCascades);
  s.tileSize = tileSize;
  const QVector3D eye = camToWorld.map(QVector3D(0, 0, 0));
  s.camFwd = camToWorld.mapVector(QVector3D(0, 0, -1)).normalized();

  const QVector3D fwdL = -sunDir.normalized(); // light travel direction
  const QVector3D up =
      std::abs(fwdL.z()) > 0.99f ? QVector3D(1, 0, 0) : QVector3D(0, 0, -1);
  QMatrix4x4 lrot; // world -> light view, rotation only (eye at the origin)
  lrot.lookAt(QVector3D(0, 0, 0), fwdL, up);

  const float th = std::tan(fovYDeg * 0.5f * 0.0174532925f);
  const float k2 = th * th * (aspect * aspect + 1.0f); // corner radius^2 / depth^2
  float far[kMaxCascades];
  if (s.count == 1) {
    far[0] = kLowShadowRangeM;
  } else {
    for (int i = 0; i < s.count; ++i)
      far[i] = kCascadeSplitsM[i];
  }
  for (int i = 0; i < kMaxCascades; ++i)
    s.splits[i] = i < s.count ? far[i] : far[s.count - 1];
  s.splits[3] = float(s.count);

  for (int i = 0; i < s.count; ++i) {
    // The slice starts where the previous cascade's blend band starts.
    float n = 0.0f;
    if (i > 0) {
      const float p2 = i > 1 ? far[i - 2] : 0.0f;
      n = far[i - 1] - kCascadeBlend * (far[i - 1] - p2);
    }
    const float f = far[i];
    // Bounding sphere of the slice, centred on the view axis at depth c.
    float c = 0.5f * (n + f) * (1.0f + k2);
    c = std::min(c, f);
    const float r = std::sqrt((f - c) * (f - c) + f * f * k2);
    // Window half-extent with the PCF margin; texel size follows from it.
    const float T = float(tileSize);
    const float h = r * T / (T - 2.0f * kShadowMarginTexels);
    const float texel = 2.0f * h / T;

    QVector3D cl = lrot.map(eye + s.camFwd * c);
    if (snap) {
      cl.setX(std::floor(cl.x() / texel) * texel);
      cl.setY(std::floor(cl.y() / texel) * texel);
    }
    const float depthC = -cl.z();
    const float zn = depthC - h - kShadowCasterM, zf = depthC + h;
    QMatrix4x4 proj;
    proj.ortho(cl.x() - h, cl.x() + h, cl.y() - h, cl.y() + h, zn, zf);
    s.vp[i] = proj * lrot;
    const float cnt = float(s.count);
    s.tile[i] = QVector4D(float(i) / cnt, 0.0f, 1.0f / cnt, 1.0f);
    s.info[i] = QVector4D(texel, zf - zn, 0.0f, 0.0f);
  }
  return s;
}

// Uniform locations of kCascadeShadowGLSL in one program (templated on the
// program type so the fitting maths above needs no OpenGL module).
struct CascadeUniforms {
  int vp = -1, tile = -1, info = -1, split = -1, texel = -1, camfwd = -1,
      on = -1, tex = -1;
  template <class Prog> // QOpenGLShaderProgram
  void resolve(Prog *p) {
    vp = p->uniformLocation("u_csmvp");
    tile = p->uniformLocation("u_csmtile");
    info = p->uniformLocation("u_csminfo");
    split = p->uniformLocation("u_csmsplit");
    texel = p->uniformLocation("u_csmtexel");
    camfwd = p->uniformLocation("u_camfwd");
    on = p->uniformLocation("u_shadowon");
    tex = p->uniformLocation("u_shadowtex");
  }
  // The program must be bound. The shadow atlas goes on texture unit 1.
  template <class Prog>
  void upload(Prog *p, const ShadowCascadeSet &s, bool on_) const {
    p->setUniformValue(on, on_ && s.count > 0 ? 1.0f : 0.0f);
    p->setUniformValue(tex, 1);
    if (s.count <= 0)
      return;
    p->setUniformValueArray(vp, s.vp, kMaxCascades);
    p->setUniformValueArray(tile, s.tile, kMaxCascades);
    p->setUniformValueArray(info, s.info, kMaxCascades);
    p->setUniformValue(split, s.splits);
    p->setUniformValue(texel, QVector2D(1.0f / float(s.atlasW()),
                                        1.0f / float(s.atlasH())));
    p->setUniformValue(camfwd, s.camFwd);
  }
};

// Shader side. Needs u_campos (the eye) in the including program. The atlas is
// a depth texture with hardware compare + linear filtering, so every tap is
// already a bilinear 2x2 PCF; four taps give a smooth ~3 texel penumbra whose
// width follows the cascade's texel size.
//
//   csmVisibility(wpos, ng, L, usePlane)
//     wpos   world position of the fragment
//     ng     unit surface normal facing the viewer (offsets the lookup)
//     L      unit direction toward the sun
//     usePlane  receiver-plane depth bias from screen derivatives (terrain);
//               off for thin geometry (grass blades), which uses a bigger constant
//   returns 1 = lit .. 0 = shadowed; 1 beyond the last cascade (faded in).
inline constexpr const char *kCascadeShadowGLSL = R"GLSL(
uniform sampler2DShadow u_shadowtex;
uniform float u_shadowon;
uniform mat4 u_csmvp[3];
uniform vec4 u_csmtile[3];
uniform vec4 u_csminfo[3];  // x texel size [m], y light depth range [m]
uniform vec4 u_csmsplit;    // far view depth of each cascade, w = count
uniform vec2 u_csmtexel;    // 1 / atlas size
uniform vec3 u_camfwd;
const float kCsmBlend = 0.12;
// Bias, in shadow texels of the cascade it is used in (so it scales per cascade).
const float kCsmBiasConst = 4.7;    // flat part
const float kCsmBiasGrazing = 11.0; // extra toward a grazing sun
const float kCsmBiasThin = 14.0;    // flat bias for thin geometry (no plane term)

float csmTap(int ci, vec3 wpos, vec3 ng, vec3 L, vec3 dWx, vec3 dWy,
             bool usePlane) {
  float texelM = u_csminfo[ci].x;
  float range = u_csminfo[ci].y;
  float ndg = clamp(dot(ng, L), 0.0, 1.0);
  float sinG = sqrt(max(1.0 - ndg * ndg, 0.0));
  // Facet-normal offset: a few texels along the true surface normal, more at
  // grazing sun, so the lookup starts clear of the caster.
  vec3 wp = wpos + ng * (0.03 + texelM * sinG);
  mat4 M = u_csmvp[ci];
  vec3 p = (M * vec4(wp, 1.0)).xyz * 0.5 + 0.5;
  float bias;
  vec2 dz = vec2(0.0);
  if (usePlane) {
    // Receiver-plane depth: how the fragment's own light depth changes per
    // shadow texel in u and v, taken from the WORLD-space derivatives so it
    // stays valid across a cascade seam. Each tap is compared against the
    // depth the surface has at that tap, not at the centre.
    vec3 dpx = 0.5 * (mat3(M) * dWx), dpy = 0.5 * (mat3(M) * dWy);
    float det = dpx.x * dpy.y - dpx.y * dpy.x;
    if (abs(det) > 1e-14)
      dz = vec2(dpx.z * dpy.y - dpx.y * dpy.z, dpx.x * dpy.z - dpx.z * dpy.x) / det;
    dz *= u_csmtexel.y; // per texel step: 1 / tile size in tile uv
    float lim = 60.0 * texelM / range;               // slope <= 60
    dz = clamp(dz, vec2(-lim), vec2(lim));
    bias = (kCsmBiasConst + kCsmBiasGrazing * (1.0 - ndg)) * texelM / range;
  } else {
    bias = kCsmBiasThin * texelM / range;
  }
  if (p.x <= 0.0 || p.x >= 1.0 || p.y <= 0.0 || p.y >= 1.0) return 1.0;
  vec2 uv0 = u_csmtile[ci].xy + p.xy * u_csmtile[ci].zw;
  // 2x2 taps half a texel off-centre; each is a bilinear compare, so together
  // they weigh a 3x3 texel block (1-2-1 tent) for a quarter of the fetches.
  float s = 0.0;
  for (int dx = 0; dx < 2; ++dx)
    for (int dy = 0; dy < 2; ++dy) {
      vec2 o = vec2(float(dx) - 0.5, float(dy) - 0.5);
      s += texture(u_shadowtex, vec3(uv0 + o * u_csmtexel, p.z - bias + dot(dz, o)));
    }
  return s * 0.25;
}

float csmVisibility(vec3 wpos, vec3 ng, vec3 L, bool usePlane) {
  // Derivatives first, in uniform control flow.
  vec3 dWx = dFdx(wpos), dWy = dFdy(wpos);
  if (u_shadowon < 0.5) return 1.0;
  float z = dot(wpos - u_campos, u_camfwd);
  int n = int(u_csmsplit.w + 0.5);
  float last = u_csmsplit[n - 1];
  if (z >= last) return 1.0;
  int ci = 0;
  if (n > 1 && z > u_csmsplit.x) ci = 1;
  if (n > 2 && z > u_csmsplit.y) ci = 2;
  float s = csmTap(ci, wpos, ng, L, dWx, dWy, usePlane);
  if (ci + 1 < n) {
    float far = u_csmsplit[ci];
    float near = ci > 0 ? u_csmsplit[ci - 1] : 0.0;
    float band = kCsmBlend * (far - near);
    if (z > far - band)
      s = mix(s, csmTap(ci + 1, wpos, ng, L, dWx, dWy, usePlane),
              smoothstep(far - band, far, z));
  }
  return mix(s, 1.0, smoothstep(0.9 * last, last, z));
}
)GLSL";

} // namespace vsim
