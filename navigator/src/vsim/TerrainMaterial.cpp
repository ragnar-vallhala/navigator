#include "TerrainTextures.h"

#include <QImage>
#include <QOpenGLContext>
#include <QOpenGLExtraFunctions>
#include <QString>

#include <cstdio>

namespace vsim::terrain {

namespace {

// Texture units of the terrain material: 0 is the program's own, 1 the shadow
// map, 2..7 are ours (3 diffuse, then 3 normal).
constexpr int kFirstUnit = 2;
constexpr int kTexCount = 6;
constexpr GLenum kMaxAnisotropyExt = 0x84FE;      // GL_TEXTURE_MAX_ANISOTROPY(_EXT)
constexpr GLenum kMaxAnisotropyLimitExt = 0x84FF; // GL_MAX_TEXTURE_MAX_ANISOTROPY(_EXT)

// grass, rock, dirt; diffuse then normal map. Poly Haven, CC0, see
// resources/terrain/LICENSES.md.
constexpr const char *kFiles[kTexCount] = {
    ":/terrain/grass_path_2_diff_1k.jpg",   ":/terrain/rocky_trail_02_diff_1k.jpg",
    ":/terrain/brown_mud_03_diff_1k.jpg",   ":/terrain/grass_path_2_nor_gl_1k.jpg",
    ":/terrain/rocky_trail_02_nor_gl_1k.jpg", ":/terrain/brown_mud_03_nor_gl_1k.jpg"};

// The anti-tiling and splat GLSL. %1..%5 are the splat constants.
const char *kGlslTemplate = R"GLSL(
// ---- Terrain material (TerrainMaterial.cpp) --------------------------------
uniform sampler2D u_t0;   // grass, rock, dirt diffuse
uniform sampler2D u_t1;
uniform sampler2D u_t2;
uniform sampler2D u_n0;   // grass, rock, dirt normal (GL convention)
uniform sampler2D u_n1;
uniform sampler2D u_n2;
uniform float u_matq;     // 1 = textured terrain (High), 0 = the plain look

float tmHash(vec2 p) { p = fract(p * vec2(123.34, 345.45)); p += dot(p, p + 34.345); return fract(p.x * p.y); }
float tmNoise(vec2 p) {
  vec2 i = floor(p), f = fract(p); f = f * f * (3.0 - 2.0 * f);
  return mix(mix(tmHash(i), tmHash(i + vec2(1, 0)), f.x),
             mix(tmHash(i + vec2(0, 1)), tmHash(i + vec2(1, 1)), f.x), f.y);
}
vec2 tmH22(vec2 p) {
  p = vec2(dot(p, vec2(127.1, 311.7)), dot(p, vec2(269.5, 183.3)));
  return fract(sin(p) * 43758.5453);
}
// Anti-tiling: the plane is cut into cells, each with its own random rotation
// and offset into the texture, and the four cells around a pixel are blended
// with smooth weights. Gradients come from the unrotated coordinates, so mips
// and anisotropic filtering stay right across cell borders.
vec3 tmStoch(sampler2D t, vec2 p, vec2 dx, vec2 dy, bool isNormal) {
  vec2 i = floor(p), f = fract(p);
  vec3 acc = vec3(0.0); float ws = 0.0, w2 = 0.0;
  for (int b = 0; b < 2; ++b)
    for (int a = 0; a < 2; ++a) {
      vec2 h = tmH22(i + vec2(a, b));
      float ang = h.x * 6.2831853, cs = cos(ang), sn = sin(ang);
      mat2 R = mat2(cs, sn, -sn, cs);
      vec2 d = abs(f - vec2(a, b));
      vec2 wv = 1.0 - d;
      float w = wv.x * wv.y; w = w * w * (3.0 - 2.0 * w);
      vec3 s = textureGrad(t, R * p + h * 7.0, R * dx, R * dy).rgb;
      if (isNormal) { // rotate the tilt back with the cell
        vec2 xy = (s.xy - 0.5) * 2.0;
        xy = vec2(cs * xy.x + sn * xy.y, -sn * xy.x + cs * xy.y);
        s = vec3(xy * 0.5 + 0.5, s.z);
      }
      acc += w * s; ws += w; w2 += w * w;
    }
  // The blend loses contrast; restore it around the texture's TRUE mean (the
  // 1x1 mip). A 0.5 pivot darkens every cell border on a dark texture and
  // draws the cell grid as a checkerboard.
  vec3 mean = textureLod(t, vec2(0.5), 10.0).rgb;
  if (isNormal) mean = vec3(0.5, 0.5, mean.z);
  vec3 m = acc / ws;
  return clamp((m - mean) / max(sqrt(w2) / ws, 0.5) + mean, 0.0, 1.0);
}
// Two scales per texture: ~3 m near, ~21 m far, crossfaded between 25 and
// 140 m, so neither the near repeat nor the far one reads. Only the scales
// that contribute are sampled.
vec3 tmTex(sampler2D t, vec2 p, vec2 dx, vec2 dy, float dist) {
  float k = smoothstep(25.0, 140.0, dist);
  vec3 a = k < 0.999 ? tmStoch(t, p / 3.2, dx / 3.2, dy / 3.2, false) : vec3(0.0);
  vec3 b = k > 0.001 ? tmStoch(t, p / 21.0, dx / 21.0, dy / 21.0, false) : vec3(0.0);
  return mix(a, b, k);
}
vec2 tmNrm(sampler2D t, vec2 p, vec2 dx, vec2 dy, float dist) {
  float k = smoothstep(25.0, 140.0, dist), fade = 1.0 - smoothstep(30.0, 150.0, dist);
  if (fade <= 0.001) return vec2(0.0);
  vec2 a = k < 0.999 ? tmStoch(t, p / 3.2, dx / 3.2, dy / 3.2, true).xy - 0.5 : vec2(0.0);
  vec2 b = k > 0.001 ? tmStoch(t, p / 21.0, dx / 21.0, dy / 21.0, true).xy - 0.5 : vec2(0.0);
  return mix(a, b * 0.6, k) * 2.0 * fade;
}
float tmLuma(vec3 c) { return dot(c, vec3(0.30, 0.59, 0.11)); }

// Surface albedo and normal of the ground at world xy p, `dist` from the eye.
// vcol is the mesh's cell colour (GroundCover); base and n are updated.
void terrainMaterial(vec3 vcol, vec2 p, float dist, inout vec3 base, inout vec3 n) {
  float slope = 1.0 - clamp(-n.z, 0.0, 1.0);
  vec2 dx = dFdx(p), dy = dFdy(p); // once, here: not inside a per-pixel branch
  // Splat rule: must match splatWeights() in TerrainMaterial.h.
  float rockW = smoothstep(%1, %2, slope + %3 * (tmNoise(p / 31.0) - 0.5));
  float dirtW = smoothstep(%4, %5, vcol.r - vcol.g * %6) * (1.0 - rockW);
  // Snow (bright, unsaturated cover) keeps the cover's hue like grass does: the
  // photos are not snow, and the rock photo is orange. Same rule as snowWeight().
  float cmax = max(vcol.r, max(vcol.g, vcol.b)), cmin = min(vcol.r, min(vcol.g, vcol.b));
  float snowW = smoothstep(%7, %8, tmLuma(vcol)) *
                (1.0 - smoothstep(%9, %10, (cmax - cmin) / max(cmax, 1e-3)));
  // The grass photo also carries the snow's brightness detail, so it is sampled
  // wherever there is snow, even on slopes that are otherwise all rock.
  bool useG = (rockW < 0.999 && dirtW < 0.999) || snowW > 0.001;
  vec3 a0 = useG ? tmTex(u_t0, p, dx, dy, dist) : vec3(0.0);
  vec3 a1 = rockW > 0.001 ? tmTex(u_t1, p, dx, dy, dist) : vec3(0.0);
  vec3 a2 = dirtW > 0.001 ? tmTex(u_t2, p, dx, dy, dist) : vec3(0.0);
  // Grass keeps the cover's hue (green fields, brown tracks) and takes only
  // the photo's brightness detail, relative to the photo's own mean so the
  // overall level stays the cover's. Rock and dirt show their own colour.
  float m0 = max(tmLuma(textureLod(u_t0, vec2(0.5), 10.0).rgb), 0.02);
  float detail = clamp(tmLuma(a0) / m0, 0.35, 1.9);
  vec3 cover = mix(vec3(tmLuma(vcol)), vcol, 0.8);
  vec3 grass = cover * detail;
  float lvl = clamp(tmLuma(vcol) / 0.38, 0.55, 1.5);
  vec3 own = mix(a0, a2 * 1.25, dirtW);
  own = mix(own, a1 * 1.15, rockW) * lvl;
  own = mix(vec3(tmLuma(own)), own, 0.7); // the rock photo is orange: calm it
  float own_k = max(rockW * 0.85, dirtW * 0.55) * (1.0 - snowW);
  // (when own_k is 0 the grass term is the whole albedo: a0 must exist)
  vec3 alb = mix(grass, own, own_k);
  // Macro variation, 50 to 200 m: brightness and a warm/cool drift, so large
  // fields never read as one repeated swatch.
  float mA = tmNoise(p / 190.0 + 17.3) - 0.5, mB = tmNoise(p / 67.0 + 41.7) - 0.5;
  alb *= (1.0 + 0.55 * mA + 0.35 * mB) * vec3(1.0 + 0.20 * mA, 1.0 + 0.05 * mB, 1.0 - 0.22 * mA);
  base = alb;
  vec2 n0 = useG ? tmNrm(u_n0, p, dx, dy, dist) : vec2(0.0);
  vec2 n1 = rockW > 0.001 ? tmNrm(u_n1, p, dx, dy, dist) : vec2(0.0);
  vec2 n2 = dirtW > 0.001 ? tmNrm(u_n2, p, dx, dy, dist) : vec2(0.0);
  vec2 nm = mix(mix(n0, n2, dirtW), n1, rockW);
  n = normalize(n + vec3(nm.x, nm.y, 0.0) * 1.1);
}
)GLSL";

} // namespace

QByteArray terrainMaterialGLSL() {
  QString s = QString::fromLatin1(kGlslTemplate);
  auto f = [](float v) { return QString::number(static_cast<double>(v), 'f', 4); };
  // Highest index first: "%1" is a prefix of "%10".
  s.replace(QStringLiteral("%10"), f(kSnowSat1))
      .replace(QStringLiteral("%9"), f(kSnowSat0))
      .replace(QStringLiteral("%8"), f(kSnowLuma1))
      .replace(QStringLiteral("%7"), f(kSnowLuma0))
      .replace(QStringLiteral("%6"), f(kDirtG))
      .replace(QStringLiteral("%5"), f(kDirtEdge1))
      .replace(QStringLiteral("%4"), f(kDirtEdge0))
      .replace(QStringLiteral("%3"), f(kRockNoise))
      .replace(QStringLiteral("%2"), f(kRockSlope1))
      .replace(QStringLiteral("%1"), f(kRockSlope0));
  return s.toLatin1();
}

TerrainTextures::~TerrainTextures() = default;

bool TerrainTextures::load(QOpenGLExtraFunctions *gl) {
  if (loaded())
    return true;
  GLuint t[kTexCount] = {};
  // Anisotropic filtering is an extension (core only from GL 4.6, absent on
  // some GLES): ask for it only when the context has it, up to 8x.
  float aniso = 0.0f;
  if (const QOpenGLContext *ctx = QOpenGLContext::currentContext();
      ctx && (ctx->hasExtension("GL_EXT_texture_filter_anisotropic") ||
              ctx->hasExtension("GL_ARB_texture_filter_anisotropic"))) {
    GLfloat maxAniso = 1.0f;
    gl->glGetFloatv(kMaxAnisotropyLimitExt, &maxAniso);
    aniso = std::min(8.0f, maxAniso);
  }
  for (int i = 0; i < kTexCount; ++i) {
    QImage img(QString::fromLatin1(kFiles[i]));
    if (img.isNull()) {
      qWarning("[TerrainMaterial] missing resource %s", kFiles[i]);
      gl->glDeleteTextures(i, t);
      return false;
    }
    img = img.convertToFormat(QImage::Format_RGBA8888);
    gl->glGenTextures(1, &t[i]);
    gl->glBindTexture(GL_TEXTURE_2D, t[i]);
    gl->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, img.width(), img.height(), 0,
                     GL_RGBA, GL_UNSIGNED_BYTE, img.constBits());
    gl->glGenerateMipmap(GL_TEXTURE_2D);
    gl->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    gl->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    gl->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    gl->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    if (aniso > 1.0f)
      gl->glTexParameterf(GL_TEXTURE_2D, kMaxAnisotropyExt, aniso);
  }
  gl->glBindTexture(GL_TEXTURE_2D, 0);
  for (int i = 0; i < kTexCount; ++i)
    tex_[i] = t[i];
  return true;
}

void TerrainTextures::release(QOpenGLExtraFunctions *gl, bool contextCurrent) {
  if (contextCurrent && loaded())
    gl->glDeleteTextures(kTexCount, tex_);
  for (GLuint &t : tex_)
    t = 0;
}

void TerrainTextures::bind(QOpenGLExtraFunctions *gl, QOpenGLShaderProgram *prog) const {
  static const char *kNames[kTexCount] = {"u_t0", "u_t1", "u_t2", "u_n0", "u_n1", "u_n2"};
  for (int i = 0; i < kTexCount; ++i) {
    gl->glActiveTexture(GL_TEXTURE0 + kFirstUnit + i);
    gl->glBindTexture(GL_TEXTURE_2D, tex_[i]);
    prog->setUniformValue(kNames[i], kFirstUnit + i);
  }
  gl->glActiveTexture(GL_TEXTURE0);
}

} // namespace vsim::terrain
