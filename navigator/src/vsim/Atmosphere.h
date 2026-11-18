// Atmosphere: the single source of truth for the simulator's sky and sun look.
// Header-only and Qt-free (<cmath> only) so the terrain, grass, vegetation and
// post-fx code can all read the same numbers. The renderer feeds the values to
// GLSL through forEachUniform(); kAtmosphereGLSL is the matching shader side
// (append it after the program's #version line and `uniform` declarations).
//
// Frame convention: world is NED (x north, y east, z DOWN), so "up" is -Z.
// The sun is described by two angles:
//   elevation  degrees above the horizon, 0 = on the horizon, 90 = overhead
//   azimuth    degrees clockwise from north (towards east), the compass bearing
//              of the sun, i.e. the direction you look to see it
// sunDirection() is the unit vector FROM the ground TOWARD the sun:
//   (cos(el) cos(az), cos(el) sin(az), -sin(el))
// Light travels along -sunDirection(); the shadow pass and the sky disc both
// use this one vector, so the disc always sits where the shadows point away
// from.
//
// The defaults reproduce the look the renderer had before this header existed:
// sun direction (0.55, 0.42, -0.72) (elevation 46.1, azimuth 37.4), the same
// sky gradient, warm key colour and ambient. At low sun the "twilight" amount
// (0 above ~35 deg, rising to 1 at ~8 deg) warms the horizon and sun, dims the
// zenith and ambient; it is exactly 0 at the default elevation.
#pragma once

#include <algorithm>
#include <cmath>

namespace vsim {

struct Rgb {
  float r = 0.0f, g = 0.0f, b = 0.0f;
};
struct Vec3f {
  float x = 0.0f, y = 0.0f, z = 0.0f;
};

// Fog constants shared by every lit shader (distance fog + height fog). The fog
// colour is the sky's colour at the horizon, never a separate constant.
inline constexpr float kFogStartM = 45.0f;       // fog begins this far from the eye
inline constexpr float kFogDensity = 1.0f / 420.0f; // optical depth per metre
inline constexpr float kFogHeightScaleM = 150.0f;   // height-fog scale height

struct Atmosphere {
  // ---- Sun ----
  float sunElevationDeg = 46.1f; // set by defaults(); see setSunDirection()
  float sunAzimuthDeg = 37.4f;
  Rgb sunColor{1.04f, 0.93f, 0.74f}; // warm golden key at the default elevation
  float sunIntensity = 1.0f;         // scales the light colour
  float sunDiscRadiusRad = 0.028f;   // visible disc (real sun is 0.0047; exaggerated)
  // ---- Sky (display-referred, no tonemap) ----
  Rgb skyZenith{0.28f, 0.35f, 0.38f};
  Rgb skyHorizon{0.52f, 0.57f, 0.58f};
  Rgb skyGround{0.34f, 0.38f, 0.38f}; // below-horizon haze, not black
  Rgb glowColor{0.30f, 0.30f, 0.26f}; // sun-side aerosol glow (wide lobe)
  float mieStrength = 1.0f;           // narrow forward-scatter lobe around the sun
  float cloudCover = 1.0f;            // 0 clear .. 1 the original dense overcast
  // ---- Ambient (hemispheric) ----
  Rgb ambientSky{0.22f, 0.25f, 0.26f};
  Rgb ambientGround{0.07f, 0.09f, 0.09f};
  float ambientStrength = 1.0f;
  // ---- Fog (see the constants above) ----
  float fogStartM = kFogStartM;
  float fogDensity = kFogDensity;
  float fogHeightM = kFogHeightScaleM;

  // The look the renderer had before this header (see the file comment).
  static Atmosphere defaults() {
    Atmosphere a;
    a.setSunDirection({0.55f, 0.42f, -0.72f});
    return a;
  }

  static constexpr float kMinElevationDeg = 1.0f;  // keeps the shadow pass sane
  static constexpr float kMaxElevationDeg = 89.0f;

  void setSun(float elevationDeg, float azimuthDeg) {
    sunElevationDeg = std::clamp(elevationDeg, kMinElevationDeg, kMaxElevationDeg);
    sunAzimuthDeg = std::fmod(azimuthDeg, 360.0f);
    if (sunAzimuthDeg < 0.0f)
      sunAzimuthDeg += 360.0f;
  }
  // Direction toward the sun in NED (need not be normalised).
  void setSunDirection(Vec3f d) {
    const float len = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    if (len < 1e-6f)
      return;
    const float kDeg = 57.29577951f;
    setSun(std::asin(std::clamp(-d.z / len, -1.0f, 1.0f)) * kDeg,
           std::atan2(d.y, d.x) * kDeg);
  }
  // Unit vector toward the sun, world NED.
  Vec3f sunDirection() const {
    const float k = 0.01745329252f;
    const float el = sunElevationDeg * k, az = sunAzimuthDeg * k;
    return {std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), -std::sin(el)};
  }

  // 0 with the sun high, 1 near the horizon: drives the warm low-sun tint.
  float twilight() const {
    const float t = std::clamp((sunElevationDeg - 8.0f) / (35.0f - 8.0f), 0.0f, 1.0f);
    return 1.0f - t * t * (3.0f - 2.0f * t);
  }

  // Colours after the low-sun adjustment: what the shaders actually receive.
  struct Resolved {
    Rgb sunLight, zenith, horizon, ground, ambientSky, ambientGround, twilightGlow;
  };
  Resolved resolved() const {
    const float t = twilight();
    auto mix = [](Rgb a, Rgb b, float k) {
      return Rgb{a.r + (b.r - a.r) * k, a.g + (b.g - a.g) * k, a.b + (b.b - a.b) * k};
    };
    auto scale = [](Rgb a, float k) { return Rgb{a.r * k, a.g * k, a.b * k}; };
    Resolved r;
    r.sunLight = scale(mix(sunColor, {1.10f, 0.58f, 0.28f}, 0.50f * t),
                       sunIntensity * (1.0f - 0.20f * t));
    r.zenith = scale(skyZenith, 1.0f - 0.35f * t);
    r.horizon = mix(skyHorizon, {0.66f, 0.55f, 0.48f}, 0.55f * t);
    r.ground = mix(skyGround, {0.40f, 0.36f, 0.34f}, 0.40f * t);
    r.ambientSky = scale(ambientSky, (1.0f - 0.30f * t) * ambientStrength);
    r.ambientGround = scale(ambientGround, (1.0f - 0.30f * t) * ambientStrength);
    r.twilightGlow = scale({0.34f, 0.17f, 0.05f}, t); // warm band round the sun
    return r;
  }

  // Colour the fog fades to with the sun well away from the view direction:
  // the horizon. (In the shader fogColor(dir) adds the sun-side glow, so fog and
  // sky agree at every azimuth.)
  Rgb fogColor() const { return resolved().horizon; }

  // Calls f(name, x, y, z) for vec3 uniforms and f(name, x) for floats. The
  // renderer forwards these to setUniformValue on each program that includes
  // kAtmosphereGLSL (sky, terrain lit, flora). u_sundir is the unit vector.
  template <class F> void forEachUniform(F &&f) const {
    const Resolved r = resolved();
    const Vec3f s = sunDirection();
    f("u_sundir", s.x, s.y, s.z);
    f("u_suncol", r.sunLight.r, r.sunLight.g, r.sunLight.b);
    f("u_skyzenith", r.zenith.r, r.zenith.g, r.zenith.b);
    f("u_skyhorizon", r.horizon.r, r.horizon.g, r.horizon.b);
    f("u_skyground", r.ground.r, r.ground.g, r.ground.b);
    f("u_glowcol", glowColor.r, glowColor.g, glowColor.b);
    f("u_twilight", r.twilightGlow.r, r.twilightGlow.g, r.twilightGlow.b);
    f("u_ambsky", r.ambientSky.r, r.ambientSky.g, r.ambientSky.b);
    f("u_ambgnd", r.ambientGround.r, r.ambientGround.g, r.ambientGround.b);
    f("u_sunradius", sunDiscRadiusRad);
    f("u_mie", mieStrength);
    f("u_cloudcover", cloudCover);
  }
};

// Shader side. Appended after the #version / in / out / uniform declarations of
// each fragment program (sky, terrain lit, flora). Exposes:
//   uniforms   u_sundir u_suncol u_skyzenith u_skyhorizon u_skyground u_glowcol
//              u_twilight u_ambsky u_ambgnd u_sunradius u_mie u_cloudcover
//   vec3 skyColor(vec3 dir)   sky radiance along a world (NED) direction,
//                             gradient + sun-side glow; NO sun disc, so fog and
//                             aerial perspective can sample it freely
//   vec3 fogColor(vec3 dir)   skyColor on the horizontal projection of dir: the
//                             colour distance fog fades to
//   vec3 sunDisc(vec3 dir)    additive sun disc core (sky shader only); the
//                             halo/bloom is part of skyColor
//   vec3 ambientLight(vec3 n) hemispheric ambient for a world normal
//   vec3 shadowFill()         sky-tinted fill that keeps shadows off black
// Fog distance constants (kFogStartM, kFogDensity, kFogHeightScaleM) are C++
// side; the programs keep their existing u_fogstart / u_fogdensity uniforms.
inline constexpr const char *kAtmosphereGLSL = R"GLSL(
uniform vec3 u_sundir;     // unit direction toward the sun (world, NED)
uniform vec3 u_suncol;     // sun light colour (low-sun tint + intensity applied)
uniform vec3 u_skyzenith;
uniform vec3 u_skyhorizon;
uniform vec3 u_skyground;
uniform vec3 u_glowcol;    // sun-side aerosol glow colour
uniform vec3 u_twilight;   // warm low-sun glow around the sun (0 at high sun)
uniform vec3 u_ambsky;
uniform vec3 u_ambgnd;
uniform float u_sunradius; // disc angular radius [rad]
uniform float u_mie;       // forward-scatter lobe strength
uniform float u_cloudcover;
vec3 skyColor(vec3 dir) {
  vec3 d = normalize(dir);
  float up = -d.z;          // NED up is -Z
  vec3 col = (up >= 0.0)
      ? mix(u_skyhorizon, u_skyzenith, pow(clamp(up, 0.0, 1.0), 0.55))
      : mix(u_skyhorizon, u_skyground, smoothstep(0.0, 1.0, clamp(-up * 1.5, 0.0, 1.0)));
  float s = max(dot(d, normalize(u_sundir)), 0.0);
  // Soft, not a step: the glow must not cut off at up == 0, or the sky just
  // below the horizon is darker than the horizon colour the fog fades to (a
  // visible seam where the far terrain ends).
  float aboveHorizon = smoothstep(-0.10, 0.0, up);
  // Wide glow (original look), a narrow Mie lobe hugging the sun, and the warm
  // Rayleigh-style low-sun band that hugs the horizon.
  col += u_glowcol * pow(s, 5.0) * aboveHorizon;
  col += u_glowcol * (0.9 * u_mie) * pow(s, 40.0) * aboveHorizon;
  // Tight bloom around the disc lives here (not in sunDisc) so the fog carries
  // it: a low sun's glow at the horizon must match the far terrain's haze.
  col += vec3(0.55) * pow(s, 220.0) * aboveHorizon;
  col += u_twilight * pow(s, 2.0) * exp(-max(up, 0.0) * 5.0) * aboveHorizon;
  return col;
}
vec3 fogColor(vec3 dir) {
  float hl = length(dir.xy);
  return skyColor(hl > 1e-4 ? vec3(dir.xy / hl, 0.0) : vec3(1.0, 0.0, 0.0));
}
vec3 sunDisc(vec3 dir) {
  vec3 d = normalize(dir);
  float c = dot(d, normalize(u_sundir));
  float r = u_sunradius;
  float cr = cos(r), ci = cos(r * 0.55);
  float core = smoothstep(cr, ci, c);                   // soft-edged disc
  float vis = smoothstep(-0.02, 0.02, -d.z);            // hidden below the horizon
  vec3 hot = mix(u_suncol, vec3(1.0), 0.45);
  return hot * (core * 3.0) * vis;
}
vec3 ambientLight(vec3 n) {
  float hemi = clamp(0.5 + 0.5 * (-n.z), 0.0, 1.0);     // 0 down .. 1 up
  return mix(u_ambgnd, u_ambsky, hemi);
}
// Fill light for surfaces the sun does not reach (cast shadow or turned away).
// The caller scales it by (1 - sun term), so lit surfaces are unchanged. Tuned
// so ambient + fill is about half of a fully sunlit up-facing surface (default
// sun: ambient ~0.25 + fill ~0.24 against ~0.95); the 0.5 below is the knob
// (0.35-0.5 of lit reads as shade, lower goes back toward black). It follows the
// ambient sky and the sun colour, so it dims and warms with the low-sun tint.
vec3 shadowFill() {
  return (u_ambsky + 0.25 * u_suncol) * 0.5;
}
)GLSL";

} // namespace vsim
