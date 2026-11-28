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
// tst_gpu_grass.cpp - the GPU grass path keeps off the ground-cover layer's
// tracks, ponds and rock, like the CPU flora path (NAVIGATOR_SIM_GRASS=ON only).
//
// 1. mask data (headless): the tile the compute shader samples agrees with
//    TerrainField::grassFactor(), is 0 on tracks/water/rock and > 0 on meadow.
// 2. GL readback (needs a real GL platform and compute: QT_QPA_PLATFORM=xcb on
//    an Xvfb, LIBGL_ALWAYS_SOFTWARE=1; skips on offscreen): the blades the
//    compute pass really emits are read back and each is checked against
//    grassFactor() at its position.
#include "GpuGrass.h"
#include "procgen/WorldPresets.h"

#include <QGuiApplication>
#include <QMatrix4x4>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QOpenGLExtraFunctions>
#include <QSurfaceFormat>
#include <QtTest>

#include <cmath>
#include <vector>

using namespace vsim;
using procgen::FieldParams;
using procgen::TerrainField;

namespace {

FieldParams meadowField() {
  FieldParams f;
  f.seed = 1337u;
  f.palette = procgen::kPalMeadow;
  return f;
}

struct Spot {
  float x = 0, y = 0;
};

// Fraction of the (2r)^2 square around (cx, cy) where grass may not grow
// (sampled on a coarse integer-indexed lattice).
float zeroFraction(const TerrainField &f, float cx, float cy, float r) {
  const int n = 24;
  int zero = 0;
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i) {
      const float x = cx - r + 2 * r * (float(i) + 0.5f) / float(n);
      const float y = cy - r + 2 * r * (float(j) + 0.5f) / float(n);
      if (GpuGrass::maskAt(f, x, y, 0.3f) < 0.02f)
        ++zero;
    }
  return float(zero) / float(n * n);
}

// A spot whose surroundings are a mix of cover (zero fraction in [lo, hi]),
// found on a lattice around the origin.
bool findMixedSpot(const TerrainField &f, float r, float lo, float hi, Spot *s) {
  float best = 1e9f;
  bool found = false;
  for (int j = -12; j <= 12; ++j)
    for (int i = -12; i <= 12; ++i) {
      const float cx = 60.0f * float(i), cy = 60.0f * float(j);
      const float z = zeroFraction(f, cx, cy, r);
      if (z >= lo && z <= hi && std::fabs(z - 0.2f) < best) {
        best = std::fabs(z - 0.2f);
        *s = {cx, cy};
        found = true;
      }
    }
  return found;
}

} // namespace

class TstGpuGrass : public QObject {
  Q_OBJECT
private slots:
  void maskMatchesGrassFactor() {
    const FieldParams fp = meadowField();
    const TerrainField f(fp);
    const float minX = -200, minY = -200, size = 400;
    const int n = 256;
    const std::vector<uint8_t> m = GpuGrass::buildMask(fp, minX, minY, size, n);
    QCOMPARE(m.size(), size_t(n * n));
    const float tm = size / float(n);
    int zeros = 0, covered = 0;
    for (int j = 0; j < n; j += 3)
      for (int i = 0; i < n; i += 3) {
        const float wx = minX + (float(i) + 0.5f) * tm;
        const float wy = minY + (float(j) + 0.5f) * tm;
        const float ref = GpuGrass::maskAt(f, wx, wy, tm);
        const int got = m[size_t(j) * size_t(n) + size_t(i)];
        QVERIFY2(std::abs(got - int(std::lround(ref * 255.0f))) <= 1,
                 qPrintable(QString("texel %1,%2: mask %3, grassFactor %4")
                                .arg(i).arg(j).arg(got).arg(ref * 255.0f)));
        zeros += got == 0;
        covered += got > 0;
      }
    // The world has both grass-free ground (tracks / ponds / rock / bare) and
    // grassy meadow, otherwise this test proves nothing.
    QVERIFY2(zeros > 20, "no grass-free ground in the sampled window");
    QVERIFY2(covered > 20, "no grassy ground in the sampled window");
  }

  // The layer is consulted at its own positions: a point the CPU path leaves
  // bare is bare in the tile, a meadow point is not.
  void maskZeroOnCoverMeadowPositive() {
    const FieldParams fp = meadowField();
    const TerrainField f(fp);
    Spot bare, grassy;
    bool haveBare = false, haveGrassy = false;
    for (int j = -40; j < 40 && !(haveBare && haveGrassy); ++j)
      for (int i = -40; i < 40 && !(haveBare && haveGrassy); ++i) {
        const float x = 8.0f * float(i), y = 8.0f * float(j);
        const float g = GpuGrass::maskAt(f, x, y, 40.0f / 128.0f);
        if (g == 0.0f && !haveBare) {
          bare = {x, y};
          haveBare = true;
        } else if (g > 0.9f && !haveGrassy) {
          grassy = {x, y};
          haveGrassy = true;
        }
      }
    QVERIFY(haveBare);
    QVERIFY(haveGrassy);
    const float sz = 40.0f;
    const int n = 128;
    for (int k = 0; k < 2; ++k) {
      const Spot &s = k == 0 ? bare : grassy;
      // Tile placed so texel (n/2, n/2) is centred exactly on the spot.
      const int mid = n / 2;
      const float tm = sz / float(n), o = (float(mid) + 0.5f) * tm;
      const std::vector<uint8_t> m =
          GpuGrass::buildMask(fp, s.x - o, s.y - o, sz, n);
      const int c = m[size_t(mid) * size_t(n) + size_t(mid)];
      if (k == 0)
        QCOMPARE(c, 0);
      else
        QVERIFY(c > 200);
    }
  }

  // Every blade the compute pass emits sits on ground the CPU path would grow
  // grass on.
  void gpuBladesAvoidCover() {
    if (QGuiApplication::platformName() == "offscreen")
      QSKIP("needs a real GL platform: QT_QPA_PLATFORM=xcb on an Xvfb with "
            "LIBGL_ALWAYS_SOFTWARE=1");
    QSurfaceFormat fmt;
    fmt.setVersion(4, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    QOpenGLContext ctx;
    ctx.setFormat(fmt);
    if (!ctx.create())
      QSKIP("no GL 4.3 context");
    QOffscreenSurface surf;
    surf.setFormat(ctx.format());
    surf.create();
    QVERIFY(ctx.makeCurrent(&surf));
    QOpenGLExtraFunctions gl(&ctx);

    const FieldParams fp = meadowField();
    const TerrainField f(fp);
    const float radius = 25.0f; // the ortho view below covers +-radius
    Spot cam;
    QVERIFY2(findMixedSpot(f, radius, 0.08f, 0.6f, &cam),
             "no spot with a mix of cover and meadow");
    const float zf = zeroFraction(f, cam.x, cam.y, radius);
    qInfo("camera (%.0f, %.0f): %.1f%% of the view is grass-free ground",
          double(cam.x), double(cam.y), double(zf * 100.0f));

    GpuGrass::Params p;
    p.field = fp;
    p.seed = fp.seed;
    p.heightM = fp.heightM;
    p.featureM = fp.featureM;
    p.macroM = fp.macroM;
    p.octaves = fp.octaves;
    p.lacunarity = fp.lacunarity;
    p.gain = fp.gain;
    p.mountainMix = fp.mountainMix;
    p.cell = 0.2f; // ring 0 region 0.2 * 256 = 51 m
    p.grid = 256;
    p.asyncMask = false;
    p.slopeLo = 0.0f; // slope/altitude out of the way: this tests the cover mask
    p.slopeHi = 0.01f;
    p.grassMaxFrac = 100.0f;
    GpuGrass g;
    g.setParams(p);
    if (!g.init(&gl))
      QSKIP("compute shaders unavailable on this GL");

    QMatrix4x4 proj, view;
    view.translate(-cam.x, -cam.y); // look straight down at the camera spot
    proj.ortho(-radius, radius, -radius, radius, -1000.0f, 1000.0f);
    g.render(&gl, proj, view, QVector3D(cam.x, cam.y, 0.0f),
             QVector3D(0.3f, 0.3f, -1.0f), 0.0f);
    gl.glFinish();
    const std::vector<GpuGrass::Blade> blades = g.readBlades(&gl, 0);
    qInfo("blades read back: %zu", blades.size());
    QVERIFY2(blades.size() > 2000, "too few blades to say anything");

    // Texel size of ring 0's tile: 1.5 * region / 512.
    int onCover = 0;
    for (const GpuGrass::Blade &b : blades)
      if (GpuGrass::maskAt(f, b.x, b.y, 0.3f) < 0.02f)
        ++onCover;
    const double frac = double(onCover) / double(blades.size());
    qInfo("%zu blades in ring 0, %d on grass-free ground (%.2f%%)",
          blades.size(), onCover, frac * 100.0);
    // Bilinear sampling of an 8-bit tile lets a few blades past the very edge of
    // a track; without the mask it is the grass-free share of the view (>8%).
    QVERIFY2(frac < 0.01, "blades grow on tracks / ponds / rock");
    g.release(&gl);
    QCOMPARE(g.liveObjects(), 0);
    ctx.doneCurrent();
  }

  // The fill shader carries its own copy of ridged(). With ridgeSoft on, the
  // heights it bakes must follow the softened CPU field, not the sharp one.
  void gpuHeightFollowsRidgeSoft() {
    if (QGuiApplication::platformName() == "offscreen")
      QSKIP("needs a real GL platform: QT_QPA_PLATFORM=xcb on an Xvfb with "
            "LIBGL_ALWAYS_SOFTWARE=1");
    QSurfaceFormat fmt;
    fmt.setVersion(4, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    QOpenGLContext ctx;
    ctx.setFormat(fmt);
    if (!ctx.create())
      QSKIP("no GL 4.3 context");
    QOffscreenSurface surf;
    surf.setFormat(ctx.format());
    surf.create();
    QVERIFY(ctx.makeCurrent(&surf));
    QOpenGLExtraFunctions gl(&ctx);

    FieldParams fp;
    fp.seed = 5u;
    fp.palette = procgen::kPalMeadow; // grass grows here; heights ignore the palette
    fp.heightM = 110.0f;
    fp.featureM = 200.0f;
    fp.mountainMix = 0.85f;
    fp.ridgeSoft = 0.25f;
    FieldParams sharp = fp;
    sharp.ridgeSoft = 0.0f;
    const TerrainField soft(fp), hard(sharp);

    GpuGrass::Params p;
    p.field = fp;
    p.seed = fp.seed;
    p.heightM = fp.heightM;
    p.featureM = fp.featureM;
    p.macroM = fp.macroM;
    p.octaves = fp.octaves;
    p.lacunarity = fp.lacunarity;
    p.gain = fp.gain;
    p.mountainMix = fp.mountainMix;
    p.ridgeSoft = fp.ridgeSoft;
    p.cell = 0.2f;
    p.grid = 256;
    p.asyncMask = false;
    p.slopeLo = 0.0f;
    p.slopeHi = 0.01f; // slope gate out of the way: this tests the heights
    p.grassMaxFrac = 100.0f;
    GpuGrass g;
    g.setParams(p);
    if (!g.init(&gl))
      QSKIP("compute shaders unavailable on this GL");
    const float radius = 25.0f;
    // A mountain spot (the macro field leaves most places flat): where the
    // softened and the sharp field differ most over the view.
    float cx = 0.0f, cy = 0.0f;
    double best = -1.0;
    for (int j = -12; j <= 12; ++j)
      for (int i = -12; i <= 12; ++i) {
        const float x = 200.0f * float(i), y = 200.0f * float(j);
        double d = 0.0;
        for (int v = -2; v <= 2; ++v)
          for (int u = -2; u <= 2; ++u)
            d += std::fabs(double(soft.height(x + 10.0f * u, y + 10.0f * v)) -
                           double(hard.height(x + 10.0f * u, y + 10.0f * v)));
        if (d > best) {
          best = d;
          cx = x;
          cy = y;
        }
      }
    qInfo("view centred at (%.0f, %.0f)", double(cx), double(cy));
    QMatrix4x4 proj, view;
    view.translate(-cx, -cy);
    proj.ortho(-radius, radius, -radius, radius, -1000.0f, 1000.0f);
    g.render(&gl, proj, view, QVector3D(cx, cy, 0.0f),
             QVector3D(0.3f, 0.3f, -1.0f), 0.0f);
    gl.glFinish();
    const std::vector<GpuGrass::Blade> blades = g.readBlades(&gl, 0);
    qInfo("blades: %zu", blades.size());
    QVERIFY2(blades.size() > 500, "too few blades to say anything");
    double dSoft = 0.0, dHard = 0.0;
    for (const GpuGrass::Blade &b : blades) {
      dSoft += std::fabs(double(-b.negH) - double(soft.height(b.x, b.y)));
      dHard += std::fabs(double(-b.negH) - double(hard.height(b.x, b.y)));
    }
    dSoft /= double(blades.size());
    dHard /= double(blades.size());
    qInfo("%zu blades: mean |GPU - CPU| %.3f m with the softened field, %.3f m "
          "against the sharp one",
          blades.size(), dSoft, dHard);
    QVERIFY2(dHard > 0.5, "the soft and sharp fields barely differ here: the "
                          "test says nothing");
    QVERIFY2(dSoft < 0.3, "GLSL ridged() does not match the CPU one");
    g.release(&gl);
    ctx.doneCurrent();
  }

  // The alpine preset has terms the plain field test above leaves at 0:
  // mountainBias, the flattened spawn pad and the relief hummocks. Blades must
  // sit on the mesh's own height there too (todo 81).
  // Ponds are carved into TerrainField::height() (WaterField), so a blade on a
  // bank must sit on the carved ground, not the bare field (the shore would
  // otherwise show floating or sunken blades).
  void gpuHeightFollowsWaterCarve() {
    if (QGuiApplication::platformName() == "offscreen")
      QSKIP("needs a real GL platform: QT_QPA_PLATFORM=xcb on an Xvfb with "
            "LIBGL_ALWAYS_SOFTWARE=1");
    QSurfaceFormat fmt;
    fmt.setVersion(4, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    QOpenGLContext ctx;
    ctx.setFormat(fmt);
    if (!ctx.create())
      QSKIP("no GL 4.3 context");
    QOffscreenSurface surf;
    surf.setFormat(ctx.format());
    surf.create();
    QVERIFY(ctx.makeCurrent(&surf));
    QOpenGLExtraFunctions gl(&ctx);

    int measured = 0;
    for (const char *id : {"meadow", "hills", "forest", "flat"}) {
      procgen::PresetData pd;
      if (!procgen::buildWorldPreset(id, 5u, pd))
        continue;
      FieldParams fp = pd.field;
      const TerrainField cpu(fp);
      if (!cpu.water().enabled())
        continue;
      // A pond: the first body on a lattice around the origin.
      procgen::WaterBody body;
      bool found = false;
      for (int j = -6; j <= 6 && !found; ++j)
        for (int i = -6; i <= 6 && !found; ++i)
          found = cpu.water().bodyAt(cpu.heightSource(), 170.0f * float(i), 170.0f * float(j), body);
      if (!found)
        continue;

      GpuGrass::Params p;
      p.field = fp;
      p.seed = fp.seed;
      p.heightM = fp.heightM;
      p.featureM = fp.featureM;
      p.macroM = fp.macroM;
      p.octaves = fp.octaves;
      p.lacunarity = fp.lacunarity;
      p.gain = fp.gain;
      p.mountainMix = fp.mountainMix;
      p.ridgeSoft = fp.ridgeSoft;
      p.meshStepM = procgen::kStreamChunkM / float(procgen::kStreamChunkRes);
      p.cell = 0.2f;
      p.grid = 384;
      p.asyncMask = false;
      p.slopeLo = 0.0f;
      p.slopeHi = 0.01f;
      p.grassMaxFrac = 100.0f;
      GpuGrass g;
      g.setParams(p);
      if (!g.init(&gl))
        QSKIP("compute shaders unavailable on this GL");

      const float reach = body.radiusM + body.bankM + 2.0f;
      const float half = std::min(40.0f, reach + 6.0f);
      QMatrix4x4 proj, view;
      view.translate(-body.cx, -body.cy);
      proj.ortho(-half, half, -half, half, -1000.0f, 1000.0f);
      g.render(&gl, proj, view, QVector3D(body.cx, body.cy, 0.0f),
               QVector3D(0.3f, 0.3f, -1.0f), 0.0f);
      gl.glFinish();
      const std::vector<GpuGrass::Blade> blades = g.readBlades(&gl, 0);
      double sum = 0.0, worst = 0.0;
      int n = 0;
      for (const GpuGrass::Blade &b : blades) {
        if (std::hypot(b.x - body.cx, b.y - body.cy) > reach)
          continue; // only the pond and its bank
        const double d = std::fabs(double(-b.negH) - double(cpu.height(b.x, b.y)));
        sum += d;
        worst = std::max(worst, d);
        ++n;
      }
      qInfo("%s: pond r %.1f m, bank %.1f m, depth %.1f m: %d blades on the "
            "pond/bank, mean |GPU - CPU| %.3f m, worst %.3f m",
            id, double(body.radiusM), double(body.bankM), double(body.depthM), n,
            n ? sum / n : 0.0, worst);
      if (n >= 50) {
        ++measured;
        QVERIFY2(sum / n < 0.1, "blades on a pond bank do not follow the carved "
                                "ground");
        QVERIFY2(worst < 0.5, "a blade floats or sinks at the shore");
      }
      g.release(&gl);
    }
    ctx.doneCurrent();
    if (measured == 0)
      QSKIP("no preset gave a pond with blades on it");
  }

  void gpuHeightFollowsAlpineTerms() {
    if (QGuiApplication::platformName() == "offscreen")
      QSKIP("needs a real GL platform: QT_QPA_PLATFORM=xcb on an Xvfb with "
            "LIBGL_ALWAYS_SOFTWARE=1");
    QSurfaceFormat fmt;
    fmt.setVersion(4, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    QOpenGLContext ctx;
    ctx.setFormat(fmt);
    if (!ctx.create())
      QSKIP("no GL 4.3 context");
    QOffscreenSurface surf;
    surf.setFormat(ctx.format());
    surf.create();
    QVERIFY(ctx.makeCurrent(&surf));
    QOpenGLExtraFunctions gl(&ctx);

    procgen::PresetData pd;
    QVERIFY(procgen::buildWorldPreset("alpine", 5u, pd));
    FieldParams fp = pd.field;
    QVERIFY(fp.mountainBias > 0.0f && fp.flatRadiusM > 0.0f && fp.reliefM > 0.0f);
    fp.palette = procgen::kPalMeadow; // grass grows here; heights ignore the palette
    const TerrainField cpu(fp);

    GpuGrass::Params p;
    p.field = fp;
    p.seed = fp.seed;
    p.heightM = fp.heightM;
    p.featureM = fp.featureM;
    p.macroM = fp.macroM;
    p.octaves = fp.octaves;
    p.lacunarity = fp.lacunarity;
    p.gain = fp.gain;
    p.mountainMix = fp.mountainMix;
    p.ridgeSoft = fp.ridgeSoft;
    p.meshStepM = procgen::kStreamChunkM / float(procgen::kStreamChunkRes);
    p.cell = 0.2f;
    p.grid = 256;
    p.asyncMask = false;
    p.slopeLo = 0.0f;
    p.slopeHi = 0.01f;
    p.grassMaxFrac = 100.0f;
    GpuGrass g;
    g.setParams(p);
    if (!g.init(&gl))
      QSKIP("compute shaders unavailable on this GL");

    // Views: the spawn pad and its blend ring, and the mountain spot with the
    // largest height range (macro field puts most places in the flat basins).
    float mx = 0.0f, my = 0.0f;
    double best = -1.0;
    for (int j = -12; j <= 12; ++j)
      for (int i = -12; i <= 12; ++i) {
        const float x = 200.0f * float(i), y = 200.0f * float(j);
        float lo = 1e9f, hi = -1e9f;
        for (int v = -2; v <= 2; ++v)
          for (int u = -2; u <= 2; ++u) {
            const float h = cpu.height(x + 10.0f * u, y + 10.0f * v);
            lo = std::min(lo, h);
            hi = std::max(hi, h);
          }
        if (double(hi - lo) > best) {
          best = double(hi - lo);
          mx = x;
          my = y;
        }
      }
    qInfo("mountain view at (%.0f, %.0f), range %.1f m", double(mx), double(my), best);
    const float views[2][2] = {{0.0f, 0.0f}, {mx, my}};
    const char *names[2] = {"pad", "mountain"};
    for (int vi = 0; vi < 2; ++vi) {
      const float cx = views[vi][0], cy = views[vi][1];
      QMatrix4x4 proj, view;
      view.translate(-cx, -cy);
      proj.ortho(-25.0f, 25.0f, -25.0f, 25.0f, -1000.0f, 1000.0f);
      g.render(&gl, proj, view, QVector3D(cx, cy, 0.0f),
               QVector3D(0.3f, 0.3f, -1.0f), 0.0f);
      gl.glFinish();
      const std::vector<GpuGrass::Blade> blades = g.readBlades(&gl, 0);
      QVERIFY2(blades.size() > 500, "too few blades to say anything");
      double sum = 0.0, worst = 0.0;
      for (const GpuGrass::Blade &b : blades) {
        const double d = std::fabs(double(-b.negH) - double(cpu.height(b.x, b.y)));
        sum += d;
        worst = std::max(worst, d);
      }
      const double mean = sum / double(blades.size());
      qInfo("%s: %zu blades, against the uncapped field: mean %.4f m, worst %.3f m "
            "(the cap is what moves them)",
            names[vi], blades.size(), mean, worst);
      // Against the mesh the player sees (slope-capped), on a subsample: each
      // surfaceHeight() builds and erodes its own window.
      double sSum = 0.0, sWorst = 0.0;
      int sN = 0;
      for (std::size_t k = 0; k < blades.size(); k += 97, ++sN) {
        const double d = std::fabs(double(-blades[k].negH) -
                                   double(procgen::surfaceHeight(
                                       cpu, blades[k].x, blades[k].y,
                                       procgen::kStreamChunkM, procgen::kStreamChunkRes)));
        sSum += d;
        sWorst = std::max(sWorst, d);
      }
      qInfo("%s: against the slope-capped mesh (%d samples) mean %.4f m, worst %.3f m",
            names[vi], sN, sSum / sN, sWorst);
      QVERIFY2(sSum / sN < 0.05, names[vi]); // the mesh the player sees
    }

    // The node grids are cached per ring: rendering again from the same camera
    // rebuilds none, nudging the camera a few cm keeps the blades on the capped
    // mesh, and a params change (new generation) rebuilds them.
    {
      QMatrix4x4 proj, view;
      proj.ortho(-25.0f, 25.0f, -25.0f, 25.0f, -1000.0f, 1000.0f);
      auto renderAt = [&](float cx, float cy) {
        view.setToIdentity();
        view.translate(-cx, -cy);
        g.render(&gl, proj, view, QVector3D(cx, cy, 0.0f),
                 QVector3D(0.3f, 0.3f, -1.0f), 0.0f);
        gl.glFinish();
      };
      renderAt(views[1][0], views[1][1]);
      const int before = g.nodeGridBuilds();
      renderAt(views[1][0], views[1][1]);
      QCOMPARE(g.nodeGridBuilds(), before); // same window: every ring hits
      const float cx = views[1][0] + 0.05f, cy = views[1][1] + 0.03f;
      renderAt(cx, cy);
      const std::vector<GpuGrass::Blade> blades = g.readBlades(&gl, 0);
      QVERIFY(blades.size() > 500);
      double sum = 0.0;
      int n = 0;
      for (std::size_t k = 0; k < blades.size(); k += 97, ++n)
        sum += std::fabs(double(-blades[k].negH) -
                         double(procgen::surfaceHeight(cpu, blades[k].x, blades[k].y,
                                                       procgen::kStreamChunkM,
                                                       procgen::kStreamChunkRes)));
      qInfo("cached grids, camera moved: mean %.4f m", sum / n);
      QVERIFY2(sum / n < 0.05, "cached node grid");
      const int mid = g.nodeGridBuilds();
      g.setParams(p); // same params, new generation: everything rebuilds
      renderAt(cx, cy);
      QVERIFY(g.nodeGridBuilds() >= mid + 4); // one per ring
    }

    // Without the cap lattice (meshStepM 0) the shader is the bare field again:
    // the three terms stay exact against TerrainField::height.
    p.meshStepM = 0.0f;
    g.setParams(p);
    for (int vi = 0; vi < 2; ++vi) {
      const float cx = views[vi][0], cy = views[vi][1];
      QMatrix4x4 proj, view;
      view.translate(-cx, -cy);
      proj.ortho(-25.0f, 25.0f, -25.0f, 25.0f, -1000.0f, 1000.0f);
      g.render(&gl, proj, view, QVector3D(cx, cy, 0.0f),
               QVector3D(0.3f, 0.3f, -1.0f), 0.0f);
      gl.glFinish();
      const std::vector<GpuGrass::Blade> blades = g.readBlades(&gl, 0);
      QVERIFY2(blades.size() > 500, names[vi]);
      double sum = 0.0;
      for (const GpuGrass::Blade &b : blades)
        sum += std::fabs(double(-b.negH) - double(cpu.height(b.x, b.y)));
      qInfo("%s, no cap: mean |GPU - height()| %.4f m", names[vi], sum / double(blades.size()));
      QVERIFY2(sum / double(blades.size()) < 0.05, names[vi]);
    }
    g.release(&gl);
    ctx.doneCurrent();
  }
};

QTEST_MAIN(TstGpuGrass)
#include "tst_gpu_grass.moc"
