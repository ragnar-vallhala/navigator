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
      const float tm = sz / float(n), o = (float(n / 2) + 0.5f) * tm;
      const std::vector<uint8_t> m =
          GpuGrass::buildMask(fp, s.x - o, s.y - o, sz, n);
      const int c = m[size_t(n / 2) * size_t(n) + size_t(n / 2)];
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
};

QTEST_MAIN(TstGpuGrass)
#include "tst_gpu_grass.moc"
