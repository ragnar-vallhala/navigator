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
// The vegetation renderer inside the real 3D sim view: a forest scene drawn
// with vegetation ON differs from the same scene OFF and is not blank, the
// instance count follows the palette (forest vs yard), LODs cross-fade without
// gaps, far trees become billboards, and the GL side tears down and rebuilds
// without leaking or raising a GL error.
//
// Needs a real GL platform plugin: run with QT_QPA_PLATFORM=xcb on an Xvfb and
// LIBGL_ALWAYS_SOFTWARE=1; on "offscreen" (ctest's default) it skips, like the
// other GL view tests. NAV_VEGETATION is forced on for the scenes.
#include "ChunkStreamer.h"
#include "SimRendererWidget.h"
#include "VegetationRenderer.h"
#include "procgen/GroundCover.h"
#include "procgen/TerrainField.h"
#include "procgen/Vegetation.h"

#include <QApplication>
#include <QGuiApplication>
#include <QOpenGLContext>
#include <QSettings>
#include <QStandardPaths>
#include <QSurfaceFormat>
#include <QVBoxLayout>
#include <QtTest>

#include <cmath>

using namespace vsim;
using namespace vsim::procgen;

namespace {

constexpr float kChunkM = kStreamChunkM;
constexpr int kRes = kStreamChunkRes;

struct World {
  std::unique_ptr<TerrainField> field;
  VegParams params;
  std::vector<std::vector<VegInstance>> veg; // per chunk, same order as keys
  std::vector<qint64> keys;
  std::vector<std::pair<int, int>> cells;
};

World buildWorld(int palette, int lo, int hi) {
  World w;
  FieldParams fp;
  fp.seed = 11u;
  fp.palette = palette;
  fp.heightM = 20.0f; // gentle: the camera stays above ground by construction
  w.field = std::make_unique<TerrainField>(fp);
  w.params = vegParamsForPalette(palette);
  for (int cy = lo; cy <= hi; ++cy)
    for (int cx = lo; cx <= hi; ++cx) {
      w.keys.push_back(ChunkStreamer::keyOf(cx, cy));
      w.cells.push_back({cx, cy});
      w.veg.push_back(scatterVegetation(*w.field, cx, cy, kChunkM, w.params));
    }
  return w;
}

size_t totalInstances(const World &w) {
  size_t n = 0;
  for (const auto &v : w.veg)
    n += v.size();
  return n;
}

void loadTerrain(SimRendererWidget *r, const World &w) {
  for (size_t i = 0; i < w.keys.size(); ++i) {
    const ProcMesh m = meshFieldChunk(*w.field, w.cells[i].first, w.cells[i].second,
                                      kChunkM, kRes);
    const ChunkMeshData d = toChunkMeshData(w.keys[i], m);
    r->setWorldChunk(d.key, d.positions, d.normals, d.colors);
  }
}

void loadVegetation(SimRendererWidget *r, const World &w) {
  for (size_t i = 0; i < w.keys.size(); ++i)
    r->setChunkVegetation(w.keys[i], w.veg[i], (w.cells[i].first + 0.5f) * kChunkM,
                          (w.cells[i].second + 0.5f) * kChunkM, kChunkM * 0.5f);
}

void aimCamera(SimRendererWidget *r, const World &w, float x, float y, float aboveM,
               float heading, float down) {
  const float g = surfaceHeight(*w.field, x, y, kChunkM, kRes);
  r->setCameraPoseForTest(QVector3D(x, y, -(g + aboveM)), heading, down);
  SimSnapshot s;
  s.pos_w = {x, y, -(g + aboveM)};
  r->setSnapshot(s);
}

// Frames until textures/impostors have settled (llvmpipe decodes slowly).
QImage settledFrame(SimRendererWidget *r, int minFrames = 4) {
  QElapsedTimer t;
  t.start();
  QImage img;
  int frames = 0;
  while (t.elapsed() < 120000) {
    img = r->grabFrame();
    ++frames;
    if (frames >= minFrames && !r->vegetationBusy())
      break;
  }
  for (int i = 0; i < 2; ++i) // two more: bakes and fades done
    img = r->grabFrame();
  return img;
}

int distinctColours(const QImage &img) {
  QSet<QRgb> s;
  for (int y = 0; y < img.height(); y += 3)
    for (int x = 0; x < img.width(); x += 3)
      s.insert(img.pixel(x, y));
  return int(s.size());
}

// Share of pixels whose colour differs noticeably between two frames.
double changedFraction(const QImage &a, const QImage &b) {
  const QImage x = a.convertToFormat(QImage::Format_RGB32);
  const QImage y = b.convertToFormat(QImage::Format_RGB32);
  if (x.size() != y.size())
    return 1.0;
  long n = 0, tot = 0;
  for (int j = 0; j < x.height(); ++j)
    for (int i = 0; i < x.width(); ++i, ++tot) {
      const QRgb p = x.pixel(i, j), q = y.pixel(i, j);
      if (std::abs(qRed(p) - qRed(q)) + std::abs(qGreen(p) - qGreen(q)) +
              std::abs(qBlue(p) - qBlue(q)) >
          36)
        ++n;
    }
  return double(n) / double(tot);
}

struct Rig {
  QWidget host;
  SimRendererWidget *r = nullptr;
  explicit Rig(bool trees = false) {
    auto *lay = new QVBoxLayout(&host);
    lay->setContentsMargins(0, 0, 0, 0);
    r = new SimRendererWidget;
    r->setGlErrorCheck(true);
    r->setVegetationTreesEnabled(trees); // explicit, whatever the default is
    auto *box = QWidget::createWindowContainer(r, &host);
    box->setFixedSize(800, 500);
    lay->addWidget(box);
  }
  bool show() {
    host.show();
    return QTest::qWaitForWindowExposed(r);
  }
};

// Four trees of one kind on bare ground, 40 m east of the origin and spread 20 m
// apart sideways (so about as far from a camera behind them), each with its own
// yaw. Several trees keep the pixel counts of far rows large enough to compare.
constexpr int kStandTrees = 4;
World standWorld(int kind) {
  World w = buildWorld(kPalForest, -1, 0);
  for (auto &v : w.veg)
    v.clear();
  for (size_t i = 0; i < w.cells.size(); ++i)
    if (w.cells[i].first == 0 && w.cells[i].second == 0)
      for (int n = 0; n < kStandTrees; ++n) {
        const float y = -30.0f + 20.0f * float(n);
        VegInstance t;
        t.species = kVegTree;
        t.kind = uint8_t(kind);
        t.variant = 0;
        t.scale = 1.0f;
        t.yaw = 0.3f + 1.3f * float(n);
        t.tint = 0.5f;
        t.pos = {40.0f, y, -surfaceHeight(*w.field, 40.0f, y, kChunkM, kRes)};
        w.veg[i].push_back(t);
      }
  return w;
}

// Camera `d` metres (slant) from the stand's middle (6 m up), `elevDeg` above the
// horizontal, looking straight at it.
void aimAtStand(SimRendererWidget *r, const World &w, float d, float elevDeg) {
  const float e = elevDeg * 3.14159265f / 180.0f;
  aimCamera(r, w, 40.0f - d * std::cos(e), 0.0f, 6.0f + d * std::sin(e), 0.0f, e);
}

struct TreePixels {
  int foliage = 0; // green-dominant pixels that differ from the empty scene
  int all = 0;
  double lum = 0.0; // mean luminance of those pixels
  // Screen-door measure: pixels the trees do not cover but three of whose four
  // neighbours they do (pin holes), plus covered pixels with at most one covered
  // neighbour (loose dots), over the covered pixels. A solid crown or a clean
  // edge has few; a dithered one has many.
  int pinholes = 0, dots = 0;
  double stipple() const { return all > 0 ? double(pinholes + dots) / all : 0.0; }
};

// What the trees add to the frame `off` (the same pose with vegetation off).
TreePixels treeAdded(const QImage &onIn, const QImage &offIn) {
  const QImage on = onIn.convertToFormat(QImage::Format_RGB32);
  const QImage off = offIn.convertToFormat(QImage::Format_RGB32);
  TreePixels t;
  double sum = 0;
  for (int y = 0; y < on.height(); ++y)
    for (int x = 0; x < on.width(); ++x) {
      const QRgb a = on.pixel(x, y), b = off.pixel(x, y);
      if (std::abs(qRed(a) - qRed(b)) + std::abs(qGreen(a) - qGreen(b)) +
              std::abs(qBlue(a) - qBlue(b)) <= 24)
        continue;
      ++t.all;
      sum += 0.299 * qRed(a) + 0.587 * qGreen(a) + 0.114 * qBlue(a);
      if (qGreen(a) >= qRed(a))
        ++t.foliage;
    }
  t.lum = t.all ? sum / t.all : 0.0;
  const int W = on.width(), H = on.height();
  std::vector<unsigned char> cov(size_t(W) * size_t(H), 0);
  for (int y = 0; y < H; ++y)
    for (int x = 0; x < W; ++x) {
      const QRgb a = on.pixel(x, y), b = off.pixel(x, y);
      cov[size_t(y) * size_t(W) + size_t(x)] =
          std::abs(qRed(a) - qRed(b)) + std::abs(qGreen(a) - qGreen(b)) + std::abs(qBlue(a) - qBlue(b)) > 24 ? 1 : 0;
    }
  for (int y = 1; y + 1 < H; ++y)
    for (int x = 1; x + 1 < W; ++x) {
      const size_t i = size_t(y) * size_t(W) + size_t(x);
      const int n = cov[i - 1] + cov[i + 1] + cov[i - size_t(W)] + cov[i + size_t(W)];
      if (cov[i] ? n <= 1 : n >= 3)
        ++(cov[i] ? t.dots : t.pinholes);
    }
  return t;
}

// The stand seen from `d` m at `elev` degrees down, drawn at `lod` (0-2 mesh,
// 3 impostor, -1 by distance). The impostor atlas needs a few frames to bake.
TreePixels standView(SimRendererWidget *r, const World &w, float d, float elev, int lod) {
  aimAtStand(r, w, d, elev);
  r->setVegetationEnabled(false);
  const QImage off = r->grabFrame();
  r->setVegetationEnabled(true);
  r->setVegetationForcedLod(lod);
  QImage on = settledFrame(r);
  for (int i = 0; i < 40 && (lod == 3 || (lod == -1 && d > 135.0f)) &&
                  r->vegetationStats().impostors == 0;
       ++i)
    on = r->grabFrame();
  r->setVegetationForcedLod(-1);
  return treeAdded(on, off);
}

} // namespace

class TstVegetationRender : public QObject {
  Q_OBJECT
private slots:
  void initTestCase() {
    qputenv("NAV_VEGETATION", "1");
    qunsetenv("NAV_VEGETATION_TREES"); // the default: trees on
    QStandardPaths::setTestModeEnabled(true); // no stored sim/vegetationTrees
    if (QGuiApplication::platformName() == "offscreen")
      QSKIP("the GL view needs a real platform: set QT_QPA_PLATFORM=xcb, DISPLAY "
            "to an Xvfb and LIBGL_ALWAYS_SOFTWARE=1");
    QSurfaceFormat fmt; // what the app asks for in main()
    fmt.setVersion(4, 6);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    fmt.setSamples(4);
    QSurfaceFormat::setDefaultFormat(fmt);
  }

  // Pure: what is drawn at a distance. No GL.
  void lodChoiceIsGapFreeAndMonotonic() {
    int n = 0;
    const VegSpeciesInfo *tab = vegSpeciesTable(&n);
    QVERIFY(n >= 10);
    for (int i = 0; i < n; ++i) {
      const VegSpeciesInfo &s = tab[i];
      for (bool bill : {false, true}) {
        const float maxD = (bill || !s.impostor) ? s.maxDist : s.fallbackMax;
        int lastLod = -1;
        const int steps = int((maxD + 20.0f - 0.5f) / 0.37f) + 1;
        for (int st = 0; st < steps; ++st) {
          const float d = 0.5f + float(st) * 0.37f;
          const VegLodChoice c = vegChooseLod(s, d, bill);
          if (d >= maxD) {
            QCOMPARE(c.n, 0);
            continue;
          }
          QVERIFY2(c.n >= 1 && c.n <= 2, s.id); // never a gap before maxDist
          for (int k = 0; k < c.n; ++k) {
            QVERIFY(c.lod[k] >= 0 && c.lod[k] <= 3);
            if (c.lod[k] == 3)
              QVERIFY2(bill && s.impostor, s.id);
          }
          if (c.n == 2) // exactly complementary dither partners
            QVERIFY(c.fade[0] <= 1.0f && c.fade[1] >= 2.0f &&
                    std::fabs(c.fade[1] - 2.0f - c.fade[0]) < 1e-5f);
          QVERIFY2(c.lod[0] >= lastLod, s.id); // coarser with distance
          lastLod = c.lod[0];
        }
      }
    }
    // Trees: LOD0 close, LOD1 mid, impostor far.
    const VegSpeciesInfo &t = tab[0];
    QCOMPARE(vegChooseLod(t, 10.0f, true).lod[0], 0);
    QCOMPARE(vegChooseLod(t, 60.0f, true).lod[0], 1);
    QCOMPARE(vegChooseLod(t, 120.0f, true).lod[0], 2);
    QCOMPARE(vegChooseLod(t, 300.0f, true).lod[0], 3);
    QCOMPARE(vegChooseLod(t, 300.0f, false).n, 0); // past the 250 m fallback
  }

  // Instance counts follow the palette: dense forest, almost nothing in a yard.
  // The billboard shader locates a tile from the atlas layout; if its numbers
  // differ from the bake's constants, impostors sample the wrong tiles.
  void billboardShaderUsesAtlasConstants() {
    const QByteArray src = vsim::vegBillboardVertexSource();
    QVERIFY2(!src.contains('@'), "unresolved placeholder in the billboard shader");
    const QByteArray tiles = QByteArray::number(double(vsim::kVegAtlasTiles), 'f', 1);
    const QByteArray az = QByteArray::number(double(vsim::kVegAtlasAzimuths), 'f', 1);
    QVERIFY(src.contains("mod(tile, " + tiles + ")"));
    QVERIFY(src.contains("floor(tile / " + tiles + ")"));
    QVERIFY(src.contains("+ q) / " + tiles + ";"));
    QVERIFY(src.contains("* " + az + ";"));
    QVERIFY(src.contains("mod(i0 + 1.0, " + az + ")"));
    QVERIFY(src.contains("tileUv(u_tile + " + az + ", ")); // the top-down tile follows the azimuths
    // The atlas must be able to hold every azimuth of at least one variant.
    QVERIFY(vsim::kVegAtlasTiles * vsim::kVegAtlasTiles >= vsim::kVegAtlasViews);
    QCOMPARE(vsim::kVegAtlasViews, vsim::kVegAtlasAzimuths + 1);
    // The nine tree variants (fir 2, pine 3, jacaranda 2, island 2) all get a tile set.
    QVERIFY(9 * vsim::kVegAtlasViews <= vsim::kVegAtlasTiles * vsim::kVegAtlasTiles);
    // A stale tile count would show up as a literal divide by a different number.
    if (vsim::kVegAtlasTiles != 8)
      QVERIFY(!src.contains("tile / 8.0") && !src.contains("+ q) / 8.0"));
  }

  void instanceCountFollowsPalette() {
    const World forest = buildWorld(kPalForest, -1, 0);
    const World yard = buildWorld(kPalYard, -1, 0);
    qInfo("instances over 4 chunks: forest %zu, yard %zu", totalInstances(forest),
          totalInstances(yard));
    QVERIFY(totalInstances(forest) > 2000);
    QVERIFY(totalInstances(yard) * 20 < totalInstances(forest));
    Rig rig(true);
    loadVegetation(rig.r, forest);
    QCOMPARE(rig.r->vegetationInstanceCount(), totalInstances(forest));
    rig.r->removeWorldChunk(forest.keys[0]); // vegetation leaves with its chunk
    QCOMPARE(rig.r->vegetationInstanceCount(),
             totalInstances(forest) - forest.veg[0].size());
    rig.r->clearWorldChunks();
    QCOMPARE(rig.r->vegetationInstanceCount(), size_t(0));
  }

  // With trees switched off (NAV_VEGETATION_TREES=0 or the setting), a forest
  // palette still gets shrubs, ground plants, grass tufts and rocks, but no
  // tree instances, impostors or atlas.
  void treesOffHasNoTreesButGroundCover() {
    const World forest = buildWorld(kPalForest, -1, 0);
    Rig rig; // trees off (explicit)
    QVERIFY(!rig.r->vegetationTreesEnabled());
    loadTerrain(rig.r, forest);
    loadVegetation(rig.r, forest);
    QCOMPARE(rig.r->vegetationInstanceCount(int(kVegTree)), size_t(0));
    QVERIFY(rig.r->vegetationInstanceCount(int(kVegBush)) > 100);
    QVERIFY(rig.r->vegetationInstanceCount(int(kVegGrassClump)) > 1000);
    QVERIFY(rig.r->vegetationInstanceCount(int(kVegRock)) > 0);
    QVERIFY(rig.show());
    aimCamera(rig.r, forest, -60.0f, -60.0f, 3.0f, 0.8f, 0.12f);
    const QImage img = settledFrame(rig.r);
    const VegetationRenderer::Stats st = rig.r->vegetationStats();
    qInfo("default: %d instances drawn, %d billboards, %d baked variants", st.instancesDrawn,
          st.impostors, st.impostorVariants);
    QVERIFY(st.instancesDrawn > 50);
    QCOMPARE(st.impostors, 0);
    QCOMPARE(st.impostorVariants, 0);
    QVERIFY(distinctColours(img) > 40);
    QCOMPARE(rig.r->glErrorCount(), 0);
    rig.r->setVegetationTreesEnabled(true); // switching them on adds the trees
    QVERIFY(rig.r->vegetationInstanceCount(int(kVegTree)) > 100);
  }

  void envOverridesSetting() {
    qputenv("NAV_VEGETATION", "0");
    QVERIFY(!VegetationRenderer::enabledByConfig());
    qputenv("NAV_VEGETATION", "1");
    QVERIFY(VegetationRenderer::enabledByConfig());
    // No env var and no stored setting: trees are on.
    qunsetenv("NAV_VEGETATION_TREES");
    QVERIFY(VegetationRenderer::treesEnabledByConfig());
    qputenv("NAV_VEGETATION_TREES", "1");
    QVERIFY(VegetationRenderer::treesEnabledByConfig());
    for (const char *off : {"0", "false", "FALSE", "off", "Off"}) {
      qputenv("NAV_VEGETATION_TREES", off);
      QVERIFY2(!VegetationRenderer::treesEnabledByConfig(), off);
    }
    // The stored setting turns them off when no env var overrides it, and the
    // env var wins over the setting.
    qunsetenv("NAV_VEGETATION_TREES");
    {
      QSettings s;
      s.setValue("sim/vegetationTrees", false);
    }
    QVERIFY(!VegetationRenderer::treesEnabledByConfig());
    qputenv("NAV_VEGETATION_TREES", "1");
    QVERIFY(VegetationRenderer::treesEnabledByConfig());
    qunsetenv("NAV_VEGETATION_TREES");
    {
      QSettings s;
      s.remove("sim/vegetationTrees");
    }
    QVERIFY(VegetationRenderer::treesEnabledByConfig());
  }

  // Forest scene near the ground: vegetation ON differs from OFF, neither is
  // blank, and what was drawn is accounted for.
  void forestOnDiffersFromOff() {
    const World forest = buildWorld(kPalForest, -1, 0);
    Rig rig(true);
    loadTerrain(rig.r, forest);
    loadVegetation(rig.r, forest);
    QVERIFY(rig.show());
    aimCamera(rig.r, forest, -60.0f, -60.0f, 3.0f, 0.8f, 0.12f);
    rig.r->setVegetationEnabled(false);
    const QImage off = settledFrame(rig.r, 3);
    QCOMPARE(rig.r->vegetationStats().instancesDrawn, 0);
    rig.r->setVegetationEnabled(true);
    const QImage on = settledFrame(rig.r);
    const VegetationRenderer::Stats st = rig.r->vegetationStats();
    qInfo("ON: %d instances, %d draw calls, %lld tris, %d chunks, %d species ready",
          st.instancesDrawn, st.drawCalls, st.triangles, st.chunksDrawn,
          st.speciesReady);
    QVERIFY(st.instancesDrawn > 50);
    QVERIFY(st.drawCalls > 0 && st.speciesReady > 0);
    QVERIFY(distinctColours(off) > 40);
    QVERIFY(distinctColours(on) > 40);
    const double ch = changedFraction(on, off);
    qInfo("changed pixels with vegetation on: %.1f%%", 100.0 * ch);
    QVERIFY(ch > 0.02);
    QCOMPARE(rig.r->glErrorCount(), 0);
  }

  // Looking down a long forest the far trees are billboards (when the context
  // can bake the atlas), and the frame stays valid.
  void farTreesBecomeBillboards() {
    const World forest = buildWorld(kPalForest, -2, 1);
    Rig rig(true);
    loadTerrain(rig.r, forest);
    loadVegetation(rig.r, forest);
    QVERIFY(rig.show());
    aimCamera(rig.r, forest, -200.0f, -150.0f, 12.0f, 0.75f, 0.06f);
    const QImage img = settledFrame(rig.r);
    const VegetationRenderer::Stats st = rig.r->vegetationStats();
    qInfo("far view: %d instances, %d billboards, %d draw calls, %d baked variants",
          st.instancesDrawn, st.impostors, st.drawCalls, st.impostorVariants);
    QVERIFY(distinctColours(img) > 40);
    QVERIFY(st.instancesDrawn > 500);
    if (st.impostorVariants > 0)
      QVERIFY(st.impostors > 100);
    else
      qWarning("no impostor atlas on this context: mesh LOD fallback in use");
    QCOMPARE(rig.r->glErrorCount(), 0);
  }

  // The impostor bake records which covered texels are foliage: bark must be a
  // small part of every tree and sit low, foliage the rest and higher. (The
  // billboard shader uses that mask to tint foliage only; it has no test of
  // its own, only this readback of what the bake wrote.)
  void impostorBakeSeparatesBarkFromFoliage() {
    const World forest = buildWorld(kPalForest, -2, 1);
    Rig rig(true);
    rig.r->setVegetationBakeStats(true);
    loadTerrain(rig.r, forest);
    loadVegetation(rig.r, forest);
    QVERIFY(rig.show());
    aimCamera(rig.r, forest, -200.0f, -150.0f, 12.0f, 0.75f, 0.06f);
    settledFrame(rig.r);
    if (rig.r->vegetationStats().impostorVariants == 0)
      QSKIP("no impostor atlas on this context");
    const VegetationRenderer::BakeStats b = rig.r->vegetationBakeStats();
    qInfo("bake: %lld foliage px, %lld bark px", b.foliagePx, b.barkPx);
    QVERIFY(b.foliagePx > 1000 && b.barkPx > 100);
    QVERIFY(double(b.barkPx) < 0.4 * double(b.foliagePx));
    const double fh = b.foliageHeightSum / double(b.foliagePx);
    const double bh = b.barkHeightSum / double(b.barkPx);
    qInfo("mean height: foliage %.2f, bark %.2f", fh, bh);
    QVERIFY(bh < fh - 0.05);
    QCOMPARE(rig.r->glErrorCount(), 0);
  }

  // Far trees seen from above keep their crowns. Four trees of one kind, 160 m
  // and 250 m away (the impostor range), camera level with the crowns, 45, 60
  // and 90 degrees down: the foliage pixels of the natural draw (impostor) against
  // the same trees forced to LOD0. LOD0 gives 240 or more pixels in every row of
  // the three kinds asserted (the single-tree rows were too small at 250 m), so it
  // is the reference; LOD1 is logged beside it. The island tree is expected to
  // overshoot, side views included (its impostor fills its whole card): it is
  // held to what main drew (it improved from above), not to the band.
  void farTreesKeepTheirFoliageFromAbove_data() {
    QTest::addColumn<int>("kind");
    for (int k = 0; k < 4; ++k)
      QTest::newRow(k == 0 ? "fir" : k == 1 ? "pine" : k == 2 ? "jacaranda" : "island") << k;
  }
  void farTreesKeepTheirFoliageFromAbove() {
    QFETCH(int, kind);
    World w = standWorld(kind);
    Rig rig(true);
    loadTerrain(rig.r, w);
    loadVegetation(rig.r, w);
    QVERIFY(rig.show());
    QString failures;
    for (float d : {160.0f, 250.0f}) {
      int side = 0; // the foliage the same tree shows level with the crowns
      for (float e : {0.0f, 45.0f, 60.0f, 90.0f}) {
        const TreePixels nat = standView(rig.r, w, d, e, -1);
        const bool impostors = rig.r->vegetationStats().impostors > 0;
        const TreePixels ref0 = standView(rig.r, w, d, e, 0);
        const TreePixels ref1 = standView(rig.r, w, d, e, 1);
        // The same mesh from half the distance has four times the pixels (the
        // reference for a thin crown that one pixel row more or less moves by a
        // fifth on a different rasteriser).
        const TreePixels refNear = standView(rig.r, w, d * 0.5f, e, 0);
        const double nearPx = std::max(1.0, refNear.foliage / 4.0);
        if (rig.r->vegetationStats().impostorVariants == 0)
          QSKIP("no impostor atlas on this context");
        const double r0 = double(nat.foliage) / nearPx;
        const double r1 = ref1.foliage ? double(nat.foliage) / ref1.foliage : 0.0;
        qInfo("kind %d, %.0f m, %.0f deg down: foliage px natural %d, LOD0 %d, LOD0 at half distance /4 %.0f (x%.2f), LOD1 %d (x%.2f), impostors drawn %d",
              kind, double(d), double(e), nat.foliage, ref0.foliage, nearPx, r0, ref1.foliage, r1, int(impostors));
        if (!impostors || nearPx < (kind < 3 ? 150 : 40)) // the island tree is small
          failures += QString("[no impostors or reference too small at %1 m, %2 deg] ").arg(double(d)).arg(double(e));
        else if (kind < 3 && (r0 < 0.70 || r0 > 1.75))
          failures += QString("[%1 m %2 deg: x%3] ").arg(double(d)).arg(double(e)).arg(r0, 0, 'f', 2);
        else if (kind == 3) {
          // The island tree is a poor fit for the mesh reference (its crown is
          // mostly trunk at this size, and no rasteriser agrees on it), so it is
          // held to itself: from above it shows no more of the tree than it
          // does from the side. Main overshot this (x1.75 at 45 degrees).
          if (e == 0.0f)
            side = nat.foliage;
          else if (nat.foliage > side * 1.10)
            failures += QString("[island %1 m %2 deg: %3 px > %4 px from the side] ").arg(double(d)).arg(double(e)).arg(nat.foliage).arg(side);
        }
      }
    }
    QVERIFY2(failures.isEmpty(), qPrintable(failures));
    QCOMPARE(rig.r->glErrorCount(), 0);
  }

  // The side view did not move: with the camera level with the crowns the top
  // tile has no weight and the billboard is the quad it was, over tiles baked as
  // before. The relation is self-contained, so it holds on any rasteriser: half
  // a degree up draws what level draws (the blend has begun, but covers a few
  // pixels), and the impostor stays in the band of what the mesh draws.
  // (On main's own build the level numbers were fir 1129/431, pine 681/227,
  // jacaranda 2161/893 and island 917/384 px on the reference llvmpipe, which
  // the level row matched exactly there.)
  void impostorSideViewIsUnchangedFromMain_data() {
    QTest::addColumn<int>("kind");
    QTest::addColumn<float>("dist");
    QTest::newRow("fir 160") << 0 << 160.0f;
    QTest::newRow("fir 250") << 0 << 250.0f;
    QTest::newRow("pine 160") << 1 << 160.0f;
    QTest::newRow("pine 250") << 1 << 250.0f;
    QTest::newRow("jacaranda 160") << 2 << 160.0f;
    QTest::newRow("jacaranda 250") << 2 << 250.0f;
    QTest::newRow("island 160") << 3 << 160.0f;
    QTest::newRow("island 250") << 3 << 250.0f;
  }
  void impostorSideViewIsUnchangedFromMain() {
    QFETCH(int, kind);
    QFETCH(float, dist);
    World w = standWorld(kind);
    Rig rig(true);
    loadTerrain(rig.r, w);
    loadVegetation(rig.r, w);
    QVERIFY(rig.show());
    const TreePixels t = standView(rig.r, w, dist, 0.0f, -1);
    if (rig.r->vegetationStats().impostorVariants == 0)
      QSKIP("no impostor atlas on this context");
    const TreePixels up = standView(rig.r, w, dist, 0.1f, -1);
    const TreePixels mesh = standView(rig.r, w, dist, 0.0f, 0);
    qInfo("kind %d, %.0f m side: foliage px %d, at 0.1 deg %d, LOD0 %d; luminance %.1f, at 0.1 deg %.1f", kind, double(dist),
          t.foliage, up.foliage, mesh.foliage, t.lum, up.lum);
    QVERIFY2(t.foliage > 100, "trees not found");
    QVERIFY2(std::abs(double(up.foliage) / t.foliage - 1.0) <= 0.07, // the pose itself moved a tenth of a degree
             qPrintable(QString("foliage %1 level vs %2 at a tenth of a degree").arg(t.foliage).arg(up.foliage)));
    QVERIFY2(std::abs(up.lum / t.lum - 1.0) <= 0.02,
             qPrintable(QString("luminance %1 level vs %2 at a tenth of a degree").arg(t.lum).arg(up.lum)));
    if (kind < 3) {
      const double r = double(t.foliage) / std::max(1, mesh.foliage);
      QVERIFY2(r >= 0.6 && r <= 1.7, qPrintable(QString("x%1 of the mesh").arg(r)));
    }
    QCOMPARE(rig.r->glErrorCount(), 0);
  }

  // No pop where a tree changes from mesh to impostor (130 m), for every kind,
  // level with the crowns and from 45 and 60 degrees up: the foliage the trees
  // add to the frame (scaled by distance squared) and its mean luminance, step
  // by step across the edge. The luminance of the impostor is higher than the
  // mesh's (see the report); the blend hides it as a slow drift, not a step.
  void impostorCrossFadeHasNoPop_data() {
    QTest::addColumn<int>("kind");
    QTest::addColumn<float>("elev");
    for (int k = 0; k < 4; ++k)
      for (float e : {0.0f, 45.0f, 60.0f})
        QTest::newRow(qPrintable(QString("%1 @%2 deg").arg(k == 0 ? "fir" : k == 1 ? "pine" : k == 2 ? "jacaranda" : "island").arg(int(e))))
            << k << e;
  }
  void impostorCrossFadeHasNoPop() {
    QFETCH(int, kind);
    QFETCH(float, elev);
    World w = standWorld(kind);
    Rig rig(true);
    loadTerrain(rig.r, w);
    loadVegetation(rig.r, w);
    QVERIFY(rig.show());
    // Two metres apart across the whole fade, so the difference between the mesh
    // and the impostor (which varies from one rasteriser to the next) is spread
    // over many steps instead of landing in one.
    const float dists[] = {116, 118, 120, 122, 124, 126, 128, 130, 132, 134, 136, 138, 140, 142, 144, 146};
    std::vector<double> fol, lum;
    for (float d : dists) {
      const TreePixels t = standView(rig.r, w, d, elev, -1);
      fol.push_back(t.foliage * double(d) * double(d) / (116.0 * 116.0));
      lum.push_back(t.lum);
    }
    if (rig.r->vegetationStats().impostorVariants == 0)
      QSKIP("no impostor atlas on this context");
    double worstF = 1.0, worstL = 1.0;
    for (size_t i = 1; i < fol.size(); ++i) {
      QVERIFY2(fol[i - 1] > 20 && fol[i] > 20, "trees not found");
      const double rf = fol[i] / fol[i - 1], rl = lum[i] / lum[i - 1];
      if (std::abs(std::log(rf)) > std::abs(std::log(worstF)))
        worstF = rf;
      if (std::abs(std::log(rl)) > std::abs(std::log(worstL)))
        worstL = rl;
    }
    qInfo("kind %d, %.0f deg: worst foliage step x%.2f, worst luminance step x%.2f, luminance drift x%.2f over %g-%g m",
          kind, double(elev), worstF, worstL, lum.back() / lum.front(), double(dists[0]), double(dists[15]));
    QVERIFY2(worstF >= 0.7 && worstF <= 1.0 / 0.7, qPrintable(QString("foliage step x%1").arg(worstF)));
    QVERIFY2(worstL >= 0.8 && worstL <= 1.25, qPrintable(QString("luminance step x%1").arg(worstL)));
    QCOMPARE(rig.r->glErrorCount(), 0);
  }

  // The blend between the side tiles and the top tile has no step: the pine at
  // 160 m with the camera climbing from level to straight above in 5 degree steps
  // (then 89.9 and exactly 90, where the quad's heading is undefined and the
  // bake's up vector would be): the foliage pixels and luminance between
  // neighbours stay within a stated bound, and exactly overhead draws the same
  // as 89.9.
  void pineElevationSweepIsSmooth_data() {
    QTest::addColumn<float>("dist");
    QTest::addColumn<int>("lod");
    QTest::newRow("160 m natural") << 160.0f << -1;
    QTest::newRow("100 m impostor") << 100.0f << 3; // forced: the pine's mesh runs to 130 m
    QTest::newRow("60 m impostor") << 60.0f << 3;
  }
  void pineElevationSweepIsSmooth() {
    QFETCH(float, dist);
    QFETCH(int, lod);
    World w = standWorld(1);
    Rig rig(true);
    loadTerrain(rig.r, w);
    loadVegetation(rig.r, w);
    QVERIFY(rig.show());
    std::vector<float> elevs;
    for (int e = 0; e <= 85; e += 5)
      elevs.push_back(float(e));
    elevs.push_back(89.9f);
    elevs.push_back(90.0f);
    std::vector<TreePixels> px;
    for (float e : elevs) {
      px.push_back(standView(rig.r, w, dist, e, lod));
      qInfo("pine %.0f m, %.1f deg: foliage px %d, luminance %.1f, stipple %.3f, impostors %d", double(dist), double(e),
            px.back().foliage, px.back().lum, px.back().stipple(), rig.r->vegetationStats().impostors);
    }
    if (rig.r->vegetationStats().impostorVariants == 0)
      QSKIP("no impostor atlas on this context");
    double worstF = 1.0, worstL = 1.0;
    for (size_t i = 1; i + 2 < px.size(); ++i) { // up to 85 -> 89.9
      QVERIFY2(px[i - 1].foliage > 100 && px[i].foliage > 100, "trees not found");
      const double rf = double(px[i].foliage) / px[i - 1].foliage, rl = px[i].lum / px[i - 1].lum;
      if (std::abs(std::log(rf)) > std::abs(std::log(worstF)))
        worstF = rf;
      if (std::abs(std::log(rl)) > std::abs(std::log(worstL)))
        worstL = rl;
    }
    qInfo("pine sweep %.0f m: worst foliage step x%.2f, worst luminance step x%.2f per 5 degrees", double(dist), worstF, worstL);
    QVERIFY2(worstF >= 0.80 && worstF <= 1.28, qPrintable(QString("foliage step x%1").arg(worstF)));
    QVERIFY2(worstL >= 0.93 && worstL <= 1.08, qPrintable(QString("luminance step x%1").arg(worstL)));
    const TreePixels &p899 = px[px.size() - 2], &p90 = px.back();
    QVERIFY2(p90.foliage > 100, "nothing drawn from straight above");
    QVERIFY2(std::abs(double(p90.foliage) / p899.foliage - 1.0) <= 0.05,
             qPrintable(QString("90 deg %1 vs 89.9 deg %2").arg(p90.foliage).arg(p899.foliage)));
    QCOMPARE(rig.r->glErrorCount(), 0);
  }

  // The crown keeps its tone as the top tile comes in: the mean luminance of the
  // tree pixels from 20, 30 and 45 degrees up stays within 15 percent of the
  // side view of the same trees (the top tile used to read darker than the side
  // tiles, and a dark saucer showed where it replaced the crown).
  void impostorKeepsItsToneFromAbove_data() {
    QTest::addColumn<int>("kind");
    QTest::newRow("fir") << 0;
    QTest::newRow("pine") << 1;
    QTest::newRow("jacaranda") << 2;
  }
  void impostorKeepsItsToneFromAbove() {
    QFETCH(int, kind);
    World w = standWorld(kind);
    Rig rig(true);
    loadTerrain(rig.r, w);
    loadVegetation(rig.r, w);
    QVERIFY(rig.show());
    const TreePixels side = standView(rig.r, w, 160.0f, 0.0f, -1);
    if (rig.r->vegetationStats().impostorVariants == 0)
      QSKIP("no impostor atlas on this context");
    QVERIFY2(side.foliage > 100, "trees not found");
    QString failures;
    for (float e : {20.0f, 30.0f, 45.0f}) {
      const TreePixels t = standView(rig.r, w, 160.0f, e, -1);
      const double r = t.lum / side.lum;
      qInfo("TONE kind %d, %.0f deg: luminance %.1f (side %.1f), x%.3f", kind, double(e), t.lum, side.lum, r);
      if (r < 0.85 || r > 1.15)
        failures += QString("[%1 deg: x%2] ").arg(double(e)).arg(r, 0, 'f', 3);
    }
    QVERIFY2(failures.isEmpty(), qPrintable(failures));
    QCOMPARE(rig.r->glErrorCount(), 0);
  }

  // The impostor's coverage has no screen-door pattern where the top tile blends in.
  void impostorTopBlendHasNoScreenDoor_data() {
    QTest::addColumn<int>("kind");
    QTest::addColumn<float>("dist");
    for (int k = 0; k < 4; ++k)
      for (float d : {100.0f, 160.0f})
        QTest::newRow(qPrintable(QString("%1 %2 m").arg(k == 0 ? "fir" : k == 1 ? "pine" : k == 2 ? "jacaranda" : "island").arg(int(d))))
            << k << d;
  }
  void impostorTopBlendHasNoScreenDoor() {
    QFETCH(int, kind);
    QFETCH(float, dist);
    World w = standWorld(kind);
    Rig rig(true);
    loadTerrain(rig.r, w);
    loadVegetation(rig.r, w);
    QVERIFY(rig.show());
    QString failures;
    for (float e : {10.0f, 20.0f, 30.0f, 45.0f, 60.0f}) {
      const TreePixels imp = standView(rig.r, w, dist, e, 3);
      const TreePixels l0 = standView(rig.r, w, dist, e, 0);
      qInfo("STIP kind %d %.0f m %.0f deg: impostor %.3f (pin %d dots %d of %d), LOD0 %.3f (pin %d dots %d of %d)", kind,
            double(dist), double(e), imp.stipple(), imp.pinholes, imp.dots, imp.all, l0.stipple(), l0.pinholes, l0.dots,
            l0.all);
      QVERIFY2(imp.all > 150 && l0.all > 150, "trees not found");
      // No noisier than the mesh draws the same tree (plus a little slack).
      if (imp.stipple() > l0.stipple() + 0.05)
        failures += QString("[%1 deg: %2 vs LOD0 %3] ").arg(double(e)).arg(imp.stipple(), 0, 'f', 3).arg(l0.stipple(), 0, 'f', 3);
      // And far below what the screen-door pick drew at 100 m (fir 0.453 at 20
      // degrees, 1.329 at 30; pine 0.520 and 1.194): a quarter of that at most.
      if (dist == 100.0f && (kind == 0 || kind == 1) && (e == 20.0f || e == 30.0f)) {
        const double was = kind == 0 ? (e == 20.0f ? 0.453 : 1.329) : (e == 20.0f ? 0.520 : 1.194);
        if (imp.stipple() > 0.25 * was)
          failures += QString("[%1 deg: %2 is over a quarter of the old %3] ").arg(double(e)).arg(imp.stipple(), 0, 'f', 3).arg(was);
      }
    }
    QVERIFY2(failures.isEmpty(), qPrintable(failures));
    QCOMPARE(rig.r->glErrorCount(), 0);
  }

  // The top-down tile exists for every variant and the whole bake is repeatable:
  // two renderers baking the same forest give the same atlas (a hash of every
  // tile's colour and mask, read back), and the top tiles are not empty.
  void impostorBakeHasTopViewsAndIsRepeatable() {
    unsigned long long hash[2] = {0, 0};
    long long top[2] = {0, 0};
    for (int run = 0; run < 2; ++run) {
      const World forest = buildWorld(kPalForest, -2, 1);
      Rig rig(true);
      rig.r->setVegetationBakeStats(true);
      loadTerrain(rig.r, forest);
      loadVegetation(rig.r, forest);
      QVERIFY(rig.show());
      aimCamera(rig.r, forest, -200.0f, -150.0f, 12.0f, 0.75f, 0.06f);
      settledFrame(rig.r);
      if (rig.r->vegetationStats().impostorVariants == 0)
        QSKIP("no impostor atlas on this context");
      const VegetationRenderer::BakeStats b = rig.r->vegetationBakeStats();
      hash[run] = b.hash;
      top[run] = b.topPx;
      qInfo("run %d: %d variants baked, top tiles cover %lld texels, atlas hash %llx", run,
            rig.r->vegetationStats().impostorVariants, b.topPx, b.hash);
      QCOMPARE(rig.r->glErrorCount(), 0);
    }
    // Nine variants, each with a top tile of at least a few hundred covered texels.
    QVERIFY(top[0] > 2700);
    QCOMPARE(top[0], top[1]);
    QVERIFY(hash[0] != 0);
    QCOMPARE(hash[0], hash[1]);
  }

  // A trunk keeps its brightness when the mesh gives way to the impostor. One
  // tree on bare ground, the same camera and light and so the same fog: the
  // frame is taken with every tree forced to mesh LOD 2, then forced to the
  // impostor, and the trunk texels (the tree's pixels in the lowest fifth of its
  // silhouette, near its axis) are compared. The billboard shader itself has no
  // other test; this reads back what it draws.
  void trunkBrightnessSurvivesTheImpostorSwitch_data() {
    QTest::addColumn<int>("kind");
    QTest::addColumn<bool>("strict");
    QTest::addColumn<float>("view"); // which side of the tree the camera stands on (radians)
    // The primary view (from the west) is asserted for the pine and both
    // broadleaf trees; the fir's thin trunk is mostly hidden by needles (about 30
    // pixels), so it is only logged. The two other views are logged too: the
    // mesh trunk is lit by the sun and the cascaded shadows, the billboard has a
    // model of the sun on the visible half-cylinder and no shadow, so how well
    // they agree depends on the sun's side of the tree (see the report).
    for (float view : {3.14159f, 1.9f, 4.6f}) {
      const QByteArray sfx = "@" + QByteArray::number(double(view), 'f', 1);
      const bool primary = view > 3.0f && view < 3.3f;
      QTest::newRow(("fir" + sfx).constData()) << 0 << false << view;
      QTest::newRow(("pine" + sfx).constData()) << 1 << primary << view;
      QTest::newRow(("jacaranda" + sfx).constData()) << 2 << primary << view;
      QTest::newRow(("island" + sfx).constData()) << 3 << primary << view;
    }
  }
  void trunkBrightnessSurvivesTheImpostorSwitch() {
    QFETCH(int, kind);
    QFETCH(bool, strict);
    QFETCH(float, view);
    World w = buildWorld(kPalForest, -1, 0);
    for (auto &v : w.veg)
      v.clear();
    for (size_t i = 0; i < w.cells.size(); ++i)
      if (w.cells[i].first == 0 && w.cells[i].second == 0) {
        VegInstance t;
        t.species = kVegTree;
        t.kind = uint8_t(kind);
        t.variant = 0;
        t.scale = 1.0f;
        t.yaw = 0.3f;
        t.tint = 0.5f;
        t.pos = {40.0f, 0.0f, -surfaceHeight(*w.field, 40.0f, 0.0f, kChunkM, kRes)};
        w.veg[i].push_back(t);
      }
    Rig rig(true);
    loadTerrain(rig.r, w);
    loadVegetation(rig.r, w);
    QVERIFY(rig.show());
    // 40 m from the tree, looking at it from the angle `view` (pi: from the west)
    aimCamera(rig.r, w, 40.0f + 40.0f * std::cos(view), 40.0f * std::sin(view), 6.0f, view + 3.14159f, 0.04f);
    rig.r->setVegetationEnabled(false);
    const QImage off = rig.r->grabFrame().convertToFormat(QImage::Format_RGB32);
    rig.r->setVegetationEnabled(true);
    auto trunkMean = [&](int lod, int *count) {
      rig.r->setVegetationForcedLod(lod);
      QImage on = settledFrame(rig.r);
      for (int i = 0; i < 40 && lod == 3 && rig.r->vegetationStats().impostors == 0; ++i)
        on = rig.r->grabFrame();
      on = on.convertToFormat(QImage::Format_RGB32);
      int y0 = on.height(), y1 = -1;
      std::vector<QPoint> px;
      for (int y = 0; y < on.height(); ++y)
        for (int x = 0; x < on.width(); ++x) {
          const QRgb a = on.pixel(x, y), b = off.pixel(x, y);
          if (std::abs(qRed(a) - qRed(b)) + std::abs(qGreen(a) - qGreen(b)) +
                  std::abs(qBlue(a) - qBlue(b)) > 40) {
            px.push_back({x, y});
            y0 = std::min(y0, y);
            y1 = std::max(y1, y);
          }
        }
      *count = 0;
      if (px.size() < 50)
        return 0.0;
      const int cut = y1 - (y1 - y0) / 5;
      // Bark, not needles: low branches now hang into this band, and foliage is
      // the green-dominant part of it.
      auto isBark = [&](const QPoint &p) {
        const QRgb c = on.pixel(p);
        return qGreen(c) <= qRed(c) + 4;
      };
      std::vector<int> xs;
      for (const QPoint &p : px)
        if (p.y() >= cut && isBark(p))
          xs.push_back(p.x());
      if (xs.size() < 8)
        return 0.0;
      std::nth_element(xs.begin(), xs.begin() + long(xs.size() / 2), xs.end());
      const int cx = xs[xs.size() / 2];
      double sum = 0;
      for (const QPoint &p : px)
        if (p.y() >= cut && std::abs(p.x() - cx) <= 5 && isBark(p)) {
          const QRgb c = on.pixel(p);
          sum += 0.299 * qRed(c) + 0.587 * qGreen(c) + 0.114 * qBlue(c);
          ++*count;
        }
      return *count ? sum / *count : 0.0;
    };
    int nm = 0, ni = 0;
    const double mesh = trunkMean(2, &nm);
    const double imp = trunkMean(3, &ni);
    qInfo("kind %d: trunk mean luminance mesh %.1f (%d px), impostor %.1f (%d px), ratio %.2f", kind,
          mesh, nm, imp, ni, mesh > 0 ? imp / mesh : 0.0);
    if (rig.r->vegetationStats().impostorVariants == 0)
      QSKIP("no impostor atlas on this context");
    if (!strict)
      return;
    QVERIFY2(nm >= 8 && ni >= 8, "trunk texels not found");
    QVERIFY2(std::abs(imp - mesh) <= 0.15 * std::max(imp, mesh),
             qPrintable(QString("mesh %1 vs impostor %2").arg(mesh).arg(imp)));
    QCOMPARE(rig.r->glErrorCount(), 0);
  }

  // Every tree kind must reach the screen, conifers included: a pool or table
  // that never picks a kind looks exactly like a forest without them.
  void everyTreeKindIsDrawn() {
    const World forest = buildWorld(kPalForest, -2, 1);
    Rig rig(true);
    loadTerrain(rig.r, forest);
    loadVegetation(rig.r, forest);
    QVERIFY(rig.show());
    int meshes[procgen::kVegTreeKindCount] = {0}, imp[procgen::kVegTreeKindCount] = {0};
    for (float heading : {0.4f, 2.0f, 3.6f, 5.2f}) {
      aimCamera(rig.r, forest, -60.0f, -60.0f, 20.0f, heading, 0.15f);
      settledFrame(rig.r);
      const VegetationRenderer::Stats st = rig.r->vegetationStats();
      for (int k = 0; k < procgen::kVegTreeKindCount; ++k) {
        meshes[k] += st.treeMeshes[k];
        imp[k] += st.treeImpostors[k];
      }
    }
    for (int k = 0; k < procgen::kVegTreeKindCount; ++k) {
      qInfo("tree kind %d (%s): %d meshes, %d impostors", k, procgen::kVegTreeKinds[k].id, meshes[k], imp[k]);
      QVERIFY2(meshes[k] + imp[k] > 0, procgen::kVegTreeKinds[k].id);
    }
    QCOMPARE(rig.r->glErrorCount(), 0);
  }

  // Context teardown: the GL side frees completely, comes back from the CPU
  // instance lists, draws the same and leaks nothing.
  void teardownAndRebuild() {
    const World forest = buildWorld(kPalForest, -1, 0);
    Rig rig(true);
    loadTerrain(rig.r, forest);
    loadVegetation(rig.r, forest);
    QVERIFY(rig.show());
    aimCamera(rig.r, forest, -60.0f, -60.0f, 3.0f, 0.8f, 0.12f);
    const QImage ref = settledFrame(rig.r);
    const int drawn = rig.r->vegetationStats().instancesDrawn;
    QVERIFY(drawn > 50);
    const int live = rig.r->liveGlObjects();
    QVERIFY(live > 30);
    QOpenGLContext *ctx = rig.r->context();
    emit ctx->aboutToBeDestroyed(); // the genuine signal: renderer frees all
    QCOMPARE(rig.r->liveGlObjects(), 0);
    const QImage again = settledFrame(rig.r);
    QVERIFY(distinctColours(again) > 40);
    QCOMPARE(rig.r->vegetationStats().instancesDrawn, drawn);
    QVERIFY(changedFraction(ref, again) < 0.03);
    QCOMPARE(rig.r->liveGlObjects(), live); // same objects, none leaked
    for (int i = 0; i < 2; ++i) {           // and again on the same context
      rig.r->resetGlResources();
      settledFrame(rig.r);
      QCOMPARE(rig.r->liveGlObjects(), live);
    }
    QCOMPARE(rig.r->glErrorCount(), 0);
  }
};

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  TstVegetationRender t;
  return QTest::qExec(&t, argc, argv);
}
#include "tst_vegetation_render.moc"
