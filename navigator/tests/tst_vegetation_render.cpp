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
    // The atlas must be able to hold every azimuth of at least one variant.
    QVERIFY(vsim::kVegAtlasTiles * vsim::kVegAtlasTiles >= vsim::kVegAtlasAzimuths);
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
