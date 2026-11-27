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
#include "TerrainTextures.h"
#include "VegetationRenderer.h"
#include "procgen/EdgeTrack.h"
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
#include <numeric>
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
    r->setWorldChunk(d.key, d.positions, d.normals, d.colors, d.splitColors, d.edge, d.edgeColors);
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

// Share of pixels that differ at all (any channel) between two frames.
double anyDiffFraction(const QImage &a, const QImage &b) {
  const QImage x = a.convertToFormat(QImage::Format_RGB32);
  const QImage y = b.convertToFormat(QImage::Format_RGB32);
  if (x.size() != y.size())
    return 1.0;
  long n = 0;
  for (int j = 0; j < x.height(); ++j)
    for (int i = 0; i < x.width(); ++i)
      n += x.pixel(i, j) != y.pixel(i, j);
  return double(n) / double(x.width() * x.height());
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
    // The caster tests switch the shadow casters on. Every other test measures
    // the trees against the same frame without them: a cast shadow on the ground
    // is a change those counts would take for foliage.
    r->setVegCastersOffForTest(true);
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
constexpr double kFarStippleMax = 0.8; // pinholes + dots over tree pixels, far fade (dither 1.5 to 1.7, blend 0.1 to 0.5)
// `seed` moves a lone tree a little and turns it: the per-instance mirror and lean
// follow its position, so other seeds are other looks of the same kind.
World standWorld(int kind, int trees = kStandTrees, int variant = 0, int seed = 0) {
  World w = buildWorld(kPalForest, -1, 0);
  for (auto &v : w.veg)
    v.clear();
  for (size_t i = 0; i < w.cells.size(); ++i)
    if (w.cells[i].first == 0 && w.cells[i].second == 0)
      for (int n = 0; n < trees; ++n) {
        const float y = trees == 1 ? 0.37f * float(seed) : -30.0f + 20.0f * float(n);
        VegInstance t;
        t.species = kVegTree;
        t.kind = uint8_t(kind);
        t.variant = uint8_t(variant);
        t.scale = 1.0f;
        t.yaw = 0.3f + 1.3f * float(n) + 0.9f * float(seed);
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

// A pixel belongs to the trees when the frame differs from the same pose with
// vegetation off by more than this (sum of the channel differences). It has to
// be small: a dark crown against the lit ground differs by only a few levels,
// so a bigger cut makes the coverage depend on the crown's brightness (a dimmer
// but identical crown would "lose" pixels). TVR_DIFF overrides it for studies.
int coverDiff() {
  static const int d = qEnvironmentVariableIsSet("TVR_DIFF") ? qEnvironmentVariableIntValue("TVR_DIFF") : 8;
  return d;
}
inline bool treeDiffers(QRgb a, QRgb b) {
  return std::abs(qRed(a) - qRed(b)) + std::abs(qGreen(a) - qGreen(b)) + std::abs(qBlue(a) - qBlue(b)) > coverDiff();
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
      if (!treeDiffers(a, b))
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
      cov[size_t(y) * size_t(W) + size_t(x)] = treeDiffers(a, b) ? 1 : 0;
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

// The outline of what the trees add to a frame: per image row, the span between
// the leftmost and rightmost pixel that has a covered neighbour on its row (a
// stray dot does not count).
struct Outline {
  int top = -1, bottom = -1; // first and last covered row
  std::vector<int> span;     // per row
  int height() const { return top < 0 ? 0 : bottom - top + 1; }
  int maxSpan() const { return span.empty() ? 0 : *std::max_element(span.begin(), span.end()); }
  // The widest row of band `i` of `n` equal bands from the top (an envelope, so
  // the gaps between a pine's tiers do not read as a neck).
  int band(int i, int n) const {
    if (top < 0) return 0;
    const int h = height();
    int m = 0;
    for (int y = top + i * h / n; y < top + (i + 1) * h / n; ++y)
      m = std::max(m, span[size_t(y)]);
    return m;
  }
};

Outline outlineOf(const QImage &onIn, const QImage &offIn) {
  const QImage on = onIn.convertToFormat(QImage::Format_RGB32);
  const QImage off = offIn.convertToFormat(QImage::Format_RGB32);
  const int W = on.width(), H = on.height();
  std::vector<unsigned char> cov(size_t(W) * size_t(H), 0), tmp(cov.size(), 0);
  for (int y = 0; y < H; ++y)
    for (int x = 0; x < W; ++x) {
      const QRgb a = on.pixel(x, y), b = off.pixel(x, y);
      cov[size_t(y) * size_t(W) + size_t(x)] = treeDiffers(a, b) ? 1 : 0;
    }
  // A closing (dilate, then erode, 5x5): the sparse needle cards and the
  // impostor's holes fill in, the silhouette stays.
  auto pass = [&](const std::vector<unsigned char> &in, std::vector<unsigned char> &out, bool dilate) {
    for (int y = 0; y < H; ++y)
      for (int x = 0; x < W; ++x) {
        bool any = false, all = true;
        for (int dy = -2; dy <= 2; ++dy)
          for (int dx = -2; dx <= 2; ++dx) {
            const int xx = x + dx, yy = y + dy;
            const bool v = xx >= 0 && yy >= 0 && xx < W && yy < H && in[size_t(yy) * size_t(W) + size_t(xx)];
            any = any || v;
            all = all && v;
          }
        out[size_t(y) * size_t(W) + size_t(x)] = (dilate ? any : all) ? 1 : 0;
      }
  };
  pass(cov, tmp, true);
  pass(tmp, cov, false);
  Outline o;
  o.span.assign(size_t(H), 0);
  for (int y = 0; y < H; ++y) {
    int lo = -1, hi = -1, n = 0;
    for (int x = 0; x < W; ++x)
      if (cov[size_t(y) * size_t(W) + size_t(x)]) {
        if (lo < 0) lo = x;
        hi = x;
        ++n;
      }
    if (n >= 3) {
      o.span[size_t(y)] = hi - lo + 1;
      if (o.top < 0) o.top = y;
      o.bottom = y;
    }
  }
  return o;
}

Outline soloOutline(SimRendererWidget *r, const World &w, float d, float elev, int lod) {
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
  return outlineOf(on, off);
}

// One object (a tree or a rock) alone at (40, 0), the camera 7 m before it at
// eye height. Returns how much the frame with vegetation on is darker than the
// same pose with it off at ground point (40 - ahead, 0), as on / off luma over a
// small window: below 1 where the contact blob darkens the ground. The tree
// itself covers none of the probed pixels (they are nearer the camera than its
// foot), and the sun shadow map and the terrain are the same on and off, as
// vegetation never casts.
constexpr float kProbeCamBackM = 7.0f, kProbeEyeM = 1.7f, kProbeDownRad = 0.22f;

double windowLuma(const QImage &im, int cx, int cy) {
  double sum = 0;
  int n = 0;
  for (int y = cy - 2; y <= cy + 2; ++y)
    for (int x = cx - 2; x <= cx + 2; ++x) {
      if (x < 0 || y < 0 || x >= im.width() || y >= im.height())
        continue;
      const QRgb c = im.pixel(x, y);
      sum += 0.299 * qRed(c) + 0.587 * qGreen(c) + 0.114 * qBlue(c);
      ++n;
    }
  return n ? sum / n : 0.0;
}

// Screen position of the point `ahead` metres before the object, at the height
// of its foot (the blob is a flat disc there; from a grazing camera the slope
// under it would move the probe off the disc by more than a few pixels).
QPoint probePixel(SimRendererWidget *r, const World &w, float ahead) {
  const float g = surfaceHeight(*w.field, 40.0f, 0.0f, kChunkM, kRes);
  const QPointF p = r->worldToPixelForTest(QVector3D(40.0f - ahead, 0.0f, -g));
  return QPoint(int(std::lround(p.x())), int(std::lround(p.y())));
}

void aimProbe(SimRendererWidget *r, const World &w) {
  aimCamera(r, w, 40.0f - kProbeCamBackM, 0.0f, kProbeEyeM, 0.0f, kProbeDownRad);
}

World loneWorld(uint8_t species, int kind) {
  World w = standWorld(0, 0);
  for (size_t i = 0; i < w.cells.size(); ++i)
    if (w.cells[i].first == 0 && w.cells[i].second == 0) {
      VegInstance t;
      t.species = species;
      t.kind = uint8_t(kind);
      t.scale = 1.0f;
      t.yaw = 0.3f;
      t.tint = 0.5f;
      t.pos = {40.0f, 0.0f, -surfaceHeight(*w.field, 40.0f, 0.0f, kChunkM, kRes)};
      w.veg[i].push_back(t);
    }
  return w;
}

// Trees and rocks as shadow casters: a lone tree `distM` ahead of a raised
// camera with the sun in the north (behind the tree), so its shadow falls on the
// ground toward the camera.
QPoint casterGroundPx(SimRendererWidget *r, const World &w, float x, float y) {
  const float g = surfaceHeight(*w.field, x, y, kChunkM, kRes);
  const QPointF p = r->worldToPixelForTest(QVector3D(x, y, -g));
  return QPoint(int(std::lround(p.x())), int(std::lround(p.y())));
}
// High shadows, the sun in the north, the camera 14 m up and distM before x = 40.
void casterScene(SimRendererWidget *r, const World &w, float distM, float sunEl = 30.0f,
                 float down = 0.2f) {
  r->setSun(sunEl, 0.0f);
  r->setTerrainQuality(vsim::terrain::Quality::High);
  r->setShadowQuality(ShadowQuality::High);
  aimCamera(r, w, 40.0f - distM, 0.0f, 14.0f, 0.0f, down);
}

} // namespace

namespace {
// ---- Parcel-border tracks are drawn per pixel (EdgeTrack.h) -----------------

  // A point on a tracked border of the hills palette (flat ground, track at full
  // strength) and the unit direction the border runs in, found from the signed
  // distance's gradient.
  struct BorderPoint { float x = 0, y = 0, tx = 1, ty = 0; bool ok = false; };
// The tracked border with the longest straight run, picked by geometry alone (never by how
  // well the test's own measurement comes out on it): every clean candidate is walked along its
  // tangent in 3 m steps up to 45 m each way while it stays a tracked line, and the longest
  // run wins (the first of equals). A run of at least 12 m (4 steps each way, as before) is required.
BorderPoint findBorder(const TerrainField &f, float x0, float x1, float y0, float y1) {
    const float step = kChunkM / float(kRes);
    auto at = [&](float x, float y) {
      const float h = f.height(x, y);
      const PgVec3 n = f.normal(x, y, step);
      return f.colorAtSplit(x, y, h, n.z < 0.0f ? -n.z : 0.0f);
    };
    auto tracked = [&](float x, float y) {
      const auto q = at(x, y);
      return q.edgeA < 0.35f && q.edgeK > 0.9f;
    };
    BorderPoint b;
    int bestRun = 0;
    for (int iy = 0; float(iy) * 2.0f < y1 - y0; ++iy)
      for (int ix = 0; float(ix) * 0.5f < x1 - x0; ++ix) {
        const float x = x0 + float(ix) * 0.5f, y = y0 + float(iy) * 2.0f;
        const auto s = at(x, y);
        if (s.edgeA > 0.05f || s.edgeK < 0.95f)
          continue;
        const float e = 0.5f;
        const float gx = (at(x + e, y).edgeS - at(x - e, y).edgeS) / (2 * e);
        const float gy = (at(x, y + e).edgeS - at(x, y - e).edgeS) / (2 * e);
        const float gl = std::hypot(gx, gy);
        if (gl < 0.5f || gl > 2.0f)
          continue; // not a clean single border (corner or jump)
        const float tx = -gy / gl, ty = gx / gl;
        int run = 0; // steps of 3 m kept in both directions
        for (int k = 1; k <= 15; ++k) {
          const float t = float(k) * 3.0f;
          if (!tracked(x + tx * t, y + ty * t) || !tracked(x - tx * t, y - ty * t))
            break;
          run = k;
        }
        if (run >= 4 && run > bestRun) {
          bestRun = run;
          b = {x, y, tx, ty, true};
        }
      }
    return b;
  }

  // Left and right edge of the dirt band down the middle of a frame whose camera
  // looks along the border: per row where redness (r - g) crosses the middle
  // between the band and the grass at the image sides. Returns the RMS of each
  // edge about its best smooth (quadratic) curve (pixels, averaged) and the mean 25-75 %
  // transition width.
  struct BandEdges { double rms = 0, width = 0; int rows = 0; };
BandEdges bandEdges(const QImage &imgIn) {
    const QImage img = imgIn.convertToFormat(QImage::Format_RGB32);
    const int W = img.width();
    BandEdges out;
    std::vector<double> ys, xl, xr, wl;
    for (int y = 220; y < 440; ++y) {
      std::vector<double> v(size_t(W), 0.0);
      for (int x = 3; x < W - 3; ++x) {
        double a = 0;
        for (int k = -3; k <= 3; ++k) {
          const QRgb p = img.pixel(x + k, y);
          a += qRed(p) - qGreen(p);
        }
        v[size_t(x)] = a / 7.0;
      }
      auto mean = [&](int a, int b) { double s = 0; for (int x = a; x < b; ++x) s += v[size_t(x)]; return s / (b - a); };
      const double band = mean(W / 2 - 6, W / 2 + 6), grass = 0.5 * (mean(20, 120) + mean(W - 120, W - 20));
      if (band - grass < 14.0)
        continue; // no band in this row
      auto cross = [&](double lvl, int dir) { // subpixel, from the middle outward
        for (int x = W / 2; x > 3 && x < W - 4; x += dir)
          if (v[size_t(x)] < lvl) {
            const double v0 = v[size_t(x - dir)], v1 = v[size_t(x)];
            return double(x - dir) + dir * (v0 - lvl) / (v0 - v1);
          }
        return -1.0;
      };
      const double l50 = cross(grass + 0.5 * (band - grass), -1), r50 = cross(grass + 0.5 * (band - grass), +1);
      const double l25 = cross(grass + 0.25 * (band - grass), -1), l75 = cross(grass + 0.75 * (band - grass), -1);
      if (l50 < 0 || r50 < 0 || l25 < 0 || l75 < 0)
        continue;
      ys.push_back(y); xl.push_back(l50); xr.push_back(r50); wl.push_back(std::fabs(l25 - l75));
    }
    // RMS of an edge about its best-fit quadratic in the row. The parcel borders curve on
    // purpose (a two-scale warp), so a straight line would count the intended bend as
    // jaggedness in both samplings; what is measured here is the departure from a smooth
    // curve. Normal equations on the row centred at its middle, solved by Cramer's rule.
    auto rmsSmooth = [&](const std::vector<double> &x) {
      const size_t n = ys.size();
      const double y0 = 0.5 * (ys.front() + ys.back()), sc = 0.5 * (ys.back() - ys.front());
      double s1 = 0, s2 = 0, s3 = 0, s4 = 0, t0 = 0, t1 = 0, t2 = 0;
      for (size_t i = 0; i < n; ++i) {
        const double u = (ys[i] - y0) / sc, u2 = u * u;
        s1 += u; s2 += u2; s3 += u2 * u; s4 += u2 * u2;
        t0 += x[i]; t1 += u * x[i]; t2 += u2 * x[i];
      }
      const double N = double(n);
      auto det3 = [](double a11, double a12, double a13, double a21, double a22, double a23,
                     double a31, double a32, double a33) {
        return a11 * (a22 * a33 - a23 * a32) - a12 * (a21 * a33 - a23 * a31) + a13 * (a21 * a32 - a22 * a31);
      };
      const double D = det3(N, s1, s2, s1, s2, s3, s2, s3, s4);
      double c0 = 0, c1 = 0, c2 = 0;
      if (D != 0) {
        c0 = det3(t0, s1, s2, t1, s2, s3, t2, s3, s4) / D;
        c1 = det3(N, t0, s2, s1, t1, s3, s2, t2, s4) / D;
        c2 = det3(N, s1, t0, s1, s2, t1, s2, s3, t2) / D;
      }
      double r = 0;
      for (size_t i = 0; i < n; ++i) {
        const double u = (ys[i] - y0) / sc, e = x[i] - (c0 + c1 * u + c2 * u * u);
        r += e * e;
      }
      return std::sqrt(r / N);
    };
    out.rows = int(ys.size());
    if (out.rows < 40)
      return out;
    out.rms = 0.5 * (rmsSmooth(xl) + rmsSmooth(xr));
    out.width = std::accumulate(wl.begin(), wl.end(), 0.0) / double(wl.size());
    return out;
  }

// The same ground twice, as the renderer gets it: the new mesh data (baked
// colour plus the track-free colour and track distances) and the old one (the
// baked colour only, which is what the mesher gave before).
void loadHillsTerrain(SimRendererWidget *r, const World &w, bool split) {
  for (size_t i = 0; i < w.keys.size(); ++i) {
    const ProcMesh m = meshFieldChunk(*w.field, w.cells[i].first, w.cells[i].second, kChunkM, kRes);
    const ChunkMeshData d = toChunkMeshData(w.keys[i], m);
    if (split)
      r->setWorldChunk(d.key, d.positions, d.normals, d.colors, d.splitColors, d.edge, d.edgeColors);
    else
      r->setWorldChunk(d.key, d.positions, d.normals, d.colors);
  }
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

  // A pine's coarser mesh LODs keep its crown: from 40 to 80 m they show about
  // what the full mesh shows (ratios of renders on one rasteriser, so it holds
  // on any of them). Level with the crowns LOD1 drew 0.71 of LOD0 before its
  // low branches and card segments were restored, which read as a bare pole.
  void pineLod1KeepsItsFoliage() {
    World w = standWorld(1);
    Rig rig(true);
    loadTerrain(rig.r, w);
    loadVegetation(rig.r, w);
    QVERIFY(rig.show());
    QString failures;
    for (float d : {40.0f, 60.0f, 80.0f})
      for (float e : {0.0f, 20.0f}) {
        const TreePixels l0 = standView(rig.r, w, d, e, 0);
        const TreePixels l1 = standView(rig.r, w, d, e, 1);
        const TreePixels l2 = standView(rig.r, w, d, e, 2);
        const double ratio = l0.foliage ? double(l1.foliage) / l0.foliage : 0.0;
        const double ratio2 = l0.foliage ? double(l2.foliage) / l0.foliage : 0.0;
        qInfo("pine LOD1 %.0f m %.0f deg: LOD0 %d px, LOD1 %d px (x%.3f), LOD2 %d px (x%.3f)", double(d), double(e), l0.foliage,
              l1.foliage, ratio, l2.foliage, ratio2);
        if (l0.foliage < 150)
          failures += QString("[reference too small at %1 m] ").arg(double(d));
        else if (ratio < 0.85 || ratio > 1.15)
          failures += QString("[%1 m %2 deg: LOD1 x%3] ").arg(double(d)).arg(double(e)).arg(ratio, 0, 'f', 2);
        else if (ratio2 < 0.62 || ratio2 > 1.15)
          failures += QString("[%1 m %2 deg: LOD2 x%3] ").arg(double(d)).arg(double(e)).arg(ratio2, 0, 'f', 2);
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

  // The far fade-out (the last 14% before the billboard range ends, from about
  // 480 m) is an alpha blend, not a one-pixel screen door: the dither read as a
  // checkerboard on the tree line. At 520 m the fade is about one half, which the
  // dither turned into the most alternating pattern (bayer at 0.5); the blend
  // leaves the crown's own texture and outline only. Low terrain draws the
  // same billboards.
  void farFadeOutIsBlendedNotDithered_data() {
    QTest::addColumn<int>("kind");
    QTest::addColumn<bool>("high");
    QTest::newRow("fir High") << 0 << true;
    QTest::newRow("jacaranda High") << 2 << true;
    QTest::newRow("jacaranda Low") << 2 << false;
  }
  void farFadeOutIsBlendedNotDithered() {
    QFETCH(int, kind);
    QFETCH(bool, high);
    World w = standWorld(kind);
    Rig rig(true);
    QVERIFY(rig.show());
    rig.r->setTerrainQuality(high ? vsim::terrain::Quality::High : vsim::terrain::Quality::Low);
    loadTerrain(rig.r, w);
    loadVegetation(rig.r, w);
    standView(rig.r, w, 160.0f, 0.0f, -1); // the atlas bakes from a nearer view
    const TreePixels t = standView(rig.r, w, 520.0f, 0.0f, -1);
    if (rig.r->vegetationStats().impostorVariants == 0)
      QSKIP("no impostor atlas on this context");
    qInfo("kind %d %s, 520 m: tree px %d, stipple %.3f", kind, high ? "High" : "Low", t.all, t.stipple());
    QVERIFY2(t.all > 30, qPrintable(QString("trees not found (%1 px)").arg(t.all)));
    QVERIFY2(t.stipple() < kFarStippleMax, qPrintable(QString("stipple %1").arg(t.stipple())));
    QCOMPARE(rig.r->glErrorCount(), 0);
  }

  // Two variants in the far fade-out: every variant's opaque billboard pass is
  // drawn before any blended far pass. Per variant the far pass would let a later
  // variant's opaque billboard (depth write on, the faded quad wrote none) paint
  // over an earlier variant's faded one that is in front of it. The draw order is
  // read from the renderer's stats, and the frame must still show the trees.
  void farFadePassesRunAfterEveryOpaquePass() {
    World w = standWorld(0, 2);
    for (size_t i = 0; i < w.cells.size(); ++i)
      if (w.cells[i].first == 0 && w.cells[i].second == 0)
        for (int n = 0; n < 2; ++n) {
          VegInstance t;
          t.species = kVegTree;
          t.kind = 2;
          t.scale = 1.0f;
          t.yaw = 0.7f + float(n);
          t.tint = 0.5f;
          const float y = -10.0f + 20.0f * float(n);
          t.pos = {40.0f, y, -surfaceHeight(*w.field, 40.0f, y, kChunkM, kRes)};
          w.veg[i].push_back(t);
        }
    Rig rig(true);
    QVERIFY(rig.show());
    loadTerrain(rig.r, w);
    loadVegetation(rig.r, w);
    standView(rig.r, w, 160.0f, 0.0f, -1); // the atlas bakes from a nearer view
    const TreePixels t = standView(rig.r, w, 520.0f, 0.0f, -1);
    if (rig.r->vegetationStats().impostorVariants < 2)
      QSKIP("no impostor atlas on this context");
    const std::string passes = rig.r->vegetationStats().impostorPasses;
    qInfo("far fade passes '%s', tree px %d", passes.c_str(), t.all);
    const size_t firstFar = passes.find('f');
    QVERIFY2(firstFar != std::string::npos && passes.rfind('f') > firstFar,
             "two variants should each have a far fade-out pass");
    QVERIFY2(passes.find('o', firstFar) == std::string::npos, passes.c_str());
    QVERIFY2(t.all > 60, qPrintable(QString("trees not found (%1 px)").arg(t.all)));
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

  // A lone conifer keeps its shape from above: the impostor's outline against
  // the same tree drawn as a mesh at the same pose. A top tile that replaces the
  // side tiles turns the crown into a flat saucer (too wide), loses the spire and
  // trunk (too short) and leaves a neck under the saucer (too thin). Ratios of two
  // renders at the same resolution, so they hold across rasterisers.
  void impostorKeepsTheTreesShapeFromAbove_data() {
    QTest::addColumn<int>("kind");
    QTest::addColumn<float>("dist");
    QTest::addColumn<float>("elev");
    QTest::addColumn<int>("variant");
    QTest::addColumn<int>("seed");
    for (int kind : {0, 1})
      for (int var : {0, 2}) // variant 1 is the same shape as 0, and a fir has one
        if (kind == 1 || var == 0)
          for (float d : {100.0f, 160.0f})
            for (float e : {10.0f, 20.0f, 30.0f, 45.0f, 70.0f, 85.0f})
              for (int seed : {0, 3}) // another position and heading: the mirror state changes
                if (seed == 0 || e > 40.0f)
                  QTest::newRow(qPrintable(QString("%1 v%2 %3 m %4 deg s%5").arg(kind ? "pine" : "fir").arg(var).arg(double(d)).arg(double(e)).arg(seed)))
                      << kind << d << e << var << seed;
  }
  void impostorKeepsTheTreesShapeFromAbove() {
    QFETCH(int, kind);
    QFETCH(float, dist);
    QFETCH(float, elev);
    QFETCH(int, variant);
    QFETCH(int, seed);
    World w = standWorld(kind, 1, variant, seed);
    Rig rig(true);
    loadTerrain(rig.r, w);
    loadVegetation(rig.r, w);
    QVERIFY(rig.show());
    // The impostor: forced at 100 m, where the mesh would be drawn; natural at 160 m.
    const Outline imp = soloOutline(rig.r, w, dist, elev, dist < 135.0f ? 3 : -1);
    if (rig.r->vegetationStats().impostorVariants == 0)
      QSKIP("no impostor atlas on this context");
    const Outline ref = soloOutline(rig.r, w, dist, elev, 1);
    QVERIFY2(ref.height() > 20 && imp.height() > 0, "tree not found");
    const double hRatio = double(imp.height()) / double(ref.height());
    const double wRatio = double(imp.maxSpan()) / double(std::max(1, ref.maxSpan()));
    // Six bands from the top, widest row of each (imp/ref).
    QString rows;
    int sumI = 0, sumR = 0;
    double waist = 9.0, over = 0.0;
    for (int i = 0; i < 6; ++i) {
      const int bi = imp.band(i, 6), br = ref.band(i, 6);
      rows += QString(" %1/%2").arg(bi).arg(br);
      sumI += bi;
      sumR += br;
      if (i >= 1 && i <= 3) { // below the tip, the crown's upper half
        waist = std::min(waist, double(bi) / double(std::max(1, br)));
        over = std::max(over, double(bi) / double(std::max(1, br)));
      }
    }
    const double area = double(sumI) / double(std::max(1, sumR));
    qInfo("SHAPE %s v%d %.0f m %.0f deg: height %d/%d x%.2f, widest %d/%d x%.2f, area x%.2f, neck x%.2f, upper bulge x%.2f; bands top to bottom imp/ref:%s",
          kind ? "pine" : "fir", variant, double(dist), double(elev), imp.height(), ref.height(), hRatio, imp.maxSpan(),
          ref.maxSpan(), wRatio, area, waist, over, qPrintable(rows));
    // Bounds sit between the current shader and two earlier ones: the top tile
    // replacing the side tiles in a disc (bulges and fills the crown out at 20
    // and 30 degrees), and its successor that faded the side tiles out under the
    // disc (at 45 degrees a fir became a flat-topped ball, height x0.58 of the
    // mesh, with no spire). A pine's impostor is 15 to 30 percent shorter than
    // its mesh even without the top tile (its thin trunk and tip fall below the
    // coverage threshold), so its height bound is looser than the fir's.
    const bool high = elev >= 40.0f;
    const double minHeight = kind == 0 ? 0.75 : 0.62;
    QVERIFY2(hRatio >= minHeight, qPrintable(QString("spire or trunk lost: height x%1").arg(hRatio)));
    QVERIFY2(wRatio <= 1.25, qPrintable(QString("flat saucer: widest row x%1").arg(wRatio)));
    QVERIFY2(area <= (high ? 1.25 : 1.4), qPrintable(QString("crown filled out: area x%1").arg(area)));
    // Straight down the top tile is the whole crown, a disc: the cone's outline would cut
    // it to a wedge (area x0.77-0.91 and widest row x0.81-0.94 of the mesh's at 70 and
    // 85 degrees), so the clip is gone by 70 degrees and the impostor keeps its width.
    if (elev >= 60.0f) {
      QVERIFY2(area >= 0.95, qPrintable(QString("crown clipped to a wedge: area x%1").arg(area)));
      QVERIFY2(wRatio >= 0.97, qPrintable(QString("crown clipped to a wedge: widest row x%1").arg(wRatio)));
    }
    if (elev >= 20.0f)
      QVERIFY2(waist >= 0.5, qPrintable(QString("neck under the crown: x%1").arg(waist)));
    // A cone seen from 45 degrees stays a cone: the top tile's disc, as wide as the
    // base, used to make a ball on the spire (upper bands x1.27 to x1.42 of the mesh's
    // for a fir at 100 m; the cone mask in the top tile brings them to x1.17 at most).
    if (high && elev < 60.0f && dist < 135.0f)
      QVERIFY2(over <= 1.22, qPrintable(QString("ball on the spire: upper bands x%1").arg(over)));
    if (elev >= 20.0f && !high)
      QVERIFY2(over <= 2.2, qPrintable(QString("bulb on the crown: upper bands x%1").arg(over)));
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
  // The impostor must be about as bright as the mesh it replaces. A lone tree on
  // bare ground, the mesh forced to LOD1 and then the impostor, mean luminance of
  // the crown pixels (green over red in the mesh view) both cover. The mesh
  // lights a tree differently when instVary mirrors it (the winding flips, so
  // gl_FrontFacing inverts and the whole shading normal turns over), and the
  // impostor follows that per instance: x = 40 hashes to a mirrored tree and
  // x = 43 to an unmirrored one (fract(sin(dot(p, (12.9898, 78.233))) * 43758.5453)
  // is 0.60 and 0.22; mirrored above 0.5). Fir and pine are asserted to a band
  // (mean over 0 and 45 degrees, three suns); jacaranda and island are measured
  // and printed only: island stays 1.0-1.5 and jacaranda reaches 1.5 behind-the-
  // crown at sun 25 (see the commit message).
  void impostorLuminanceMatchesMesh_data() {
    QTest::addColumn<int>("kind");
    QTest::addColumn<bool>("mirrored");
    for (int k = 0; k < 4; ++k)
      for (bool m : {true, false})
        QTest::newRow(qPrintable(QString("%1 %2").arg(k == 0 ? "fir" : k == 1 ? "pine" : k == 2 ? "jacaranda" : "island").arg(m ? "mirrored" : "unmirrored")))
            << k << m;
  }
  void impostorLuminanceMatchesMesh() {
    QFETCH(int, kind);
    QFETCH(bool, mirrored);
    const float px = mirrored ? 40.0f : 43.0f;
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
        t.pos = {px, 0.0f, -surfaceHeight(*w.field, px, 0.0f, kChunkM, kRes)};
        w.veg[i].push_back(t);
      }
    Rig rig(true);
    loadTerrain(rig.r, w);
    loadVegetation(rig.r, w);
    QVERIFY(rig.show());
    QString failures;
    for (const auto sunp : {std::pair<float, float>{46.0f, 37.0f}, {25.0f, 200.0f}, {70.0f, 300.0f}}) {
      double sum = 0;
      int rows = 0;
      for (float e : {0.0f, 45.0f}) {
        rig.r->setSun(sunp.first, sunp.second);
        const float er = e * 3.14159f / 180.0f;
        aimCamera(rig.r, w, px + 50.0f * std::cos(er), 0.0f, 6.0f + 50.0f * std::sin(er), 3.14159f, er);
        rig.r->setVegetationEnabled(false);
        const QImage off = rig.r->grabFrame().convertToFormat(QImage::Format_RGB32);
        rig.r->setVegetationEnabled(true);
        QImage img[4];
        for (int lod : {1, 3}) {
          rig.r->setVegetationForcedLod(lod);
          QImage on = settledFrame(rig.r);
          for (int i = 0; i < 40 && lod == 3 && rig.r->vegetationStats().impostors == 0; ++i)
            on = rig.r->grabFrame();
          img[lod] = on.convertToFormat(QImage::Format_RGB32);
        }
        rig.r->setVegetationForcedLod(-1);
        double sm = 0, si = 0;
        int n = 0;
        for (int y = 0; y < off.height(); ++y)
          for (int x = 0; x < off.width(); ++x) {
            const QRgb o = off.pixel(x, y), a = img[1].pixel(x, y), b = img[3].pixel(x, y);
            if (!treeDiffers(a, o) || !treeDiffers(b, o) || qGreen(a) <= qRed(a) + 4)
              continue;
            auto lum = [](QRgb c) { return 0.299 * qRed(c) + 0.587 * qGreen(c) + 0.114 * qBlue(c); };
            ++n;
            sm += lum(a);
            si += lum(b);
          }
        QVERIFY2(n > 150 && sm > 0, "crown pixels not found");
        const double r = si / sm;
        qInfo("LUMBAND kind %d %s sun %.0f/%.0f %.0f deg: impostor/mesh crown luminance x%.2f (%d px)", kind, mirrored ? "mirrored" : "unmirrored",
              double(sunp.first), double(sunp.second), double(e), r, n);
        sum += r;
        ++rows;
      }
      const double mean = sum / rows;
      if (kind <= 1 && (mean < 0.75 || mean > 1.25))
        failures += QString("[sun %1/%2: x%3] ").arg(double(sunp.first)).arg(double(sunp.second)).arg(mean, 0, 'f', 2);
    }
    QVERIFY2(failures.isEmpty(), qPrintable("impostor crown luminance vs mesh, outside 0.75..1.25: " + failures));
  }

  void borderTrackIsSmoothAndCrisp_data() {
    QTest::addColumn<float>("aboveM");
    QTest::addColumn<float>("downRad");
    QTest::addColumn<bool>("high");
    QTest::newRow("High h6") << 6.0f << 0.8f << true;
    QTest::newRow("High h10") << 10.0f << 0.9f << true;
  }
  void borderTrackIsSmoothAndCrisp() {
    QFETCH(float, aboveM);
    QFETCH(float, downRad);
    QFETCH(bool, high);
    Rig rig;
    QVERIFY(rig.show());
    rig.r->setTerrainQuality(high ? vsim::terrain::Quality::High : vsim::terrain::Quality::Low);
    QVERIFY(high); // Low keeps the baked track: see lowKeepsTheBakedBorderTracks
    rig.r->setVegetationEnabled(false);
    World w;
    FieldParams fp;
    fp.seed = 11u;
    fp.palette = kPalHills;
    fp.heightM = 30.0f;
    w.field = std::make_unique<TerrainField>(fp);
    for (int cy = -1; cy <= 1; ++cy)
      for (int cx = -1; cx <= 1; ++cx) {
        w.keys.push_back(ChunkStreamer::keyOf(cx, cy));
        w.cells.push_back({cx, cy});
      }
    const BorderPoint b = findBorder(*w.field, -170.0f, 170.0f, -170.0f, 170.0f);
    QVERIFY2(b.ok, "no straight tracked border in the hills world");
    const float D = aboveM / std::tan(downRad);
    const float heading = std::atan2(b.ty, b.tx);
    BandEdges res[2];
    for (int split = 0; split < 2; ++split) {
      rig.r->clearWorldChunks();
      loadHillsTerrain(rig.r, w, split == 1);
      aimCamera(rig.r, w, b.x - b.tx * D, b.y - b.ty * D, aboveM, heading, downRad);
      res[split] = bandEdges(settledFrame(rig.r, 3));
    }
    qInfo("ZZ border track at (%.1f,%.1f): old rms %.2f px width %.2f px (%d rows) | new rms %.2f px width %.2f px (%d rows)",
          double(b.x), double(b.y), res[0].rms, res[0].width, res[0].rows, res[1].rms, res[1].width, res[1].rows);
    // The band must exist in both frames (a missing track would measure as 0),
    // and drawn per pixel it is much straighter and its edge much sharper than
    // the node-sampled one: measured about 1.8x (h6) to 2.4x (h10) on the smooth-curve rms
    // and 2.4x on width at High.
    QVERIFY2(res[0].rows >= 40 && res[1].rows >= 40, "no dirt band in the frame");
    const QByteArray where = QByteArray("border at (") + QByteArray::number(double(b.x), 'f', 1) + ", " +
                             QByteArray::number(double(b.y), 'f', 1) + "): rms old " +
                             QByteArray::number(res[0].rms, 'f', 2) + " px, new " +
                             QByteArray::number(res[1].rms, 'f', 2) + " px; width old " +
                             QByteArray::number(res[0].width, 'f', 1) + " px, new " +
                             QByteArray::number(res[1].width, 'f', 1) + " px";
    QVERIFY2(res[1].rms < 0.6 * res[0].rms, where.constData());
    QVERIFY2(res[1].rms < 3.5, where.constData());
    QVERIFY2(res[1].width < 0.75 * res[0].width, where.constData());
    QCOMPARE(rig.r->glErrorCount(), 0);
  }

  // Low keeps the old baked border tracks: the chunk's node colour is exactly
  // colorAt() (what the mesher gave before), and with the new per-pixel data
  // present the frame equals the one drawn from that colour alone, pixel for
  // pixel. A live switch to High and back changes nothing at Low.
  void lowKeepsTheBakedBorderTracks() {
    Rig rig;
    QVERIFY(rig.show());
    rig.r->setTerrainQuality(vsim::terrain::Quality::Low);
    rig.r->setVegetationEnabled(false);
    World w;
    FieldParams fp;
    fp.seed = 11u;
    fp.palette = kPalHills;
    fp.heightM = 30.0f;
    w.field = std::make_unique<TerrainField>(fp);
    for (int cy = -1; cy <= 1; ++cy)
      for (int cx = -1; cx <= 1; ++cx) {
        w.keys.push_back(ChunkStreamer::keyOf(cx, cy));
        w.cells.push_back({cx, cy});
      }
    const ProcMesh m = meshFieldChunk(*w.field, 0, 0, kChunkM, kRes);
    int differs = 0, trackNodes = 0;
    for (size_t k = 0; k < m.positions.size(); k += 7) {
      const PgVec3 p = m.positions[k], n = m.normals[k];
      const PgVec3 c = w.field->colorAt(p.x, p.y, -p.z, n.z < 0.0f ? -n.z : 0.0f).color;
      differs += (c.x != m.colors[k].x || c.y != m.colors[k].y || c.z != m.colors[k].z);
      trackNodes += (m.colors[k].x != m.splitColors[k].x || m.colors[k].y != m.splitColors[k].y);
    }
    QCOMPARE(differs, 0);
    QVERIFY2(trackNodes > 20, "the chunk must hold baked border tracks for this to mean anything");
    const float D = 10.0f / std::tan(0.6f);
    QImage f[2];
    for (int split = 0; split < 2; ++split) {
      rig.r->clearWorldChunks();
      loadHillsTerrain(rig.r, w, split == 1);
      aimCamera(rig.r, w, 8.0f - D, 12.0f, 10.0f, 0.0f, 0.6f);
      f[split] = settledFrame(rig.r, 3);
    }
    QCOMPARE(anyDiffFraction(f[0], f[1]), 0.0);
    // High with the same data differs (the track is drawn per pixel), and back
    // to Low is the baked frame again.
    rig.r->setTerrainQuality(vsim::terrain::Quality::High);
    const QImage hi = settledFrame(rig.r, 3);
    QVERIFY(anyDiffFraction(f[1], hi) > 0.0);
    rig.r->setTerrainQuality(vsim::terrain::Quality::Low);
    QCOMPARE(anyDiffFraction(f[1], settledFrame(rig.r, 3)), 0.0);
    QCOMPARE(rig.r->glErrorCount(), 0);
  }

  // A palette without parcels has no border tracks: the new mesh data must leave
  // its frame exactly as it was, at both qualities.
  void worldsWithoutParcelsRenderUnchanged_data() {
    QTest::addColumn<bool>("high");
    QTest::newRow("High") << true;
    QTest::newRow("Low") << false;
  }
  void worldsWithoutParcelsRenderUnchanged() {
    QFETCH(bool, high);
    Rig rig;
    QVERIFY(rig.show());
    rig.r->setTerrainQuality(high ? vsim::terrain::Quality::High : vsim::terrain::Quality::Low);
    rig.r->setVegetationEnabled(false);
    World w;
    FieldParams fp;
    fp.seed = 11u;
    fp.palette = kPalForest;
    fp.heightM = 40.0f;
    w.field = std::make_unique<TerrainField>(fp);
    for (int cy = -1; cy <= 1; ++cy)
      for (int cx = -1; cx <= 1; ++cx) {
        w.keys.push_back(ChunkStreamer::keyOf(cx, cy));
        w.cells.push_back({cx, cy});
      }
    QImage f[2];
    for (int split = 0; split < 2; ++split) {
      rig.r->clearWorldChunks();
      loadHillsTerrain(rig.r, w, split == 1);
      aimCamera(rig.r, w, -6.0f, 12.0f, 10.0f, 0.0f, 0.6f);
      f[split] = settledFrame(rig.r, 3);
    }
    QCOMPARE(changedFraction(f[0], f[1]), 0.0);
    QVERIFY(f[0].convertToFormat(QImage::Format_RGB32) == f[1].convertToFormat(QImage::Format_RGB32));
    QCOMPARE(rig.r->glErrorCount(), 0);
  }

  // The textured ground skips the grass photo where the dirt (or rock) weight
  // reaches 0.999, but the grass term is still blended in at 45% (15% on rock).
  // With no photo sample its brightness detail must be neutral; clamp(0) was
  // 0.35 and drew hard-edged dark slivers wherever the cover crossed the 0.999
  // contour (hills, High only). A flat chunk whose node colour steps across the
  // contour must read the same on both sides of it.
  void dirtWeightContourLeavesNoBrightnessStep() {
    Rig rig;
    QVERIFY(rig.show());
    rig.r->setTerrainQuality(vsim::terrain::Quality::High);
    rig.r->setVegetationEnabled(false);
    const World w = buildWorld(kPalFlat, -1, 1);
    const float cell = kChunkM / float(kRes);
    const float yc = 12.0f;
    const float yLo = std::floor(yc / cell) * cell, yHi = yLo + cell;
    // dirt weight 0.98 on the low side, exactly 1 on the high side (kDirtEdge1 is 0.10)
    const float g = 0.30f, b = 0.20f;
    const float rLow = vsim::terrain::kDirtG * g + 0.088f, rHigh = vsim::terrain::kDirtG * g + 0.108f;
    for (size_t i = 0; i < w.keys.size(); ++i) {
      const ProcMesh m = meshFieldChunk(*w.field, w.cells[i].first, w.cells[i].second, kChunkM, kRes);
      ChunkMeshData d = toChunkMeshData(w.keys[i], m);
      for (size_t k = 0; k < d.colors.size(); ++k)
        d.colors[k] = QVector3D(d.positions[k].y() <= yLo + 0.01f ? rLow : rHigh, g, b);
      rig.r->setWorldChunk(d.key, d.positions, d.normals, d.colors);
    }
    const float h = 12.0f, D = h / std::tan(0.6f);
    aimCamera(rig.r, w, 8.0f - D, yc, h, 0.0f, 0.6f);
    const QImage im = settledFrame(rig.r, 3);
    auto bandLuma = [&](float y) {
      const float gz = surfaceHeight(*w.field, 8.0f, y, kChunkM, kRes);
      const QPointF p = rig.r->worldToPixelForTest(QVector3D(8.0f, y, -gz));
      double sum = 0;
      int n = 0;
      for (int dy = -8; dy <= 8; ++dy)
        for (int dx = -8; dx <= 8; ++dx) {
          const int x = int(p.x()) + dx, yy = int(p.y()) + dy;
          if (x < 0 || yy < 0 || x >= im.width() || yy >= im.height()) continue;
          const QRgb c = im.pixel(x, yy);
          sum += 0.299 * qRed(c) + 0.587 * qGreen(c) + 0.114 * qBlue(c);
          ++n;
        }
      return n ? sum / n : 0.0;
    };
    const double lo = bandLuma(yLo - 2.0f), hi = bandLuma(yHi + 2.0f);
    qInfo("ZZ dirt contour: luma below %.2f above %.2f ratio %.4f", lo, hi, hi / lo);
    QVERIFY(lo > 20.0 && hi > 20.0);
    QVERIFY2(hi / lo > 0.93 && hi / lo < 1.07, "a brightness step across the dirt weight 0.999 contour");
    QCOMPARE(rig.r->glErrorCount(), 0);
  }

  // The GLSL track weight is generated from the constants of procgen/EdgeTrack.h
  // (as the splat rule is from TerrainMaterial.h), so it cannot drift from the
  // C++ rule the tests and the mesher use.
  void edgeTrackGlslIsGeneratedFromTheSharedConstants() {
    const QString g = QString::fromLatin1(vsim::terrain::terrainMaterialGLSL());
    auto f = [](float v) { return QString::number(double(v), 'f', 4); };
    const int at = g.indexOf("float edgeTrackWeight(vec2 trk)");
    QVERIFY(at >= 0);
    const QString fn = g.mid(at);
    QVERIFY(fn.contains("trk.y >= " + f(vsim::procgen::kEdgeValidMax)));
    QVERIFY(fn.contains(f(vsim::procgen::kEdgeTrackMax) + " * (1.0 - smoothstep(" + f(vsim::procgen::kEdgeTrackD0) +
                        ", " + f(vsim::procgen::kEdgeTrackD1) + ", abs(trk.x)))"));
  }

  // The survey grid is a close-ground hint: present 30 m from the eye, gone by
  // 60 m (its long straight lines read as seams from the air). Measured as the
  // pixels a frame changes between grid on and grid off, looking straight down
  // from 30 m and from 60 m over flat ground.
  void surveyGridIsGoneBeyondSixtyMetres_data() {
    QTest::addColumn<bool>("high");
    QTest::newRow("High") << true;
    QTest::newRow("Low") << false;
  }
  void surveyGridIsGoneBeyondSixtyMetres() {
    QFETCH(bool, high);
    Rig rig;
    QVERIFY(rig.show());
    rig.r->setTerrainQuality(high ? vsim::terrain::Quality::High : vsim::terrain::Quality::Low);
    rig.r->setVegetationEnabled(false);
    World w;
    FieldParams fp;
    fp.seed = 11u;
    fp.palette = kPalForest;
    fp.heightM = 3.0f; // flat: the grid only draws on level ground
    w.field = std::make_unique<TerrainField>(fp);
    for (int cy = -1; cy <= 1; ++cy)
      for (int cx = -1; cx <= 1; ++cx) {
        w.keys.push_back(ChunkStreamer::keyOf(cx, cy));
        w.cells.push_back({cx, cy});
      }
    loadTerrain(rig.r, w);
    double changed[3] = {};
    const float eye[3] = {20.0f, 30.0f, 60.0f};
    for (int i = 0; i < 3; ++i) {
      aimCamera(rig.r, w, 0.0f, 0.0f, eye[i], 0.0f, 1.5f);
      rig.r->setSurveyGrid(true);
      const QImage on = settledFrame(rig.r, 3);
      rig.r->setSurveyGrid(false);
      const QImage off = settledFrame(rig.r, 3);
      changed[i] = anyDiffFraction(on, off);
      if (i == 0)
        QVERIFY2(changed[0] > 0.0, "the grid must show at 20 m");
    }
    qInfo("ZZ grid pixels changed by the grid: 20 m %.5f, 30 m %.5f, 60 m %.5f", changed[0], changed[1], changed[2]);
    QVERIFY2(changed[1] > 0.0, "the grid lines are still there at 30 m");
    QCOMPARE(changed[2], 0.0);
    QCOMPARE(rig.r->glErrorCount(), 0);
  }

  // The survey grid is a View option, off by default and stored under
  // "sim/surveyGrid": a renderer made with nothing stored has it off and draws
  // no grid pixel (identical to an explicit off, at 20 m over flat ground where
  // the grid is strongest); the stored value is read back and applied.
  void surveyGridIsAnOffByDefaultSetting() {
    QStandardPaths::setTestModeEnabled(true);
    {
      QSettings s;
      s.remove("sim/surveyGrid");
    }
    QVERIFY(!SimRendererWidget::surveyGridByConfig());
    Rig rig;
    QVERIFY(rig.show());
    QVERIFY(!rig.r->surveyGrid());
    rig.r->setVegetationEnabled(false);
    World w;
    FieldParams fp;
    fp.seed = 11u;
    fp.palette = kPalForest;
    fp.heightM = 3.0f;
    w.field = std::make_unique<TerrainField>(fp);
    for (int cy = -1; cy <= 1; ++cy)
      for (int cx = -1; cx <= 1; ++cx) {
        w.keys.push_back(ChunkStreamer::keyOf(cx, cy));
        w.cells.push_back({cx, cy});
      }
    loadTerrain(rig.r, w);
    aimCamera(rig.r, w, 0.0f, 0.0f, 20.0f, 0.0f, 1.5f);
    const QImage byDefault = settledFrame(rig.r, 3);
    rig.r->setSurveyGrid(false);
    QCOMPARE(anyDiffFraction(byDefault, settledFrame(rig.r, 3)), 0.0);
    rig.r->setSurveyGrid(true);
    QVERIFY2(anyDiffFraction(byDefault, settledFrame(rig.r, 3)) > 0.01, "switching it on must show lines");
    rig.r->setSurveyGrid(false);
    QCOMPARE(anyDiffFraction(byDefault, settledFrame(rig.r, 3)), 0.0); // and off again: zero grid pixels
    // The setting round-trips and a renderer built after it follows it.
    {
      QSettings s;
      s.setValue("sim/surveyGrid", true);
    }
    QVERIFY(SimRendererWidget::surveyGridByConfig());
    {
      Rig on;
      QVERIFY(on.show());
      QVERIFY(on.r->surveyGrid());
    }
    {
      QSettings s;
      s.setValue("sim/surveyGrid", false);
    }
    QVERIFY(!SimRendererWidget::surveyGridByConfig());
    {
      QSettings s;
      s.remove("sim/surveyGrid");
    }
  }

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

  // A jacaranda's coarser mesh LODs keep its crown mass: from 50 to 130 m they
  // show about what the full mesh shows (ratios of renders on one rasteriser,
  // so it holds on any of them). LOD2 drew 0.57 to 0.65 of LOD0 before its
  // leaf budget was raised, against 1.25 for the impostor, so a tree handing
  // over between them doubled its foliage and the dithered cross-fade showed
  // a dot grid.
  void jacarandaLod2KeepsItsFoliage() {
    World w = standWorld(2);
    Rig rig(true);
    loadTerrain(rig.r, w);
    loadVegetation(rig.r, w);
    QVERIFY(rig.show());
    QString failures;
    for (float d : {50.0f, 100.0f, 130.0f})
      for (float e : {0.0f, 20.0f, 45.0f}) {
        const TreePixels l0 = standView(rig.r, w, d, e, 0);
        const TreePixels l2 = standView(rig.r, w, d, e, 2);
        const double ratio = l0.foliage ? double(l2.foliage) / l0.foliage : 0.0;
        qInfo("jacaranda LOD2 %.0f m %.0f deg: LOD0 %d px, LOD2 %d px, x%.3f", double(d), double(e), l0.foliage, l2.foliage, ratio);
        if (l0.foliage < 150)
          failures += QString("[reference too small at %1 m] ").arg(double(d));
        else if (ratio < 0.78 || ratio > 1.40)
          failures += QString("[%1 m %2 deg: LOD2 x%3] ").arg(double(d)).arg(double(e)).arg(ratio, 0, 'f', 2);
      }
    QVERIFY2(failures.isEmpty(), qPrintable(failures));
    QCOMPARE(rig.r->glErrorCount(), 0);
  }

  // Where a tree hands over between the mesh LOD 2 and its impostor (the
  // cross-fade, 121 to 139 m) it must not show a dot grid: the two layers were
  // dithered into complementary pixels, so neither covered a crown solidly and
  // the ground showed through as a regular pattern (stipple 0.64 to 0.99 against
  // 0.1 to 0.6 for either layer alone). Over the band, level and from above, the
  // worst stipple stays near the noisier of the two layers drawn alone.
  void crossFadeLeavesNoDotGrid_data() {
    QTest::addColumn<int>("kind");
    for (int k = 0; k < 4; ++k)
      QTest::newRow(k == 0 ? "fir" : k == 1 ? "pine" : k == 2 ? "jacaranda" : "island") << k;
  }
  void crossFadeLeavesNoDotGrid() {
    QFETCH(int, kind);
    World w = standWorld(kind);
    Rig rig(true);
    loadTerrain(rig.r, w);
    loadVegetation(rig.r, w);
    QVERIFY(rig.show());
    QString failures;
    for (float e : {20.0f, 45.0f}) {
      const TreePixels imp = standView(rig.r, w, 130.0f, e, 3);
      const TreePixels mesh = standView(rig.r, w, 130.0f, e, 2);
      QVERIFY2(imp.all > 150 && mesh.all > 150, "trees not found");
      const double ref = std::max(imp.stipple(), mesh.stipple());
      double worst = 0.0, worstD = 0.0;
      for (int step = 0; step <= 10; ++step) {
        const float d = 120.0f + 2.0f * float(step);
        const double s = standView(rig.r, w, d, e, -1).stipple();
        if (s > worst) {
          worst = s;
          worstD = double(d);
        }
      }
      qInfo("DOTGRID kind %d %.0f deg: impostor %.3f, mesh LOD2 %.3f, worst over 120-140 m %.3f at %.0f m", kind, double(e),
            imp.stipple(), mesh.stipple(), worst, worstD);
      if (rig.r->vegetationStats().impostorVariants == 0)
        QSKIP("no impostor atlas on this context");
      if (worst > 1.2 * ref + 0.22)
        failures += QString("[%1 deg: %2 at %3 m vs layers %4] ").arg(double(e)).arg(worst, 0, 'f', 3).arg(worstD).arg(ref, 0, 'f', 3);
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

  // ---- trees and rocks as shadow casters ----------------------------------
  void treesCastIntoTheFirstCascades_data() {
    QTest::addColumn<int>("kind");
    QTest::addColumn<float>("distM");
    QTest::addColumn<int>("maxTrisPerInstance"); // the coarsest LOD, not LOD0/1
    QTest::newRow("jacaranda c1") << 2 << 70.0f << 5900;
    QTest::newRow("fir c1") << 0 << 70.0f << 300;
    QTest::newRow("island c1") << 3 << 70.0f << 1500;
  }
  // The ground in a tree's shadow is darker with casters on than off, ground
  // beside it is not, and what was drawn is the coarsest LOD in cascade 1 only.
  void treesCastIntoTheFirstCascades() {
    QFETCH(int, kind);
    QFETCH(float, distM);
    QFETCH(int, maxTrisPerInstance);
    World w = loneWorld(kVegTree, kind);
    Rig rig(true);
    loadTerrain(rig.r, w);
    loadVegetation(rig.r, w);
    QVERIFY(rig.show());
    casterScene(rig.r, w, distM);
    rig.r->setVegCasterFirstCascadeForTest(1);
    rig.r->setVegCastersOffForTest(true);
    const QImage off = settledFrame(rig.r).convertToFormat(QImage::Format_RGB32);
    rig.r->setVegCastersOffForTest(false);
    const QImage on = settledFrame(rig.r).convertToFormat(QImage::Format_RGB32);
    const auto &cs = rig.r->vegCasterStatsForTest();
    QVERIFY2(cs.instances[1] >= 1, "the tree is drawn into cascade 1");
    QCOMPARE(cs.instances[0], 0);
    QCOMPARE(cs.instances[2], 0);
    QVERIFY2(cs.triangles[1] / cs.instances[1] <= maxTrisPerInstance, "coarsest LOD");
    const QPoint a = casterGroundPx(rig.r, w, 40.0f - 12.0f, 0.0f);
    const QPoint b = casterGroundPx(rig.r, w, 40.0f - 12.0f, 14.0f);
    const double shadowRatio = windowLuma(on, a.x(), a.y()) / windowLuma(off, a.x(), a.y());
    const double controlRatio = windowLuma(on, b.x(), b.y()) / windowLuma(off, b.x(), b.y());
    qInfo("caster probe: shadow %.3f control %.3f (tris/instance %lld)", shadowRatio, controlRatio,
          cs.triangles[1] / cs.instances[1]);
    QVERIFY2(shadowRatio < 0.9, "ground in the tree's shadow is darker with casters");
    QVERIFY2(controlRatio > 0.97, "ground beside the shadow is not");
    QCOMPARE(rig.r->glErrorCount(), 0);
  }

  // The flags decide: trees off, vegetation off, Low quality, and the test switch
  // each leave the frame as it is without casters, and switching one back on with
  // the camera still shows the shadow again (a tile reused from before the switch
  // would not).
  void casterFlagsAndLowGateTheShadow() {
    World w = loneWorld(kVegTree, 2);
    Rig rig(true);
    loadTerrain(rig.r, w);
    loadVegetation(rig.r, w);
    QVERIFY(rig.show());
    casterScene(rig.r, w, 70.0f);
    rig.r->setVegCasterFirstCascadeForTest(1);
    settledFrame(rig.r); // the projection the probe needs is the drawn one
    const QPoint a = casterGroundPx(rig.r, w, 28.0f, 0.0f);
    auto ratio = [&](const QImage &x, const QImage &y) {
      return windowLuma(x, a.x(), a.y()) / windowLuma(y, a.x(), a.y());
    };
    auto frame = [&]() { return settledFrame(rig.r).convertToFormat(QImage::Format_RGB32); };
    rig.r->setVegCastersOffForTest(true);
    const QImage none = frame();
    rig.r->setVegCastersOffForTest(false);
    const QImage withTrees = frame();
    QVERIFY(ratio(withTrees, none) < 0.9);
    // Trees off: the shadow goes (a stale tile would keep it).
    rig.r->setVegetationTreesEnabled(false);
    QVERIFY2(ratio(frame(), none) > 0.97, "trees off");
    rig.r->setVegetationTreesEnabled(true);
    QVERIFY2(ratio(frame(), none) < 0.9, "trees back on");
    // Vegetation off altogether, then on.
    rig.r->setVegetationEnabled(false);
    rig.r->grabFrame();
    const QImage vegOff = frame();
    rig.r->setVegetationEnabled(true);
    QVERIFY2(ratio(frame(), vegOff) < 0.9, "vegetation back on");
    // Low: one 1024 map over 150 m (texel 0.35 m): no casters at all.
    rig.r->setShadowQuality(ShadowQuality::Low);
    const QImage lowOn = frame();
    const auto &cs = rig.r->vegCasterStatsForTest();
    const int passes = cs.passes;
    rig.r->setVegCastersOffForTest(true);
    const QImage lowOff = frame();
    QCOMPARE(cs.passes, passes); // never drawn at Low
    QVERIFY2(changedFraction(lowOn, lowOff) < 0.001, "Low is the same with or without casters");
    QCOMPARE(rig.r->glErrorCount(), 0);
  }

  // The finest cascade (texel 5 cm, the drone's) takes trees too, so ground near
  // the camera is shadowed like the rest: a tree 24 m ahead shadows the ground
  // 6 m in front of it. Without the finest cascade it does not.
  void treesCastIntoTheFinestCascade() {
    World w = loneWorld(kVegTree, 2);
    Rig rig(true);
    loadTerrain(rig.r, w);
    loadVegetation(rig.r, w);
    QVERIFY(rig.show());
    casterScene(rig.r, w, 24.0f, 30.0f, 0.5f);
    settledFrame(rig.r); // the projection the probes need is the drawn one
    const QPoint a = casterGroundPx(rig.r, w, 40.0f - 6.0f, 0.0f);
    const QPoint c = casterGroundPx(rig.r, w, 40.0f - 6.0f, 14.0f);
    auto frame = [&]() { return settledFrame(rig.r).convertToFormat(QImage::Format_RGB32); };
    rig.r->setVegCastersOffForTest(true);
    const QImage none = frame();
    rig.r->setVegCastersOffForTest(false);
    frame(); // as the renderer ships: the finest cascade has them
    qInfo("default casters: %d/%d/%d passes %d", rig.r->vegCasterStatsForTest().instances[0],
          rig.r->vegCasterStatsForTest().instances[1], rig.r->vegCasterStatsForTest().instances[2],
          rig.r->vegCasterStatsForTest().passes);
    QVERIFY2(rig.r->vegCasterStatsForTest().instances[0] >= 1, "the default includes cascade 0");
    rig.r->setVegCasterFirstCascadeForTest(1);
    const QImage c1only = frame();
    QCOMPARE(rig.r->vegCasterStatsForTest().instances[0], 0);
    rig.r->setVegCasterFirstCascadeForTest(0);
    const QImage withC0 = frame();
    QVERIFY(rig.r->vegCasterStatsForTest().instances[0] >= 1);
    const double r1 = windowLuma(c1only, a.x(), a.y()) / windowLuma(none, a.x(), a.y());
    const double r0 = windowLuma(withC0, a.x(), a.y()) / windowLuma(none, a.x(), a.y());
    const double rc = windowLuma(withC0, c.x(), c.y()) / windowLuma(none, c.x(), c.y());
    qInfo("finest cascade: c1 only %.3f, with c0 %.3f, control %.3f", r1, r0, rc);
    QVERIFY2(r1 > 0.97, "without the finest cascade the near ground is not shadowed");
    QVERIFY2(r0 < 0.9, "with it, it is");
    QVERIFY2(rc > 0.97, "ground beside it is not");
    QCOMPARE(rig.r->glErrorCount(), 0);
  }

  // Beyond kVegCasterFarRangeM trees stop casting into the coarser cascades (a
  // 150-200 m tree is a few fogged pixels and its band was most of the caster
  // cost); the finest cascade keeps every tree near the camera, whatever the range.
  void farCastersAreCulledOnlyBeyondTheRange() {
    World w = loneWorld(kVegTree, 2);
    Rig rig(true);
    loadTerrain(rig.r, w);
    loadVegetation(rig.r, w);
    QVERIFY(rig.show());
    rig.r->setVegCastersOffForTest(false);
    rig.r->setVegCasterFirstCascadeForTest(0);
    const auto &cs = rig.r->vegCasterStatsForTest();
    // The tree is 100 m ahead: cascade 1, inside the range.
    casterScene(rig.r, w, 100.0f);
    settledFrame(rig.r);
    QVERIFY2(cs.instances[1] >= 1, "100 m: inside the range, drawn");
    // 180 m ahead: still cascade 1's slice, beyond the range.
    casterScene(rig.r, w, 180.0f);
    settledFrame(rig.r);
    const int beyond = cs.instances[1];
    rig.r->setVegCasterFarRangeForTest(0.0f); // no limit
    casterScene(rig.r, w, 181.0f);            // a different pose, so the tile is redrawn
    settledFrame(rig.r);
    const int unlimited = cs.instances[1];
    qInfo("180 m: %d with the range, %d without", beyond, unlimited);
    QVERIFY2(unlimited >= 1, "the tree is in cascade 1's window, so without the range it is drawn");
    QCOMPARE(beyond, 0);
    // The finest cascade is never limited, even by a tiny range.
    rig.r->setVegCasterFarRangeForTest(5.0f);
    casterScene(rig.r, w, 24.0f);
    settledFrame(rig.r);
    QVERIFY2(cs.instances[0] >= 1, "cascade 0 ignores the range");
    QCOMPARE(rig.r->glErrorCount(), 0);
  }

  // The casters are culled to the light window: a forest seen from its corner
  // draws far fewer than the forest holds.
  void casterListIsCulledToTheLightWindow() {
    World w = buildWorld(kPalForest, -2, 2);
    size_t held = 0;
    for (const auto &v : w.veg)
      for (const VegInstance &t : v)
        held += (t.species == kVegTree || t.species == kVegRock) ? 1 : 0;
    Rig rig(true);
    loadTerrain(rig.r, w);
    loadVegetation(rig.r, w);
    QVERIFY(rig.show());
    rig.r->setSun(30.0f, 0.0f);
    rig.r->setTerrainQuality(vsim::terrain::Quality::High);
    rig.r->setShadowQuality(ShadowQuality::High);
    rig.r->setVegCastersOffForTest(false);
    rig.r->setVegCasterFirstCascadeForTest(0);
    aimCamera(rig.r, w, -380.0f, -380.0f, 12.0f, 3.1f, 0.2f); // corner, looking out
    settledFrame(rig.r);
    const auto &cs = rig.r->vegCasterStatsForTest();
    const int drawn = cs.instances[0] + cs.instances[1];
    qInfo("culled casters: drew %d of %zu", drawn, held);
    QVERIFY(drawn > 0);
    QVERIFY2(size_t(drawn) < held / 3, "casters outside the cascade windows are skipped");
  }

  // The push that keeps a crown clear of its own hull also lifts the start of a
  // shadow; scaled with the height above the base it must not detach the trunk's
  // shadow from its foot: at 30 deg the ground beside a lone jacaranda is
  // shadowed from within 0.7 m of the base (0.55 m with no push and with the
  // ramp, 0.9 m flat -1, 1.8 m flat -2).
  void lowSunShadowStaysAtTheTrunkFoot() {
    World w = loneWorld(kVegTree, 2);
    Rig rig(true);
    rig.r->freezeAnimationForTest(true); // clock stays 0, as in the caster pass
    loadTerrain(rig.r, w);
    loadVegetation(rig.r, w);
    QVERIFY(rig.show());
    rig.r->setTerrainQuality(vsim::terrain::Quality::High);
    rig.r->setShadowQuality(ShadowQuality::High);
    rig.r->setVegCasterFirstCascadeForTest(0);
    rig.r->setSun(30.0f, 90.0f);
    aimCamera(rig.r, w, 33.0f, 0.0f, 1.5f, 0.0f, 0.18f);
    rig.r->setVegCastersOffForTest(true);
    const QImage off = settledFrame(rig.r).convertToFormat(QImage::Format_RGB32);
    rig.r->setVegCastersOffForTest(false);
    const QImage on = settledFrame(rig.r).convertToFormat(QImage::Format_RGB32);
    float onset = -1.0f;
    for (int i = 3; i <= 70 && onset < 0.0f; ++i) {
      const float t = 0.05f * float(i);
      const QPoint a = casterGroundPx(rig.r, w, 40.0f, -t);
      const QPoint b = casterGroundPx(rig.r, w, 40.0f, -t - 0.15f);
      QVERIFY(a.x() > 4 && a.y() > 4 && a.x() < on.width() - 4 && a.y() < on.height() - 4);
      const double ra = windowLuma(on, a.x(), a.y()) / windowLuma(off, a.x(), a.y());
      const double rb = windowLuma(on, b.x(), b.y()) / windowLuma(off, b.x(), b.y());
      if (ra < 0.85 && rb < 0.85)
        onset = t;
    }
    qInfo("shadow starts %.2f m from the trunk's foot", double(onset));
    QVERIFY2(onset > 0.0f, "the tree casts a shadow on the ground beside it");
    QVERIFY2(onset < 0.7f, "the shadow starts at the trunk's foot");
  }

  // The clock is frozen at 0 in this test and the one above: the wind sways the
  // trees in the main pass (floraTime_) but the caster pass draws them at time
  // 0, so the ratios would vary with the sway otherwise.
  // The crowns are not darkened by their own caster: seen from the sun's side the
  // lit face of a crown keeps its brightness with casters on (a coarse LOD
  // shadowing the finer mesh behind it would show as acne), in the finest
  // cascade and in cascade 1. The bound is 0.88: pinned on llvmpipe the ratios
  // are 0.922 jacaranda and 0.903 fir in the finest cascade (margin 0.003 over
  // 0.9, thin for another driver), 0.98 and 0.994 in cascade 1; with the flat
  // 1 m push they are 0.863 and 0.852, and with no push 0.785 and 0.699.
  void crownsDoNotShadowThemselves_data() {
    QTest::addColumn<int>("kind");
    QTest::addColumn<float>("distM");
    QTest::newRow("jacaranda c0") << 2 << 24.0f;
    QTest::newRow("fir c0") << 0 << 24.0f;
    QTest::newRow("jacaranda c1") << 2 << 80.0f;
    QTest::newRow("island c1") << 3 << 80.0f;
  }
  void crownsDoNotShadowThemselves() {
    QFETCH(int, kind);
    QFETCH(float, distM);
    World w = loneWorld(kVegTree, kind);
    Rig rig(true);
    rig.r->freezeAnimationForTest(true); // clock stays 0, as in the caster pass
    loadTerrain(rig.r, w);
    loadVegetation(rig.r, w);
    QVERIFY(rig.show());
    casterScene(rig.r, w, distM, 50.0f);
    rig.r->setSun(50.0f, 180.0f); // behind the camera: the visible face is the lit one
    rig.r->setVegCasterFirstCascadeForTest(0);
    rig.r->setVegetationEnabled(false);
    rig.r->grabFrame();
    const QImage bare = rig.r->grabFrame().convertToFormat(QImage::Format_RGB32);
    rig.r->setVegetationEnabled(true);
    rig.r->setVegCastersOffForTest(true);
    const QImage off = settledFrame(rig.r).convertToFormat(QImage::Format_RGB32);
    rig.r->setVegCastersOffForTest(false);
    const QImage on = settledFrame(rig.r).convertToFormat(QImage::Format_RGB32);
    // Crown pixels: the trees' pixels with casters off; their mean luminance.
    double lumOff = 0.0, lumOn = 0.0;
    long n = 0;
    for (int y = 0; y < off.height(); ++y)
      for (int x = 0; x < off.width(); ++x)
        if (treeDiffers(off.pixel(x, y), bare.pixel(x, y))) {
          const QRgb p = off.pixel(x, y), q = on.pixel(x, y);
          lumOff += 0.299 * qRed(p) + 0.587 * qGreen(p) + 0.114 * qBlue(p);
          lumOn += 0.299 * qRed(q) + 0.587 * qGreen(q) + 0.114 * qBlue(q);
          ++n;
        }
    QVERIFY2(n > 300, "the tree is in view");
    const double ratio = lumOn / lumOff;
    qInfo("crown luminance with casters / without: %.3f (%ld px)", ratio, n);
    QVERIFY2(ratio > 0.88, "a crown's lit face keeps its brightness");
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

  // The soft contact blob under a trunk or a rock: the ground a little way from
  // the foot is darker than ground 3 m away, by the blob alone (the sun shadow
  // map and the terrain are the same with vegetation on and off).
  void contactBlobDarkensTheFoot_data() {
    QTest::addColumn<int>("species");
    QTest::addColumn<int>("kind");
    QTest::addColumn<float>("nearM"); // probe distance before the foot
    QTest::addColumn<double>("maxRatio");
    using B = VegContactBlob;
    // 60% of the way out in the blob, clear of the object itself.
    QTest::newRow("jacaranda") << int(kVegTree) << 2
                          << 0.6f * B::kTrunkRadiusMult * kVegTreeKinds[2].trunkRadiusM << 0.88;
    QTest::newRow("rock") << int(kVegRock) << 0
                          << 0.6f * 0.5f * kVegRockSpanM * B::kRockRadiusMult << 0.94;
  }
  void contactBlobDarkensTheFoot() {
    QFETCH(int, species);
    QFETCH(int, kind);
    QFETCH(float, nearM);
    QFETCH(double, maxRatio);
    World w = loneWorld(uint8_t(species), kind);
    Rig rig(true);
    loadTerrain(rig.r, w);
    loadVegetation(rig.r, w);
    QVERIFY(rig.show());
    rig.r->setSun(46.0f, 37.0f);
    rig.r->setTerrainQuality(vsim::terrain::Quality::High);
    aimProbe(rig.r, w);
    rig.r->setVegetationEnabled(false);
    rig.r->grabFrame();
    const QImage off = rig.r->grabFrame().convertToFormat(QImage::Format_RGB32);
    rig.r->setVegetationEnabled(true);
    const QImage on = settledFrame(rig.r).convertToFormat(QImage::Format_RGB32);
    QVERIFY(rig.r->vegetationStats().blobs >= 1);
    const QPoint a = probePixel(rig.r, w, nearM);
    const QPoint b = probePixel(rig.r, w, 3.0f);
    const double nearRatio = windowLuma(on, a.x(), a.y()) / windowLuma(off, a.x(), a.y());
    const double farRatio = windowLuma(on, b.x(), b.y()) / windowLuma(off, b.x(), b.y());
    qInfo("blob probe: near %.3f far %.3f", nearRatio, farRatio);
    QVERIFY2(farRatio > 0.97, "ground 3 m away must be untouched by the blob");
    QVERIFY2(nearRatio < maxRatio, "ground at the foot must be darker with the blob");
    QCOMPARE(rig.r->glErrorCount(), 0);
  }

  // No trees, no tree blob: a world of one tree with trees off draws no blob and
  // leaves the frame as it is with vegetation off.
  void treesOffMeansNoBlob() {
    World w = loneWorld(kVegTree, 1);
    Rig rig(false);
    loadTerrain(rig.r, w);
    loadVegetation(rig.r, w);
    QVERIFY(rig.show());
    rig.r->setSun(46.0f, 37.0f);
    rig.r->setTerrainQuality(vsim::terrain::Quality::High);
    aimProbe(rig.r, w);
    rig.r->setVegetationEnabled(false);
    rig.r->grabFrame();
    const QImage off = rig.r->grabFrame();
    rig.r->setVegetationEnabled(true);
    const QImage on = settledFrame(rig.r);
    QCOMPARE(rig.r->vegetationStats().blobs, 0);
    QVERIFY(changedFraction(on, off) < 0.001);
    // and with them on, the blob is there
    rig.r->setVegetationTreesEnabled(true);
    settledFrame(rig.r);
    QVERIFY(rig.r->vegetationStats().blobs >= 1);
    // vegetation off altogether: nothing is reported either
    rig.r->setVegetationEnabled(false);
    rig.r->grabFrame();
    QCOMPARE(rig.r->vegetationStats().blobs, 0);
  }

  // The drone's blob lies on the ground under it while it is low, is gone far
  // above the ground, and needs the terrain field to know where the ground is.
  // It does not depend on the vegetation: the same with trees off, vegetation
  // off, and no vegetation loaded at all.
  void droneBlobSitsUnderTheDrone_data() {
    QTest::addColumn<bool>("trees");
    QTest::addColumn<bool>("vegetation");
    QTest::addColumn<bool>("loaded");
    QTest::newRow("vegetation on") << true << true << true;
    QTest::newRow("trees off") << false << true << true;
    QTest::newRow("vegetation off") << true << false << true;
    QTest::newRow("vegetation unloaded") << true << true << false;
  }
  void droneBlobSitsUnderTheDrone() {
    QFETCH(bool, trees);
    QFETCH(bool, vegetation);
    QFETCH(bool, loaded);
    World w = loneWorld(kVegTree, 1);
    for (auto &v : w.veg)
      for (auto &t : v)
        t.pos.y = 60.0f; // out of view
    FieldParams fp;
    fp.seed = 11u;
    fp.palette = kPalForest;
    fp.heightM = 20.0f;
    auto field = std::make_shared<const TerrainField>(fp);
    Rig rig(trees);
    rig.r->setMotorsEditable(false); // the simulator running: the drone is flying
    rig.r->setTerrainField(field);
    loadTerrain(rig.r, w);
    if (loaded)
      loadVegetation(rig.r, w);
    rig.r->setVegetationEnabled(vegetation);
    QVERIFY(rig.show());
    rig.r->setSun(46.0f, 37.0f);
    rig.r->setTerrainQuality(vsim::terrain::Quality::High);
    const float g = surfaceHeight(*w.field, 40.0f, 0.0f, kChunkM, kRes);
    auto frame = [&](float aboveM) {
      aimCamera(rig.r, w, 34.0f, 0.0f, 3.5f, 0.0f, 0.3f);
      SimSnapshot s;
      s.pos_w = {40.0f, 0.0f, -(g + aboveM)};
      rig.r->setSnapshot(s);
      return settledFrame(rig.r).convertToFormat(QImage::Format_RGB32);
    };
    const QImage high = frame(60.0f);
    QVERIFY(!rig.r->droneBlobDrawnForTest());
    const QImage low = frame(1.5f);
    QVERIFY(rig.r->droneBlobDrawnForTest());
    // Ground under the drone, seen from behind: the blob darkens it (the drone
    // itself is above, and the analytic sun shadow falls elsewhere).
    const QPointF p = rig.r->worldToPixelForTest(QVector3D(40.0f, 0.0f, -g - 0.02f));
    const double ratio = windowLuma(low, int(p.x()), int(p.y())) /
                         windowLuma(high, int(p.x()), int(p.y()));
    qInfo("drone blob ratio %.3f", ratio);
    QVERIFY(ratio < 0.9);
    QCOMPARE(rig.r->glErrorCount(), 0);
  }

  // The drone casts a real shadow: its hull (solid) and its rotors (full radius,
  // dithered to the blade solidity kRotorCoverage) are drawn into the sun's depth
  // map, so the ground down the sun ray is darker with the drone low than with it
  // 60 m up in the same frame. The camera looks south from the shadow's side, so
  // the drone is behind its own shadow and hides nothing. Rotor coverage 0 / default / 1 is rendered for each row:
  // the default must darken the footprint a little more than no rotor shadow and
  // far less than a solid disc, which pins the dither to its coverage.
  void droneShadowFallsDownTheSunRay_data() {
    QTest::addColumn<bool>("high");
    QTest::addColumn<float>("aboveM");
    QTest::addColumn<float>("sunEl");
    QTest::newRow("high 0.5 m sun 50") << true << 0.5f << 50.0f;
    QTest::newRow("high 1.5 m sun 50") << true << 1.5f << 50.0f;
    QTest::newRow("high 3 m sun 50") << true << 3.0f << 50.0f;
    QTest::newRow("high 1.5 m sun 20") << true << 1.5f << 20.0f;
    // Landed: the shadow falls right under the drone, which must still get it
    // (the map's depth bias alone would hide a caster that low).
    QTest::newRow("high landed 0.05 m sun 50") << true << 0.05f << 50.0f;
    QTest::newRow("low 1.5 m sun 50") << false << 1.5f << 50.0f;
  }
  void droneShadowFallsDownTheSunRay() {
    QFETCH(bool, high);
    QFETCH(float, aboveM);
    QFETCH(float, sunEl);
    World w = loneWorld(kVegTree, 1);
    for (auto &v : w.veg)
      for (auto &t : v)
        t.pos.y = 60.0f; // out of view
    FieldParams fp;
    fp.seed = 11u;
    fp.palette = kPalForest;
    fp.heightM = 20.0f;
    auto field = std::make_shared<const TerrainField>(fp);
    Rig rig(true);
    rig.r->setMotorsEditable(false);
    rig.r->setTerrainField(field);
    loadTerrain(rig.r, w);
    loadVegetation(rig.r, w);
    QVERIFY(rig.show());
    rig.r->setShadowQuality(high ? vsim::ShadowQuality::High : vsim::ShadowQuality::Low);
    rig.r->setSun(sunEl, 180.0f); // from the south: the shadow falls north, away
    rig.r->setTerrainQuality(vsim::terrain::Quality::High);
    const float g = surfaceHeight(*w.field, 40.0f, 0.0f, kChunkM, kRes);
    auto frame = [&](float above) {
      aimCamera(rig.r, w, 50.0f, 0.0f, 12.0f, 3.14159265f, 0.9f);
      SimSnapshot s;
      s.pos_w = {40.0f, 0.0f, -(g + above)};
      rig.r->setSnapshot(s);
      return settledFrame(rig.r).convertToFormat(QImage::Format_RGB32);
    };
    const QImage far = frame(60.0f);
    // Where the sun ray from the drone meets the terrain (it slopes about 10%).
    const float tanEl = std::tan(qDegreesToRadians(sunEl));
    float sx = 40.0f;
    for (int i = 0; i < 20; ++i)
      sx = 40.0f + (g + aboveM - surfaceHeight(*w.field, sx, 0.0f, kChunkM, kRes)) / tanEl;
    const float sg = surfaceHeight(*w.field, sx, 0.0f, kChunkM, kRes);
    // The drone's footprint (rotor span about 0.64 m), and its mean luma.
    constexpr float kHalf = 0.32f;
    int x0 = 1 << 20, x1 = -1, y0 = 1 << 20, y1 = -1;
    for (int c = 0; c < 4; ++c) {
      const QPointF q = rig.r->worldToPixelForTest(QVector3D(
          sx + ((c & 1) ? kHalf : -kHalf),
          ((c & 2) ? kHalf : -kHalf), -sg - 0.02f));
      x0 = std::min(x0, int(q.x())), x1 = std::max(x1, int(q.x()));
      y0 = std::min(y0, int(q.y())), y1 = std::max(y1, int(q.y()));
    }
    auto meanLuma = [&](const QImage &im) {
      double sum = 0;
      int n = 0;
      for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x, ++n) {
          const QRgb c = im.pixel(x, y);
          sum += 0.299 * qRed(c) + 0.587 * qGreen(c) + 0.114 * qBlue(c);
        }
      return n ? sum / n : 0.0;
    };
    const double ref = meanLuma(far);
    auto ratioAt = [&](float cover) {
      rig.r->setRotorCoverageForTest(cover);
      const QImage near = frame(aboveM);
      return meanLuma(near) / ref;
    };
    const double r0 = high ? ratioAt(0.0f) : 0.0;
    const double rd = ratioAt(-1.0f);
    // The same frame with no drone caster at all: what the drone's shadow does,
    // with the drone's own pixels identical in both (so it also works landed).
    rig.r->setDroneCasterOffForTest(true);
    const double rOff = ratioAt(-1.0f);
    rig.r->setDroneCasterOffForTest(false);
    qInfo("with caster %.4f, without %.4f -> shadow x%.4f", rd, rOff, rd / rOff);
    const double r1 = high ? ratioAt(1.0f) : 0.0;
    // High: the drone is in the map. Low: its texel is too coarse, so the
    // analytic cylinder keeps the shadow.
    QCOMPARE(rig.r->droneInShadowMapForTest(), high);
    qInfo("drone shadow %s %.1f m sun %.0f: footprint %dx%d px, ratio hull-only %.4f, "
          "default %.4f, solid rotors %.4f",
          high ? "high" : "low", double(aboveM), double(sunEl), x1 - x0 + 1, y1 - y0 + 1, r0,
          rd, r1);
    if (!high) {
      QVERIFY(rd < 0.97); // the cylinder
    } else {
      if (aboveM > 0.2f)
        QVERIFY(r0 < 0.99);        // the hull shadows the ground (landed: the
                                   // drone's own pixels cover it, so not here)
      QVERIFY(rd / rOff < 0.995);  // the caster darkens the footprint, landed or not
      QVERIFY(r0 - r1 > 0.004);    // solid rotor discs add a measurable shadow
      const double frac = (r0 - rd) / (r0 - r1); // the default is a fraction of solid
      qInfo("rotor shadow fraction of solid %.3f (coverage %.2f)", frac, 0.15);
      QVERIFY(frac > 0.05 && frac < 0.25);
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
