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
// Vegetation assets load from the embedded resource: every manifest entry
// parses, indices and submeshes are valid, bounding boxes match the manifest,
// LODs get cheaper, textures are non-empty and the embedded blob stays small.
// No GL.
#include "VegetationAssets.h"

#include <QDirIterator>
#include <QFile>
#include <QGuiApplication>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QtTest>

#include <algorithm>
#include <cmath>

using namespace vsim;

class TstVegetationAssets : public QObject {
  Q_OBJECT
  VegAssets assets_;
  QJsonObject manifest_;

private slots:
  void initTestCase() {
    QString err;
    QVERIFY2(loadVegetationAssets(assets_, &err), qPrintable(err));
    QFile f(":/vegetation/manifest.json");
    QVERIFY(f.open(QIODevice::ReadOnly));
    manifest_ = QJsonDocument::fromJson(f.readAll()).object();
  }

  void everyManifestEntryLoads() {
    const QJsonArray sp = manifest_.value("species").toArray();
    QCOMPARE(int(assets_.species.size()), int(sp.size()));
    QVERIFY(sp.size() >= 10);
    for (int i = 0; i < sp.size(); ++i) {
      const QJsonObject so = sp[i].toObject();
      const VegSpeciesAssets &s = assets_.species[size_t(i)];
      QCOMPARE(s.id, so.value("id").toString());
      QCOMPARE(int(s.variants.size()), int(so.value("variants").toArray().size()));
      QCOMPARE(int(s.materials.size()), int(so.value("materials").toArray().size()));
      QVERIFY(!s.variants.empty());
      QVERIFY(!s.materials.empty());
    }
  }

  // The scatter stands a rock's origin on the terrain, so every rock model's
  // pivot must be its lowest point (the renderer adds no offset): within a
  // centimetre once scaled to the 1.3 m span the renderer draws it at.
  void rockPivotIsItsBase() {
    int seen = 0;
    for (const VegSpeciesAssets &s : assets_.species) {
      if (s.category != QStringLiteral("rock"))
        continue;
      for (const VegVariant &v : s.variants) {
        const float k = 1.3f / std::max(v.footprintX, v.footprintZ);
        const QByteArray tag = v.name.toLatin1();
        QVERIFY2(std::fabs(v.bboxMin.y()) * k < 0.01f, tag.constData());
        ++seen;
      }
    }
    QVERIFY(seen >= 4);
  }

  void meshesAreValid() {
    for (const VegSpeciesAssets &s : assets_.species)
      for (const VegVariant &v : s.variants)
        for (int l = 0; l < kVegLodCount; ++l) {
          const VegMesh &m = v.lods[l];
          const QByteArray tag = (v.name + " lod" + QString::number(l)).toLatin1();
          QVERIFY2(m.triangleCount() > 0, tag);
          QVERIFY2(!m.submeshes.empty(), tag);
          for (uint32_t ix : m.indices)
            QVERIFY2(ix < m.vertexCount(), tag);
          uint32_t covered = 0;
          for (const VegSubmesh &sm : m.submeshes) {
            QVERIFY2(uint64_t(sm.firstIndex) + sm.indexCount <= m.indices.size(), tag);
            QVERIFY2(sm.material >= 0 && sm.material < int(s.materials.size()), tag);
            covered += sm.indexCount;
          }
          QCOMPARE(covered, uint32_t(m.indices.size()));
        }
  }

  void bboxesMatchManifest() {
    const QJsonArray sp = manifest_.value("species").toArray();
    for (int i = 0; i < sp.size(); ++i) {
      const QJsonArray vs = sp[i].toObject().value("variants").toArray();
      for (int j = 0; j < vs.size(); ++j) {
        const QJsonArray lods = vs[j].toObject().value("lods").toArray();
        const VegVariant &v = assets_.species[size_t(i)].variants[size_t(j)];
        for (int l = 0; l < kVegLodCount; ++l) {
          const QJsonObject lo = lods[l].toObject();
          QCOMPARE(int(v.lods[l].triangleCount()), lo.value("triangles").toInt());
          QCOMPARE(int(v.lods[l].vertexCount()), lo.value("vertices").toInt());
          const QJsonArray mn = lo.value("bboxMin").toArray();
          const QJsonArray mx = lo.value("bboxMax").toArray();
          const float a[3] = {v.lods[l].bboxMin.x(), v.lods[l].bboxMin.y(), v.lods[l].bboxMin.z()};
          const float b[3] = {v.lods[l].bboxMax.x(), v.lods[l].bboxMax.y(), v.lods[l].bboxMax.z()};
          for (int k = 0; k < 3; ++k) {
            QVERIFY(std::fabs(a[k] - float(mn[k].toDouble())) < 1e-4f);
            QVERIFY(std::fabs(b[k] - float(mx[k].toDouble())) < 1e-4f);
          }
          // The stored box also encloses the vertices (within rounding).
          float lo3[3] = {1e30f, 1e30f, 1e30f}, hi3[3] = {-1e30f, -1e30f, -1e30f};
          for (uint32_t q = 0; q < v.lods[l].vertexCount(); ++q)
            for (int k = 0; k < 3; ++k) {
              const float x = v.lods[l].vertices[size_t(q) * 8 + size_t(k)];
              lo3[k] = std::min(lo3[k], x);
              hi3[k] = std::max(hi3[k], x);
            }
          for (int k = 0; k < 3; ++k) {
            QVERIFY(std::fabs(lo3[k] - a[k]) < 1e-3f);
            QVERIFY(std::fabs(hi3[k] - b[k]) < 1e-3f);
          }
        }
        QVERIFY(v.nominalHeight > 0.0f);
        QVERIFY(v.bboxMin.y() > -0.01f); // base at y = 0
      }
    }
  }

  void lodsGetCheaper() {
    for (const VegSpeciesAssets &s : assets_.species)
      for (const VegVariant &v : s.variants) {
        const QByteArray tag = v.name.toLatin1();
        QVERIFY2(v.lods[0].triangleCount() > v.lods[1].triangleCount(), tag);
        QVERIFY2(v.lods[1].triangleCount() > v.lods[2].triangleCount(), tag);
      }
  }

  void texturesAreNonEmpty() {
    for (const VegSpeciesAssets &s : assets_.species)
      for (const VegMaterial &m : s.materials) {
        const QByteArray tag = (s.id + "/" + m.name).toLatin1();
        QVERIFY2(m.diffuse.width() > 0 && m.diffuse.height() > 0, tag);
        QVERIFY2(m.normal.width() > 0 && m.arm.width() > 0, tag);
        QVERIFY2(m.diffuse.format() == QImage::Format_RGBA8888, tag);
        // Not a uniform image: a loaded texture carries detail.
        bool varies = false;
        for (int y = 0; y < m.diffuse.height() && !varies; y += 7)
          for (int x = 0; x < m.diffuse.width(); x += 7)
            if (m.diffuse.pixel(x, y) != m.diffuse.pixel(0, 0)) {
              varies = true;
              break;
            }
        QVERIFY2(varies, tag);
        if (m.cutout()) {
          QVERIFY2(m.alphaCutoff > 0.0f && m.alphaCutoff < 1.0f, tag);
          int opaque = 0, clear = 0;
          for (int y = 0; y < m.alpha.height(); y += 5)
            for (int x = 0; x < m.alpha.width(); x += 5)
              (m.alpha.pixelColor(x, y).red() > 127 ? opaque : clear)++;
          QVERIFY2(opaque > 0 && clear > 0, tag);
        }
      }
  }

  void cutoutMaterialsExist() {
    const VegSpeciesAssets *fir = assets_.find("fir_tree_01");
    QVERIFY(fir);
    bool anyCut = false;
    for (const VegMaterial &m : fir->materials)
      anyCut = anyCut || m.cutout();
    QVERIFY(anyCut);
  }

  // Full-size trees: at least two conifers and two broadleaf species, each with
  // a modelled height of 8 m or more and a measured trunk.
  void fullSizeTreesShipped() {
    const QJsonArray sp = manifest_.value("species").toArray();
    const QStringList conifers = {"fir_tree_01", "pine_sapling_medium"};
    const QStringList broadleaf = {"jacaranda_tree", "island_tree_01"};
    for (const QStringList &group : {conifers, broadleaf})
      for (const QString &id : group) {
        const VegSpeciesAssets *s = assets_.find(id);
        QVERIFY2(s, qPrintable(id));
        QCOMPARE(s->category, QStringLiteral("tree"));
        bool inManifest = false;
        for (const QJsonValue &v : sp)
          if (v.toObject().value("id").toString() == id) {
            inManifest = true;
            QVERIFY(v.toObject().value("license").toString() == "CC0");
          }
        QVERIFY2(inManifest, qPrintable(id));
        for (const VegVariant &v : s->variants) {
          QVERIFY2(v.nominalHeight >= 5.0f, qPrintable(v.name));
          QVERIFY2(v.trunkRadius > 0.05f && v.trunkRadius < 2.0f, qPrintable(v.name));
          // a crown worth the name at LOD 0 (broadleaf: a real model; conifers
          // are built from cards, see cardConifersHaveTiers)
          if (group == broadleaf)
            QVERIFY2(v.lods[0].triangleCount() >= 15000, qPrintable(v.name));
        }
      }
  }

  // Card conifers: 6-10 needle-branch tiers at LOD 0, fewer at each coarser LOD
  // and a cheaper mesh each time.
  void cardConifersHaveTiers() {
    for (const char *id : {"fir_tree_01", "pine_sapling_medium"}) {
      const VegSpeciesAssets *s = assets_.find(id);
      QVERIFY2(s, id);
      QVERIFY2(s->variants.size() >= 2, id);
      for (const VegVariant &v : s->variants) {
        QVERIFY2(v.lods[0].tiers >= 6 && v.lods[0].tiers <= 10, qPrintable(v.name));
        QVERIFY2(v.lods[1].tiers >= 3 && v.lods[1].tiers < v.lods[0].tiers, qPrintable(v.name));
        QVERIFY2(v.lods[2].tiers >= 1 && v.lods[2].tiers <= v.lods[1].tiers, qPrintable(v.name));
        QVERIFY2(v.lods[1].triangleCount() < v.lods[0].triangleCount(), qPrintable(v.name));
        QVERIFY2(v.lods[2].triangleCount() < v.lods[1].triangleCount(), qPrintable(v.name));
        // a trunk plus cards: bark and a cut-out needle material
        QCOMPARE(int(v.lods[0].submeshes.size()), 2);
        QVERIFY(!s->materials[size_t(v.lods[0].submeshes[0].material)].cutout());
        QVERIFY(s->materials[size_t(v.lods[0].submeshes[1].material)].cutout());
      }
    }
  }

  // The pine reads as a pine: a trunk that shows below the first whorl, and
  // needle whorls with clear air between them. Needle area is binned by height
  // (triangle centroids); a whorl is a run of occupied 10 cm bins, two whorls
  // are separated by at least 30 cm with next to no needle area. Every LOD keeps
  // one separated whorl per tier.
  void pineHasSeparatedWhorls() {
    const VegSpeciesAssets *s = assets_.find("pine_sapling_medium");
    QVERIFY(s);
    for (const VegVariant &v : s->variants)
      for (int l = 0; l < kVegLodCount; ++l) {
        const VegMesh &m = v.lods[l];
        const VegSubmesh &sm = m.submeshes[1]; // needles
        const float H = m.bboxMax.y();
        const float bin = 0.1f;
        std::vector<double> area(size_t(H / bin) + 2, 0.0);
        double total = 0;
        float lowest = H;
        for (uint32_t i = sm.firstIndex; i + 2 < sm.firstIndex + sm.indexCount; i += 3) {
          QVector3D p[3];
          for (int k = 0; k < 3; ++k) {
            const float *f = &m.vertices[size_t(m.indices[i + uint32_t(k)]) * VegMesh::kFloatsPerVertex];
            p[k] = QVector3D(f[0], f[1], f[2]);
          }
          const double a = 0.5 * double(QVector3D::crossProduct(p[1] - p[0], p[2] - p[0]).length());
          const float cy = (p[0].y() + p[1].y() + p[2].y()) / 3.0f;
          area[std::min(size_t(cy / bin), area.size() - 1)] += a;
          total += a;
          lowest = std::min(lowest, std::min({p[0].y(), p[1].y(), p[2].y()}));
        }
        // The crown's whorls are counted above 0.30 H: the few low drooping
        // branches below it fill the bottom third and are not whorls.
        int whorls = 0, empty = 99;
        double lowArea = 0;
        for (size_t b = 0; b < area.size(); ++b)
          if (float(b) * bin < 0.30f * H)
            lowArea += area[b];
        for (size_t b = size_t(0.30f * H / bin); b < area.size(); ++b) {
          const double a = area[b];
          if (a > 0.003 * total) {
            if (empty >= 3) ++whorls;
            empty = 0;
          } else {
            ++empty;
          }
        }
        qInfo("%s lod%d: %d whorls of %d tiers, lowest needle %.1f of %.1f, %.0f%% of needle area below 0.3 H",
              qPrintable(v.name), l, whorls, m.tiers, double(lowest), double(H), 100.0 * lowArea / total);
        QVERIFY2(whorls >= m.tiers - 3 && whorls <= m.tiers + 1, // the leader may read as one more
                 qPrintable(QString("%1 lod%2: %3 whorls vs %4 tiers").arg(v.name).arg(l).arg(whorls).arg(m.tiers)));
        // the bottom third is not a bare pole: low branches droop down toward the ground
        QVERIFY2(lowest < 0.15f * H, qPrintable(QString("%1 lod%2: lowest needle %3 of %4").arg(v.name).arg(l).arg(double(lowest)).arg(double(H))));
        QVERIFY2(lowArea > 0.04 * total, qPrintable(QString("%1 lod%2: %3 of the needle area is low").arg(v.name).arg(l).arg(lowArea / total)));
      }
  }

  // The pine's trunk is sturdy and tapers (measured on the trunk submesh at
  // every LOD), and its whorls are widest low down and narrow toward the top.
  void pineTrunkTapersAndWhorlsNarrow() {
    const VegSpeciesAssets *s = assets_.find("pine_sapling_medium");
    QVERIFY(s);
    for (const VegVariant &v : s->variants)
      for (int l = 0; l < kVegLodCount; ++l) {
        const VegMesh &m = v.lods[l];
        auto radiusAt = [&](const VegSubmesh &sm, float y0, float y1) {
          float r = 0;
          for (uint32_t i = sm.firstIndex; i < sm.firstIndex + sm.indexCount; ++i) {
            const float *f = &m.vertices[size_t(m.indices[i]) * VegMesh::kFloatsPerVertex];
            if (f[1] >= y0 && f[1] <= y1)
              r = std::max(r, std::hypot(f[0], f[2]));
          }
          return r;
        };
        const float H = m.bboxMax.y();
        const float footR = radiusAt(m.submeshes[0], 0.0f, 0.5f);
        const float topR = radiusAt(m.submeshes[0], 0.6f * H, H);
        qInfo("%s lod%d: trunk radius foot %.2f, upper %.2f", qPrintable(v.name), l,
              double(footR), double(topR));
        QVERIFY2(footR >= 0.24f, qPrintable(v.name));
        QVERIFY2(topR > 0.0f && topR <= 0.5f * footR, qPrintable(v.name));
        // whorl reach: lowest third of the needle heights against the highest third
        const VegSubmesh &nd = m.submeshes[1];
        float lo = H, hi = 0;
        for (uint32_t i = nd.firstIndex; i < nd.firstIndex + nd.indexCount; ++i) {
          const float y = m.vertices[size_t(m.indices[i]) * VegMesh::kFloatsPerVertex + 1];
          lo = std::min(lo, y);
          hi = std::max(hi, y);
        }
        const float third = (hi - lo) / 3.0f;
        const float lowR = radiusAt(nd, lo, lo + third);
        const float upR = radiusAt(nd, hi - third, hi);
        qInfo("%s lod%d: whorl reach low %.2f, high %.2f", qPrintable(v.name), l, double(lowR), double(upR));
        QVERIFY2(lowR >= 2.0f * upR, qPrintable(v.name));
        QVERIFY2(lowR >= 2.5f, qPrintable(v.name));
      }
    // and it wears a real bark (lighter than the old near-black one)
    const VegMaterial &bark = s->materials[0];
    const QImage d = bark.diffuse.convertToFormat(QImage::Format_RGB888);
    double sum = 0;
    for (int y = 0; y < d.height(); ++y)
      for (int x = 0; x < d.width(); ++x)
        sum += qGray(d.pixel(x, y));
    const double mean = sum / (double(d.width()) * d.height()) / 255.0;
    qInfo("pine bark mean gray %.2f", mean);
    QVERIFY(mean > 0.28);
  }

  // The conifer needle albedo must not be brighter than the broadleaf leaves
  // (mean linear luminance over the covered texels).
  void coniferIsNotBrighterThanBroadleaf() {
    auto meanLum = [](const VegMaterial &m) {
      const QImage d = m.diffuse.convertToFormat(QImage::Format_RGBA8888);
      const QImage a = m.alpha.scaled(d.size()).convertToFormat(QImage::Format_Grayscale8);
      auto lin = [](int c) {
        const double x = c / 255.0;
        return x <= 0.04045 ? x / 12.92 : std::pow((x + 0.055) / 1.055, 2.4);
      };
      double sum = 0;
      long n = 0;
      for (int y = 0; y < d.height(); ++y)
        for (int x = 0; x < d.width(); ++x) {
          if (qGray(a.pixel(x, y)) < 128)
            continue;
          const QRgb c = d.pixel(x, y);
          sum += 0.2126 * lin(qRed(c)) + 0.7152 * lin(qGreen(c)) + 0.0722 * lin(qBlue(c));
          ++n;
        }
      return n ? sum / double(n) : 0.0;
    };
    auto cutoutLum = [&](const char *id) {
      double best = 0;
      for (const VegMaterial &m : assets_.find(id)->materials)
        if (m.cutout())
          best = std::max(best, meanLum(m));
      return best;
    };
    const double broad = std::min(cutoutLum("jacaranda_tree"), cutoutLum("island_tree_01"));
    QVERIFY(broad > 0.01);
    for (const char *id : {"fir_tree_01", "pine_sapling_medium"}) {
      const double c = cutoutLum(id);
      qInfo("%s needle luminance %.4f, broadleaf %.4f", id, c, broad);
      QVERIFY2(c > 0.005 && c <= broad, id);
    }
  }

  void broadleafSpeciesHaveTwoDistinctVariants() {
    for (const char *id : {"jacaranda_tree", "island_tree_01"}) {
      const VegSpeciesAssets *s = assets_.find(id);
      QVERIFY2(s && s->variants.size() >= 2, id);
      const VegVariant &a = s->variants[0], &b = s->variants[1];
      QVERIFY2(std::fabs(a.nominalHeight - b.nominalHeight) > 0.05f * a.nominalHeight ||
                   std::fabs(a.footprintX - b.footprintX) > 0.05f * a.footprintX,
               id);
    }
  }

  void embeddedSizeIsBounded() {
    qint64 total = 0;
    int n = 0;
    QDirIterator it(":/vegetation", QDirIterator::Subdirectories);
    while (it.hasNext()) {
      it.next();
      if (it.fileInfo().isFile()) {
        total += it.fileInfo().size();
        ++n;
      }
    }
    qInfo("embedded vegetation resource: %d files, %.1f MB", n, double(total) / 1e6);
    QVERIFY(n > 100);
    QVERIFY(total < qint64(30) * 1000 * 1000);
  }

  void rejectsBrokenMesh() {
    VegMesh m;
    QString err;
    QVERIFY(!parseVMesh(QByteArray("VMSH"), m, &err));
    QVERIFY(!parseVMesh(QByteArray(64, 'x'), m, &err));
    QFile f(":/vegetation/rock_07/rock_07_lod2.vmesh");
    QVERIFY(f.open(QIODevice::ReadOnly));
    QByteArray good = f.readAll();
    QVERIFY(parseVMesh(good, m, &err));
    QVERIFY(!parseVMesh(good.left(good.size() - 5), m, &err));
    QByteArray bad = good;
    bad[bad.size() - 1] = char(0x7f); // a high index byte: out of range
    bad[bad.size() - 2] = char(0x7f);
    QVERIFY(!parseVMesh(bad, m, &err));
  }
};

int main(int argc, char **argv) {
  qputenv("QT_QPA_PLATFORM", qgetenv("QT_QPA_PLATFORM").isEmpty() ? "offscreen" : qgetenv("QT_QPA_PLATFORM"));
  QGuiApplication app(argc, argv);
  TstVegetationAssets t;
  return QTest::qExec(&t, argc, argv);
}
#include "tst_vegetation_assets.moc"
