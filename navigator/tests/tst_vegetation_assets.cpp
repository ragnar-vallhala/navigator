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
    const VegSpeciesAssets *fir = assets_.find("fir_sapling");
    QVERIFY(fir);
    bool anyCut = false;
    for (const VegMaterial &m : fir->materials)
      anyCut = anyCut || m.cutout();
    QVERIFY(anyCut);
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
    QVERIFY(total < 30 * 1000 * 1000);
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
