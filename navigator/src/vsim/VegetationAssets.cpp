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
#include "VegetationAssets.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <cmath>
#include <cstring>

// Static-library resources are not linked in unless something references them.
static void initVegetationResource() { Q_INIT_RESOURCE(vegetation_assets); }

namespace vsim {

namespace {

bool fail(QString *error, const QString &what) {
  if (error)
    *error = what;
  return false;
}

template <class T> bool readLe(const QByteArray &b, qsizetype off, T &v) {
  if (off < 0 || off + qsizetype(sizeof(T)) > b.size())
    return false;
  std::memcpy(&v, b.constData() + off, sizeof(T)); // x86/ARM little endian
  return true;
}

QVector3D vec3(const QJsonValue &v) {
  const QJsonArray a = v.toArray();
  return QVector3D(float(a.at(0).toDouble()), float(a.at(1).toDouble()),
                   float(a.at(2).toDouble()));
}

bool loadImage(const QString &path, QImage::Format fmt, QImage &out,
               QString *error) {
  QImage img;
  if (!img.load(path))
    return fail(error, path + ": cannot decode image");
  out = img.convertToFormat(fmt);
  return !out.isNull() || fail(error, path + ": cannot convert image");
}

bool readFile(const QString &path, QByteArray &out, QString *error) {
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly))
    return fail(error, path + ": cannot open");
  out = f.readAll();
  return true;
}

} // namespace

const VegSpeciesAssets *VegAssets::find(const QString &id) const {
  for (const VegSpeciesAssets &s : species)
    if (s.id == id)
      return &s;
  return nullptr;
}

bool parseVMesh(const QByteArray &b, VegMesh &out, QString *error) {
  constexpr qsizetype kHeader = 48;
  if (b.size() < kHeader || std::memcmp(b.constData(), "VMSH", 4) != 0)
    return fail(error, "not a vmesh (bad magic)");
  uint32_t version = 0, nv = 0, ni = 0, ns = 0, ib = 0;
  readLe(b, 4, version);
  readLe(b, 8, nv);
  readLe(b, 12, ni);
  readLe(b, 16, ns);
  readLe(b, 20, ib);
  if (version != 1)
    return fail(error, "unsupported vmesh version");
  if ((ib != 2 && ib != 4) || ni % 3 != 0 || nv == 0 || ni == 0)
    return fail(error, "inconsistent vmesh header");
  const qsizetype need = kHeader + qsizetype(ns) * 12 +
                         qsizetype(nv) * 32 + qsizetype(ni) * qsizetype(ib);
  if (ns > 64 || b.size() != need)
    return fail(error, "vmesh size does not match its header");

  float bb[6];
  for (int i = 0; i < 6; ++i)
    readLe(b, 24 + 4 * i, bb[i]);
  out.bboxMin = QVector3D(bb[0], bb[1], bb[2]);
  out.bboxMax = QVector3D(bb[3], bb[4], bb[5]);

  out.submeshes.clear();
  qsizetype off = kHeader;
  for (uint32_t i = 0; i < ns; ++i, off += 12) {
    VegSubmesh s;
    uint32_t mat = 0;
    readLe(b, off, s.firstIndex);
    readLe(b, off + 4, s.indexCount);
    readLe(b, off + 8, mat);
    s.material = int(mat);
    if (uint64_t(s.firstIndex) + s.indexCount > ni || s.indexCount % 3 != 0)
      return fail(error, "submesh index range out of bounds");
    out.submeshes.push_back(s);
  }

  out.vertices.resize(size_t(nv) * VegMesh::kFloatsPerVertex);
  std::memcpy(out.vertices.data(), b.constData() + off,
              size_t(nv) * 32);
  for (float f : out.vertices)
    if (!std::isfinite(f))
      return fail(error, "non-finite vertex data");
  off += qsizetype(nv) * 32;

  out.indices.resize(ni);
  for (uint32_t i = 0; i < ni; ++i) {
    uint32_t v = 0;
    if (ib == 2) {
      uint16_t s = 0;
      readLe(b, off + qsizetype(i) * 2, s);
      v = s;
    } else {
      readLe(b, off + qsizetype(i) * 4, v);
    }
    if (v >= nv)
      return fail(error, "index out of range");
    out.indices[i] = v;
  }
  return true;
}

bool loadVegetationMaterialImages(VegMaterial &m, QString *error) {
  initVegetationResource();
  if (!loadImage(m.diffusePath, QImage::Format_RGBA8888, m.diffuse, error) ||
      !loadImage(m.normalPath, QImage::Format_RGBA8888, m.normal, error) ||
      !loadImage(m.armPath, QImage::Format_RGBA8888, m.arm, error))
    return false;
  if (!m.alphaPath.isEmpty() &&
      !loadImage(m.alphaPath, QImage::Format_Grayscale8, m.alpha, error))
    return false;
  return true;
}

bool loadVegetationAssets(VegAssets &out, QString *error, const QString &root,
                          bool withImages) {
  initVegetationResource();
  out.species.clear();
  QByteArray json;
  if (!readFile(root + "/manifest.json", json, error))
    return false;
  QJsonParseError pe;
  const QJsonDocument doc = QJsonDocument::fromJson(json, &pe);
  if (!doc.isObject())
    return fail(error, "manifest.json: " + pe.errorString());

  for (const QJsonValue &sv : doc.object().value("species").toArray()) {
    const QJsonObject so = sv.toObject();
    VegSpeciesAssets sp;
    sp.id = so.value("id").toString();
    sp.category = so.value("category").toString();
    const QString dir = root + "/" + so.value("directory").toString() + "/";
    if (sp.id.isEmpty() || dir.endsWith("//"))
      return fail(error, "manifest.json: species without id or directory");

    for (const QJsonValue &mv : so.value("materials").toArray()) {
      const QJsonObject mo = mv.toObject();
      VegMaterial m;
      m.name = mo.value("name").toString();
      m.doubleSided = mo.value("doubleSided").toBool(true);
      m.diffusePath = dir + mo.value("diffuse").toString();
      m.normalPath = dir + mo.value("normal").toString();
      m.armPath = dir + mo.value("arm").toString();
      const QString alpha = mo.value("alpha").toString();
      if (!alpha.isEmpty() && mo.value("alphaMode").toString() == "MASK") {
        m.alphaPath = dir + alpha;
        m.alphaCutoff = float(mo.value("alphaCutoff").toDouble(0.5));
        m.impostorCutoff = float(mo.value("impostorCutoff").toDouble(double(m.alphaCutoff)));
      }
      if (withImages && !loadVegetationMaterialImages(m, error))
        return false;
      sp.materials.push_back(std::move(m));
    }

    for (const QJsonValue &vv : so.value("variants").toArray()) {
      const QJsonObject vo = vv.toObject();
      VegVariant v;
      v.name = vo.value("name").toString();
      v.bboxMin = vec3(vo.value("bboxMin"));
      v.bboxMax = vec3(vo.value("bboxMax"));
      v.nominalHeight = float(vo.value("nominalHeight").toDouble());
      const QJsonArray fp = vo.value("footprint").toArray();
      v.footprintX = float(fp.at(0).toDouble());
      v.footprintZ = float(fp.at(1).toDouble());
      v.trunkRadius = float(vo.value("trunkRadius").toDouble());
      const QJsonArray lods = vo.value("lods").toArray();
      if (lods.size() != kVegLodCount)
        return fail(error, v.name + ": expected 3 LODs");
      for (int l = 0; l < kVegLodCount; ++l) {
        const QString file = lods.at(l).toObject().value("file").toString();
        v.lods[l].tiers = lods.at(l).toObject().value("tiers").toInt(0);
        QByteArray bytes;
        QString why;
        if (!readFile(dir + file, bytes, error))
          return false;
        if (!parseVMesh(bytes, v.lods[l], &why))
          return fail(error, dir + file + ": " + why);
        for (const VegSubmesh &s : v.lods[l].submeshes)
          if (s.material < 0 || s.material >= int(sp.materials.size()))
            return fail(error, dir + file + ": material index out of range");
      }
      sp.variants.push_back(std::move(v));
    }
    out.species.push_back(std::move(sp));
  }
  return !out.species.empty() || fail(error, "manifest.json: no species");
}

} // namespace vsim
