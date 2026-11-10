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
#include "ProceduralWorld.h"

#include "procgen/TerrainGen.h"

#include <algorithm>

namespace vsim {

bool isFieldArenaBiome(const QString &biome) {
  return procgen::isFieldBiome(biome.toLower().toStdString());
}

bool isKnownBiome(const QString &biome) {
  return biome.compare(QStringLiteral("meadow"), Qt::CaseInsensitive) == 0 ||
         isFieldArenaBiome(biome);
}

QList<QPair<QString, QString>> worldPresetList() {
  QList<QPair<QString, QString>> l;
  for (const auto &p : procgen::worldPresets())
    l.append({QString::fromLatin1(p.id), QString::fromUtf8(p.label)});
  return l;
}

QList<QPair<QString, QString>> windPresetList() {
  QList<QPair<QString, QString>> l;
  for (const auto &p : procgen::windPresets())
    l.append({QString::fromLatin1(p.id), QString::fromUtf8(p.label)});
  return l;
}

bool applyWorldPreset(WorldConfig &w, const QString &id, quint32 seed) {
  procgen::PresetData d;
  if (!procgen::buildWorldPreset(id.toStdString(), seed, d))
    return false;
  w.proceduralBiome = QString::fromStdString(d.biome);
  w.proceduralSeed = seed;
  if (d.biome == "meadow") {
    // The original finite meadow keeps its own generator and defaults.
    w.proceduralSizeM = 256.0f;
    w.proceduralResolution = 192;
    w.field = procgen::FieldParams{};
    w.field.seed = seed;
    w.flora = procgen::FloraParams{};
    w.flora.seed = seed;
    w.obstacles.clear();
    return true;
  }
  w.proceduralSizeM = d.sizeM;
  w.proceduralResolution = d.resolution;
  w.field = d.field;
  w.flora = d.flora;
  w.obstacles.clear();
  w.obstacles.reserve(static_cast<int>(d.obstacles.size()));
  for (const procgen::PgObstacle &o : d.obstacles) {
    Obstacle ob;
    ob.type = o.type;
    ob.pos = QVector3D(o.x, o.y, o.zc);
    ob.size = QVector3D(o.sx, o.sy, o.sz);
    ob.rotate = QVector3D(0.0f, 0.0f, o.yawDeg);
    ob.restitution = 0.3f;
    w.obstacles.push_back(ob);
  }
  return true;
}

WindConfig windPresetConfig(const QString &id, quint32 seed, bool *ok) {
  procgen::WindPresetData d;
  const bool good = procgen::buildWindPreset(id.toStdString(), seed, d);
  if (ok)
    *ok = good;
  WindConfig c;
  if (!good)
    return c;
  c.steady = QVector3D(d.steadyN, d.steadyE, d.steadyD);
  c.gustAmp = d.gustAmp;
  c.gustPeriod = d.gustPeriod;
  c.turbSigma = d.turbSigma;
  c.turbTau = d.turbTau;
  c.enabled = d.enabled;
  return c;
}

procgen::TerrainField arenaField(const WorldConfig &w) {
  procgen::FieldParams p = w.field;
  p.seed = static_cast<uint32_t>(w.proceduralSeed);
  return procgen::TerrainField(p);
}

procgen::TerrainParams meadowParams(const WorldConfig &w) {
  procgen::TerrainParams p;
  p.seed = static_cast<uint32_t>(w.proceduralSeed);
  p.sizeM = w.proceduralSizeM > 1.0f ? w.proceduralSizeM : 256.0f;
  p.resolution = std::clamp(w.proceduralResolution, 2, 1024);
  return p;
}

procgen::Heightfield proceduralMeadowHeightfield(const WorldConfig &w) {
  return procgen::generateHeightfield(meadowParams(w));
}

LoadedMesh generateProceduralWorld(const WorldConfig &w) {
  LoadedMesh out;
  if (w.proceduralBiome.isEmpty() || !isKnownBiome(w.proceduralBiome))
    return out; // valid == false

  procgen::ProcMesh m;
  if (isFieldArenaBiome(w.proceduralBiome)) {
    const procgen::TerrainField f = arenaField(w);
    const float sz = w.proceduralSizeM > 1.0f ? w.proceduralSizeM : 256.0f;
    m = procgen::meshArena(f, sz, std::clamp(w.proceduralResolution, 4, 1024));
  } else {
    m = procgen::generateMeadow(meadowParams(w));
  }
  if (m.empty())
    return out;

  const std::size_t nverts = m.positions.size();
  out.positions.reserve(nverts);
  out.normals.reserve(nverts);
  out.colors.reserve(nverts);

  QVector3D lo(1e9f, 1e9f, 1e9f), hi(-1e9f, -1e9f, -1e9f);
  for (std::size_t i = 0; i < nverts; ++i) {
    const procgen::PgVec3 &v = m.positions[i];
    const procgen::PgVec3 &nrm = m.normals[i];
    const procgen::PgVec3 &c = m.colors[i];
    out.positions.emplace_back(v.x, v.y, v.z);
    out.normals.emplace_back(nrm.x, nrm.y, nrm.z);
    out.colors.emplace_back(c.x, c.y, c.z);
    lo.setX(std::min(lo.x(), v.x));
    lo.setY(std::min(lo.y(), v.y));
    lo.setZ(std::min(lo.z(), v.z));
    hi.setX(std::max(hi.x(), v.x));
    hi.setY(std::max(hi.y(), v.y));
    hi.setZ(std::max(hi.z(), v.z));
  }
  out.bboxMin = lo;
  out.bboxMax = hi;
  out.valid = true;
  return out;
}

} // namespace vsim
