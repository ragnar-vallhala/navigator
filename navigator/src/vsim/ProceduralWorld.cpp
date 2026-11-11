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

namespace vsim {

bool isFieldArenaBiome(const QString &biome) {
  return procgen::isFieldBiome(biome.toLower().toStdString());
}

bool isStreamedBiome(const QString &biome) {
  // "meadow" is the legacy finite biome of old saved worlds: it streams now.
  return biome.compare(QStringLiteral("endless"), Qt::CaseInsensitive) == 0 ||
         biome.compare(QStringLiteral("meadow"), Qt::CaseInsensitive) == 0 ||
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
  // Streamed: size is just the obstacle-layout extent; resolution is legacy.
  w.proceduralSizeM = d.sizeM;
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
    ob.style = o.style;
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

} // namespace vsim
