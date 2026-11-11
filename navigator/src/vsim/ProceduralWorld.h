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
// ProceduralWorld.h — Qt adapter over the Qt-free procgen core: which biomes
// stream, and the world/wind preset lists and appliers for the UI.
#pragma once

#include "VsimTypes.h"           // vsim::WorldConfig
#include "procgen/WorldPresets.h"

#include <QList>
#include <QPair>

namespace vsim {

// True for the preset biomes (hills/alpine/flat/desert/forest/racing/yard):
// terrain from w.field, streamed endlessly like the "endless" biome.
bool isFieldArenaBiome(const QString &biome);

// True for every procedural biome the app streams: "endless", the preset
// biomes above, and the legacy "meadow" of old saved worlds. There is no finite
// procedural arena any more; only imported meshes are finite.
bool isStreamedBiome(const QString &biome);

// World presets (id, label) and wind presets (id, label), for the UI.
QList<QPair<QString, QString>> worldPresetList();
QList<QPair<QString, QString>> windPresetList();

// Replace the procedural fields of `w` (biome, field, flora, size, resolution,
// obstacles) with preset `id` generated from `seed`. Returns false (w untouched)
// for an unknown id. Deterministic: same id + seed -> same config.
bool applyWorldPreset(WorldConfig &w, const QString &id, quint32 seed);

// Wind preset `id` for `seed`; *ok=false (and a disabled WindConfig) if unknown.
WindConfig windPresetConfig(const QString &id, quint32 seed, bool *ok = nullptr);

} // namespace vsim
