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
// VegetationAssets.h — CPU side of the vegetation models (Qt only, no GL).
//
// Reads assets/vegetation (embedded as the Qt resource ":/vegetation") into
// plain memory: per species, per variant, per LOD a triangle mesh, and per
// species the materials its submeshes refer to. The file formats and colour
// spaces are documented in tools/vegetation/build_vegetation.py:
//   * geometry: glTF frame, +Y up, metres, base at y = 0, pivot at the base
//     centre; interleaved position(3) normal(3) uv(2)
//   * UV (0,0) is the TOP-left of the image: upload the QImage unflipped;
//     bark/leaf textures tile, so wrap = repeat
//   * diffuse: sRGB; normal: linear, tangent space, +Y up; arm: linear,
//     R = ambient occlusion, G = roughness; alpha: 8-bit coverage, tested
//     against alphaCutoff
#pragma once

#include <QImage>
#include <QString>
#include <QVector3D>

#include <cstdint>
#include <vector>

namespace vsim {

inline constexpr int kVegLodCount = 3;

struct VegMaterial {
  QString name;
  // Resource paths of the images (set by the loader even when the images are
  // not decoded, see loadVegetationMaterialImages).
  QString diffusePath, normalPath, armPath, alphaPath;
  QImage diffuse; // sRGB, Format_RGBA8888
  QImage normal;  // linear, Format_RGBA8888
  QImage arm;     // linear, Format_RGBA8888
  QImage alpha;   // linear coverage, Format_Grayscale8; null when opaque
  float alphaCutoff = 0.5f;
  bool doubleSided = true;
  bool cutout() const { return !alphaPath.isEmpty(); }
};

struct VegSubmesh {
  uint32_t firstIndex = 0;
  uint32_t indexCount = 0;
  int material = 0; // index into VegSpecies::materials
};

struct VegMesh {
  static constexpr int kFloatsPerVertex = 8; // position, normal, uv
  std::vector<float> vertices;
  std::vector<uint32_t> indices; // CCW triangles, absolute vertex ids
  std::vector<VegSubmesh> submeshes;
  QVector3D bboxMin, bboxMax;
  int tiers = 0; // card conifers: needle-branch tiers in this LOD, else 0
  uint32_t vertexCount() const {
    return uint32_t(vertices.size() / kFloatsPerVertex);
  }
  uint32_t triangleCount() const { return uint32_t(indices.size() / 3); }
};

struct VegVariant {
  QString name;
  VegMesh lods[kVegLodCount]; // 0 = full detail
  QVector3D bboxMin, bboxMax; // whole-variant box (LOD 0 frame)
  float nominalHeight = 0.0f; // metres, as modelled
  float footprintX = 0.0f;    // metres, as modelled
  float footprintZ = 0.0f;
  float trunkRadius = 0.0f;   // metres as modelled, 0 when not measured (trees)
};

struct VegSpeciesAssets {
  QString id;
  QString category; // tree | shrub | ground_plant | rock
  std::vector<VegMaterial> materials;
  std::vector<VegVariant> variants;
};

struct VegAssets {
  std::vector<VegSpeciesAssets> species;
  const VegSpeciesAssets *find(const QString &id) const;
};

// Loads every species listed in <root>/manifest.json. On failure returns false
// and sets *error (a file name and the reason); `out` is then unspecified.
// `root` defaults to the embedded resource. With withImages = false only the
// meshes and the material descriptions are read; decode a species' textures
// later (and on any thread) with loadVegetationMaterialImages.
bool loadVegetationAssets(VegAssets &out, QString *error,
                          const QString &root = QStringLiteral(":/vegetation"),
                          bool withImages = true);

// Decodes the textures of one material from its recorded paths.
bool loadVegetationMaterialImages(VegMaterial &m, QString *error);

// Parses one .vmesh (see the format in build_vegetation.py). Validates counts,
// submesh ranges and index values; false + *error on any inconsistency.
bool parseVMesh(const QByteArray &bytes, VegMesh &out, QString *error);

} // namespace vsim
