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
#pragma once

#include "VsimTypes.h"

#include <QJsonObject>

namespace vsim {

// Portable vehicle file (.vveh) <-> GeometryConfig: the format the geometry
// editor's Save/Load Vehicle uses. The built-in airframe presets are checked
// against files in this format too.
QJsonObject geometryToJson(const GeometryConfig &c);
// Missing keys keep the GeometryConfig defaults.
GeometryConfig geometryFromJson(const QJsonObject &root);

} // namespace vsim
