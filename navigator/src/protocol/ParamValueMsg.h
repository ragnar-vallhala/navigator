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

#include <QMetaType>
#include <QString>
#include <QtGlobal>

// A decoded PARAM_VALUE as a plain GCS struct: the receive router fills it and
// hands it on, so nothing outside the router sees a generated navlink type.
namespace params {
namespace ParamCodec {
struct ValueMsg {
  QString id;
  quint16 index = 0;
  quint16 count = 0;
  quint32 generation = 0;
  quint8 type = 0; // wire type number
  quint8 raw[8] = {};
  quint8 flags = 0; // ValueFlag bits
};
} // namespace ParamCodec
} // namespace params

Q_DECLARE_METATYPE(params::ParamCodec::ValueMsg)
