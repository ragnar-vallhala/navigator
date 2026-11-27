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

#include "ParamCatalog.h"
#include "ParamValueMsg.h"

#include <QByteArray>
#include <QString>

extern "C" {
#include "navlink_msgs.h"
}

// NavLink v2 frames for the parameter service (GCS -> FC) and the decode of
// PARAM_VALUE (FC -> GCS). Values travel as their real type in 8 bytes, little
// endian, zero padded (never a float bit-cast).
namespace params {
namespace ParamCodec {

// The value as a double; false for a type this client does not know.
bool valueOf(const ValueMsg &m, double &out);
// `v` as the 8 wire bytes of `t`. False when `v` is not exactly representable
// (non-finite, fractional for an integer type, outside the type).
bool toWire(Type t, double v, quint8 out[8]);
// Fill a ValueMsg from a decoded navlink_param_value_t.
ValueMsg fromWire(const navlink_param_value_t &w);

QByteArray encodeRequestList(quint8 devId = 42);
// An id is at most 16 Latin-1 characters; a longer or non-Latin-1 one encodes to
// an empty frame (it is never sent cut short, which would name another
// parameter). Same for encodeSet.
QByteArray encodeRequestReadById(const QString &id, quint8 devId = 42);
QByteArray encodeRequestReadByIndex(quint16 index, quint8 devId = 42);
// reqSeq must be non-zero: 0 means "no acknowledgement wanted".
QByteArray encodeSet(const QString &id, Type t, double v, quint8 reqSeq,
                     quint8 devId = 42);
// op 0 commit, 1 discard; group 0 = every group and every unsaved parameter.
QByteArray encodeCommit(quint8 reqSeq, quint8 op, quint8 group, quint8 devId = 42);

} // namespace ParamCodec
} // namespace params
