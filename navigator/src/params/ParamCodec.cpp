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
#include "ParamCodec.h"

#include "TxSeq.h"
#include "../protocol/NavlinkText.h"

#include <cmath>
#include <cstring>

extern "C" {
#include "navlink_msgs.h"
}

namespace params {
namespace ParamCodec {
namespace {
QByteArray frame(const uint8_t *buf, size_t n) {
  return QByteArray(reinterpret_cast<const char *>(buf), static_cast<int>(n));
}
// Writes the id into the 16-byte field, NUL-padded. False (nothing usable
// written) if it does not fit or is not Latin-1: truncating it would address a
// different parameter, so the caller sends nothing instead.
bool putId(char (&dst)[16], const QString &id) {
  std::memset(dst, 0, sizeof dst);
  const QByteArray b = id.toLatin1();
  if (id.isEmpty() || size_t(b.size()) > sizeof dst ||
      QString::fromLatin1(b) != id) {
    std::memset(dst, 0, sizeof dst);
    return false;
  }
  std::memcpy(dst, b.constData(), size_t(b.size()));
  return true;
}
} // namespace

bool toWire(Type t, double v, quint8 out[8]) {
  std::memset(out, 0, 8);
  if (!std::isfinite(v))
    return false;
  switch (t) {
  case Type::F32: {
    if (std::fabs(v) > double(3.4028234663852886e38))
      return false;
    const float f = float(v);
    std::memcpy(out, &f, 4);
    return true;
  }
  case Type::U8: case Type::U16: case Type::U32: {
    const double hi = t == Type::U8 ? 255.0 : t == Type::U16 ? 65535.0 : 4294967295.0;
    if (v != std::floor(v) || v < 0 || v > hi)
      return false;
    const quint32 u = quint32(v);
    for (int i = 0; i < typeSize(t); ++i)
      out[i] = quint8(u >> (8 * i));
    return true;
  }
  case Type::I8: case Type::I16: case Type::I32: {
    const double hi = t == Type::I8 ? 127.0 : t == Type::I16 ? 32767.0 : 2147483647.0;
    if (v != std::floor(v) || v < -hi - 1 || v > hi)
      return false;
    const qint32 s = qint32(v);
    const quint32 u = quint32(s);
    for (int i = 0; i < typeSize(t); ++i)
      out[i] = quint8(u >> (8 * i));
    return true;
  }
  }
  return false;
}

bool valueOf(const ValueMsg &m, double &out) {
  auto u32 = [&](int n) {
    quint32 u = 0;
    for (int i = 0; i < n; ++i)
      u |= quint32(m.raw[i]) << (8 * i);
    return u;
  };
  switch (Type(m.type)) {
  case Type::F32: {
    float f;
    std::memcpy(&f, m.raw, 4);
    out = double(f);
    return true;
  }
  case Type::U8: out = double(u32(1)); return true;
  case Type::U16: out = double(u32(2)); return true;
  case Type::U32: out = double(u32(4)); return true;
  case Type::I8: out = double(qint8(u32(1))); return true;
  case Type::I16: out = double(qint16(u32(2))); return true;
  case Type::I32: out = double(qint32(u32(4))); return true;
  }
  return false;
}

ValueMsg fromWire(const navlink_param_value_t &w) {
  ValueMsg m;
  m.id = navtext::fieldLatin1(w.param_id);
  m.index = w.index;
  m.count = w.count;
  m.generation = w.generation;
  m.type = w.type;
  std::memcpy(m.raw, w.value, 8);
  m.flags = w.flags;
  return m;
}

QByteArray encodeRequestList(quint8 devId) {
  navlink_param_request_list_t m{};
  m.target_sys = devId;
  m.target_comp = 1;
  uint8_t buf[NAVLINK_MAX_FRAME];
  return frame(buf, navlink_param_request_list_encode(buf, &m, TxSeq::next(), 0xFF, 1));
}

QByteArray encodeRequestReadById(const QString &id, quint8 devId) {
  navlink_param_request_read_t m{};
  m.target_sys = devId;
  m.target_comp = 1;
  if (!putId(m.param_id, id))
    return {}; // an id that does not fit 16 bytes is never sent cut short
  m.index = -1;
  uint8_t buf[NAVLINK_MAX_FRAME];
  return frame(buf, navlink_param_request_read_encode(buf, &m, TxSeq::next(), 0xFF, 1));
}

QByteArray encodeRequestReadByIndex(quint16 index, quint8 devId) {
  navlink_param_request_read_t m{};
  m.target_sys = devId;
  m.target_comp = 1;
  m.index = qint16(index);
  uint8_t buf[NAVLINK_MAX_FRAME];
  return frame(buf, navlink_param_request_read_encode(buf, &m, TxSeq::next(), 0xFF, 1));
}

QByteArray encodeSet(const QString &id, Type t, double v, quint8 reqSeq,
                     quint8 devId) {
  navlink_param_set_t m{};
  m.target_sys = devId;
  m.target_comp = 1;
  if (!putId(m.param_id, id))
    return {}; // an id that does not fit 16 bytes is never sent cut short
  m.type = quint8(t);
  if (!toWire(t, v, m.value) || reqSeq == 0)
    return {}; // never put a value on the wire that the type cannot hold
  m.req_seq = reqSeq;
  uint8_t buf[NAVLINK_MAX_FRAME];
  return frame(buf, navlink_param_set_encode(buf, &m, TxSeq::next(), 0xFF, 1));
}

QByteArray encodeCommit(quint8 reqSeq, quint8 op, quint8 group, quint8 devId) {
  if (reqSeq == 0 || op > 1)
    return {};
  navlink_cmd_param_commit_t m{};
  m.target_sys = devId;
  m.target_comp = 1;
  m.req_seq = reqSeq;
  m.op = op;
  m.group = group;
  uint8_t buf[NAVLINK_MAX_FRAME];
  return frame(buf, navlink_cmd_param_commit_encode(buf, &m, TxSeq::next(), 0xFF, 1));
}

} // namespace ParamCodec
} // namespace params
