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
#include "ParamCatalog.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>

#include <cmath>
#include <limits>

namespace params {

int typeSize(Type t) {
  switch (t) {
  case Type::U8:
  case Type::I8:
    return 1;
  case Type::U16:
  case Type::I16:
    return 2;
  case Type::U32:
  case Type::I32:
  case Type::F32:
    return 4;
  }
  return 0;
}

const char *typeName(Type t) {
  switch (t) {
  case Type::U8: return "U8";
  case Type::I8: return "I8";
  case Type::U16: return "U16";
  case Type::I16: return "I16";
  case Type::U32: return "U32";
  case Type::I32: return "I32";
  case Type::F32: return "F32";
  }
  return "?";
}

namespace {
bool typeFromName(const QString &n, Type &t) {
  static const struct { const char *name; Type t; } kT[] = {
      {"U8", Type::U8},   {"I8", Type::I8},   {"U16", Type::U16},
      {"I16", Type::I16}, {"U32", Type::U32}, {"I32", Type::I32},
      {"F32", Type::F32}};
  for (const auto &e : kT)
    if (n == QLatin1String(e.name)) {
      t = e.t;
      return true;
    }
  return false;
}

// The representable range of an integer type; F32 is any finite float.
void typeRange(Type t, double &lo, double &hi) {
  switch (t) {
  case Type::U8: lo = 0; hi = 255; return;
  case Type::I8: lo = -128; hi = 127; return;
  case Type::U16: lo = 0; hi = 65535; return;
  case Type::I16: lo = -32768; hi = 32767; return;
  case Type::U32: lo = 0; hi = 4294967295.0; return;
  case Type::I32: lo = -2147483648.0; hi = 2147483647.0; return;
  case Type::F32:
    hi = double(std::numeric_limits<float>::max());
    lo = -hi;
    return;
  }
}

bool fail(QString *err, const QString &m) {
  if (err)
    *err = m;
  return false;
}
} // namespace

bool Catalog::accepts(const Entry &e, double v, QString *why) {
  auto no = [&](const QString &m) {
    if (why)
      *why = m;
    return false;
  };
  if (!std::isfinite(v))
    return no(QStringLiteral("not a finite number"));
  double lo, hi;
  typeRange(e.type, lo, hi);
  if (e.type != Type::F32 && v != std::floor(v))
    return no(QStringLiteral("%1 takes whole numbers").arg(typeName(e.type)));
  if (v < lo || v > hi)
    return no(QStringLiteral("outside what %1 can hold").arg(typeName(e.type)));
  if (v < e.min || v > e.max)
    return no(QStringLiteral("outside %1 .. %2").arg(e.min).arg(e.max));
  return true;
}

bool Catalog::load(const QByteArray &json, Catalog &out, QString *err) {
  out = Catalog{};
  QJsonParseError pe;
  const QJsonDocument doc = QJsonDocument::fromJson(json, &pe);
  if (doc.isNull() || !doc.isObject())
    return fail(err, QStringLiteral("not a JSON object (%1)").arg(pe.errorString()));
  const QJsonObject root = doc.object();
  if (root.value(QLatin1String("catalog")).toInt(-1) != 1)
    return fail(err, QStringLiteral("unknown catalog version"));
  const QJsonValue crcV = root.value(QLatin1String("table_crc"));
  if (!crcV.isDouble() || crcV.toDouble() < 0 || crcV.toDouble() > 4294967295.0 ||
      crcV.toDouble() != std::floor(crcV.toDouble()))
    return fail(err, QStringLiteral("table_crc is missing or not a 32-bit number"));

  Catalog c;
  c.m_crc = static_cast<quint32>(crcV.toDouble());

  const QJsonObject groups = root.value(QLatin1String("groups")).toObject();
  for (auto it = groups.begin(); it != groups.end(); ++it) {
    bool ok = false;
    const int gid = it.key().toInt(&ok);
    if (!ok || gid < 1 || gid > 255)
      return fail(err, QStringLiteral("bad group id '%1'").arg(it.key()));
    Group g;
    g.id = gid;
    g.name = it.value().toObject().value(QLatin1String("name")).toString();
    g.doc = it.value().toObject().value(QLatin1String("doc")).toString();
    c.m_groups.push_back(g);
  }

  static const QRegularExpression idRe(QStringLiteral("^[A-Z][A-Z0-9_]*$"));
  const QJsonArray arr = root.value(QLatin1String("params")).toArray();
  if (arr.isEmpty())
    return fail(err, QStringLiteral("no parameters"));
  for (int i = 0; i < arr.size(); ++i) {
    const QJsonObject o = arr[i].toObject();
    Entry e;
    e.id = o.value(QLatin1String("id")).toString();
    if (e.id.isEmpty() || e.id.size() > kMaxIdLen || !idRe.match(e.id).hasMatch())
      return fail(err, QStringLiteral("entry %1: bad id '%2'").arg(i).arg(e.id));
    if (c.m_byId.contains(e.id))
      return fail(err, QStringLiteral("duplicate id %1").arg(e.id));
    if (!typeFromName(o.value(QLatin1String("type")).toString(), e.type))
      return fail(err, QStringLiteral("%1: unknown type").arg(e.id));
    for (const char *k : {"min", "max", "default"})
      if (!o.value(QLatin1String(k)).isDouble())
        return fail(err, QStringLiteral("%1: %2 is not a number").arg(e.id, k));
    e.min = o.value(QLatin1String("min")).toDouble();
    e.max = o.value(QLatin1String("max")).toDouble();
    e.def = o.value(QLatin1String("default")).toDouble();
    if (!std::isfinite(e.min) || !std::isfinite(e.max) || !std::isfinite(e.def))
      return fail(err, QStringLiteral("%1: a bound is not finite").arg(e.id));
    if (e.min > e.max)
      return fail(err, QStringLiteral("%1: min is above max").arg(e.id));
    double lo, hi;
    typeRange(e.type, lo, hi);
    if (e.min < lo || e.max > hi)
      return fail(err, QStringLiteral("%1: range does not fit %2").arg(e.id, typeName(e.type)));
    if (e.def < e.min || e.def > e.max)
      return fail(err, QStringLiteral("%1: default is outside the range").arg(e.id));
    if (e.type != Type::F32 &&
        (e.min != std::floor(e.min) || e.max != std::floor(e.max) ||
         e.def != std::floor(e.def)))
      return fail(err, QStringLiteral("%1: integer type with a fractional bound").arg(e.id));
    const int fb = o.value(QLatin1String("flags_byte")).toInt(-1);
    if (fb < 0 || (fb & ~0x0F))
      return fail(err, QStringLiteral("%1: bad flags").arg(e.id));
    e.flags = static_cast<quint8>(fb);
    e.group = o.value(QLatin1String("group")).toInt(0);
    if (e.group < 0 || e.group > 255)
      return fail(err, QStringLiteral("%1: bad group").arg(e.id));
    e.unit = o.value(QLatin1String("unit")).toString();
    e.doc = o.value(QLatin1String("doc")).toString();
    e.rangeWhy = o.value(QLatin1String("range_why")).toString();
    e.index = o.value(QLatin1String("index")).toInt(-1);
    if (e.index != i)
      return fail(err, QStringLiteral("%1: index %2 is not its position %3")
                           .arg(e.id).arg(e.index).arg(i));
    c.m_byId.insert(e.id, i);
    c.m_entries.push_back(e);
  }
  if (!c.m_byId.contains(QLatin1String(kCrcId)))
    return fail(err, QStringLiteral("%1 is missing").arg(kCrcId));
  if (!(c.entries()[c.m_byId.value(QLatin1String(kCrcId))].flags & ReadOnly))
    return fail(err, QStringLiteral("%1 must be read-only").arg(kCrcId));

  // Groups commit as one unit: every member must exist in the group table, none
  // read-only, and locked/unlocked must not mix (the firmware refuses a group
  // that does, so a catalog with one cannot be the firmware's).
  QHash<int, int> lockedOf; // group -> 0 unlocked, 1 locked
  for (const Entry &e : c.m_entries) {
    if (!e.group)
      continue;
    if (!c.group(e.group))
      return fail(err, QStringLiteral("%1: group %2 is not defined").arg(e.id).arg(e.group));
    if (e.flags & ReadOnly)
      return fail(err, QStringLiteral("%1: a read-only entry in group %2").arg(e.id).arg(e.group));
    const int l = (e.flags & LockedWhenArmed) ? 1 : 0;
    if (lockedOf.contains(e.group) && lockedOf[e.group] != l)
      return fail(err, QStringLiteral("group %1 mixes locked and unlocked entries").arg(e.group));
    lockedOf[e.group] = l;
  }
  out = std::move(c);
  return true;
}

bool Catalog::loadFile(const QString &path, Catalog &out, QString *err) {
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly))
    return fail(err, QStringLiteral("cannot open %1").arg(path));
  return load(f.readAll(), out, err);
}

const Entry *Catalog::byId(const QString &id) const {
  const auto it = m_byId.constFind(id);
  return it == m_byId.constEnd() ? nullptr : &m_entries[it.value()];
}
const Entry *Catalog::byIndex(int index) const {
  return (index >= 0 && index < m_entries.size()) ? &m_entries[index] : nullptr;
}
const Group *Catalog::group(int id) const {
  for (const Group &g : m_groups)
    if (g.id == id)
      return &g;
  return nullptr;
}
QVector<int> Catalog::membersOf(int group) const {
  QVector<int> r;
  for (const Entry &e : m_entries)
    if (group == 0 ? e.group != 0 : e.group == group)
      r.push_back(e.index);
  return r;
}

} // namespace params
