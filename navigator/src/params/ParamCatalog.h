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

#include <QByteArray>
#include <QHash>
#include <QString>
#include <QVector>

// The parameter catalog the GCS knows (NavLink spec 12.2.1). It is loaded from
// params_catalog.json, which navlink's generator writes from params.json, the
// single source the firmware table is built from. The loader never reads
// params.json. A catalog the generator would have refused is refused here too,
// so a hand-edited or truncated file cannot put a bad range in front of a user.
namespace params {

// Wire type numbers (dialect param_type). Only the types a catalog may use.
enum class Type : quint8 { U8 = 1, I8, U16, I16, U32, I32, F32 = 9 };

// Catalog flags (params.json flags_byte).
enum Flag : quint8 {
  ReadOnly = 0x01,
  LockedWhenArmed = 0x02,
  RebootRequired = 0x04,
  Persisted = 0x08,
};

// PARAM_VALUE.flags bits.
enum ValueFlag : quint8 {
  Staged = 0x01,
  Unsaved = 0x02,
  NonDefault = 0x04,
  RebootPending = 0x08,
};

struct Entry {
  QString id;
  Type type = Type::F32;
  double min = 0, max = 0, def = 0;
  quint8 flags = 0;
  int group = 0; // 0 = applies at once, no group
  QString unit, doc, rangeWhy;
  int index = 0; // position in the table (PARAM_VALUE.index)
};

struct Group {
  int id = 0;
  QString name, doc;
};

class Catalog {
public:
  static constexpr const char *kCrcId = "SYS_PTBL_CRC";
  static constexpr int kMaxIdLen = 16;

  // Parse and validate. On failure returns false with the reason in *err and
  // leaves `out` empty.
  static bool load(const QByteArray &json, Catalog &out, QString *err);
  static bool loadFile(const QString &path, Catalog &out, QString *err);

  bool isEmpty() const { return m_entries.isEmpty(); }
  int count() const { return m_entries.size(); }
  quint32 tableCrc() const { return m_crc; }
  const QVector<Entry> &entries() const { return m_entries; }
  const QVector<Group> &groups() const { return m_groups; }
  const Entry *byId(const QString &id) const;
  const Entry *byIndex(int index) const;
  const Group *group(int id) const;
  QVector<int> membersOf(int group) const; // entry indexes, table order

  // True when `v` is finite, fits `e.type` (integral for integer types) and
  // lies inside [min,max]. `why` receives the reason when it does not.
  static bool accepts(const Entry &e, double v, QString *why = nullptr);

private:
  QVector<Entry> m_entries;
  QVector<Group> m_groups;
  QHash<QString, int> m_byId;
  quint32 m_crc = 0;
};

int typeSize(Type t);
const char *typeName(Type t);

} // namespace params
