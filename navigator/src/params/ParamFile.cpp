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
#include "ParamFile.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QSet>
#include <cmath>

namespace params {

QString ParamFile::valueText(const Entry &e, double v) {
  if (e.type != Type::F32)
    return QString::number(qint64(std::llround(v)));
  // The shortest decimal that reads back as the same float: 0.1f is "0.1", not
  // 0.10000000149.
  const float f = float(v);
  for (int p = 1; p <= 9; ++p) {
    const QByteArray s = QByteArray::number(double(f), 'g', p);
    if (s.toFloat() == f)
      return QString::fromLatin1(s);
  }
  return QString::number(double(f), 'g', 9);
}

bool ParamFile::equalValues(const Entry &e, double a, double b) {
  return e.type == Type::F32 ? float(a) == float(b) : a == b;
}

static bool inFile(const Entry &e) { return e.group != 0 && !(e.flags & ReadOnly); }

bool ParamFile::save(const Catalog &cat, const QHash<QString, VehicleValue> &vehicle, int sysid,
                     const QDateTime &when, QByteArray *out, QString *why) {
  const auto fail = [&](const QString &m) {
    if (why)
      *why = m;
    return false;
  };
  if (cat.isEmpty())
    return fail(QStringLiteral("there is no parameter table"));
  QVector<QPair<QString, QString>> list; // id, value text
  int staged = 0;
  QStringList missing;
  for (const Entry &e : cat.entries()) {
    if (!inFile(e))
      continue;
    const auto it = vehicle.constFind(e.id);
    if (it == vehicle.constEnd() || !it->have) {
      missing << e.id;
      continue;
    }
    if (it->flags & Staged)
      ++staged;
    list.push_back({e.id, valueText(e, it->v)});
  }
  if (!missing.isEmpty())
    return fail(QStringLiteral("the table is incomplete (no value for %1)").arg(missing.first()));
  if (staged > 0)
    return fail(QStringLiteral("%1 value(s) are staged, not yet on the vehicle: commit or discard "
                               "them first, a file must hold what the vehicle runs")
                    .arg(staged));
  if (list.isEmpty())
    return fail(QStringLiteral("the table has no parameters to save"));

  // Written by hand, not by QJsonDocument: it prints a double with 17 digits,
  // and the file wants each number as its shortest text.
  QByteArray b = "{\n";
  b += "  \"format\": " + QByteArray::number(kFormat) + ",\n";
  b += "  \"table_crc\": " + QByteArray::number(qulonglong(cat.tableCrc())) + ",\n";
  b += "  \"saved_at\": \"" + when.toUTC().toString(Qt::ISODate).toLatin1() + "\",\n";
  b += "  \"sysid\": " + QByteArray::number(sysid) + ",\n";
  b += "  \"params\": [\n";
  for (int i = 0; i < list.size(); ++i) {
    b += "    {\"id\": \"" + list[i].first.toLatin1() + "\", \"value\": " +
         list[i].second.toLatin1() + "}";
    b += i + 1 < list.size() ? ",\n" : "\n";
  }
  b += "  ]\n}\n";
  *out = b;
  return true;
}

bool ParamFile::parse(const QByteArray &data, const Catalog &cat, Loaded *out, QString *why) {
  const auto fail = [&](const QString &m) {
    if (why)
      *why = m;
    return false;
  };
  *out = Loaded();
  if (data.size() > kMaxBytes)
    return fail(QStringLiteral("the file is too large for a parameter file (over %1 KB)")
                    .arg(kMaxBytes / 1024));
  QJsonParseError pe;
  const QJsonDocument doc = QJsonDocument::fromJson(data, &pe);
  if (pe.error != QJsonParseError::NoError || !doc.isObject())
    return fail(QStringLiteral("not a parameter file (not JSON: %1)").arg(pe.errorString()));
  const QJsonObject root = doc.object();
  if (root.value(QStringLiteral("format")).toInt(-1) != kFormat)
    return fail(QStringLiteral("a parameter file of a format this version does not read (format %1)")
                    .arg(root.value(QStringLiteral("format")).toVariant().toString()));
  if (!root.value(QStringLiteral("params")).isArray())
    return fail(QStringLiteral("not a parameter file (no list of parameters)"));

  const QJsonValue crc = root.value(QStringLiteral("table_crc"));
  if (crc.isDouble() && crc.toDouble() >= 0 && crc.toDouble() <= 4294967295.0) {
    out->meta.haveCrc = true;
    out->meta.tableCrc = quint32(crc.toDouble());
  }
  out->meta.savedAt = root.value(QStringLiteral("saved_at")).toString();
  out->meta.sysid = root.value(QStringLiteral("sysid")).toInt(-1);

  QSet<QString> seen;
  for (const QJsonValue &v : root.value(QStringLiteral("params")).toArray()) {
    const QJsonObject o = v.toObject();
    const QString id = o.value(QStringLiteral("id")).toString();
    const auto problem = [&](const QString &m) { out->problems.push_back({id, m}); };
    if (id.isEmpty() || id.size() > Catalog::kMaxIdLen) {
      out->problems.push_back({QString(), QStringLiteral("an entry without a usable id")});
      continue;
    }
    if (seen.contains(id)) {
      problem(QStringLiteral("listed more than once"));
      continue;
    }
    seen.insert(id);
    const Entry *e = cat.byId(id);
    if (!e) {
      problem(QStringLiteral("not a parameter of this table"));
      continue;
    }
    if (!inFile(*e)) {
      problem(QStringLiteral("not a tuning parameter (live or read-only), it does not belong in a file"));
      continue;
    }
    const QJsonValue val = o.value(QStringLiteral("value"));
    if (!val.isDouble()) {
      problem(QStringLiteral("the value is not a number"));
      continue;
    }
    const double d = val.toDouble();
    QString r;
    if (!std::isfinite(d) || !Catalog::accepts(*e, d, &r)) {
      problem(r.isEmpty() ? QStringLiteral("the value is not a finite number") : r);
      continue;
    }
    out->rows.push_back({id, d});
  }
  return true;
}

bool ParamFile::readFile(const QString &path, const Catalog &cat, Loaded *out, QString *why) {
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly)) {
    if (why)
      *why = QStringLiteral("cannot open %1: %2").arg(path, f.errorString());
    return false;
  }
  if (f.size() > kMaxBytes) { // never read a big file to find out it is one
    if (why)
      *why = QStringLiteral("the file is too large for a parameter file (over %1 KB)")
                 .arg(kMaxBytes / 1024);
    return false;
  }
  return parse(f.read(kMaxBytes + 1), cat, out, why);
}

bool ParamFile::writeFile(const QString &path, const QByteArray &data, QString *why) {
  QSaveFile f(path);
  if (!f.open(QIODevice::WriteOnly) || f.write(data) != data.size() || !f.commit()) {
    if (why)
      *why = QStringLiteral("cannot write %1: %2").arg(path, f.errorString());
    return false;
  }
  return true;
}

QVector<ParamFile::DiffRow> ParamFile::diff(const Loaded &f, const Catalog &cat,
                                            const QHash<QString, VehicleValue> &vehicle) {
  QVector<DiffRow> rows;
  for (const Row &r : f.rows) {
    const Entry *e = cat.byId(r.id);
    if (!e)
      continue;
    DiffRow d;
    d.id = r.id;
    d.group = e->group;
    d.file = r.value;
    const auto it = vehicle.constFind(r.id);
    d.haveVehicle = it != vehicle.constEnd() && it->have;
    if (d.haveVehicle) {
      d.vehicle = it->v;
      d.same = equalValues(*e, it->v, r.value);
    }
    rows.push_back(d);
  }
  return rows;
}

bool ParamFile::sameTable(const Loaded &f, const Catalog &cat, QString *why) {
  if (!f.meta.haveCrc) {
    if (why)
      *why = QStringLiteral("the file does not say which table it was made for");
    return false;
  }
  if (f.meta.tableCrc != cat.tableCrc()) {
    if (why)
      *why = QStringLiteral("the file was made for another parameter table (%1, this one is %2)")
                 .arg(f.meta.tableCrc, 8, 16, QLatin1Char('0'))
                 .arg(cat.tableCrc(), 8, 16, QLatin1Char('0'));
    return false;
  }
  return true;
}

} // namespace params
