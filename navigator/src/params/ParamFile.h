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

#include <QDateTime>
#include <QHash>
#include <QString>
#include <QVector>

namespace params {

// A parameter file: the values of the vehicle's writable, grouped parameters at
// one moment, as JSON. Pure (QtCore only, no widgets, no link): the panel's
// Save / Load buttons use it, and so can anything else.
//
//   {"format":1, "table_crc":<n>, "saved_at":"<UTC ISO time>", "sysid":<n>,
//    "params":[{"id":"RATE_ROLL_KP","value":0.05}, ...]}
//
// The list is an array, not an object, so a repeated id is seen (a JSON object
// would keep one of them without a word). Values are the float's shortest
// decimal that reads back as the same float, so a save followed by a load is
// "same" for every row. Group 0 (live values) and read-only parameters are not
// in a file: a file is a tune, not a snapshot of everything.
//
// Loading is parse first, check second, and sends nothing: a file with any
// problem (an unknown or read-only id, a value outside the catalog's range, a
// repeated id, not a number) reports it per id and must not be applied.
// saved_at and sysid are only shown; they are never checked. A table_crc that is
// not the vehicle's means the values were made for another table: the diff is
// shown, the apply is refused.
class ParamFile {
public:
  static constexpr int kFormat = 1;
  static constexpr qint64 kMaxBytes = qint64(1024) * 1024; // a tune is a few KB

  struct VehicleValue {
    double v = 0;
    quint8 flags = 0; // ValueFlag bits
    bool have = false;
  };
  struct Meta {
    bool haveCrc = false;
    quint32 tableCrc = 0;
    QString savedAt; // as written; empty when absent
    int sysid = -1;  // -1 when absent
  };
  struct Row {
    QString id;
    double value = 0;
  };
  struct Problem {
    QString id; // empty: the problem is not about one parameter
    QString text;
  };
  struct Loaded {
    Meta meta;
    QVector<Row> rows;         // the good ones, in file order
    QVector<Problem> problems; // anything that makes the file unfit to apply
    bool ok() const { return problems.isEmpty(); }
  };
  struct DiffRow {
    QString id;
    int group = 0;
    bool haveVehicle = false;
    double vehicle = 0, file = 0;
    bool same = false; // equal as the vehicle holds them (floats compared as floats)
  };

  // The file for the vehicle's current values. False, with the reason, when it
  // would not be what the vehicle runs: a value missing, or any value Staged.
  static bool save(const Catalog &cat, const QHash<QString, VehicleValue> &vehicle, int sysid,
                   const QDateTime &when, QByteArray *out, QString *why);
  // False only when this is not a parameter file at all (too large, not JSON,
  // not an object, a format this version does not know); then *why says why and
  // nothing is shown. Otherwise true, with any problems in out->problems.
  static bool parse(const QByteArray &data, const Catalog &cat, Loaded *out, QString *why);

  static bool readFile(const QString &path, const Catalog &cat, Loaded *out, QString *why);
  static bool writeFile(const QString &path, const QByteArray &data, QString *why); // atomic

  // One row per row of the file, in file order.
  static QVector<DiffRow> diff(const Loaded &f, const Catalog &cat,
                               const QHash<QString, VehicleValue> &vehicle);
  // False, with the reason, when the file was made for another table.
  static bool sameTable(const Loaded &f, const Catalog &cat, QString *why);

  static QString valueText(const Entry &e, double v); // what is written
  static bool equalValues(const Entry &e, double a, double b);
};

} // namespace params
