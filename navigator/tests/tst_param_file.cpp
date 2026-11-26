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
#include <QtTest>
#include "ParamFile.h"

#include <QJsonDocument>
#include <QTemporaryDir>
#include <cmath>

using namespace params;
using VV = ParamFile::VehicleValue;

namespace {
QString catalogPath() {
  return qEnvironmentVariable("PARAMS_CATALOG_JSON",
                              QStringLiteral(NAVLINK_GEN_DIR "/params_catalog.json"));
}
const QDateTime kWhen(QDate(2026, 11, 27), QTime(1, 30, 0), Qt::UTC);
} // namespace

class TstParamFile : public QObject {
  Q_OBJECT
  Catalog cat;
  // The vehicle holding the catalog's defaults, as floats the way PARAM_VALUE carries them.
  QHash<QString, VV> vehicle() const {
    QHash<QString, VV> h;
    for (const Entry &e : cat.entries())
      h.insert(e.id, VV{e.type == Type::F32 ? double(float(e.def)) : e.def, quint8(Persisted), true});
    return h;
  }
  static QByteArray file(const QByteArray &params, const QByteArray &head = "\"format\":1,\"table_crc\":1") {
    return "{" + head + ",\"params\":[" + params + "]}";
  }

private slots:
  void initTestCase() {
    QString err;
    QVERIFY2(Catalog::loadFile(catalogPath(), cat, &err), qPrintable(err));
  }
  void aSaveThenALoadIsAllSame();
  void oddFloatsRoundTrip();
  void valuesAreWrittenAsTheirShortestText();
  void groupZeroAndReadOnlyAreNotInTheFile();
  void saveRefusesWhileAnythingIsStaged();
  void saveRefusesAnIncompleteTable();
  void theTimeAndSysidAreKeptAndNotChecked();
  void aRepeatedIdIsAProblem();
  void unknownLiveAndReadOnlyIdsAreProblems();
  void aValueOutOfRangeOrNotANumberIsAProblem();
  void aBigFileIsRefusedWithoutBeingRead();
  void notAParameterFileIsRefused();
  void aDifferentTableIsReportedAndTheDiffStillWorks();
  void theDiffMarksChangedAndUnknownRows();
  void aFileIsWrittenWholeOrNotAtAll();
};

void TstParamFile::aSaveThenALoadIsAllSame() {
  QByteArray b;
  QString why;
  QVERIFY2(ParamFile::save(cat, vehicle(), 42, kWhen, &b, &why), qPrintable(why));
  ParamFile::Loaded l;
  QVERIFY2(ParamFile::parse(b, cat, &l, &why), qPrintable(why));
  QVERIFY2(l.ok(), qPrintable(l.problems.value(0).id + ": " + l.problems.value(0).text));
  QVERIFY(ParamFile::sameTable(l, cat, &why));
  const auto rows = ParamFile::diff(l, cat, vehicle());
  QVERIFY(rows.size() > 20);
  for (const auto &r : rows)
    QVERIFY2(r.same, qPrintable(r.id));
}

void TstParamFile::oddFloatsRoundTrip() {
  // Values a float cannot hold exactly as a short decimal, and extremes.
  const double odd[] = {0.1, 1e-5, 123456.79, 0.3, 1.0 / 3.0, 16777217.0, 2.5e-3, 0.0005};
  auto v = vehicle();
  int n = 0;
  for (const Entry &e : cat.entries())
    if (e.type == Type::F32 && e.group != 0 && !(e.flags & ReadOnly)) {
      const double want = std::min(std::max(odd[n % 8], e.min), e.max);
      v[e.id].v = double(float(want));
      ++n;
    }
  QVERIFY(n > 8);
  QByteArray b;
  QVERIFY(ParamFile::save(cat, v, 1, kWhen, &b, nullptr));
  ParamFile::Loaded l;
  QVERIFY(ParamFile::parse(b, cat, &l, nullptr));
  QVERIFY(l.ok());
  for (const auto &r : ParamFile::diff(l, cat, v))
    QVERIFY2(r.same, qPrintable(r.id));
}

void TstParamFile::valuesAreWrittenAsTheirShortestText() {
  const Entry *f = cat.byId(QStringLiteral("NOTCH_Q"));
  const Entry *u = cat.byId(QStringLiteral("NOTCH_EN"));
  QVERIFY(f && u);
  QCOMPARE(ParamFile::valueText(*f, double(0.1f)), QStringLiteral("0.1"));
  QCOMPARE(ParamFile::valueText(*f, 8.0), QStringLiteral("8"));
  QCOMPARE(ParamFile::valueText(*f, double(1e-5f)), QStringLiteral("1e-05"));
  QCOMPARE(ParamFile::valueText(*u, 1.0), QStringLiteral("1"));
  QVERIFY(ParamFile::equalValues(*f, double(0.1f), 0.1)); // compared as floats
  QVERIFY(!ParamFile::equalValues(*f, 8.0, 8.001));
}

void TstParamFile::groupZeroAndReadOnlyAreNotInTheFile() {
  QByteArray b;
  QVERIFY(ParamFile::save(cat, vehicle(), 1, kWhen, &b, nullptr));
  QVERIFY(!b.contains("GLPF"));
  QVERIFY(!b.contains("SYS_PTBL_CRC"));
  QVERIFY(b.contains("RATE_ROLL_KP"));
  QVERIFY(b.contains("NOTCH_FMIN"));
}

void TstParamFile::saveRefusesWhileAnythingIsStaged() {
  auto v = vehicle();
  v[QStringLiteral("RATE_ROLL_KP")].flags |= Staged;
  v[QStringLiteral("NOTCH_Q")].flags |= Staged;
  QByteArray b;
  QString why;
  QVERIFY(!ParamFile::save(cat, v, 1, kWhen, &b, &why));
  QVERIFY2(why.contains(QLatin1String("2 value(s) are staged")), qPrintable(why));
  QVERIFY(b.isEmpty());
}

void TstParamFile::saveRefusesAnIncompleteTable() {
  auto v = vehicle();
  v[QStringLiteral("ANG_PITCH_KP")].have = false;
  QByteArray b;
  QString why;
  QVERIFY(!ParamFile::save(cat, v, 1, kWhen, &b, &why));
  QVERIFY2(why.contains(QLatin1String("ANG_PITCH_KP")), qPrintable(why));
  v.remove(QStringLiteral("ANG_PITCH_KP"));
  QVERIFY(!ParamFile::save(cat, v, 1, kWhen, &b, &why));
}

void TstParamFile::theTimeAndSysidAreKeptAndNotChecked() {
  QByteArray b;
  QVERIFY(ParamFile::save(cat, vehicle(), 42, kWhen, &b, nullptr));
  ParamFile::Loaded l;
  QVERIFY(ParamFile::parse(b, cat, &l, nullptr));
  QCOMPARE(l.meta.sysid, 42);
  QCOMPARE(l.meta.savedAt, QStringLiteral("2026-11-27T01:30:00Z"));
  QVERIFY(l.ok()); // neither makes a problem
  // A file without them loads as well.
  ParamFile::Loaded m;
  QVERIFY(ParamFile::parse(file("{\"id\":\"NOTCH_Q\",\"value\":8}"), cat, &m, nullptr));
  QCOMPARE(m.meta.sysid, -1);
  QVERIFY(m.meta.savedAt.isEmpty());
  QVERIFY(m.ok());
}

void TstParamFile::aRepeatedIdIsAProblem() {
  ParamFile::Loaded l;
  QVERIFY(ParamFile::parse(
      file("{\"id\":\"NOTCH_Q\",\"value\":8},{\"id\":\"NOTCH_Q\",\"value\":9}"), cat, &l, nullptr));
  QVERIFY(!l.ok());
  QCOMPARE(l.problems.size(), 1);
  QCOMPARE(l.problems[0].id, QStringLiteral("NOTCH_Q"));
  QVERIFY(l.problems[0].text.contains(QLatin1String("more than once")));
  QCOMPARE(l.rows.size(), 1); // the first one stays shown
}

void TstParamFile::unknownLiveAndReadOnlyIdsAreProblems() {
  ParamFile::Loaded l;
  QVERIFY(ParamFile::parse(file("{\"id\":\"NO_SUCH\",\"value\":1},"
                                "{\"id\":\"SYS_PTBL_CRC\",\"value\":1},"
                                "{\"id\":\"GLPF_ROLL_RC\",\"value\":0.01},"
                                "{\"id\":\"NOTCH_Q\",\"value\":8}"),
                           cat, &l, nullptr));
  QVERIFY(!l.ok());
  QCOMPARE(l.rows.size(), 1);
  QCOMPARE(l.rows[0].id, QStringLiteral("NOTCH_Q"));
  QStringList ids;
  for (const auto &p : l.problems)
    ids << p.id;
  QVERIFY(ids.contains(QStringLiteral("NO_SUCH")));
  QVERIFY(ids.contains(QStringLiteral("SYS_PTBL_CRC")));
  QVERIFY(ids.contains(QStringLiteral("GLPF_ROLL_RC")));
}

void TstParamFile::aValueOutOfRangeOrNotANumberIsAProblem() {
  const char *bad[] = {"{\"id\":\"NOTCH_Q\",\"value\":99}",       // above max
                       "{\"id\":\"NOTCH_Q\",\"value\":-1}",        // below min
                       "{\"id\":\"NOTCH_EN\",\"value\":0.5}",      // not an integer
                       "{\"id\":\"NOTCH_Q\",\"value\":\"nan\"}",   // text
                       "{\"id\":\"NOTCH_Q\",\"value\":null}",      // nothing
                       "{\"id\":\"NOTCH_Q\"}",                      // no value
                       "{\"id\":\"NOTCH_Q\",\"value\":1e999}"};    // beyond a double
  for (const char *b : bad) {
    ParamFile::Loaded l;
    QString why;
    if (!ParamFile::parse(file(b), cat, &l, &why))
      continue; // a parser that refuses the whole file is as good
    QVERIFY2(!l.ok(), b);
    QVERIFY2(l.rows.isEmpty(), b);
  }
}

void TstParamFile::aBigFileIsRefusedWithoutBeingRead() {
  QTemporaryDir dir;
  QVERIFY(dir.isValid());
  const QString path = dir.filePath(QStringLiteral("big.json"));
  QFile f(path);
  QVERIFY(f.open(QIODevice::WriteOnly));
  QVERIFY(f.resize(ParamFile::kMaxBytes + 1));
  f.close();
  ParamFile::Loaded l;
  QString why;
  QVERIFY(!ParamFile::readFile(path, cat, &l, &why));
  QVERIFY2(why.contains(QLatin1String("too large")), qPrintable(why));
  // A file that would otherwise load, padded past the cap: refused for its size.
  const QByteArray ok = file("{\"id\":\"NOTCH_Q\",\"value\":8}");
  QVERIFY(ParamFile::parse(ok, cat, &l, &why));
  QVERIFY(!ParamFile::parse(ok + QByteArray(int(ParamFile::kMaxBytes), ' '), cat, &l, &why));
  QVERIFY2(why.contains(QLatin1String("too large")), qPrintable(why));
  QVERIFY(!ParamFile::readFile(dir.filePath(QStringLiteral("none.json")), cat, &l, &why));
}

void TstParamFile::notAParameterFileIsRefused() {
  ParamFile::Loaded l;
  QString why;
  QVERIFY(!ParamFile::parse("not json", cat, &l, &why));
  QVERIFY(!ParamFile::parse("[1,2,3]", cat, &l, &why));
  QVERIFY(!ParamFile::parse("{\"format\":2,\"params\":[]}", cat, &l, &why));
  QVERIFY2(why.contains(QLatin1String("format")), qPrintable(why));
  QVERIFY(!ParamFile::parse("{\"format\":1}", cat, &l, &why));
  QVERIFY(!ParamFile::parse("{\"format\":1,\"params\":{\"NOTCH_Q\":8}}", cat, &l, &why)); // an object
}

void TstParamFile::aDifferentTableIsReportedAndTheDiffStillWorks() {
  ParamFile::Loaded l;
  QVERIFY(ParamFile::parse(file("{\"id\":\"NOTCH_Q\",\"value\":9}", "\"format\":1,\"table_crc\":12345"),
                           cat, &l, nullptr));
  QVERIFY(l.ok()); // the values themselves are fine
  QString why;
  QVERIFY(!ParamFile::sameTable(l, cat, &why));
  QVERIFY2(why.contains(QLatin1String("another parameter table")), qPrintable(why));
  const auto rows = ParamFile::diff(l, cat, vehicle());
  QCOMPARE(rows.size(), 1);
  QVERIFY(!rows[0].same);
  // No crc at all is refused too.
  ParamFile::Loaded n;
  QVERIFY(ParamFile::parse(file("{\"id\":\"NOTCH_Q\",\"value\":9}", "\"format\":1"), cat, &n, nullptr));
  QVERIFY(!ParamFile::sameTable(n, cat, &why));
}

void TstParamFile::theDiffMarksChangedAndUnknownRows() {
  ParamFile::Loaded l;
  QVERIFY(ParamFile::parse(file("{\"id\":\"NOTCH_Q\",\"value\":8},"
                                "{\"id\":\"NOTCH_FMIN\",\"value\":100},"
                                "{\"id\":\"NOTCH_FMAX\",\"value\":300}"),
                           cat, &l, nullptr));
  auto v = vehicle(); // NOTCH_Q 8 (same), FMIN 60 (differs), FMAX 450 (differs)
  v[QStringLiteral("NOTCH_FMAX")].have = false; // not read
  const auto rows = ParamFile::diff(l, cat, v);
  QCOMPARE(rows.size(), 3);
  QVERIFY(rows[0].same);
  QVERIFY(!rows[1].same);
  QCOMPARE(rows[1].vehicle, 60.0);
  QCOMPARE(rows[1].file, 100.0);
  QCOMPARE(rows[1].group, 5);
  QVERIFY(!rows[2].haveVehicle);
  QVERIFY(!rows[2].same);
}

void TstParamFile::aFileIsWrittenWholeOrNotAtAll() {
  QTemporaryDir dir;
  QVERIFY(dir.isValid());
  QByteArray b;
  QVERIFY(ParamFile::save(cat, vehicle(), 7, kWhen, &b, nullptr));
  const QString path = dir.filePath(QStringLiteral("tune.json"));
  QString why;
  QVERIFY2(ParamFile::writeFile(path, b, &why), qPrintable(why));
  ParamFile::Loaded l;
  QVERIFY2(ParamFile::readFile(path, cat, &l, &why), qPrintable(why));
  QVERIFY(l.ok());
  QCOMPARE(l.meta.sysid, 7);
  QVERIFY(!ParamFile::writeFile(dir.filePath(QStringLiteral("no/such/dir/x.json")), b, &why));
  QVERIFY(QFile::exists(path));
  QVERIFY(QJsonDocument::fromJson(b).isObject()); // the text is real JSON
}

QTEST_APPLESS_MAIN(TstParamFile)
#include "tst_param_file.moc"
