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
// User preset store: save / load / list / delete round trip, name sanitising
// (traversal, empty, overlong, reserved characters), case-insensitive
// uniqueness, never shadowing a built-in id or name, and a corrupt file being
// skipped. Pure core, runs in the GCS-only build too; temp dir only.

#include "UserPresets.h"

#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QTemporaryDir>
#include <QtTest>

using vsim::UserPresetStore;
using Kind = UserPresetStore::Kind;
using Res = UserPresetStore::SaveResult;

class TstUserPresets : public QObject {
  Q_OBJECT

  static QJsonObject payload(double mass) {
    return QJsonObject{{"mass", mass}, {"note", "x"}};
  }
  static void writeRaw(const QString &path, const QByteArray &bytes) {
    QVERIFY(QDir().mkpath(QFileInfo(path).absolutePath()));
    QFile f(path);
    QVERIFY(f.open(QIODevice::WriteOnly));
    f.write(bytes);
  }

private slots:
  void roundTripVehicleAndWorld() {
    QTemporaryDir tmp;
    UserPresetStore st(tmp.path());
    QString saved;
    QCOMPARE(st.save(Kind::Vehicle, "  My Quad ", payload(1.5), false, &saved),
             Res::Ok);
    QCOMPARE(saved, QString("My Quad")); // trimmed
    QCOMPARE(st.save(Kind::World, "Yard", payload(7), false), Res::Ok);
    QJsonObject v, w;
    QVERIFY(st.load(Kind::Vehicle, "My Quad", &v));
    QCOMPARE(v, payload(1.5));
    QVERIFY(st.load(Kind::World, "Yard", &w));
    QCOMPARE(w, payload(7));
    // Kinds are separate namespaces and directories.
    QVERIFY(!st.exists(Kind::World, "My Quad"));
    QVERIFY(QFile::exists(tmp.filePath("vehicles/My Quad.vveh")));
    QVERIFY(QFile::exists(tmp.filePath("worlds/Yard.vworld")));
    // Case-insensitive load.
    QVERIFY(st.load(Kind::Vehicle, "my quad", &v));
  }

  void listIsSortedCaseInsensitively() {
    QTemporaryDir tmp;
    UserPresetStore st(tmp.path());
    for (const char *n : {"charlie", "Alpha", "bravo", "Delta"})
      QCOMPARE(st.save(Kind::Vehicle, n, payload(1)), Res::Ok);
    QCOMPARE(st.list(Kind::Vehicle),
             QStringList({"Alpha", "bravo", "charlie", "Delta"}));
    QVERIFY(st.list(Kind::World).isEmpty());
  }

  void deleteRemovesOnlyThatPreset() {
    QTemporaryDir tmp;
    UserPresetStore st(tmp.path());
    st.save(Kind::Vehicle, "a", payload(1));
    st.save(Kind::Vehicle, "b", payload(2));
    QVERIFY(st.remove(Kind::Vehicle, "A")); // case-insensitive
    QCOMPARE(st.list(Kind::Vehicle), QStringList({"b"}));
    QVERIFY(!st.remove(Kind::Vehicle, "a")); // already gone
    QVERIFY(!st.remove(Kind::Vehicle, "../b"));
    QCOMPARE(st.list(Kind::Vehicle), QStringList({"b"}));
  }

  void sanitiseRejectsBadNames() {
    QTemporaryDir tmp;
    UserPresetStore st(tmp.path());
    const QString tooLong(UserPresetStore::kMaxNameLength + 1, 'x');
    for (const QString &bad :
         {QString("../x"), QString(".."), QString("a/b"), QString("a\\b"),
          QString(""), QString("   "), tooLong, QString("a..b"), QString(".hid"),
          QString("end."), QString("a:b"), QString("q?"), QString("NUL"),
          QString("tab\there")}) {
      QString why;
      QVERIFY2(UserPresetStore::sanitize(bad, &why).isEmpty(), qPrintable(bad));
      QVERIFY(!why.isEmpty());
      QCOMPARE(st.save(Kind::Vehicle, bad, payload(1)), Res::InvalidName);
    }
    // Nothing escaped or was written anywhere.
    QVERIFY(!QFile::exists(tmp.filePath("x.vveh")));
    QVERIFY(!QFile::exists(tmp.filePath("../x.vveh")));
    QVERIFY(st.list(Kind::Vehicle).isEmpty());
    // The cap itself is allowed; unicode is fine.
    QCOMPARE(UserPresetStore::sanitize(QString(UserPresetStore::kMaxNameLength, 'x')).size(),
             UserPresetStore::kMaxNameLength);
    QCOMPARE(UserPresetStore::sanitize(QString::fromUtf8(" Qu\xc3\xa4\x64 ")),
             QString::fromUtf8("Qu\xc3\xa4\x64"));
  }

  void duplicateNamesAreCaseInsensitive() {
    QTemporaryDir tmp;
    UserPresetStore st(tmp.path());
    QCOMPARE(st.save(Kind::Vehicle, "Racer", payload(1)), Res::Ok);
    QCOMPARE(st.save(Kind::Vehicle, "racer", payload(2)), Res::Exists);
    QCOMPARE(st.save(Kind::Vehicle, "RACER", payload(2)), Res::Exists);
    QJsonObject o;
    QVERIFY(st.load(Kind::Vehicle, "Racer", &o));
    QCOMPARE(o, payload(1)); // untouched
    // Overwrite replaces it and leaves a single entry under the new casing.
    QString saved;
    QCOMPARE(st.save(Kind::Vehicle, "racer", payload(3), true, &saved), Res::Ok);
    QCOMPARE(saved, QString("racer"));
    QCOMPARE(st.list(Kind::Vehicle), QStringList({"racer"}));
    QVERIFY(st.load(Kind::Vehicle, "RACER", &o));
    QCOMPARE(o, payload(3));
  }

  void builtInIdsAndNamesAreNeverShadowed() {
    QTemporaryDir tmp;
    UserPresetStore st(tmp.path());
    st.setReserved(Kind::Vehicle, {"5in", "5-inch freestyle"});
    QCOMPARE(st.save(Kind::Vehicle, "5in", payload(1)), Res::Reserved);
    QCOMPARE(st.save(Kind::Vehicle, "5IN", payload(1)), Res::Reserved);
    QCOMPARE(st.save(Kind::Vehicle, "5-Inch Freestyle", payload(1), true),
             Res::Reserved); // overwrite does not bypass it
    QCOMPARE(st.save(Kind::Vehicle, "Custom", payload(1)), Res::Reserved);
    // Reserved per kind: a world may use a vehicle's built-in name.
    QCOMPARE(st.save(Kind::World, "5in", payload(1)), Res::Ok);
    QVERIFY(st.list(Kind::Vehicle).isEmpty());
    // A file dropped in by hand under a reserved name is not listed either.
    writeRaw(tmp.filePath("vehicles/5in.vveh"), "{}");
    QTest::ignoreMessage(QtWarningMsg, QRegularExpression("built-in preset"));
    QVERIFY(st.list(Kind::Vehicle).isEmpty());
  }

  void corruptAndForeignFilesAreSkipped() {
    QTemporaryDir tmp;
    UserPresetStore st(tmp.path());
    QCOMPARE(st.save(Kind::Vehicle, "good", payload(1)), Res::Ok);
    writeRaw(tmp.filePath("vehicles/broken.vveh"), "{ not json");
    writeRaw(tmp.filePath("vehicles/array.vveh"), "[1,2]");
    writeRaw(tmp.filePath("vehicles/empty.vveh"), "");
    writeRaw(tmp.filePath("vehicles/notes.txt"), "{}");
    writeRaw(tmp.filePath("vehicles/bad:name.vveh"), "{}");
    QTest::ignoreMessage(QtWarningMsg, QRegularExpression("skipping corrupt"));
    QTest::ignoreMessage(QtWarningMsg, QRegularExpression("skipping corrupt"));
    QTest::ignoreMessage(QtWarningMsg, QRegularExpression("skipping corrupt"));
    QTest::ignoreMessage(QtWarningMsg, QRegularExpression("skipping"));
    QCOMPARE(st.list(Kind::Vehicle), QStringList({"good"}));
    QJsonObject o;
    QTest::ignoreMessage(QtWarningMsg, QRegularExpression("skipping corrupt"));
    QVERIFY(!st.load(Kind::Vehicle, "broken", &o));
    QVERIFY(!st.load(Kind::Vehicle, "missing", &o));
    QVERIFY(!st.load(Kind::Vehicle, "../good", &o));
    QVERIFY(st.load(Kind::Vehicle, "good", &o));
  }

  void missingRootIsEmptyNotAnError() {
    QTemporaryDir tmp;
    UserPresetStore st(tmp.filePath("does/not/exist"));
    QVERIFY(st.list(Kind::Vehicle).isEmpty());
    QCOMPARE(st.save(Kind::Vehicle, "a", payload(1)), Res::Ok); // creates dirs
    QCOMPARE(st.list(Kind::Vehicle), QStringList({"a"}));
  }
};

QTEST_MAIN(TstUserPresets)
#include "tst_user_presets.moc"
