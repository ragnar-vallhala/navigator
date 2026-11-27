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
// tst_navlink_text.cpp - NavLink char[N] fields are NUL-padded, not
// NUL-terminated. A field that fills its array must come out as exactly N
// characters, never more, and a short one must stop at its NUL.
//
// 1. navtext::fieldLatin1 on a struct whose next bytes are non-zero (a strlen
//    would run on into them).
// 2. Real frames through NavlinkRouter: a full STATUSTEXT, FAULT_STATUS,
//    PERF_TASKNAME and PARAM_VALUE.
// 3. ParamCodec never sends an id cut to 16 characters.
#include "NavlinkRouter.h"
#include "NavlinkText.h"
#include "params/ParamCodec.h"

#include <QtTest>
#include <cstring>

extern "C" {
#include "navlink_msgs.h"
}

namespace {

QByteArray bytesOf(const uint8_t *buf, size_t n) {
  return QByteArray(reinterpret_cast<const char *>(buf), static_cast<int>(n));
}

} // namespace

class TstNavlinkText : public QObject {
  Q_OBJECT
private slots:
  void helperStopsAtTheArrayEnd() {
    struct {
      char a[4];
      char b[4];
    } s;
    std::memcpy(s.a, "ABCD", 4); // fills the array: no NUL of its own
    std::memcpy(s.b, "WXYZ", 4);
    QCOMPARE(navtext::fieldLatin1(s.a), QStringLiteral("ABCD"));
    QCOMPARE(navtext::fieldLength(s.a), 4);
    std::memcpy(s.a, "AB\0D", 4); // stops at the first NUL, ignores what follows
    QCOMPARE(navtext::fieldLatin1(s.a), QStringLiteral("AB"));
    std::memset(s.a, 0, 4);
    QVERIFY(navtext::fieldLatin1(s.a).isEmpty());
  }

  void fullStatusTextIsExactly50() {
    NavlinkRouter r;
    QString got;
    r.onStatusText = [&](uint8_t, uint16_t, const QString &t) { got = t; };
    navlink_statustext_t m{};
    m.severity = 3;
    std::memset(m.text, 'A', sizeof m.text); // no terminator
    uint8_t buf[NAVLINK_MAX_FRAME];
    r.feed(bytesOf(buf, navlink_statustext_encode(buf, &m, 0, 1, 1)));
    QCOMPARE(got, QString(50, QLatin1Char('A')));
  }

  void fullFaultTextIsExactly50() {
    NavlinkRouter r;
    QString got;
    r.onFault = [&](const FaultEvent &e) { got = e.text; };
    navlink_fault_status_t m{};
    m.id = 0xFFFE; // not in the fault table, so its own text is what shows
    m.severity = 2;
    m.active = 1;
    std::memset(m.text, 'B', sizeof m.text);
    uint8_t buf[NAVLINK_MAX_FRAME];
    r.feed(bytesOf(buf, navlink_fault_status_encode(buf, &m, 0, 1, 1)));
    QCOMPARE(got, QString(50, QLatin1Char('B')));
  }

  void fullTaskNameIsExactly32() {
    NavlinkRouter r;
    QString got;
    r.onTaskName = [&](int, const QString &n) { got = n; };
    navlink_perf_taskname_t m{};
    m.task_id = 5;
    std::memset(m.name, 'C', sizeof m.name);
    uint8_t buf[NAVLINK_MAX_FRAME];
    r.feed(bytesOf(buf, navlink_perf_taskname_encode(buf, &m, 0, 1, 1)));
    QCOMPARE(got, QString(32, QLatin1Char('C')));
  }

  void sixteenCharParamIdIsExactly16() {
    NavlinkRouter r;
    QString got;
    r.onParamValue = [&](const params::ParamCodec::ValueMsg &v) { got = v.id; };
    navlink_param_value_t m{};
    std::memset(m.param_id, 'D', sizeof m.param_id);
    m.index = 1;
    m.count = 2;
    uint8_t buf[NAVLINK_MAX_FRAME];
    r.feed(bytesOf(buf, navlink_param_value_encode(buf, &m, 0, 1, 1)));
    QCOMPARE(got, QString(16, QLatin1Char('D')));
  }

  void shortFieldsStopAtTheirNul() {
    NavlinkRouter r;
    QString got;
    r.onTaskName = [&](int, const QString &n) { got = n; };
    navlink_perf_taskname_t m{};
    m.task_id = 1;
    std::memcpy(m.name, "idle\0junk-after-the-nul", 24);
    uint8_t buf[NAVLINK_MAX_FRAME];
    r.feed(bytesOf(buf, navlink_perf_taskname_encode(buf, &m, 0, 1, 1)));
    QCOMPARE(got, QStringLiteral("idle"));
  }

  void paramIdsThatDoNotFitAreNeverSentCutShort() {
    const QString ok(16, QLatin1Char('E'));
    QVERIFY(!params::ParamCodec::encodeRequestReadById(ok).isEmpty());
    QVERIFY(!params::ParamCodec::encodeSet(ok, params::Type::U8, 1, 1).isEmpty());
    const QString tooLong(17, QLatin1Char('E'));
    QVERIFY(params::ParamCodec::encodeRequestReadById(tooLong).isEmpty());
    QVERIFY(params::ParamCodec::encodeSet(tooLong, params::Type::U8, 1, 1).isEmpty());
    // Not Latin-1 would become '?' and could name another parameter.
    const QString odd = QStringLiteral("RATE_中");
    QVERIFY(params::ParamCodec::encodeRequestReadById(odd).isEmpty());
    QVERIFY(params::ParamCodec::encodeRequestReadById(QString()).isEmpty());
  }
};

QTEST_GUILESS_MAIN(TstNavlinkText)
#include "tst_navlink_text.moc"
