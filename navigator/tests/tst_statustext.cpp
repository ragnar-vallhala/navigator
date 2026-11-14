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
// STATUSTEXT through NavlinkRouter::feed: severity prefix and chunk
// reassembly (order, duplicates, timeout, caps) with an injected clock.
#include "NavlinkRouter.h"

#include <QtTest>
#include <algorithm>
#include <cstring>

extern "C" {
#include "navlink_msgs.h"
}

namespace {
QByteArray frame(uint8_t sev, const QByteArray &text, uint16_t id,
                 uint8_t seq) {
  navlink_statustext_t m{};
  m.severity = sev;
  std::memcpy(m.text, text.constData(),
              std::min(sizeof(m.text), static_cast<size_t>(text.size())));
  m.id = id;
  m.chunk_seq = seq;
  uint8_t buf[NAVLINK_MAX_FRAME];
  const size_t n = navlink_statustext_encode(buf, &m, 0, 1, 1);
  return QByteArray(reinterpret_cast<const char *>(buf),
                    static_cast<qsizetype>(n));
}

struct Rig {
  NavlinkRouter r;
  QStringList lines;
  qint64 now = 1000;
  Rig() {
    r.clockMs = [this] { return now; };
    r.onLog = [this](const QString &l) { lines << l; };
  }
  void send(uint8_t sev, const QByteArray &t, uint16_t id, uint8_t seq) {
    r.feed(frame(sev, t, id, seq));
  }
};

const QByteArray kFull50 = QByteArray(50, 'a');
} // namespace

class TstStatusText : public QObject {
  Q_OBJECT
private slots:
  void severityLabels() {
    QCOMPARE(NavlinkRouter::severityLabel(0), QStringLiteral("EMERG"));
    QCOMPARE(NavlinkRouter::severityLabel(1), QStringLiteral("ALERT"));
    QCOMPARE(NavlinkRouter::severityLabel(2), QStringLiteral("CRIT"));
    QCOMPARE(NavlinkRouter::severityLabel(3), QStringLiteral("ERR"));
    QCOMPARE(NavlinkRouter::severityLabel(4), QStringLiteral("WARN"));
    QCOMPARE(NavlinkRouter::severityLabel(5), QStringLiteral("NOTICE"));
    QCOMPARE(NavlinkRouter::severityLabel(6), QStringLiteral("INFO"));
    QCOMPARE(NavlinkRouter::severityLabel(7), QStringLiteral("DEBUG"));
    QCOMPARE(NavlinkRouter::severityLabel(9), QStringLiteral("SEV 9"));
  }

  void singleChunkHasSeverityPrefix() {
    Rig g;
    uint8_t sev = 99;
    uint16_t id = 99;
    QString text;
    g.r.onStatusText = [&](uint8_t s, uint16_t i, const QString &t) {
      sev = s;
      id = i;
      text = t;
    };
    g.send(4, "low battery", 0, 0);
    QCOMPARE(g.lines, QStringList{QStringLiteral("[WARN] low battery")});
    QCOMPARE(sev, uint8_t(4));
    QCOMPARE(id, uint16_t(0));
    QCOMPARE(text, QStringLiteral("low battery"));
    g.send(77, "odd", 0, 0);
    QCOMPARE(g.lines.last(), QStringLiteral("[SEV 77] odd"));
  }

  void singleChunkWithIdIsImmediate() {
    Rig g;
    g.send(6, "hi", 5, 0);
    QCOMPARE(g.lines, QStringList{QStringLiteral("[INFO] hi")});
  }

  void twoChunksInOrder() {
    Rig g;
    g.send(3, kFull50, 7, 0);
    QVERIFY(g.lines.isEmpty());
    g.send(3, "tail", 7, 1);
    QCOMPARE(g.lines,
             QStringList{QStringLiteral("[ERR] ") + kFull50 + "tail"});
  }

  void twoChunksOutOfOrder() {
    Rig g;
    g.send(3, "tail", 7, 1);
    QVERIFY(g.lines.isEmpty());
    g.send(3, kFull50, 7, 0);
    QCOMPARE(g.lines,
             QStringList{QStringLiteral("[ERR] ") + kFull50 + "tail"});
  }

  void threeChunksWithDuplicate() {
    Rig g;
    const QByteArray b(50, 'b');
    g.send(5, kFull50, 9, 0);
    g.send(5, kFull50, 9, 0); // resend
    g.send(5, b, 9, 1);
    g.send(5, b, 9, 1); // resend
    QVERIFY(g.lines.isEmpty());
    g.send(5, "end", 9, 2);
    QCOMPARE(g.lines, QStringList{QStringLiteral("[NOTICE] ") + kFull50 + b +
                                  "end"});
  }

  void missingChunkFlushedAsIncomplete() {
    Rig g;
    g.send(4, kFull50, 11, 0);
    g.send(4, "last", 11, 2); // seq 1 never arrives
    QVERIFY(g.lines.isEmpty());
    g.now += NavlinkRouter::kStatusTextStaleMs - 1;
    g.r.flushStale(g.now);
    QVERIFY(g.lines.isEmpty());
    g.now += 1;
    g.r.flushStale(g.now);
    QCOMPARE(g.lines.size(), 1);
    QCOMPARE(g.lines[0], QStringLiteral("[WARN] ") + kFull50 + "last [...]");
    g.r.flushStale(g.now + 100000); // nothing left to flush twice
    QCOMPARE(g.lines.size(), 1);
  }

  void staleFlushedByNextFrame() {
    Rig g;
    g.send(4, kFull50, 11, 0);
    g.now += NavlinkRouter::kStatusTextStaleMs;
    g.send(6, "other", 0, 0); // the feed path flushes
    QCOMPARE(g.lines.size(), 2);
    QVERIFY(g.lines[0].endsWith(" [...]"));
    QCOMPARE(g.lines[1], QStringLiteral("[INFO] other"));
  }

  void quietLinkFlushedByTimerCall() {
    Rig g;
    g.send(3, kFull50, 21, 0); // partial; the link then goes quiet
    g.r.flushStale();          // clock not advanced yet
    QVERIFY(g.lines.isEmpty());
    g.now += NavlinkRouter::kStatusTextStaleMs + 1;
    g.r.flushStale(g.now);
    QCOMPARE(g.lines,
             QStringList{QStringLiteral("[ERR] ") + kFull50 + " [...]"});
    g.r.flushStale(g.now + 100000);
    g.r.flushStale();
    QCOMPARE(g.lines.size(), 1);
  }

  void flushAllEmitsEverythingOldestFirst() {
    Rig g;
    g.send(3, kFull50, 1, 0);
    g.now += 10;
    g.send(4, QByteArray(50, 'b'), 2, 0);
    g.now += 10;
    g.send(6, "x", 3, 1); // seq 0 missing
    QVERIFY(g.lines.isEmpty());
    g.r.flushAll();
    QCOMPARE(g.lines.size(), 3);
    QCOMPARE(g.lines[0], QStringLiteral("[ERR] ") + kFull50 + " [...]");
    QCOMPARE(g.lines[1],
             QStringLiteral("[WARN] ") + QByteArray(50, 'b') + " [...]");
    QCOMPARE(g.lines[2], QStringLiteral("[INFO] x [...]"));
    g.r.flushAll();
    g.r.flushStale(g.now + 100000);
    QCOMPARE(g.lines.size(), 3);
  }

  void interleavedGroups() {
    Rig g;
    g.send(3, kFull50, 1, 0);
    g.send(6, QByteArray(50, 'x'), 2, 0);
    g.send(3, "one", 1, 1);
    QCOMPARE(g.lines,
             QStringList{QStringLiteral("[ERR] ") + kFull50 + "one"});
    g.send(6, "two", 2, 1);
    QCOMPARE(g.lines.size(), 2);
    QCOMPARE(g.lines[1],
             QStringLiteral("[INFO] ") + QByteArray(50, 'x') + "two");
  }

  void groupCapEvictsOldest() {
    Rig g;
    // Each group holds seq 1 only (seq 0 missing), so none completes.
    for (int id = 1; id <= NavlinkRouter::kStatusTextMaxGroups; id++)
      g.send(6, QByteArray(5, static_cast<char>('a' + id)),
             static_cast<uint16_t>(id), 1);
    QVERIFY(g.lines.isEmpty());
    g.send(6, "new", 100, 1); // the 9th group evicts id 1
    QCOMPARE(g.lines.size(), 1);
    QCOMPARE(g.lines[0], QStringLiteral("[INFO] bbbbb [...]"));
  }

  void byteCapEvictsOldest() {
    Rig g;
    g.send(6, kFull50, 1, 1); // oldest group
    // Fill group 2 with full chunks (no terminal) until the byte cap trips.
    for (int s = 0; s < 40 && g.lines.isEmpty(); s++)
      g.send(6, kFull50, 2, static_cast<uint8_t>(s));
    QCOMPARE(g.lines.size(), 1);
    QCOMPARE(g.lines[0], QStringLiteral("[INFO] ") + kFull50 + " [...]");
  }

  void fiftyBytesNoNulNotChunked() {
    Rig g;
    g.send(6, kFull50, 0, 0); // id 0: whole message, no NUL, no loss
    QCOMPARE(g.lines, QStringList{QStringLiteral("[INFO] ") + kFull50});
  }

  void textStopsAtNul() {
    Rig g;
    QByteArray t("ab");
    t.append('\0');
    t.append("zz");
    g.send(6, t, 0, 0);
    QCOMPARE(g.lines, QStringList{QStringLiteral("[INFO] ab")});
  }
};

QTEST_MAIN(TstStatusText)
#include "tst_statustext.moc"
