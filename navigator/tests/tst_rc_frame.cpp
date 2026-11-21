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
// rcframe: the RC bridge's failsafe policy -- link loss sends the feeder's
// NO_SIGNAL (never channels / arm), and the arm override cannot arm while the
// link is lost or RC is disabled.

#include "RcFrame.h"

#include <QtTest>

#include <string>
#include <vector>

class TstRcFrame : public QObject {
  Q_OBJECT
private slots:
  void normalFrameIsCsv();
  void linkLostSendsNoSignalOnly();
  void armOverrideSuppressedWhenLinkLost();
  void armOverrideIgnoredWhenDisabled();
  void armOverrideIgnoredWithNoSource();
  void openFailureIsLostSource();
  void reopenTimer();
  void armOverrideAppliesWhenLive();
  void deviceLossErrnoClassification();
  void uartLineClassification();
  void fullRangeThrottleUnchanged();
  void centredThrottleCannotArmInFullRange();
  void halfRangeCentreAndBelowAreMinimum();
  void halfRangeFullUpIsMaximum();
  void throttleDeadzoneSnapsToMin();
  void throttleInvertAppliesFirst();
  void axisToUsEndpoints();
  void strictFrameAcceptsRealLines();
  void strictFrameRejectsCorruptLines();
  void strictFrameRejectsEveryDroppedByte();
  void assemblerIsChunkingInvariant();
  void assemblerHandlesCrLfAndBlankLines();
  void assemblerDropsOverlongLineAndResyncs();
  void escapeRawIsPrintable();
};

static std::vector<std::string> feedAll(const std::string &stream,
                                        size_t chunk, int *dropped = nullptr) {
  rcframe::LineAssembler a;
  std::vector<std::string> out;
  if (dropped)
    *dropped = 0;
  for (size_t i = 0; i < stream.size(); i += chunk) {
    for (size_t j = i; j < std::min(stream.size(), i + chunk); ++j) {
      std::string line;
      switch (a.push(stream[j], &line)) {
      case rcframe::Feed::Line:
        out.push_back(line);
        break;
      case rcframe::Feed::Dropped:
        if (dropped)
          ++*dropped;
        break;
      case rcframe::Feed::None:
        break;
      }
    }
  }
  return out;
}

void TstRcFrame::normalFrameIsCsv() {
  QCOMPARE(QString::fromStdString(
               rcframe::formatFrame(false, 1500, 1500, 1000, 1500, 2000, 1000)),
           QStringLiteral("1500,1500,1000,1500,2000,1000\n"));
}

void TstRcFrame::linkLostSendsNoSignalOnly() {
  const std::string f =
      rcframe::formatFrame(true, 1500, 1500, 1000, 1500, 2000, 1000);
  QCOMPARE(QString::fromStdString(f), QStringLiteral("NO_SIGNAL\n"));
  QVERIFY(f.find(',') == std::string::npos); // no channel data at all
}

void TstRcFrame::armOverrideSuppressedWhenLinkLost() {
  for (int ov : {-1, 0, 1})
    QCOMPARE(rcframe::resolveArm(true, true, true, ov, 1500), -1);
  QCOMPARE(rcframe::resolveArm(true, false, true, 1, 1500), -1);
}

void TstRcFrame::armOverrideIgnoredWhenDisabled() {
  // Disabled RC stays neutral: the override must not force arm=2000.
  QCOMPARE(rcframe::resolveArm(false, false, true, 1, 1500), 1500);
  QCOMPARE(rcframe::resolveArm(false, false, true, 0, 1500), 1500);
}

void TstRcFrame::armOverrideIgnoredWithNoSource() {
  // No input device: live neutral link, but the override must not arm.
  for (int ov : {-1, 0, 1})
    QCOMPARE(rcframe::resolveArm(false, true, false, ov, 1500), 1500);
  // A lost source still wins (no frame at all).
  QCOMPARE(rcframe::resolveArm(true, true, false, 1, 1500), -1);
}

void TstRcFrame::openFailureIsLostSource() {
  QVERIFY(rcframe::sourceLostAtOpen(true, false));  // selected, won't open
  QVERIFY(!rcframe::sourceLostAtOpen(true, true));  // selected, opened
  QVERIFY(!rcframe::sourceLostAtOpen(false, false)); // nothing selected
}

void TstRcFrame::reopenTimer() {
  int w = 500;
  int fires = 0;
  for (int t = 0; t < 1000; t += 20)
    fires += rcframe::reopenDue(w, 20, 500);
  QCOMPARE(fires, 2); // every 500 ms
  int z = 0;
  QVERIFY(rcframe::reopenDue(z, 20, 500)); // immediate when wait is 0
  QCOMPARE(z, 500);
}

void TstRcFrame::armOverrideAppliesWhenLive() {
  QCOMPARE(rcframe::resolveArm(false, true, true, -1, 1234), 1234);
  QCOMPARE(rcframe::resolveArm(false, true, true, 1, 1234), 2000);
  QCOMPARE(rcframe::resolveArm(false, true, true, 0, 1234), 1000);
}

void TstRcFrame::deviceLossErrnoClassification() {
  QVERIFY(rcframe::deviceLostErrno(ENODEV)); // unplugged joystick
  QVERIFY(rcframe::deviceLostErrno(EIO));
  QVERIFY(!rcframe::deviceLostErrno(EAGAIN)); // no event right now
  QVERIFY(!rcframe::deviceLostErrno(EINTR));
}

void TstRcFrame::uartLineClassification() {
  using rcframe::Line;
  QCOMPARE(rcframe::classifyLine("1500,1500,1000,1500,1000,1000"), Line::Frame);
  QCOMPARE(rcframe::classifyLine("NO_SIGNAL"), Line::NoSignal);
  QCOMPARE(rcframe::classifyLine("  NO_SIGNAL"), Line::NoSignal);
  QCOMPARE(rcframe::classifyLine("boot v1.2"), Line::Junk);
  QCOMPARE(rcframe::classifyLine(""), Line::Junk);
}

// Firmware arms only below this throttle value (us).
static constexpr int kArmMaxUs = 1100;

void TstRcFrame::fullRangeThrottleUnchanged() {
  // A throttle that rests fully down, as before: -32767 -> 1000, 0 -> 1500.
  QCOMPARE(rcframe::throttleToUs(-32767, false, false), 1000);
  QCOMPARE(rcframe::throttleToUs(0, false, false), 1500);
  QCOMPARE(rcframe::throttleToUs(32767, false, false), 2000);
  QCOMPARE(rcframe::throttleToUs(16383, false, false), 1749); // integer truncation
}

void TstRcFrame::centredThrottleCannotArmInFullRange() {
  // The reported failure: a spring-centred stick gives 1500 us, never below
  // the 1100 us arm limit. Half-range mode fixes exactly this.
  QVERIFY(rcframe::throttleToUs(0, false, false) >= kArmMaxUs);
  QVERIFY(rcframe::throttleToUs(0, false, true) < kArmMaxUs);
}

void TstRcFrame::halfRangeCentreAndBelowAreMinimum() {
  for (int v : {0, -1, -1000, -20000, -32767})
    QCOMPARE(rcframe::throttleToUs(v, false, true), 1000);
}

void TstRcFrame::halfRangeFullUpIsMaximum() {
  QCOMPARE(rcframe::throttleToUs(32767, false, true), 2000);
  const int mid = rcframe::throttleToUs(16383, false, true);
  QVERIFY(mid > 1450 && mid < 1550);
  QVERIFY(rcframe::throttleToUs(8000, false, true) <
          rcframe::throttleToUs(24000, false, true)); // monotonic
}

void TstRcFrame::throttleDeadzoneSnapsToMin() {
  // 1000 + 4% of the range is inside the dead band, 1000 + 6% is not.
  QCOMPARE(rcframe::throttleToUs(1300, false, true), 1000); // ~1040 us
  QVERIFY(rcframe::throttleToUs(2300, false, true) > 1050);  // ~1070 us
  // Full range: a stick resting a little above the bottom still arms.
  QCOMPARE(rcframe::throttleToUs(-30500, false, false), 1000); // ~1035 us
}

void TstRcFrame::throttleInvertAppliesFirst() {
  // Inverted axis: stick fully up reads -32767 and means minimum throttle.
  QCOMPARE(rcframe::throttleToUs(32767, true, false), 1000);
  QCOMPARE(rcframe::throttleToUs(-32767, true, false), 2000);
  QCOMPARE(rcframe::throttleToUs(-32767, true, true), 2000);
  QCOMPARE(rcframe::throttleToUs(32767, true, true), 1000);
}

void TstRcFrame::axisToUsEndpoints() {
  // Non-throttle channels keep the plain mapping, deadzone-free.
  QCOMPARE(rcframe::axisToUs(-32767, false), 1000);
  QCOMPARE(rcframe::axisToUs(0, false), 1500);
  QCOMPARE(rcframe::axisToUs(32767, false), 2000);
  QCOMPARE(rcframe::axisToUs(-30500, false), 1035); // not snapped
  QCOMPARE(rcframe::axisToUs(32767, true), 1000);
}

void TstRcFrame::strictFrameAcceptsRealLines() {
  int v[8] = {};
  // What the Arduino bridge prints: 5 to 8 channels.
  QCOMPARE(rcframe::parseStrictFrame("1500,1500,1000,1500,2000", v), 5);
  QCOMPARE(v[4], 2000);
  QCOMPARE(rcframe::parseStrictFrame("1500,1498,1000,1502,1000,1000", v), 6);
  QCOMPARE(v[3], 1502);
  QCOMPARE(rcframe::parseStrictFrame(
               "1500,1500,1000,1500,1000,1000,1500,1500", v), 8);
  // Whitespace around fields is fine; the pulse range 800..2200 is accepted.
  QCOMPARE(rcframe::parseStrictFrame(" 1500 , 1500,1000 ,1500,1000", v), 5);
  QCOMPARE(rcframe::parseStrictFrame("800,2200,1000,1500,1000", v), 5);
}

void TstRcFrame::strictFrameRejectsCorruptLines() {
  int v[8] = {};
  for (const char *bad : {
           "",                                  // empty
           "1500,1500,1000",                    // truncated: 3 fields
           "1500,1500,1000,1500",               // truncated: 4 fields
           "1500,1500,1000,1500,2",             // last field cut short
           "00,1500,1000,1500,2000,1000",       // started mid-line
           "500,1500,1000,1500,2000,1000",      // lost a leading digit
           "1500,1500,100,1500,2000,1000",      // lost a digit mid-line
           "150015001000,1500,2000",            // lost a comma
           "1500,,1000,1500,2000",              // empty field
           "1500,1500,1000,1500,2000,",         // trailing comma
           "1500,15x0,1000,1500,2000",          // stray character
           "-1500,1500,1000,1500,2000",         // negative
           "1500;1500;1000;1500;2000",          // wrong separator
           "1500,1500,1000,1500,2000,1000,1500,1500,1500", // 9 fields
           "99999999,1500,1000,1500,2000",      // absurd
           "799,1500,1000,1500,2000",           // just below the pulse range
           "1500,1500,1000,1500,2201",          // just above it
       })
    QCOMPARE(rcframe::parseStrictFrame(bad, v), 0);
}

void TstRcFrame::strictFrameRejectsEveryDroppedByte() {
  // Delete each single character of a good frame in turn: whatever that does
  // to a field, the result must never be accepted as a different frame.
  const std::string good = "1500,1498,1050,1502,2000,1000";
  int ref[8] = {};
  QCOMPARE(rcframe::parseStrictFrame(good, ref), 6);
  for (size_t i = 0; i < good.size(); ++i) {
    std::string bad = good;
    bad.erase(i, 1);
    int v[8] = {};
    const int n = rcframe::parseStrictFrame(bad, v);
    QVERIFY2(n == 0, bad.c_str()); // all fields are 4 digits: a drop breaks one
  }
}

void TstRcFrame::assemblerIsChunkingInvariant() {
  const std::string stream =
      "1500,1500,1000,1500,2000,1000\n1501,1499,1001,1500,2000,1000\n"
      "NO_SIGNAL\n1502,1500,1000,1500,2000,1000\n";
  const auto whole = feedAll(stream, stream.size());
  QCOMPARE((int)whole.size(), 4);
  for (size_t chunk : {size_t(1), size_t(2), size_t(7), size_t(13), size_t(64)})
    QVERIFY(feedAll(stream, chunk) == whole);
}

void TstRcFrame::assemblerHandlesCrLfAndBlankLines() {
  const auto l = feedAll("\r\n1500,1500,1000,1500,2000\r\n\n\r\nNO_SIGNAL\r\n", 3);
  QCOMPARE((int)l.size(), 2);
  QCOMPARE(QString::fromStdString(l[1]), QStringLiteral("NO_SIGNAL"));
}

void TstRcFrame::assemblerDropsOverlongLineAndResyncs() {
  // A burst of junk longer than the limit, then good lines: the junk is
  // dropped whole (once) and the next line parses.
  const std::string junk(rcframe::kMaxLine + 40, '7');
  const std::string stream = "1500,1500,1000,1500,2000\n" + junk +
                             "\n1501,1500,1000,1500,2000\n";
  int dropped = 0;
  const auto l = feedAll(stream, 5, &dropped);
  QCOMPARE(dropped, 1);
  QCOMPARE((int)l.size(), 2);
  QCOMPARE(QString::fromStdString(l[1]),
           QStringLiteral("1501,1500,1000,1500,2000"));
  // Exactly kMaxLine characters is still a line.
  const std::string edge(rcframe::kMaxLine, '7');
  int d2 = 0;
  QCOMPARE((int)feedAll(edge + "\n", 4, &d2).size(), 1);
  QCOMPARE(d2, 0);
}

void TstRcFrame::escapeRawIsPrintable() {
  QCOMPARE(QString::fromStdString(rcframe::escapeRaw("1500,\x01\xff\t")),
           QStringLiteral("1500,\\x01\\xff\\x09"));
  const std::string cut = rcframe::escapeRaw(std::string(500, 'a'), 20);
  QCOMPARE((int)cut.size(), 23); // 20 chars + "..."
}

QTEST_MAIN(TstRcFrame)
#include "tst_rc_frame.moc"
