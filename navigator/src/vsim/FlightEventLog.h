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

#include <QFile>
#include <QJsonObject>
#include <QJsonValue>
#include <QList>
#include <QString>
#include <QVector>

namespace vsim {

// Per-run flight event log: ev-<stamp>.jsonl, written next to gt-<stamp>.bin.
// One JSON object per line, flushed per line so a crash leaves a readable
// prefix. No widget dependency. Format: docs/reference/flight-events.md.
//
// Timestamps (t_us) are supplied by the caller and MUST use the same base as
// the gt records: microseconds since the run's log opened (monotonic clock).
// The writer clamps a backwards timestamp up to the previous one, so the file
// is always non-decreasing.
class FlightEventLog {
public:
  static constexpr const char *kSchema = "nav-flight-events/1";
  static constexpr qint64 kRcPeriodUs = 100000; // 10 Hz while live
  // An analog RC channel (microseconds, 1000..2000) counts as changed only if
  // it moved by more than this from the last WRITTEN value (joystick jitter).
  static constexpr int kRcDeadbandUs = 2;
  // Channels that are switches (arm, ch6): any change counts.
  static constexpr int kRcFirstSwitch = 4;

  struct Header {
    QString runId;
    QString appVersion;
    QJsonValue sdkVersion; // simulator ABI version; null/undefined if unknown
    QString moduleBuildId; // simulator module build id; "" -> null
    QString vehicleSha256; // "" -> null
    QString worldPreset;   // "" -> null
    QJsonValue worldSeed;  // null/undefined when unknown
    QString gtFile;        // file name only, no directory
    qint64 t0UnixMs = 0;
  };

  FlightEventLog() = default;
  ~FlightEventLog() { close(); }
  FlightEventLog(const FlightEventLog &) = delete;
  FlightEventLog &operator=(const FlightEventLog &) = delete;

  // Creates (truncates) `path` and writes the header line. False on failure.
  bool open(const QString &path, const Header &h);
  bool isOpen() const { return m_file.isOpen(); }
  QString path() const { return m_file.fileName(); }
  qint64 eventCount() const { return m_events; }
  void close();

  // Each returns false when nothing was written (log closed, or for rc, the
  // throttle dropped it). `source` is "gamepad" / "uart" / ... or "" = null.
  bool rc(qint64 tUs, const QVector<int> &channels, const QString &source);
  bool mode(qint64 tUs, const QString &from, const QString &to,
            const QString &source = {}); // source: rc | gcs
  bool arm(qint64 tUs, bool armed);
  bool fault(qint64 tUs, const QString &id, bool active,
             const QString &detail = {});
  bool start(qint64 tUs);
  bool stop(qint64 tUs, const QString &reason); // user | fault | exit
  bool marker(qint64 tUs, const QString &text);

  // Lowercase hex SHA-256 of `data`.
  static QString sha256Hex(const QByteArray &data);

  // Tolerant reader: skips blank lines and any line that is not a JSON object
  // (a truncated last line after a crash, typically). Never throws.
  static QList<QJsonObject> readEvents(const QString &path);

  // ---- run listing + retention (one run = gt-<stamp>.bin + ev-<stamp>.jsonl)
  struct Run {
    QString stamp;
    QString gtPath; // "" when absent
    QString evPath; // "" when absent
    qint64 gtBytes = 0;
    qint64 evBytes = 0;
  };
  // Runs in `dir`, newest stamp first. Only files named exactly gt-*.bin or
  // ev-*.jsonl are considered.
  static QList<Run> listRuns(const QString &dir);
  // Number of event lines after the header (cheap: counts newlines).
  static int countEvents(const QString &evPath);
  // Capped variant for the GUI thread: reads at most `capBytes`; *capped is set
  // when the file was longer (the count is then a lower bound).
  static int countEvents(const QString &evPath, qint64 capBytes, bool *capped);
  // True when `stamp` is the run being recorded (never delete such a run).
  static bool isLiveRun(const QString &stamp, const QString &currentStamp,
                        bool recording) {
    return recording && !stamp.isEmpty() && stamp == currentStamp;
  }
  // The rc change test (exposed for tests): last written vs new channels.
  static bool rcChanged(const QVector<int> &lastWritten,
                        const QVector<int> &now);
  // Deletes the oldest runs (by stamp) so at most `keep` remain, never the run
  // `protectStamp` (which counts toward `keep`). keep <= 0 keeps everything.
  // Returns the number of files removed.
  static int prune(const QString &dir, int keep, const QString &protectStamp);

private:
  bool writeObj(qint64 tUs, const char *kind, const QJsonObject &o);
  QFile m_file;
  qint64 m_lastT = 0;
  qint64 m_events = 0;
  qint64 m_lastRcT = 0;
  bool m_haveRc = false;
  QVector<int> m_lastRc;
};

} // namespace vsim
