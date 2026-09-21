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
#include <QString>

#include "RecordFormat.h"

// Writer: tees inbound telemetry byte-chunks to a length-prefixed .bin so a
// session can be replayed later (FR-LOG-05). The header is written on open();
// each writeFrame() appends one [t_us][len][bytes] record. The caller supplies
// t_us (monotonic, microseconds) so the sink stays clock-free and unit-testable.
class RecordSink {
public:
  RecordSink() = default;
  ~RecordSink();

  bool open(const QString &path, quint32 protocolVersion,
            quint64 startWallClockMs);
  bool isOpen() const { return m_file.isOpen(); }
  void writeFrame(quint64 tUs, const QByteArray &bytes);
  void close();
  QString path() const { return m_file.fileName(); }

private:
  QFile m_file;
};

// Sequential reader for the same format — used by the 1C round-trip test and
// by ReplaySource (Phase 2D) to scan/index a recording.
class RecordReader {
public:
  RecordReader() = default;
  ~RecordReader() { close(); }

  bool open(const QString &path); // reads + validates the header
  bool isOpen() const { return m_file.isOpen(); }
  RecordFormat::Header header() const { return m_header; }

  // Reads the next record into `out`. Returns false at clean EOF or on a
  // truncated/corrupt record.
  bool next(RecordFormat::Frame &out);
  void close();

private:
  QFile m_file;
  RecordFormat::Header m_header;
};
