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
#include "FlightEventLog.h"

#include <QCryptographicHash>
#include <QDir>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QMap>

#include <algorithm>
#include <cstdlib>

namespace vsim {

namespace {
QJsonValue strOrNull(const QString &s) {
  return s.isEmpty() ? QJsonValue(QJsonValue::Null) : QJsonValue(s);
}

QJsonValue orNull(const QJsonValue &v) {
  return v.isUndefined() ? QJsonValue(QJsonValue::Null) : v;
}

// "gt-<stamp>.bin" / "ev-<stamp>.jsonl" -> stamp, else "".
QString stampOf(const QString &name, const QString &prefix,
                const QString &suffix) {
  if (name.size() <= prefix.size() + suffix.size() ||
      !name.startsWith(prefix) || !name.endsWith(suffix))
    return {};
  return name.mid(prefix.size(), name.size() - prefix.size() - suffix.size());
}
} // namespace

bool FlightEventLog::open(const QString &path, const Header &h) {
  close();
  m_file.setFileName(path);
  if (!m_file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    return false;
  m_lastT = 0;
  m_events = 0;
  m_lastRcT = 0;
  m_haveRc = false;
  m_lastRc.clear();
  QJsonObject o;
  o["schema"] = QString::fromLatin1(kSchema);
  o["run_id"] = strOrNull(h.runId);
  o["app_version"] = strOrNull(h.appVersion);
  o["sdk_version"] = orNull(h.sdkVersion);
  o["module_build_id"] = strOrNull(h.moduleBuildId);
  o["vehicle_sha256"] = strOrNull(h.vehicleSha256);
  o["world_preset"] = strOrNull(h.worldPreset);
  o["world_seed"] = orNull(h.worldSeed);
  o["gt_file"] = strOrNull(h.gtFile);
  o["t0_unix_ms"] = h.t0UnixMs;
  m_file.write(QJsonDocument(o).toJson(QJsonDocument::Compact) + '\n');
  m_file.flush();
  return true;
}

void FlightEventLog::close() {
  if (m_file.isOpen()) {
    m_file.flush();
    m_file.close();
  }
}

bool FlightEventLog::writeObj(qint64 tUs, const char *kind,
                              const QJsonObject &o) {
  if (!m_file.isOpen())
    return false;
  tUs = std::max(tUs, m_lastT);
  m_lastT = tUs;
  QJsonObject line;
  line["t_us"] = tUs;
  line["kind"] = QString::fromLatin1(kind);
  for (auto it = o.begin(); it != o.end(); ++it)
    line[it.key()] = it.value();
  m_file.write(QJsonDocument(line).toJson(QJsonDocument::Compact) + '\n');
  m_file.flush(); // per line: a crash leaves a readable prefix
  ++m_events;
  return true;
}

bool FlightEventLog::rc(qint64 tUs, const QVector<int> &channels,
                        const QString &source) {
  if (!m_file.isOpen())
    return false;
  const bool changed = !m_haveRc || rcChanged(m_lastRc, channels);
  if (!changed && tUs - m_lastRcT < kRcPeriodUs)
    return false;
  QJsonArray a;
  for (int v : channels)
    a.append(v);
  QJsonObject o;
  o["channels"] = a;
  o["source"] = strOrNull(source);
  m_haveRc = true;
  m_lastRc = channels;
  m_lastRcT = tUs;
  return writeObj(tUs, "rc", o);
}

bool FlightEventLog::mode(qint64 tUs, const QString &from, const QString &to,
                          const QString &source) {
  return writeObj(tUs, "mode",
                  {{"from", strOrNull(from)},
                   {"to", strOrNull(to)},
                   {"source", strOrNull(source)}});
}

bool FlightEventLog::arm(qint64 tUs, bool armed) {
  return writeObj(tUs, "arm", {{"armed", armed}});
}

bool FlightEventLog::fault(qint64 tUs, const QString &id, bool active,
                           const QString &detail) {
  return writeObj(
      tUs, "fault",
      {{"id", id}, {"active", active}, {"detail", strOrNull(detail)}});
}

bool FlightEventLog::start(qint64 tUs) { return writeObj(tUs, "start", {}); }

bool FlightEventLog::stop(qint64 tUs, const QString &reason) {
  return writeObj(tUs, "stop", {{"reason", reason}});
}

bool FlightEventLog::marker(qint64 tUs, const QString &text) {
  return writeObj(tUs, "marker", {{"text", text}});
}

QString FlightEventLog::sha256Hex(const QByteArray &data) {
  return QString::fromLatin1(
      QCryptographicHash::hash(data, QCryptographicHash::Sha256).toHex());
}

QList<QJsonObject> FlightEventLog::readEvents(const QString &path) {
  QList<QJsonObject> out;
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly))
    return out;
  const QList<QByteArray> lines = f.readAll().split('\n');
  for (const QByteArray &ln : lines) {
    if (ln.trimmed().isEmpty())
      continue;
    QJsonParseError err;
    const QJsonDocument d = QJsonDocument::fromJson(ln, &err);
    if (err.error == QJsonParseError::NoError && d.isObject())
      out.append(d.object());
  }
  return out;
}

QList<FlightEventLog::Run> FlightEventLog::listRuns(const QString &dir) {
  QMap<QString, Run> byStamp;
  const QFileInfoList files = QDir(dir).entryInfoList(
      {QStringLiteral("gt-*.bin"), QStringLiteral("ev-*.jsonl")},
      QDir::Files | QDir::NoSymLinks);
  for (const QFileInfo &fi : files) {
    const QString gt = stampOf(fi.fileName(), "gt-", ".bin");
    const QString ev = stampOf(fi.fileName(), "ev-", ".jsonl");
    const QString stamp = gt.isEmpty() ? ev : gt;
    if (stamp.isEmpty())
      continue;
    Run &r = byStamp[stamp];
    r.stamp = stamp;
    if (!gt.isEmpty()) {
      r.gtPath = fi.absoluteFilePath();
      r.gtBytes = fi.size();
    } else {
      r.evPath = fi.absoluteFilePath();
      r.evBytes = fi.size();
    }
  }
  QList<Run> out;
  for (auto it = byStamp.cend(); it != byStamp.cbegin();) // newest first
    out.append(*--it);
  return out;
}

bool FlightEventLog::rcChanged(const QVector<int> &last,
                               const QVector<int> &now) {
  if (last.size() != now.size())
    return true;
  for (int i = 0; i < now.size(); ++i) {
    const int d = std::abs(now[i] - last[i]);
    if (i >= kRcFirstSwitch ? d != 0 : d > kRcDeadbandUs)
      return true;
  }
  return false;
}

int FlightEventLog::countEvents(const QString &evPath, qint64 capBytes,
                                bool *capped) {
  if (capped)
    *capped = false;
  QFile f(evPath);
  if (!f.open(QIODevice::ReadOnly))
    return 0;
  int lines = 0;
  qint64 left = capBytes;
  while (!f.atEnd() && left > 0) {
    const QByteArray b = f.read(std::min<qint64>(1 << 16, left));
    lines += b.count('\n');
    left -= b.size();
  }
  if (capped && !f.atEnd())
    *capped = true;
  return std::max(0, lines - 1);
}

int FlightEventLog::countEvents(const QString &evPath) {
  QFile f(evPath);
  if (!f.open(QIODevice::ReadOnly))
    return 0;
  int lines = 0;
  while (!f.atEnd())
    lines += f.read(1 << 16).count('\n');
  return std::max(0, lines - 1); // minus the header
}

int FlightEventLog::prune(const QString &dir, int keep,
                          const QString &protectStamp) {
  if (keep <= 0)
    return 0;
  const QList<Run> runs = listRuns(dir); // newest first
  const bool protectedPresent = std::any_of(
      runs.begin(), runs.end(),
      [&](const Run &r) { return r.stamp == protectStamp; });
  int room = keep - (protectedPresent ? 1 : 0); // for the other runs
  int removed = 0;
  for (const Run &r : runs) {
    if (r.stamp == protectStamp)
      continue;
    if (room > 0) {
      --room;
      continue;
    }
    for (const QString &p : {r.gtPath, r.evPath})
      if (!p.isEmpty() && QFile::remove(p))
        ++removed;
  }
  return removed;
}

} // namespace vsim
