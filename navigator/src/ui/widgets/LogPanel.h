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

#include <QCheckBox>
#include <QElapsedTimer>
#include <QGroupBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QString>

#include <deque>

class LogPanel : public QGroupBox {
  Q_OBJECT

public:
  explicit LogPanel(QWidget *parent = nullptr);

  // Timestamp prefix style (Settings ▸ Logging ▸ Timestamps).
  enum TimestampMode { Local = 0, Utc = 1, Elapsed = 2 };

public slots:
  void appendLog(const QString &msg);
  void clearLog();
  // Freeze/unfreeze the scrollback view (mockup Pause); disk logging continues.
  void setPaused(bool paused);

  // ---- Settings ▸ Logging & Recording --------------------------------------
  void setMaxLines(int maxLines);  // ring-buffer cap
  void setTimestampMode(int mode); // Local / UTC / Elapsed (T+)
  // Dump the full session buffer (unfiltered) to a text file. Returns true on
  // success; used by MainWindow's "export on disconnect".
  bool exportToFile(const QString &path) const;

private:
  // One buffered line: the wall-clock + monotonic instants it was logged at (so
  // the timestamp can be re-rendered in any mode) and the raw message text
  // (without the timestamp prefix).
  struct Entry {
    qint64 wallMs;
    qint64 elapsedMs;
    QString msg;
  };

  QString stampFor(const Entry &e) const; // timestamp prefix in the cur mode
  QString format(const Entry &e) const;   // "[stamp] msg"
  void rebuildView();                     // re-render m_text from m_entries

  QPlainTextEdit *m_text;
  QCheckBox *m_autoScroll;
  QPushButton *m_pauseBtn = nullptr;
  bool m_paused = false;

  std::deque<Entry> m_entries; // full ring buffer (capped at m_maxLines)
  int m_maxLines = 2000;
  int m_tsMode = Local;
  QElapsedTimer m_clock; // for Elapsed (T+) timestamps; started at construction
};
