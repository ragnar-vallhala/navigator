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

#include "Types.h" // FaultEvent
#include <QAbstractListModel>
#include <QVector>

// The GCS's persistent fault list, fed by FAULT_STATUS. One row per fault id,
// kept in first-seen order:
//   - a raised event for an id already listed only bumps its count / last-seen
//     (the FC re-sends every raised fault every 5 s, so this must not add rows)
//   - a cleared event removes the row
//   - Acknowledge marks every row acknowledged (stays listed, no longer counted
//     by unackedCount()); a later resend keeps it acknowledged
//   - clear() drops every row; a fault still raised reappears on its next resend
// Severity is the syslog scale (0 EMERGENCY .. 7 DEBUG); <= ERROR (3) is what
// the badge reports as "serious".
class FaultsModel : public QAbstractListModel {
  Q_OBJECT

public:
  enum Role {
    IdRole = Qt::UserRole + 1,
    NameRole,     // faults.json key, or empty for an unknown id
    TextRole,     // the firmware's text, "" if none
    SeverityRole, // int, 0..7
    CountRole,    // times seen since first raised
    FirstSeenRole, // qint64 ms since epoch
    LastSeenRole,  // qint64 ms since epoch
    AckedRole      // bool
  };

  static constexpr int kErrorSeverity = 3; // NAVLINK_SEVERITY_ERROR

  explicit FaultsModel(QObject *parent = nullptr);

  int rowCount(const QModelIndex &parent = {}) const override;
  QVariant data(const QModelIndex &index, int role) const override;
  QHash<int, QByteArray> roleNames() const override;

  // Raise or clear one fault. nowMs is injectable for tests.
  void apply(const FaultEvent &e, qint64 nowMs = -1);

  // Rows not yet acknowledged / the subset at severity <= ERROR.
  int unackedCount() const;
  int unackedSeriousCount() const;
  // Lowest severity number among unacknowledged rows, or -1 if none.
  int worstUnackedSeverity() const;

  // "IMU_INIT_FAILED", or "Fault 4660 (0x1234)" for an id with no name.
  static QString displayName(uint16_t id, const QString &name);

public slots:
  void acknowledgeAll();
  void clear();

signals:
  // Emitted whenever unackedCount() / unackedSeriousCount() may have changed.
  void countsChanged(int unacked, int serious);

private:
  struct Row {
    uint16_t id;
    uint8_t severity;
    QString name;
    QString text;
    int count;
    qint64 firstMs;
    qint64 lastMs;
    bool acked;
  };
  int indexOfId(uint16_t id) const;
  void emitCounts();

  QVector<Row> m_rows;
};
