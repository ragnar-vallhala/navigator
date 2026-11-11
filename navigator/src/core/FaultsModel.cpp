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
#include "FaultsModel.h"

#include <QDateTime>

FaultsModel::FaultsModel(QObject *parent) : QAbstractListModel(parent) {}

int FaultsModel::rowCount(const QModelIndex &parent) const {
  return parent.isValid() ? 0 : static_cast<int>(m_rows.size());
}

QVariant FaultsModel::data(const QModelIndex &index, int role) const {
  if (!index.isValid() || index.row() < 0 || index.row() >= m_rows.size())
    return {};
  const Row &r = m_rows[index.row()];
  switch (role) {
  case Qt::DisplayRole:
    return r.text.isEmpty() ? displayName(r.id, r.name)
                            : displayName(r.id, r.name) + QStringLiteral(": ") +
                                  r.text;
  case IdRole:
    return r.id;
  case NameRole:
    return r.name;
  case TextRole:
    return r.text;
  case SeverityRole:
    return r.severity;
  case CountRole:
    return r.count;
  case FirstSeenRole:
    return r.firstMs;
  case LastSeenRole:
    return r.lastMs;
  case AckedRole:
    return r.acked;
  default:
    return {};
  }
}

QHash<int, QByteArray> FaultsModel::roleNames() const {
  auto n = QAbstractListModel::roleNames();
  n[IdRole] = "faultId";
  n[NameRole] = "name";
  n[TextRole] = "text";
  n[SeverityRole] = "severity";
  n[CountRole] = "count";
  n[FirstSeenRole] = "firstSeen";
  n[LastSeenRole] = "lastSeen";
  n[AckedRole] = "acked";
  return n;
}

QString FaultsModel::displayName(uint16_t id, const QString &name) {
  if (!name.isEmpty())
    return name;
  return QStringLiteral("Fault %1 (0x%2)")
      .arg(id)
      .arg(id, 4, 16, QLatin1Char('0'));
}

int FaultsModel::indexOfId(uint16_t id) const {
  for (int i = 0; i < m_rows.size(); ++i)
    if (m_rows[i].id == id)
      return i;
  return -1;
}

void FaultsModel::apply(const FaultEvent &e, qint64 nowMs) {
  if (nowMs < 0)
    nowMs = QDateTime::currentMSecsSinceEpoch();
  const int at = indexOfId(e.id);

  if (!e.active) {
    if (at < 0)
      return;
    beginRemoveRows({}, at, at);
    m_rows.removeAt(at);
    endRemoveRows();
    emitCounts();
    return;
  }

  if (at < 0) {
    const int n = static_cast<int>(m_rows.size());
    beginInsertRows({}, n, n);
    m_rows.push_back(
        {e.id, e.severity, e.name, e.text, 1, nowMs, nowMs, false});
    endInsertRows();
    emitCounts();
    return;
  }

  Row &r = m_rows[at];
  r.count++;
  r.lastMs = nowMs;
  // The firmware may restate severity / text; the latest frame wins.
  const bool sevChanged = r.severity != e.severity;
  r.severity = e.severity;
  if (!e.text.isEmpty())
    r.text = e.text;
  if (!e.name.isEmpty())
    r.name = e.name;
  const QModelIndex ix = index(at);
  emit dataChanged(ix, ix);
  if (sevChanged)
    emitCounts();
}

int FaultsModel::unackedCount() const {
  int n = 0;
  for (const Row &r : m_rows)
    n += r.acked ? 0 : 1;
  return n;
}

int FaultsModel::unackedSeriousCount() const {
  int n = 0;
  for (const Row &r : m_rows)
    n += (!r.acked && r.severity <= kErrorSeverity) ? 1 : 0;
  return n;
}

int FaultsModel::worstUnackedSeverity() const {
  int w = -1;
  for (const Row &r : m_rows)
    if (!r.acked && (w < 0 || r.severity < w))
      w = r.severity;
  return w;
}

void FaultsModel::acknowledgeAll() {
  bool any = false;
  for (Row &r : m_rows) {
    if (!r.acked) {
      r.acked = true;
      any = true;
    }
  }
  if (!any)
    return;
  if (!m_rows.isEmpty())
    emit dataChanged(index(0), index(static_cast<int>(m_rows.size()) - 1));
  emitCounts();
}

void FaultsModel::clear() {
  if (m_rows.isEmpty())
    return;
  beginResetModel();
  m_rows.clear();
  endResetModel();
  emitCounts();
}

void FaultsModel::emitCounts() {
  emit countsChanged(unackedCount(), unackedSeriousCount());
}
