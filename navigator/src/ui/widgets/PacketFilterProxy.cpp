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
#include "PacketFilterProxy.h"

#include "PacketLogModel.h"

PacketFilterProxy::PacketFilterProxy(QObject *parent)
    : QSortFilterProxyModel(parent) {
  setAllTypes(true);
}

void PacketFilterProxy::setTypeEnabled(int t, bool on) {
  if (on)
    m_types.insert(t);
  else
    m_types.remove(t);
  invalidateFilter();
}

void PacketFilterProxy::setAllTypes(bool on) {
  m_types.clear();
  if (on) {
    for (int t = 0x0; t <= 0xA; ++t)
      m_types.insert(t);
    m_types.insert(0xFF); // RAW
  }
  invalidateFilter();
}

void PacketFilterProxy::setDirection(Direction d) {
  m_dir = d;
  invalidateFilter();
}

void PacketFilterProxy::setDeviceFilter(int dev) {
  m_dev = dev;
  invalidateFilter();
}

void PacketFilterProxy::setSearch(const QString &s) {
  m_search = s.trimmed().toLower();
  m_searchHex = m_search;
  m_searchHex.remove(' ');
  invalidateFilter();
}

bool PacketFilterProxy::setExpression(const QString &expr) {
  const bool ok = m_expr.compile(expr);
  invalidateFilter();
  return ok;
}

bool PacketFilterProxy::filterAcceptsRow(int row, const QModelIndex &) const {
  const auto *model = qobject_cast<PacketLogModel *>(sourceModel());
  if (!model)
    return true;
  const PacketEntry *e = model->entry(row);
  if (!e)
    return false;

  // Type chips.
  if (!m_types.contains(e->type))
    return false;
  // Direction.
  if (m_dir == RxOnly && e->tx)
    return false;
  if (m_dir == TxOnly && !e->tx)
    return false;
  // Device.
  if (m_dev >= 0 && (e->type == 0xFF || e->dev != m_dev))
    return false;

  // Search: hex bytes or info text.
  if (!m_search.isEmpty()) {
    const QString hex = QString::fromLatin1(e->raw.toHex());
    const bool hexHit = !m_searchHex.isEmpty() && hex.contains(m_searchHex);
    const bool infoHit = e->info.toLower().contains(m_search);
    if (!hexHit && !infoHit)
      return false;
  }

  // Expression.
  if (!m_expr.empty()) {
    FilterCtx c;
    c.type = e->type;
    c.typeName = e->typeName.toLower();
    c.tx = e->tx;
    c.dev = e->dev;
    c.len = e->len;
    c.info = e->info.toLower();
    c.origin = -1;
    if (e->type == 0x6 && e->raw.size() >= 9)
      c.origin = static_cast<uint8_t>(e->raw[8]);
    if (!m_expr.eval(c))
      return false;
  }
  return true;
}
