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

#include "PacketFilterExpr.h"

#include <QSet>
#include <QSortFilterProxyModel>
#include <QString>

/**
 * Filters the packet log without ever rebuilding it. ANDs together: enabled
 * type set, direction, device, a hex/text search, and a Wireshark-style
 * expression. All setters re-filter instantly.
 */
class PacketFilterProxy : public QSortFilterProxyModel {
  Q_OBJECT
public:
  enum Direction { Both, RxOnly, TxOnly };

  explicit PacketFilterProxy(QObject *parent = nullptr);

  void setTypeEnabled(int typeNibble, bool on); // 0xFF = RAW
  void setAllTypes(bool on);
  bool typeEnabled(int typeNibble) const {
    return m_types.contains(typeNibble);
  }

  void setDirection(Direction d);
  void setDeviceFilter(int dev); // -1 = any
  void setSearch(const QString &s);
  bool setExpression(const QString &expr); // false on parse error
  QString expressionError() const { return m_expr.error(); }

protected:
  bool filterAcceptsRow(int row, const QModelIndex &parent) const override;

private:
  QSet<int> m_types; // enabled type nibbles (+0xFF)
  Direction m_dir = Both;
  int m_dev = -1;
  QString m_search;    // normalized lower-case
  QString m_searchHex; // search with spaces stripped (for hex match)
  PacketFilterExpr m_expr;
};
