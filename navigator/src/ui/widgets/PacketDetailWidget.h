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

#include "HexView.h"
#include "protocol/PacketDissector.h"

#include <QByteArray>
#include <QTreeWidget>
#include <QWidget>

/**
 * Wireshark-style per-packet detail: a dissection field tree over a hex pane.
 * Selecting a field highlights its byte range in the hex view.
 */
class PacketDetailWidget : public QWidget {
  Q_OBJECT
public:
  explicit PacketDetailWidget(QWidget *parent = nullptr);
  void setData(const QByteArray &data);
  void clear();

private:
  void setupUi();
  void addFields(QTreeWidgetItem *parent, const DissectField &f);

  QTreeWidget *m_tree = nullptr;
  HexView *m_hex = nullptr;
  QByteArray m_data;
};
