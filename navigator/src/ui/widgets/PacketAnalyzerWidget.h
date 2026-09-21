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

#include <QByteArray>
#include <QCheckBox>
#include <QFile>
#include <QLineEdit>
#include <QMap>
#include <QPushButton>
#include <QTableView>
#include <QTextStream>
#include <QWidget>

class FrequencyRibbon;
class LinkStatsPanel;
class PacketDetailWidget;
class PacketLogModel;
class PacketFilterProxy;
class DroneProtocol;

class PacketAnalyzerWidget : public QWidget {
  Q_OBJECT

public:
  explicit PacketAnalyzerWidget(QWidget *parent = nullptr);
  ~PacketAnalyzerWidget() override = default;

  void setProtocol(DroneProtocol *protocol);
  // Advanced ▸ Packet buffer: cap on rows kept in the analyzer's ring buffer.
  void setPacketCapacity(int rows);

public slots:
  void logRxPacket(const QByteArray &data);
  void logTxPacket(const QByteArray &data);

signals:
  void backToHomeRequested();

protected:
  bool eventFilter(QObject *obj, QEvent *e) override;

private slots:
  void onClearClicked();
  void onSaveClicked();
  void onSelectionChanged();
  void onExpressionEdited();

private:
  void buildUi();
  void resizeColumns();                                 // proportional fill
  void tee(const QString &dir, const QByteArray &data); // stream-to-CSV

  FrequencyRibbon *m_freqRibbon = nullptr;
  LinkStatsPanel *m_linkStats = nullptr;
  QTableView *m_table = nullptr;
  PacketLogModel *m_model = nullptr;
  PacketFilterProxy *m_proxy = nullptr;
  PacketDetailWidget *m_detailView = nullptr;

  QPushButton *m_btnClear = nullptr;
  QPushButton *m_btnStream = nullptr;
  QPushButton *m_btnSave = nullptr;
  QCheckBox *m_chkAutoScroll = nullptr;
  QLineEdit *m_searchEdit = nullptr;
  QLineEdit *m_exprEdit = nullptr;
  QMap<int, QPushButton *> m_typeChips;

  bool m_isStreaming = false;
  QFile m_streamFile;
  QTextStream m_streamOut;
};
