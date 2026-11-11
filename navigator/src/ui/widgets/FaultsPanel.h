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

#include <QString>
#include <QWidget>

class FaultsModel;
class QLabel;
class QListView;
class QPushButton;

// Lists the active faults from a FaultsModel: severity colour + name + text,
// seen-count, and Acknowledge / Clear buttons. The model is owned by the panel
// unless one is passed in. badgeText() / countsChanged() are what a status bar
// or dock tab needs to show a count.
class FaultsPanel : public QWidget {
  Q_OBJECT

public:
  // Stable id for the planned dock-panel migration.
  static constexpr const char *kPanelId = "faults";

  explicit FaultsPanel(QWidget *parent = nullptr);
  explicit FaultsPanel(FaultsModel *model, QWidget *parent = nullptr);

  FaultsModel *model() const { return m_model; }

  // "" when no active fault is unacknowledged, else that count ("3"). Cleared
  // rows never count here.
  QString badgeText() const;
  // Title for a tab / section header: "Faults", "Faults (3)" (active,
  // unacknowledged), "Faults (3, 1 cleared)" or "Faults (1 cleared)" when the
  // firmware has cleared faults nobody has dismissed yet.
  QString title() const;
  // True when there is an unacknowledged active fault or an undismissed cleared
  // one: the tab should be coloured.
  bool needsAttention() const;
  // Colour for the worst unacknowledged active severity; kWarn when only
  // cleared rows remain; kOk when nothing is listed as pending.
  QColor badgeColor() const;
  // Armed state of the vehicle: while armed, Clear is disabled (the firmware
  // sends a fault once while armed, so clearing it in flight would hide it
  // until landing). Acknowledge stays available.
  void setArmed(bool armed);

  static QColor severityColor(int severity);
  static QString severityLabel(int severity);

signals:
  void countsChanged(int unacked, int serious);

private:
  void build();
  void refresh();

  FaultsModel *m_model;
  QListView *m_view = nullptr;
  QLabel *m_empty = nullptr;
  QLabel *m_summary = nullptr;
  QPushButton *m_ackBtn = nullptr;
  QPushButton *m_clearBtn = nullptr;
  bool m_armed = false;
};
