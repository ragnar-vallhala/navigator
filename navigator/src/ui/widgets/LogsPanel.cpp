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
#include "LogsPanel.h"

#include <QColor>
#include <QTabBar>

LogsPanel::LogsPanel(QWidget *parent) : QTabWidget(parent) {
  m_log = new LogPanel(this);
  m_log->setMinimumHeight(180);
  m_faults = new FaultsPanel(this);
  setObjectName("HomeBottomTabs");
  setMinimumHeight(180);
  addTab(m_log, tr("Log"));
  addTab(m_faults, m_faults->title());
  connect(m_faults, &FaultsPanel::countsChanged, this, [this] {
    const int i = indexOf(m_faults);
    setTabText(i, m_faults->title());
    tabBar()->setTabTextColor(
        i, m_faults->needsAttention() ? m_faults->badgeColor() : QColor());
  });
}
