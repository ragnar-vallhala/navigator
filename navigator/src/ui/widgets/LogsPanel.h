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

#include "FaultsPanel.h"
#include "LogPanel.h"
#include <QTabWidget>

/**
 * Home "logs" panel (homepanel::kLogs): Log and Faults share the bottom slot as
 * tabs. The faults list is persistent but low-traffic, so it must not cost the
 * log any height; the tab title carries the unacknowledged count so a raised
 * fault is visible from Log.
 */
class LogsPanel : public QTabWidget {
  Q_OBJECT

public:
  explicit LogsPanel(QWidget *parent = nullptr);

  LogPanel *logPanel() const { return m_log; }
  FaultsPanel *faultsPanel() const { return m_faults; }

private:
  LogPanel *m_log = nullptr;
  FaultsPanel *m_faults = nullptr;
};
