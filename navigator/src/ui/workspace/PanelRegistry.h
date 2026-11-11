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
#include <functional>
#include <vector>

namespace ws {

struct PanelDef {
  QString id;    // stable, used in saved layouts: never rename once shipped
  QString title; // shown on the tab / window title
  std::function<QWidget *()> create;
};

// The list of panels an application offers. Panels are created once, in
// registration order, and re-hosted (not recreated) when layouts change.
class PanelRegistry {
public:
  void add(const QString &id, const QString &title,
           std::function<QWidget *()> create) {
    m_defs.push_back({id, title, std::move(create)});
  }
  const std::vector<PanelDef> &defs() const { return m_defs; }

private:
  std::vector<PanelDef> m_defs;
};

} // namespace ws
