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

#include "PanelRegistry.h"
#include "WorkspaceManager.h"

#include "GlPanel.h"

#include <QLabel>

// The panels Navigator will have (mapping to today's pages is in the commit
// notes / report). Every panel is a blank widget showing its name.
inline ws::PanelRegistry mockPanels() {
  ws::PanelRegistry r;
  auto add = [&r](const char *id, const char *title) {
    const QString t = QString::fromLatin1(title);
    r.add(QString::fromLatin1(id), t, [t] {
      auto *l = new QLabel(t);
      l->setAlignment(Qt::AlignCenter);
      l->setStyleSheet("font-size: 22px; color: #9aa;");
      l->setMinimumSize(120, 80);
      return l;
    });
  };
  // The simulator view is a real QOpenGLWidget, like SimRendererWidget.
  // Default: a QOpenGLWindow in a window container (see GlPanel.h for why).
  // NAV_MOCK_GL=widget swaps in a QOpenGLWidget to reproduce the ADS problem.
  r.add("sim.view", "Simulator view", []() -> QWidget * {
    if (qEnvironmentVariable("NAV_MOCK_GL") == "widget")
      return new GlPanel;
    return new GlWindowPanel;
  });
  add("sim.world", "World");
  add("sim.vehicle", "Vehicle");
  add("sim.tuning", "Tuning");
  add("dashboard", "Dashboard");
  add("packets", "Packet manager");
  add("perf", "Perf analysis");
  add("map", "Map");
  add("logs", "Logs");
  add("rc", "RC channels");
  add("motors", "Motor status");
  add("loop", "Control loop");
  add("gyro", "Gyro notch");
  add("calib", "Calibration");
  add("settings", "Settings");
  return r;
}

// Fly is the default workspace.
inline std::vector<ws::WorkspaceManager::Builtin> mockBuiltins() {
  using ws::LayoutNode;
  using ws::LayoutSpec;
  using ws::WorkspaceManager;
  const auto T = [](QStringList p, QString cur = QString()) {
    return LayoutNode::tabs(p, cur);
  };
  const auto H = Qt::Horizontal, V = Qt::Vertical;
  std::vector<WorkspaceManager::Builtin> b;
  b.push_back({"Fly", {LayoutNode::split(H,
                {T({"dashboard", "rc"}, "dashboard"),
                 LayoutNode::split(V, {T({"map"}), T({"logs", "motors"}, "logs")},
                                   {3, 2})},
                {3, 2})}});
  b.push_back({"Sim", {LayoutNode::split(H,
                {T({"sim.view"}),
                 LayoutNode::split(V,
                     {T({"sim.world", "sim.vehicle", "sim.tuning"}, "sim.world"),
                      T({"logs"})},
                     {3, 1})},
                {3, 2})}});
  b.push_back({"Tune", {LayoutNode::split(H,
                {LayoutNode::split(V, {T({"loop"}), T({"gyro"})}),
                 LayoutNode::split(V,
                     {T({"sim.tuning", "sim.vehicle", "calib"}, "sim.tuning"),
                      T({"logs"})},
                     {3, 1})},
                {3, 2})}});
  b.push_back({"Debug link", {LayoutNode::split(V,
                {LayoutNode::split(H,
                     {T({"dashboard"}), T({"packets", "perf"}, "packets")},
                     {2, 3}),
                 T({"logs"})},
                {3, 1})}});
  return b;
}
