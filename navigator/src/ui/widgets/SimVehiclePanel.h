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

#include "PanelWidgets.h"

class SimulatorWidget;

/**
 * SimVehiclePanel - Vehicle mode page: geometry editor, sensor models, fault injection.
 *
 * Panel id for the workspace registry: "sim.vehicle" (stable; never rename once it is
 * in a saved layout). The widget tree is built by SimulatorWidget's
 * private builder because the controls it creates are the widget's state
 * (m_* members); SimulatorWidget assembles the panels into its splitter/stack.
 */
class SimVehiclePanel : public ui::MinWidthScrollArea {
public:
  static constexpr const char *kPanelId = "sim.vehicle";
  explicit SimVehiclePanel(SimulatorWidget &sim, QWidget *parent = nullptr);
};
