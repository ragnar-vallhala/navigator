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
#include "SimWorldPanel.h"

#include "SimPanelsCommon.h"
#include "SimControlsPanel.h"

SimWorldPanel::SimWorldPanel(SimulatorWidget &sim, QWidget *parent)
    : ui::MinWidthScrollArea(parent) {
  sim.buildWorldPage(this);
}

void SimulatorWidget::buildWorldPage(ui::MinWidthScrollArea *scroll) {
  auto *page = new QWidget();
  auto *pv = new QVBoxLayout(page);
  pv->setContentsMargins(0, 0, 0, 0);
  pv->setSpacing(6);

  m_worldEditor = new WorldEditorWidget(page);
  connect(m_worldEditor, &WorldEditorWidget::worldApplied, this, [this] {
    if (m_sim)
      m_sim->sendWorld(m_worldEditor->config());
    persistWorld(m_worldEditor->config());
    appendLog(
        "world",
        tr("world applied (g=%1 m/s²)").arg(m_worldEditor->config().gravity));
  });
  // Wind & turbulence: its own staged Apply → push + persist.
  connect(m_worldEditor, &WorldEditorWidget::windApplied, this, [this] {
    const vsim::WindConfig w = m_worldEditor->windConfig();
    if (m_sim)
      m_sim->sendWind(w);
    persistWind(w);
    appendLog("world",
              w.enabled ? tr("wind applied (steady %1/%2/%3 m/s, turb σ=%4)")
                              .arg(w.steady.x())
                              .arg(w.steady.y())
                              .arg(w.steady.z())
                              .arg(w.turbSigma)
                        : tr("wind disabled"));
  });
  // Obstacles are visual (phase 1): live-update the 3D view + persist on any
  // add/remove/edit, no Apply needed.
  connect(m_worldEditor, &WorldEditorWidget::obstaclesChanged, this, [this] {
    if (m_renderer)
      m_renderer->setObstacles(m_worldEditor->config().obstacles);
    if (m_downRenderer)
      m_downRenderer->setObstacles(m_worldEditor->config().obstacles);
    if (m_sim)
      m_sim->sendObstacles(m_worldEditor->config().obstacles);
    persistWorld(m_worldEditor->config());
  });
  // 3D gizmo editing of obstacles <-> the editor list/form.
  connect(m_renderer, &vsim::SimRendererWidget::obstacleEdited, m_worldEditor,
          &WorldEditorWidget::setObstacleFromGizmo);
  connect(m_renderer, &vsim::SimRendererWidget::obstacleSelected,
          m_worldEditor, &WorldEditorWidget::selectObstacleRow);
  connect(m_worldEditor, &WorldEditorWidget::obstacleSelectionChanged,
          m_renderer, &vsim::SimRendererWidget::selectObstacle);
  // Imported world mesh (visual): (re)load + render + persist on any change.
  connect(m_worldEditor, &WorldEditorWidget::worldMeshChanged, this, [this] {
    loadWorldMeshToRenderer();
    persistWorld(m_worldEditor->config());
  });
  pv->addWidget(m_worldEditor);

  // ---- Training: glowing halo-gate course flown through the world ----
  {
    auto *sec = new CollapsibleSection(tr("Training"), page);
    auto *body = new QWidget();
    auto *col = new QVBoxLayout(body);
    col->setContentsMargins(0, 0, 0, 0);
    col->setSpacing(6);

    auto *row = new QHBoxLayout();
    row->addWidget(new QLabel(tr("Mode:"), body));
    m_trainingMode = new QComboBox(body);
    m_trainingMode->addItem(tr("Off"), vsim::TrainingCourse::Off);
    m_trainingMode->addItem(tr("Easy"), vsim::TrainingCourse::Easy);
    m_trainingMode->addItem(tr("Medium"), vsim::TrainingCourse::Medium);
    m_trainingMode->addItem(tr("Hard"), vsim::TrainingCourse::Hard);
    m_trainingMode->setToolTip(tr(
        "Fly the drone through the glowing halo gates. The course starts at "
        "the floor and gets progressively harder (smaller, farther, more "
        "weave). A yellow arrow over the drone points to the next gate. "
        "Selecting a mode resets the airframe to the floor."));
    connect(m_trainingMode,
            QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            [this] {
              if (m_trainingMode)
                setTrainingMode(m_trainingMode->currentData().toInt());
            });
    row->addWidget(m_trainingMode, 1);
    col->addLayout(row);

    m_trainingStatus = new QLabel(tr("Training off"), body);
    m_trainingStatus->setStyleSheet(QString("color:%1; font-size:11px;")
                                        .arg(Theme::hex(Theme::kTextMuted)));
    col->addWidget(m_trainingStatus);

    sec->setContentWidget(body);
    pv->addWidget(sec);
  }

  // ---- Simulation controls: Start/Stop/Reset, rates, status (sim.controls) ----
  pv->addWidget(new SimControlsPanel(*this, page));
  pv->addStretch();

  scroll->setWidgetResizable(true);
  scroll->setWidget(page);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
}
