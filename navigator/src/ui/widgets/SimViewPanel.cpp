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
#include "SimViewPanel.h"

#include "SimPanelsCommon.h"

SimViewPanel::SimViewPanel(SimulatorWidget &sim, QWidget *parent) : QWidget(parent) {
  sim.buildViewPanel(this);
}

void SimulatorWidget::buildViewPanel(QWidget *viewHost) {
  // The 3D view is a QOpenGLWindow in a window container (so it survives being
  // docked/undocked; see SimRendererWidget). A native window is always above its
  // sibling widgets, so nothing can overlay it: the telemetry HUD is painted by
  // the renderer itself, and the three instrument panes (horizon, down-cam,
  // contours) sit in a rail beside it instead of floating over it.
auto *viewLay = new QHBoxLayout(viewHost);
viewLay->setContentsMargins(0, 0, 0, 0);
viewLay->setSpacing(4);
m_pipRail = new QWidget(viewHost);
m_pipRail->setFixedWidth(kPipRailW);
auto *railLay = new QVBoxLayout(m_pipRail);
railLay->setContentsMargins(0, 0, 0, 0);
railLay->setSpacing(4);
viewLay->addWidget(m_pipRail);

m_renderer = new vsim::SimRendererWidget();
m_rendererBox = QWidget::createWindowContainer(m_renderer, viewHost);
m_rendererBox->setMinimumSize(400, 300);
m_rendererBox->setFocusPolicy(Qt::StrongFocus); // G/R/X/Y/Z/Esc/WASD
viewLay->addWidget(m_rendererBox, 1);


// FPV telemetry HUD: never shown as a widget; the renderer paints it over
// the frame while the sim runs (setHudVisible).
m_hud = new SimHudWidget(this);
m_hud->hide();
m_renderer->setHudWidget(m_hud);

// Artificial horizon (mockup .pip.adi), fed from the sim snapshot in updateHud.
m_horizon = new HorizonHud();
m_horizonPip = new PipPane(tr("HORIZON"), m_horizon, 150, m_pipRail);
railLay->addWidget(m_horizonPip);

// Down-cam pane (mockup .pip.downcam): a second renderer looking straight down
// at the drone, in its own window container. Fed the same pose + drone mesh
// as the main view.
m_downRenderer = new vsim::SimRendererWidget();
m_downRenderer->setDownCam(true);
m_downRenderer->setWorldVisible(true);
QWidget *downBox = QWidget::createWindowContainer(m_downRenderer);
downBox->setMinimumSize(100, 80);
m_downPip = new PipPane(tr("DOWN CAM"), downBox, 170, m_pipRail);
railLay->addWidget(m_downPip);

// Contour minimap pane: top-down hypsometric + contour view of the procedural
// terrain, centred on the drone (running) or the free-fly camera. Helps read
// height, which the perspective view hides. Driven by m_minimapTimer.
m_minimap = new ContourMinimapWidget();
m_minimapPip = new PipPane(tr("CONTOURS"), m_minimap, 200, m_pipRail);
railLay->addWidget(m_minimapPip);
railLay->addStretch(1);
m_minimapTimer = new QTimer(this);
m_minimapTimer->setInterval(120); // ~8 Hz: cheap, smooth enough to follow
connect(m_minimapTimer, &QTimer::timeout, this,
        &SimulatorWidget::updateMinimap);
m_minimapTimer->start();

}
