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
#include "AttitudePanel.h"

#include "../../core/Theme.h"
#include <QButtonGroup>
#include <QHBoxLayout>
#include <QPushButton>
#include <QVBoxLayout>

AttitudePanel::AttitudePanel(QWidget *parent, bool docked)
    : QGroupBox("Attitude", parent), m_docked(docked) {
  m_body = new QVBoxLayout(this);

  // Header: 2D / 3D segmented toggle at the top right (mockup .tt).
  auto *headerLayout = new QHBoxLayout();
  headerLayout->addStretch();
  auto *btn2d = new QPushButton("2D", this);
  auto *btn3d = new QPushButton("3D", this);
  auto *viewGroup = new QButtonGroup(this);
  viewGroup->setExclusive(true);
  for (auto *b : {btn2d, btn3d}) {
    b->setObjectName("ToggleButton");
    b->setCheckable(true);
    // Fixed height only; let the width follow the text + padding so the label
    // is never clipped (was setFixedSize, which cropped "2D"/"3D").
    b->setFixedHeight(22);
    b->setMinimumWidth(38);
    b->setCursor(Qt::PointingHandCursor);
  }
  btn2d->setChecked(true);
  btn2d->setToolTip(tr("2D HUD"));
  btn3d->setToolTip(tr("3D airframe view"));
  viewGroup->addButton(btn2d, 0);
  viewGroup->addButton(btn3d, 1);
  connect(viewGroup, &QButtonGroup::idClicked, this,
          [this](int id) { emit view3dToggled(id == 1); });
  headerLayout->addWidget(btn2d);
  headerLayout->addWidget(btn3d);
  m_body->addLayout(headerLayout);

  // Firmware system-state pill — ABOVE the ADI (mockup .state-bar). Shows
  // ARMED / STANDBY / FAILSAFE as status packets arrive; starts muted so it
  // doesn't compete with the status-bar connection pill (FR-UX-05).
  m_statusLabel = new QLabel("—", this);
  m_statusLabel->setAlignment(Qt::AlignCenter);
  m_statusLabel->setStyleSheet(
      QString("font-size: 16px; font-weight: bold; color: %1; "
              "background: %2; border: 1px solid %3; "
              "border-radius: 4px; padding: 4px; margin-bottom: 6px;")
          .arg(Theme::hex(Theme::kTextDim), Theme::hex(Theme::kBg),
               Theme::hex(Theme::kBorder)));
  m_body->addWidget(m_statusLabel);

  m_stack = new QStackedWidget(this);
  QWidget *attW = nullptr;
  QWidget *d3W = nullptr;
  if (docked) {
    auto *a = new AttitudeDockedWidget(m_stack);
    auto *d = new Drone3DDockedWidget(m_stack);
    m_attitude = a;
    m_drone3d = d;
    attW = a;
    d3W = d;
  } else {
    auto *a = new AttitudeWidget(m_stack);
    auto *d = new Drone3DWidget(m_stack);
    m_attitude = a;
    m_drone3d = d;
    attW = a;
    d3W = d;
  }
  m_stack->addWidget(attW);
  m_stack->addWidget(d3W);
  m_body->addWidget(m_stack, 1);

  // Flight-mode pill: STABILISE / ACRO + source (RC switch or GCS override),
  // driven by SYSTEM_ORIGIN_FLIGHT_MODE telemetry just like the state pill above.
  m_flightModeLabel = new QLabel("—", this);
  m_flightModeLabel->setAlignment(Qt::AlignCenter);
  m_flightModeLabel->setStyleSheet(
      QString("font-size: 13px; font-weight: bold; color: %1; "
              "background: %2; border: 1px solid %3; "
              "border-radius: 4px; padding: 3px; margin-bottom: 8px;")
          .arg(Theme::hex(Theme::kTextDim), Theme::hex(Theme::kBg),
               Theme::hex(Theme::kBorder)));
  m_body->addWidget(m_flightModeLabel);
}
