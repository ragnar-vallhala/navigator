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
#include "SimVehiclePanel.h"

#include "SimPanelsCommon.h"

SimVehiclePanel::SimVehiclePanel(SimulatorWidget &sim, QWidget *parent)
    : ui::MinWidthScrollArea(parent) {
  sim.buildVehiclePage(this);
}

void SimulatorWidget::buildVehiclePage(ui::MinWidthScrollArea *scroll) {
  m_geomEditor = new GeometryEditorWidget();
  // Persist on every config change (mesh load, edit, gizmo) — not just on
  // Apply — so the last-selected vehicle is restored on the next launch via
  // restoreGeometry() below.
  connect(m_geomEditor, &GeometryEditorWidget::previewUpdated, this, [this] {
    applyGeometryToRenderer();
    persistGeometry(m_geomEditor->config());
    // Any edit makes an earlier autotune result stale for this vehicle.
    m_customTune.onVehicleChanged();
    updateCustomWarning(false);
  });
  // Gizmo edits in the 3D view flow back into the editor (CoM frame).
  connect(m_renderer, &vsim::SimRendererWidget::motorEdited, m_geomEditor,
          &GeometryEditorWidget::setMotorFromGizmo);
  connect(m_geomEditor, &GeometryEditorWidget::presetChosen, this,
          [this](const QString &id) {
            m_presetId = id;
            m_customTune.onVehicleChanged();
            updateCustomWarning(false);
            applyGeometryToRenderer(); // the model depicts the new class
            // Preset changed (or went Custom): the old round is moot. The
            // new set is armed when Apply pushes it.
            m_retrier.cancel(QStringLiteral("gains"));
            QSettings().setValue("simulator/presetId", id);
          });
  connect(m_geomEditor, &GeometryEditorWidget::geometryApplied, this, [this] {
    applyGeometryToRenderer();
    if (m_sim) {
      armMixPush(m_geomEditor->physicsConfig());
      m_sim->sendGeometry(m_geomEditor->physicsConfig());
      pushAirframeGains(true); // the gains follow the airframe, live
    } else {
      pushFirmwareMotorGeometry(m_geomEditor->physicsConfig());
    }
    persistGeometry(m_geomEditor->config());
    appendLog(
        "geom",
        tr("geometry applied (m=%1 kg)").arg(m_geomEditor->config().mass));
  });
  // Vehicle page = geometry editor + the Fault Injection panel (mockup
  // groups them in the vehicle config column).
  auto *vehWrap = new QWidget();
  auto *vehV = new QVBoxLayout(vehWrap);
  vehV->setContentsMargins(0, 0, 0, 0);
  vehV->setSpacing(6);
  vehV->addWidget(m_geomEditor);
  vehV->addWidget(buildSensorPanel());
  vehV->addWidget(buildFaultPanel());
  vehV->addStretch();

  scroll->setWidgetResizable(true);
  scroll->setWidget(vehWrap);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
}

QWidget *SimulatorWidget::buildSensorPanel() {
  // Mockup Vehicle ▸ Sensor Models: per-sensor white-noise σ + bias clip, plus
  // an enable toggle that doubles as a dropout fault. Defaults match the
  // daemon's SensorNoise (BMX160-class).
  auto *grp = new QGroupBox(tr("Sensor Models"), this);
  auto *grid = new QGridLayout(grp);
  grid->setHorizontalSpacing(10);
  grid->setVerticalSpacing(4);
  grid->addWidget(new QLabel(tr("Sensor"), grp), 0, 0);
  grid->addWidget(new QLabel(tr("On"), grp), 0, 1);
  grid->addWidget(new QLabel(tr("Noise σ"), grp), 0, 2);
  grid->addWidget(new QLabel(tr("Bias clip"), grp), 0, 3);

  struct Def {
    const char *name;
    double sigma;
    double clip;
    int dec;
    double step;
  };
  const Def defs[3] = {
      {"Accel (m/s²)", 0.03, 0.08, 4, 0.005},
      {"Gyro (rad/s)", 0.0014, 0.012, 5, 0.0005},
      {"Mag (µT)", 0.3, 5.0, 3, 0.1},
  };
  for (int i = 0; i < 3; ++i) {
    auto &r = m_sensorRow[i];
    grid->addWidget(new QLabel(tr(defs[i].name), grp), i + 1, 0);
    r.en = new QCheckBox(grp);
    r.en->setChecked(true);
    r.en->setToolTip(tr("Off = drop this sensor's feed (dropout fault)"));
    grid->addWidget(r.en, i + 1, 1, Qt::AlignCenter);
    r.sigma = new QDoubleSpinBox(grp);
    r.sigma->setRange(0.0, 100.0);
    r.sigma->setDecimals(defs[i].dec);
    r.sigma->setSingleStep(defs[i].step);
    r.sigma->setValue(defs[i].sigma);
    grid->addWidget(r.sigma, i + 1, 2);
    r.clip = new QDoubleSpinBox(grp);
    r.clip->setRange(0.0, 100.0);
    r.clip->setDecimals(defs[i].dec);
    r.clip->setSingleStep(defs[i].step);
    r.clip->setValue(defs[i].clip);
    grid->addWidget(r.clip, i + 1, 3);

    connect(r.en, &QCheckBox::toggled, this, [this] { pushNoise(); });
    connect(r.sigma, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            [this] { pushNoise(); });
    connect(r.clip, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            [this] { pushNoise(); });
  }
  return grp;
}

QWidget *SimulatorWidget::buildFaultPanel() {
  // Mockup Vehicle ▸ Fault Injection: kill any rotor mid-flight, drop the RC
  // feed (firmware failsafe), or a GPS glitch (no GPS model in SITL yet).
  auto *grp = new QGroupBox(tr("Fault Injection"), this);
  auto *v = new QVBoxLayout(grp);
  v->setSpacing(6);

  auto *killRow = new QHBoxLayout();
  killRow->addWidget(new QLabel(tr("Kill motor:"), grp));
  for (int i = 0; i < 4; ++i) {
    m_killBtn[i] = new QPushButton(QString("M%1").arg(i + 1), grp);
    m_killBtn[i]->setCheckable(true);
    m_killBtn[i]->setFixedWidth(40);
    m_killBtn[i]->setToolTip(
        tr("Cut rotor %1 (dead ESC) while flying").arg(i + 1));
    connect(m_killBtn[i], &QPushButton::toggled, this, [this, i](bool on) {
      m_motorKill[i] = on;
      pushFaults();
      appendLog("fault", on ? tr("motor %1 killed").arg(i + 1)
                            : tr("motor %1 restored").arg(i + 1));
    });
    killRow->addWidget(m_killBtn[i]);
  }
  killRow->addStretch();
  v->addLayout(killRow);

  auto *rcLoss = new QCheckBox(tr("RC link loss (→ firmware failsafe)"), grp);
  rcLoss->setToolTip(tr("Stop feeding RC so the firmware enters failsafe"));
  connect(rcLoss, &QCheckBox::toggled, this, [this](bool on) {
    // Link loss is distinct from the RC-enable box: it makes the bridge send
    // NO_SIGNAL so the firmware's RC failsafe fires. The enable box is left
    // alone, so unchecking this restores exactly the prior RC state.
    if (m_rc)
      m_rc->setLinkLost(on);
    appendLog("fault",
              on ? tr("RC link loss injected") : tr("RC link restored"));
  });
  v->addWidget(rcLoss);

  auto *gps = new QCheckBox(tr("GPS glitch"), grp);
  gps->setEnabled(false); // no GPS model in the SITL yet
  gps->setToolTip(tr("No GPS is simulated yet — placeholder for parity"));
  v->addWidget(gps);

  return grp;
}
