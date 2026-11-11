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
#include "SimControlsPanel.h"

#include "SimPanelsCommon.h"

SimControlsPanel::SimControlsPanel(SimulatorWidget &sim, QWidget *parent) : QWidget(parent) {
  // Same policy as the CollapsibleSection it wraps, so the panel sizes as the
  // bare section did inside the World page.
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  sim.buildSimControls(this);
}

void SimulatorWidget::buildSimControls(QWidget *host) {
  auto *simSec = new CollapsibleSection(tr("Simulation"), host);
  auto *simBody = new QWidget();
  auto *sv = new QVBoxLayout(simBody);
  sv->setContentsMargins(0, 0, 0, 0);
  sv->setSpacing(6);

  m_simStatusLabel = new QLabel(tr("● Stopped"), simBody);
  m_simStatusLabel->setStyleSheet(
      QString("color: %1;").arg(Theme::hex(Theme::kTextMuted)));
  sv->addWidget(m_simStatusLabel);

  // Non-modal notice, shown when Start is pressed on an untuned Custom
  // vehicle. Never blocks Start. The link jumps to the Autotune page.
  m_customWarnLabel = new QLabel(simBody);
  m_customWarnLabel->setObjectName(QStringLiteral("customVehicleWarning"));
  m_customWarnLabel->setWordWrap(true);
  m_customWarnLabel->setTextFormat(Qt::RichText);
  m_customWarnLabel->setStyleSheet(
      QString("color: %1; font-size: 11px;").arg(Theme::hex(Theme::kWarn)));
  m_customWarnLabel->setText(
      tr("Custom vehicle: no tuned gains. Stock gains may be unstable; run "
         "the autotuner first. <a href=\"#autotune\">Open Autotune</a>"));
  m_customWarnLabel->hide();
  connect(m_customWarnLabel, &QLabel::linkActivated, this,
          [this] { setMode(2); });
  sv->addWidget(m_customWarnLabel);

  auto *runRow = new QGridLayout();
  runRow->setContentsMargins(0, 0, 0, 0);
  QList<QWidget *> runWidgets;
  m_simStartBtn = new ui::SuccessButton(tr("Start"), simBody);
  m_simStartBtn->setToolTip(tr("Boot firmware (first time) + start the sim"));
  connect(m_simStartBtn, &QPushButton::clicked, this,
          &SimulatorWidget::startInAppSim);
  m_simStopBtn = new ui::DangerButton(tr("Stop"), simBody);
  m_simStopBtn->setEnabled(false);
  m_simStopBtn->setToolTip(tr("Stop the sim worker (firmware stays alive)"));
  connect(m_simStopBtn, &QPushButton::clicked, this,
          &SimulatorWidget::stopInAppSim);
  m_simResetBtn = new ui::GhostButton(tr("Reset"), simBody);
  m_simResetBtn->setEnabled(false);
  m_simResetBtn->setToolTip(tr("Reset the airframe to the spawn pose"));
  connect(m_simResetBtn, &QPushButton::clicked, this, [this] {
    sendSpawnReset();
  });
  m_simAttachBtn = new ui::GhostButton(tr("Attach Ext"), simBody);
  m_simAttachBtn->setToolTip(
      tr("Render an EXTERNAL pose stream "
         "(/tmp/vsim_pose) over the loaded world — e.g. a headless "
         "vayu_sitl_rtos "
         "run. No engine is started here; this view just mirrors it."));
  connect(m_simAttachBtn, &QPushButton::clicked, this,
          &SimulatorWidget::attachExternalSim);
#ifdef _WIN32
  // The external engine binary and its pose FIFO are POSIX; the in-app
  // simulator above is the Windows path.
  m_simAttachBtn->setEnabled(false);
  m_simAttachBtn->setToolTip(
      tr("Attaching to an external sim needs a POSIX host (its pose stream "
         "is a FIFO). Use Start to run the simulator in-app."));
#endif
  runWidgets << m_simStartBtn;
  runWidgets << m_simStopBtn;
  runWidgets << m_simResetBtn;
  runWidgets << m_simAttachBtn;
  m_fpvCheck = new QCheckBox(tr("FPV cam"), simBody);
  m_fpvCheck->setToolTip(
      tr("Onboard first-person camera that rides the drone "
         "(looks forward). Off = 3rd-person orbit. "
         "Available while the sim is running."));
  m_fpvCheck->setEnabled(false); // enabled on Start (locks to the drone)
  connect(m_fpvCheck, &QCheckBox::toggled, this, [this](bool on) {
    if (m_renderer)
      m_renderer->setFpv(on);
  });
  runWidgets << m_fpvCheck;

  // PiP visibility toggles (mockup Down Cam / Horizon chips).
  auto *downChk = new QCheckBox(tr("Down Cam"), simBody);
  downChk->setChecked(true);
  downChk->setToolTip(tr("Show the bird's-eye down-camera PiP"));
  connect(downChk, &QCheckBox::toggled, this, [this](bool on) {
    if (m_downPip)
      m_downPip->setVisible(on);
    syncPipRail();
  });
  runWidgets << downChk;

  auto *horizonChk = new QCheckBox(tr("Horizon"), simBody);
  horizonChk->setChecked(true);
  horizonChk->setToolTip(tr("Show the artificial-horizon PiP"));
  connect(horizonChk, &QCheckBox::toggled, this, [this](bool on) {
    if (m_horizonPip)
      m_horizonPip->setVisible(on);
    syncPipRail();
  });
  runWidgets << horizonChk;

  auto *contourChk = new QCheckBox(tr("Contours"), simBody);
  contourChk->setChecked(true);
  contourChk->setToolTip(tr("Show the top-down contour minimap "
                            "(procedural terrain height)."));
  connect(contourChk, &QCheckBox::toggled, this, [this](bool on) {
    if (m_minimapPip)
      m_minimapPip->setVisible(on);
    syncPipRail();
  });
  runWidgets << contourChk;

#ifdef VAYU_SIM_GRASS
  auto *grassChk = new QCheckBox(tr("Grass"), simBody);
  grassChk->setChecked(false); // off by default
  grassChk->setToolTip(tr("Render instanced grass + flowers on procedural "
                          "terrain."));
  connect(grassChk, &QCheckBox::toggled, this, [this](bool on) {
    m_grassEnabled = on;
    // Drive both grass paths: CPU flora visibility and the GPU-grass pass.
    if (m_renderer) {
      m_renderer->setFloraVisible(on);
      if (m_useGpuGrass)
        m_renderer->setGpuGrassActive(on);
    }
    if (m_downRenderer) {
      m_downRenderer->setFloraVisible(on);
      if (m_useGpuGrass)
        m_downRenderer->setGpuGrassActive(on);
    }
  });
  runWidgets << grassChk;
  // Apply the unchecked default to the renderers now (an unchecked box emits
  // no toggled signal at startup).
  if (m_renderer)
    m_renderer->setFloraVisible(false);
  if (m_downRenderer)
    m_downRenderer->setFloraVisible(false);
#endif // VAYU_SIM_GRASS

  m_propAudioChk = new QCheckBox(tr("Prop audio"), simBody);
  m_propAudioChk->setToolTip(tr("Propeller sound synthesized from motor rpm "
                                "(pitch + loudness rise with throttle)."));
  {
    QSettings st;
    m_propAudioChk->setChecked(st.value(kAudioKey, false).toBool());
    m_propAudio.setEnabled(m_propAudioChk->isChecked());
  }
  connect(m_propAudioChk, &QCheckBox::toggled, this, [this](bool on) {
    QSettings().setValue(kAudioKey, on);
    m_propAudio.setEnabled(on);
  });
  runWidgets << m_propAudioChk;
  sv->addLayout(runRow);

  // -- Noise seed: Start / Reset send it with the reset so the sensor-noise
  // stream is reproducible (0 in the engine would mean free-running). --
  {
    QSettings st;
    auto *seedRow = new QHBoxLayout();
    auto *seedLbl = new QLabel(tr("Noise seed"), simBody);
    auto *seedSpin = new QSpinBox(simBody);
    seedSpin->setRange(1, INT_MAX);
    seedSpin->setValue(
        std::max(1, st.value(kSeedKey, kDefaultResetSeed).toInt()));
    seedSpin->setToolTip(
        tr("Deterministic sensor-noise seed sent with every Start/Reset: the "
           "same seed reproduces the same noise. Applies on the next Start."));
    auto *seedRnd = new QCheckBox(tr("Random each Start"), simBody);
    seedRnd->setChecked(st.value(kSeedRandomKey, false).toBool());
    seedSpin->setEnabled(!seedRnd->isChecked());
    seedRnd->setToolTip(tr("Pick a fresh seed on every Start (logged); Reset "
                           "then reuses that run's seed."));
    connect(seedSpin, qOverload<int>(&QSpinBox::valueChanged), this,
            [](int v) { QSettings().setValue(kSeedKey, v); });
    connect(seedRnd, &QCheckBox::toggled, this, [seedSpin](bool on) {
      QSettings().setValue(kSeedRandomKey, on);
      seedSpin->setEnabled(!on);
    });
    seedRow->addWidget(seedLbl);
    seedRow->addWidget(seedSpin);
    seedRow->addWidget(seedRnd);
    seedRow->addStretch();
    sv->addLayout(seedRow);
  }

  // -- Loop rates (IMU/firmware, physics substeps, render) --
  {
    QSettings st;
    auto *rg = new QGridLayout();
    rg->setHorizontalSpacing(6);
    auto mkCombo = [&](const QList<int> &opts, int def, const char *key,
                       const QString &tip) {
      auto *c = new QComboBox(simBody);
      for (int v : opts)
        c->addItem(QString::number(v) + " Hz", v);
      const int cur = st.value(key, def).toInt();
      int idx = c->findData(cur);
      if (idx < 0) {
        c->addItem(QString::number(cur) + " Hz", cur);
        idx = c->count() - 1;
      }
      c->setCurrentIndex(idx);
      c->setToolTip(tip);
      connect(c, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
              [this, c, key] {
                QSettings().setValue(key, c->currentData().toInt());
                pushRatesToSim();
              });
      return c;
    };
    auto *imuC =
        mkCombo({200, 500, 1000, 2000}, kDefImuHz, kImuHzKey,
                tr("IMU emit + firmware inner-loop rate. Match your "
                   "real hardware (e.g. 1 kHz) so PID tuning transfers."));
    auto *physC =
        mkCombo({1000, 2000, 4000, 8000, 10000}, kDefPhysHz, kPhysHzKey,
                tr("RK4 physics rate. Run as substeps per IMU sample "
                   "(snapped to a multiple of the IMU rate): higher = "
                   "finer integration / less collision tunnelling, same "
                   "sample rate."));
    auto *poseC = mkCombo({30, 60, 120}, kDefPoseHz, kPoseHzKey,
                          tr("Pose / 3D-render update rate."));
    rg->addWidget(new QLabel(tr("IMU/loop:")), 0, 0);
    rg->addWidget(imuC, 0, 1);
    rg->addWidget(new QLabel(tr("Physics:")), 0, 2);
    rg->addWidget(physC, 0, 3);
    rg->addWidget(new QLabel(tr("Render:")), 1, 0);
    rg->addWidget(poseC, 1, 1);
    sv->addLayout(rg);
  }

  m_simPoseLabel = new QLabel(tr("pose: -"), simBody);
  m_simPoseLabel->setStyleSheet(QString("color: %1; font-family: monospace;")
                                    .arg(Theme::hex(Theme::kAccent)));
  sv->addWidget(m_simPoseLabel);

  // RC transmitter (USB joystick) → firmware RC. Must be set before the
  // first Start (firmware reads the RC path once at boot).
  {
    QSettings st;
    auto *rcRow = new QGridLayout();
    rcRow->setContentsMargins(0, 0, 0, 0);
    m_rcEnable = new QCheckBox(tr("RC transmitter"), simBody);
    m_rcEnable->setChecked(st.value(kRcEnableKey, true).toBool());
    m_rcEnable->setToolTip(tr("Feed a USB RC transmitter (joystick) into "
                              "the sim. Enable before the first Start."));
    connect(m_rcEnable, &QCheckBox::toggled, this, [this](bool on) {
      QSettings().setValue(kRcEnableKey, on);
      if (m_rc)
        m_rc->setEnabled(on); // live, no restart
    });

    // RC input source: a USB joystick (axes → channels) or a UART that
    // streams CSV microsecond frames straight off a serial port.
    m_rcSource = new QComboBox(simBody);
    m_rcSource->addItem(tr("USB joystick"), RcBridge::Joystick);
    m_rcSource->addItem(tr("UART (CSV)"), RcBridge::Uart);
    m_rcSource->setCurrentIndex(
        st.value(kRcSourceKey, RcBridge::Joystick).toInt() == RcBridge::Uart
            ? 1
            : 0);
    m_rcSource->setToolTip(
        tr("RC input source. USB joystick maps axes to "
           "channels; UART reads CSV µs frames directly."));
    connect(m_rcSource, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this] { applyRcSource(); });

    // Editable combo: type a device path, or drop down to pick an enumerated
    // serial port. Items + current text filled by applyRcSource().
    m_rcPath = new QComboBox(simBody);
    m_rcPath->setEditable(true);
    m_rcPath->setInsertPolicy(QComboBox::NoInsert);
    m_rcPath->setSizeAdjustPolicy(
        QComboBox::AdjustToMinimumContentsLengthWithIcon);
    // Each enumerated item shows the friendly description ("ttyUSB0 — FT232
    // USB UART") but carries the bare /dev path in itemData. Picking one
    // writes that path into the editable field — what the bridge opens.
    connect(m_rcPath, QOverload<int>::of(&QComboBox::activated), this,
            [this](int i) {
              const QVariant d = m_rcPath->itemData(i);
              if (d.isValid())
                m_rcPath->setEditText(d.toString());
              commitRcPath();
            });
    connect(m_rcPath->lineEdit(), &QLineEdit::editingFinished, this,
            [this] { commitRcPath(); });

    m_rcBaud = new QComboBox(simBody);
    for (int b : {9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600})
      m_rcBaud->addItem(QString::number(b), b);
    {
      const int idx =
          m_rcBaud->findData(st.value(kRcBaudKey, 115200).toInt());
      m_rcBaud->setCurrentIndex(idx >= 0 ? idx : 4);
    }
    m_rcBaud->setToolTip(tr("UART baud (UART source only)."));
    connect(m_rcBaud, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this] {
              const int b = m_rcBaud->currentData().toInt();
              QSettings().setValue(kRcBaudKey, b);
              if (m_rc)
                m_rc->setUartBaud(b);
            });

    // Explicit Connect for the UART source. The sim does NOT open the serial
    // device until this is pressed, so it can't fight the board telemetry for
    // the port. Disconnected by default; only meaningful for the UART source.
    m_rcConnect = new QPushButton(tr("Connect"), simBody);
    m_rcConnect->setObjectName("ToggleButton");
    m_rcConnect->setCheckable(true);
    m_rcConnect->setToolTip(
        tr("Connect/disconnect the RC UART device. The sim "
           "only grabs the serial port while connected."));
    connect(m_rcConnect, &QPushButton::clicked, this,
            [this] { setRcUartConnected(!m_rcUartConnected); });

    // Acro (rate) flight-mode toggle. This IS the simulated RC ch6 switch: it
    // drives RcBridge ch6, the firmware reads ch6 and resolves the mode (no GCS
    // override, so the switch is authoritative — matching real RC). Off = angle
    // / stabilize (bank-angle limited); on = acro (body-rate, flips allowed).
    m_acroChk = new QCheckBox(tr("Acro (ch6)"), simBody);
    m_acroChk->setToolTip(
        tr("Acro / rate mode on RC channel 6: sticks command "
           "body rate directly, no bank-angle limit. This is "
           "the ch6 switch — the firmware follows it."));
    const bool acroOn =
        st.value(QStringLiteral("sim/rcAcro"), false).toBool();
    m_acroChk->setChecked(acroOn);
    if (m_rc)
      m_rc->setAcro(acroOn);
    // Release any stale GCS mode override so the ch6 switch is in charge.
    // Mode 2 is the firmware's "release back to the RC switch" value.
    SitlModule::instance().send(CommandCodec::encodeSetFlightMode(2));
    connect(m_acroChk, &QCheckBox::toggled, this, [this](bool on) {
      QSettings().setValue(QStringLiteral("sim/rcAcro"), on);
      if (m_rc)
        m_rc->setAcro(on); // toggles ch6 -> firmware switches mode
    });
    // Optional wrapped layout so the panel can be narrower; layout only.
    auto *compactChk = new QCheckBox(tr("Compact controls"), simBody);
    compactChk->setToolTip(tr("Wrap the run and RC control rows over several "
                              "lines so the side panel can be narrower."));
    sv->addWidget(compactChk);
    sv->addLayout(rcRow);
    const QList<QWidget *> rcWidgets{m_rcEnable, m_rcSource, m_rcPath,
                                     m_rcBaud,   m_rcConnect, m_acroChk};
    connect(compactChk, &QCheckBox::toggled, this,
            [this, compactChk, rcRow, rcWidgets, runRow, runWidgets](bool on) {
              QSettings().setValue(kRcCompactKey, on);
              ui::layoutWrappedRow(rcRow, rcWidgets, on, m_rcPath);
              ui::layoutWrappedRow(runRow, runWidgets, on, nullptr, 3, 3);
              // Invalidate every enclosing layout so the panel minimum
              // (MinWidthScrollArea) follows live.
              for (QWidget *w = compactChk; w; w = w->parentWidget()) {
                if (w->layout())
                  w->layout()->invalidate();
                w->updateGeometry();
              }
            });
    const bool compactOn = st.value(kRcCompactKey, false).toBool();
    ui::layoutWrappedRow(rcRow, rcWidgets, compactOn, m_rcPath);
    ui::layoutWrappedRow(runRow, runWidgets, compactOn, nullptr, 3, 3);
    compactChk->setChecked(compactOn);

    m_flightModeLabel = new QLabel(tr("mode: —"), simBody);
    m_flightModeLabel->setStyleSheet(
        QString("color: %1; font-family: monospace; font-size: 11px;")
            .arg(Theme::hex(Theme::kTextMuted)));
    sv->addWidget(m_flightModeLabel);

    m_rcReadout = new QLabel(tr("RC: —"), simBody);
    m_rcReadout->setStyleSheet(
        QString("color: %1; font-family: monospace; font-size: 11px;")
            .arg(Theme::hex(Theme::kTextMuted)));
    sv->addWidget(m_rcReadout);

    // Live per-axis readout — move a control and see which axis changes.
    m_rcAxesLabel = new QLabel(tr("Axes: —"), simBody);
    m_rcAxesLabel->setStyleSheet(
        QString("color: %1; font-family: monospace; font-size: 11px;")
            .arg(Theme::hex(Theme::kTextDim)));
    sv->addWidget(m_rcAxesLabel);

    // Channel → axis mapping (+ invert), one row per function.
    auto *mapGrid = new QGridLayout();
    mapGrid->setHorizontalSpacing(6);
    mapGrid->setVerticalSpacing(2);
    for (int f = 0; f < 5; ++f) {
      const int savedSrc =
          st.value(QString(kRcMapAxisKey) + QString::number(f),
                   kRcDefaultAxis[f])
              .toInt();
      const bool savedInv =
          st.value(QString(kRcMapInvKey) + QString::number(f), false)
              .toBool();
      mapGrid->addWidget(new QLabel(tr(kRcFuncName[f]), simBody), f, 0);
      auto *combo = new QComboBox(simBody);
      for (int a = 0; a < 6; ++a)
        combo->addItem(tr("Axis %1").arg(a), a);
      for (int b = 0; b < 3; ++b)
        combo->addItem(tr("Button %1").arg(b), RcBridge::kButtonBase + b);
      const int idx = combo->findData(savedSrc);
      combo->setCurrentIndex(idx >= 0 ? idx : 0);
      mapGrid->addWidget(combo, f, 1);
      auto *inv = new QCheckBox(tr("invert"), simBody);
      inv->setChecked(savedInv);
      mapGrid->addWidget(inv, f, 2);
      m_rcAxisCombo[f] = combo;
      m_rcInvert[f] = inv;
      connect(combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
              this, [this, f] { pushRcMapping(f); });
      connect(inv, &QCheckBox::toggled, this,
              [this, f] { pushRcMapping(f); });
    }
    sv->addLayout(mapGrid);

    // Software arm — for TXs with no usable arm switch. Overrides the
    // Arm channel: checked = armed (2000), unchecked = use the mapping.
    m_swArm = new QCheckBox(tr("Software arm (override)"), simBody);
    m_swArm->setToolTip(tr("Force the arm channel high. Use with throttle "
                           "down to ARM when your TX has no arm switch."));
    connect(m_swArm, &QCheckBox::toggled, this, [this](bool on) {
      if (m_rc)
        m_rc->setArmOverride(on ? 1 : -1);
    });
    sv->addWidget(m_swArm);

    // Now that the device field, baud, and mapping widgets exist, apply the
    // saved source: fills the path field and enables/disables baud+mapping.
    applyRcSource();
  }

  auto *repoRow = new QHBoxLayout();
  repoRow->addWidget(new QLabel(tr("Repo root:"), simBody));
  m_repoRootEdit = new QLineEdit(m_repoRoot, simBody);
  m_repoRootEdit->setToolTip(
      tr("Path to the vayu repo. Used to find the autotune backend "
         "(build_sitl_rtos). Logs go to the Log dir below."));
  connect(m_repoRootEdit, &QLineEdit::editingFinished, this, [this] {
    m_repoRoot = m_repoRootEdit->text();
    QSettings().setValue(kRepoRootSettingKey, m_repoRoot);
  });
  repoRow->addWidget(m_repoRootEdit, 1);
  sv->addLayout(repoRow);

  auto *logRow = new QHBoxLayout();
  logRow->addWidget(new QLabel(tr("Log dir:"), simBody));
  m_logDirEdit = new QLineEdit(m_logDir, simBody);
  m_logDirEdit->setToolTip(
      tr("Directory the per-run UART2 byte log is written into"));
  connect(m_logDirEdit, &QLineEdit::editingFinished, this, [this] {
    m_logDir = m_logDirEdit->text();
    QSettings().setValue(kLogDirSettingKey, m_logDir);
  });
  logRow->addWidget(m_logDirEdit, 1);
  auto *openBtn = new ui::GhostButton(tr("Open"), simBody);
  openBtn->setToolTip(tr("Open the log directory in your file manager"));
  connect(openBtn, &QPushButton::clicked, this, [this] {
    QDesktopServices::openUrl(QUrl::fromLocalFile(m_logDir));
  });
  logRow->addWidget(openBtn);
  sv->addLayout(logRow);

  m_logPathLabel = new ui::ElidedLabel(simBody);
  m_logPathLabel->setFullText(tr("Current log: (none)"));
  m_logPathLabel->setStyleSheet(
      QString("color: %1; font-family: monospace; font-size: 11px;")
          .arg(Theme::hex(Theme::kTextMuted)));
  sv->addWidget(m_logPathLabel);

  m_log = new QPlainTextEdit(simBody);
  m_log->setReadOnly(true);
  m_log->setMaximumBlockCount(2000);
  m_log->setMinimumHeight(150);
  sv->addWidget(m_log);

  simSec->setContentWidget(simBody);
  auto *hostLay = new QVBoxLayout(host);
  hostLay->setContentsMargins(0, 0, 0, 0);
  hostLay->addWidget(simSec);
}
