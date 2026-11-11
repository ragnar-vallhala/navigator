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
#include "SimAutotunePanel.h"

#include "SimPanelsCommon.h"

SimAutotunePanel::SimAutotunePanel(SimulatorWidget &sim, QWidget *parent)
    : ui::MinWidthScrollArea(parent) {
  auto *page = new QWidget();
  sim.buildAutotunePage(page);
  setWidgetResizable(true);
  setWidget(page);
  setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
}

void SimulatorWidget::buildAutotunePage(QWidget *page) {
  auto *v = new QVBoxLayout(page);
  v->setContentsMargins(10, 10, 10, 10);
  v->setSpacing(8);

  auto *title = new QLabel(tr("PID Autotune"), page);
  title->setStyleSheet(QString("color:%1; font-size:15px; font-weight:bold;")
                           .arg(Theme::hex(Theme::kAccent)));
  v->addWidget(title);

  auto *info = new QLabel(
      tr("Searches the inner rate + outer angle gains for the CURRENT vehicle "
         "geometry, in an isolated headless test-rig sim (tools/autotune). The "
         "live sim keeps running; tuned gains persist to the shared store and "
         "load on the next sim start."),
      page);
  info->setWordWrap(true);
  info->setStyleSheet(
      QString("color:%1; font-size:11px;").arg(Theme::hex(Theme::kTextMuted)));
  v->addWidget(info);

  auto *form = new QGridLayout();
  int r = 0;
  form->addWidget(new QLabel(tr("Optimizer:"), page), r, 0);
  m_tuneOptimizer = new QComboBox(page);
  m_tuneOptimizer->addItems({"hybrid", "portfolio", "structured", "spsa",
                             "nelder-mead", "coordinate", "fdgd", "random"});
  m_tuneOptimizer->setToolTip(tr("hybrid/portfolio explore then refine (best); "
                                 "structured = manual-style one-gain-at-a-time "
                                 "sweep (P→D→I→outer); spsa is fast."));
  form->addWidget(m_tuneOptimizer, r++, 1);

  form->addWidget(new QLabel(tr("Budget (rollouts):"), page), r, 0);
  m_tuneBudget = new QSpinBox(page);
  m_tuneBudget->setRange(5, 200);
  m_tuneBudget->setValue(30);
  m_tuneBudget->setToolTip(
      tr("More rollouts = better tune, slower (~6 s each)."));
  form->addWidget(m_tuneBudget, r++, 1);

  form->addWidget(new QLabel(tr("Excitation (µs):"), page), r, 0);
  m_tuneStep = new QSpinBox(page);
  m_tuneStep->setRange(1550, 1950);
  m_tuneStep->setSingleStep(25);
  m_tuneStep->setValue(1800);
  m_tuneStep->setToolTip(
      tr("Doublet stick amplitude (1500=center, 2000=full; "
         "~21° at 1800). Lower it (e.g. 1650) to soften the "
         "excitation on twitchy airframes so a marginal seed "
         "doesn't flip."));
  form->addWidget(m_tuneStep, r++, 1);

  // Excitation waveform: step doublet vs swept-sine chirp (conditional freq row).
  form->addWidget(new QLabel(tr("Waveform:"), page), r, 0);
  m_tuneExcitation = new QComboBox(page);
  m_tuneExcitation->addItem(tr("Step (doublet)"));
  m_tuneExcitation->addItem(tr("Chirp (freq sweep)"));
  m_tuneExcitation->setToolTip(tr("Step = one doublet per axis; Chirp = swept "
                                  "sine f0→f1 over the hold, probing a band."));
  form->addWidget(m_tuneExcitation, r++, 1);

  auto *chirpLbl = new QLabel(tr("Chirp f0–f1 (Hz):"), page);
  form->addWidget(chirpLbl, r, 0);
  m_chirpRow = new QWidget(page);
  auto *chirpH = new QHBoxLayout(m_chirpRow);
  chirpH->setContentsMargins(0, 0, 0, 0);
  m_tuneChirpF0 = new QDoubleSpinBox(m_chirpRow);
  m_tuneChirpF0->setRange(0.1, 50.0);
  m_tuneChirpF0->setDecimals(1);
  m_tuneChirpF0->setValue(1.0);
  m_tuneChirpF1 = new QDoubleSpinBox(m_chirpRow);
  m_tuneChirpF1->setRange(0.1, 50.0);
  m_tuneChirpF1->setDecimals(1);
  m_tuneChirpF1->setValue(12.0);
  chirpH->addWidget(m_tuneChirpF0);
  chirpH->addWidget(new QLabel(QStringLiteral("→"), m_chirpRow));
  chirpH->addWidget(m_tuneChirpF1);
  form->addWidget(m_chirpRow, r++, 1);
  auto syncChirpVis = [this, chirpLbl] {
    const bool chirp = m_tuneExcitation->currentIndex() == 1;
    if (m_chirpRow)
      m_chirpRow->setVisible(chirp);
    chirpLbl->setVisible(chirp);
  };
  connect(m_tuneExcitation, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, [syncChirpVis](int) { syncChirpVis(); });
  syncChirpVis();

  form->addWidget(new QLabel(tr("Repeats / eval:"), page), r, 0);
  m_tuneRepeats = new QSpinBox(page);
  m_tuneRepeats->setRange(1, 5);
  m_tuneRepeats->setValue(2);
  m_tuneRepeats->setToolTip(
      tr("Rollouts averaged per evaluation (--repeats). "
         "Higher = less noisy cost, proportionally slower."));
  form->addWidget(m_tuneRepeats, r++, 1);

  form->addWidget(new QLabel(tr("Rig tether:"), page), r, 0);
  m_tuneTether = new QDoubleSpinBox(page);
  m_tuneTether->setRange(0.0, 100.0);
  m_tuneTether->setSingleStep(5.0);
  m_tuneTether->setValue(30.0);
  m_tuneTether->setToolTip(
      tr("Soft-rig spring stiffness [1/s²] (--rig-tether). The "
         "body translates during the doublet so the accel sees "
         "free-flight thrust-tilt (estimator-aware). 0 = legacy "
         "hard pin (no translation)."));
  form->addWidget(m_tuneTether, r++, 1);

  m_tuneYaw = new QCheckBox(tr("Tune yaw too"), page);
  m_tuneYaw->setToolTip(
      tr("Also tune the yaw RATE loop (--yaw): yaw_rate kp/ki/kd "
         "+ gyro LPF, excited by a yaw-rate doublet. Yaw is "
         "rate-controlled, so there is no yaw angle gain to tune."));
  form->addWidget(m_tuneYaw, r++, 1);

  m_tuneSysId = new QCheckBox(tr("System ID (analytic design)"), page);
  m_tuneSysId->setToolTip(tr(
      "Instead of an optimizer search, fly ONE broadband chirp on the fast "
      "backend, fit a physical plant model (omega/u = K/(s(tau·s+1))) per "
      "axis, "
      "and compute the gains analytically by loop-shaping (PD zero on the "
      "actuator pole; crossover at a fraction of the actuator bandwidth).\n"
      "Why: the search minimizes a tracking cost that the OUTER angle loop "
      "dominates, so it never values a stiff inner rate loop and settles on a "
      "near-zero rate_kp. System-ID has no cost to game — the rate loop gets a "
      "real gain by construction, in ONE rollout instead of a full sweep.\n"
      "Implies the fast backend. Budget/optimizer/cost-function are ignored; "
      "the "
      "chirp band + rig come from the Excitation settings. The result is "
      "applied "
      "and a verification doublet is flown to confirm it tracks."));
  form->addWidget(m_tuneSysId, r++, 1);

  form->addWidget(new QLabel(tr("Design bandwidth:"), page), r, 0);
  m_tuneSysIdBw = new QDoubleSpinBox(page);
  m_tuneSysIdBw->setRange(0.10, 0.50);
  m_tuneSysIdBw->setSingleStep(0.05);
  m_tuneSysIdBw->setDecimals(2);
  m_tuneSysIdBw->setValue(0.33);
  m_tuneSysIdBw->setToolTip(tr(
      "System-ID only: the rate-loop crossover as a fraction of the identified "
      "actuator bandwidth (1/tau). This is the aggressiveness lever — it sets "
      "rate_kp = wc/K and angle_kp = 0.25·wc.\n"
      "• Higher (→0.5): stiffer, faster disturbance rejection, but closer to "
      "the "
      "actuator pole — risks buzz; rate_kp is hard-capped at the buzz knee.\n"
      "• Lower (→0.1): gentler, more phase margin, softer response.\n"
      "0.33 (crossover at ~1/3 of the actuator BW) is a balanced default."));
  form->addWidget(m_tuneSysIdBw, r++, 1);

  form->addWidget(new QLabel(tr("Cost function:"), page), r, 0);
  m_tuneCostFn = new QComboBox(page);
  m_tuneCostFn->addItem(tr("Angle tracking (realtime cost)"));
  m_tuneCostFn->addItem(tr("Angle + rate tracking"));
  m_tuneCostFn->setToolTip(tr(
      "Cost used by the Fast backend.\n"
      "• Angle tracking: the exact realtime cost — angle-loop IAE + overshoot "
      "+ "
      "chatter, with the divergence penalty. Scores the OUTER loop.\n"
      "• Angle + rate tracking: adds a roll/pitch RATE-loop tracking term so "
      "the "
      "search also values a tracking, non-buzzy inner loop (the analysis's "
      "Part-D rate term). Note: on the current sim plant the rate loop buzzes "
      "before it stiffens, so this mainly penalises buzzy high-rate_kp tunes "
      "rather than pushing rate_kp up.\n"
      "Applies to the Fast backend only; the realtime tuner always uses angle "
      "tracking."));
  form->addWidget(m_tuneCostFn, r++, 1);

  m_tuneCompare = new QCheckBox(tr("Compare all optimizers"), page);
  m_tuneCompare->setToolTip(
      tr("Run every optimizer on equal budgets and keep the "
         "best (--compare). Much slower (N× the budget)."));
  form->addWidget(m_tuneCompare, r++, 1);
  m_tuneBuzz = new QCheckBox(tr("Throttle buzz check"), page);
  m_tuneBuzz->setChecked(true);
  m_tuneBuzz->setToolTip(
      tr("Penalize gains that limit-cycle at hover+ throttle "
         "(off = --no-buzz-check). The rig can't see this; the "
         "check sweeps throttle and scores rate-output chatter. "
         "Adds ~3 s per eval but stops a buzzy tune from winning."));
  form->addWidget(m_tuneBuzz, r++, 1);
  m_tuneValidate = new QCheckBox(tr("Free-flight validation"), page);
  m_tuneValidate->setChecked(true);
  m_tuneValidate->setToolTip(
      tr("After the search, lift off and step-recover each "
         "axis to confirm the winner flies (off = "
         "--no-validate)."));
  form->addWidget(m_tuneValidate, r++, 1);
  // AT-1: no auto-apply. The search only proposes; committing to firmware is a
  // separate explicit click (the Apply button below).
  m_tunePlot =
      new QCheckBox(tr("Live dashboard (attitude + cost window)"), page);
  m_tunePlot->setChecked(true);
  m_tunePlot->setToolTip(tr("Opens the autotuner's live plot: the drone's "
                            "attitude response to each excitation + cost "
                            "convergence, gains and per-axis traces."));
  form->addWidget(m_tunePlot, r++, 1);
  m_tuneVerbose = new QCheckBox(tr("Verbose log (print every eval)"), page);
  m_tuneVerbose->setToolTip(
      tr("Print each evaluation's gains + cost to the log "
         "(--verbose)."));
  form->addWidget(m_tuneVerbose, r++, 1);
  v->addLayout(form);

  // --- Advanced: reproducibility seeds (rarely changed) -----------------
  auto *advGroup = new QGroupBox(tr("Advanced (reproducibility)"), page);
  auto *advForm = new QGridLayout(advGroup);
  int ar = 0;
  advForm->addWidget(new QLabel(tr("Optimizer seed:"), page), ar, 0);
  m_tuneSeed = new QSpinBox(page);
  m_tuneSeed->setRange(0, 1000000);
  m_tuneSeed->setValue(1);
  m_tuneSeed->setToolTip(tr("RNG seed for the optimizer's random choices "
                            "(--seed). Same seed = same search trajectory."));
  advForm->addWidget(m_tuneSeed, ar++, 1);
  advForm->addWidget(new QLabel(tr("Sim noise seed:"), page), ar, 0);
  m_tuneSimSeed = new QSpinBox(page);
  m_tuneSimSeed->setRange(0, 2000000000);
  m_tuneSimSeed->setValue(0xC0FFEE); // DEFAULT_SIM_SEED in autotune.py
  m_tuneSimSeed->setToolTip(
      tr("Base seed for the sensor-noise reset (--sim-seed); "
         "repeat i uses seed+i. Makes eval(x) reproducible. "
         "0 = free-running noise (legacy)."));
  advForm->addWidget(m_tuneSimSeed, ar++, 1);
  v->addWidget(advGroup);

  // Persist every autotune option so a configured run is remembered next time
  // the GCS starts (the controls otherwise reset to defaults each session).
  {
    QSettings st;
    st.beginGroup(QStringLiteral("autotune"));
    m_tuneOptimizer->setCurrentText(
        st.value(QStringLiteral("optimizer"), m_tuneOptimizer->currentText())
            .toString());
    m_tuneBudget->setValue(
        st.value(QStringLiteral("budget"), m_tuneBudget->value()).toInt());
    m_tuneStep->setValue(
        st.value(QStringLiteral("stepUs"), m_tuneStep->value()).toInt());
    m_tuneRepeats->setValue(
        st.value(QStringLiteral("repeats"), m_tuneRepeats->value()).toInt());
    m_tuneTether->setValue(
        st.value(QStringLiteral("tether"), m_tuneTether->value()).toDouble());
    m_tuneSeed->setValue(
        st.value(QStringLiteral("seed"), m_tuneSeed->value()).toInt());
    m_tuneSimSeed->setValue(
        st.value(QStringLiteral("simSeed"), m_tuneSimSeed->value()).toInt());
    m_tuneYaw->setChecked(
        st.value(QStringLiteral("yaw"), m_tuneYaw->isChecked()).toBool());
    m_tuneSysId->setChecked(
        st.value(QStringLiteral("sysid"), m_tuneSysId->isChecked()).toBool());
    m_tuneSysIdBw->setValue(
        st.value(QStringLiteral("sysidBw"), m_tuneSysIdBw->value()).toDouble());
    m_tuneCostFn->setCurrentIndex(
        st.value(QStringLiteral("costFn"), m_tuneCostFn->currentIndex())
            .toInt());
    m_tuneCompare->setChecked(
        st.value(QStringLiteral("compare"), m_tuneCompare->isChecked())
            .toBool());
    m_tuneBuzz->setChecked(
        st.value(QStringLiteral("buzz"), m_tuneBuzz->isChecked()).toBool());
    m_tuneValidate->setChecked(
        st.value(QStringLiteral("validate"), m_tuneValidate->isChecked())
            .toBool());
    m_tunePlot->setChecked(
        st.value(QStringLiteral("plot"), m_tunePlot->isChecked()).toBool());
    m_tuneVerbose->setChecked(
        st.value(QStringLiteral("verbose"), m_tuneVerbose->isChecked())
            .toBool());
    st.endGroup();
  }
  // Design-bandwidth knob is meaningful only for the System-ID path.
  m_tuneSysIdBw->setEnabled(m_tuneSysId->isChecked());
  auto saveTune = [](const QString &key, const QVariant &val) {
    QSettings s;
    s.setValue(QStringLiteral("autotune/") + key, val);
  };
  connect(m_tuneOptimizer, &QComboBox::currentTextChanged, this,
          [saveTune](const QString &t) {
            saveTune(QStringLiteral("optimizer"), t);
          });
  connect(m_tuneBudget, QOverload<int>::of(&QSpinBox::valueChanged), this,
          [saveTune](int v) { saveTune(QStringLiteral("budget"), v); });
  connect(m_tuneStep, QOverload<int>::of(&QSpinBox::valueChanged), this,
          [saveTune](int v) { saveTune(QStringLiteral("stepUs"), v); });
  connect(m_tuneRepeats, QOverload<int>::of(&QSpinBox::valueChanged), this,
          [saveTune](int v) { saveTune(QStringLiteral("repeats"), v); });
  connect(m_tuneTether, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this,
          [saveTune](double v) { saveTune(QStringLiteral("tether"), v); });
  connect(m_tuneSeed, QOverload<int>::of(&QSpinBox::valueChanged), this,
          [saveTune](int v) { saveTune(QStringLiteral("seed"), v); });
  connect(m_tuneSimSeed, QOverload<int>::of(&QSpinBox::valueChanged), this,
          [saveTune](int v) { saveTune(QStringLiteral("simSeed"), v); });
  connect(m_tuneYaw, &QCheckBox::toggled, this,
          [saveTune](bool v) { saveTune(QStringLiteral("yaw"), v); });
  connect(m_tuneSysId, &QCheckBox::toggled, this, [this, saveTune](bool v) {
    saveTune(QStringLiteral("sysid"), v);
    if (m_tuneSysIdBw)
      m_tuneSysIdBw->setEnabled(v);
  });
  connect(m_tuneSysIdBw, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this,
          [saveTune](double v) { saveTune(QStringLiteral("sysidBw"), v); });
  connect(m_tuneCostFn, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, [saveTune](int v) { saveTune(QStringLiteral("costFn"), v); });
  connect(m_tuneCompare, &QCheckBox::toggled, this,
          [saveTune](bool v) { saveTune(QStringLiteral("compare"), v); });
  connect(m_tuneBuzz, &QCheckBox::toggled, this,
          [saveTune](bool v) { saveTune(QStringLiteral("buzz"), v); });
  connect(m_tuneValidate, &QCheckBox::toggled, this,
          [saveTune](bool v) { saveTune(QStringLiteral("validate"), v); });
  connect(m_tunePlot, &QCheckBox::toggled, this,
          [saveTune](bool v) { saveTune(QStringLiteral("plot"), v); });
  connect(m_tuneVerbose, &QCheckBox::toggled, this,
          [saveTune](bool v) { saveTune(QStringLiteral("verbose"), v); });

  // --- Test-rig pose: pin the airframe and tilt it on the stand ---------
  auto *rigGroup = new QGroupBox(tr("Test-rig pose"), page);
  auto *rigV = new QVBoxLayout(rigGroup);
  m_rigEnable =
      new QCheckBox(tr("Rig mode (pin translation, free rotation)"), page);
  m_rigEnable->setToolTip(
      tr("Pins the airframe so you can tilt it on a stand. "
         "Disarm first — armed, the controller fights the pose."));
  rigV->addWidget(m_rigEnable);
  auto *rigGrid = new QGridLayout();
  auto mkSlider = [&](const QString &name, int row) {
    auto *s = new QSlider(Qt::Horizontal, page);
    s->setRange(-180, 180);
    s->setValue(0);
    s->setEnabled(false);
    rigGrid->addWidget(new QLabel(name, page), row, 0);
    rigGrid->addWidget(s, row, 1);
    return s;
  };
  m_rigRoll = mkSlider(tr("Roll°"), 0);
  m_rigPitch = mkSlider(tr("Pitch°"), 1);
  m_rigYaw = mkSlider(tr("Yaw°"), 2);
  rigV->addLayout(rigGrid);
  m_rigReadout = new QLabel(tr("rig: off"), page);
  m_rigReadout->setStyleSheet(
      QString("color:%1; font-family:monospace; font-size:11px;")
          .arg(Theme::hex(Theme::kTextMuted)));
  rigV->addWidget(m_rigReadout);
  v->addWidget(rigGroup);

  connect(m_rigEnable, &QCheckBox::toggled, this, [this](bool on) {
    if (m_rigRoll)
      m_rigRoll->setEnabled(on);
    if (m_rigPitch)
      m_rigPitch->setEnabled(on);
    if (m_rigYaw)
      m_rigYaw->setEnabled(on);
    if (m_sim)
      m_sim->sendTestRig(on);
    if (on) {
      applyRigPose();
    } else {
      if (m_sim)
        m_sim->sendReset(); // back to level, free flight
      if (m_rigReadout)
        m_rigReadout->setText(tr("rig: off"));
    }
  });
  // Apply the pose ONCE per change, not on every intermediate drag value: each
  // sendRigPose is a full CTL_RESET, so streaming them while dragging floods the
  // engine (reset spam), spikes the synthetic accel (gravity re-projected
  // at each tilt), and pegs the CPU. While dragging we only preview the numbers;
  // the reset fires on release (or immediately for a keyboard/click step).
  auto previewRig = [this] {
    if (m_rigReadout && m_rigRoll)
      m_rigReadout->setText(
          tr("rig: roll %1°  pitch %2°  yaw %3°  (release to apply)")
              .arg(m_rigRoll->value())
              .arg(m_rigPitch->value())
              .arg(m_rigYaw->value()));
  };
  for (QSlider *s : {m_rigRoll, m_rigPitch, m_rigYaw}) {
    connect(s, &QSlider::valueChanged, this, [this, s, previewRig] {
      if (s->isSliderDown())
        previewRig(); // mid-drag: preview only
      else
        applyRigPose(); // click / keyboard step: apply now
    });
    connect(s, &QSlider::sliderReleased, this, [this] { applyRigPose(); });
  }
  setRigControlsEnabled(m_sim != nullptr); // disabled until the sim runs

  // Embedded convergence chart — cost per evaluation + best-so-far.
  m_tuneChart = new TuneChart(page);
  v->addWidget(m_tuneChart);

  // Live step/chirp response of the roll axis (setpoint vs measured), updated
  // once per evaluation from the latest excitation window.
  v->addWidget(
      new QLabel(tr("Roll response — setpoint (dashed) vs measured"), page));
  m_tuneResponse = new ResponsePlot(page);
  v->addWidget(m_tuneResponse);

  auto *btnRow = new QHBoxLayout();
  m_tuneStart = new QPushButton(tr("Start Autotune"), page);
  m_tuneStop = new QPushButton(tr("Stop"), page);
  m_tuneStop->setEnabled(false);
  connect(m_tuneStart, &QPushButton::clicked, this,
          [this] { startAutotune(); });
  connect(m_tuneStop, &QPushButton::clicked, this, [this] { stopAutotune(); });
  btnRow->addWidget(m_tuneStart);
  btnRow->addWidget(m_tuneStop);
  btnRow->addStretch();
  v->addLayout(btnRow);

  m_tuneResult = new QLabel(tr("—"), page);
  m_tuneResult->setWordWrap(true);
  m_tuneResult->setStyleSheet(
      QString("color:%1; font-family:monospace; font-size:11px;")
          .arg(Theme::hex(Theme::kOk)));
  v->addWidget(m_tuneResult);

  // AT-1: Proposed Gains (the search's best) + an explicit Apply to firmware.
  m_tuneProposed = new QLabel(tr("Proposed gains: —"), page);
  m_tuneProposed->setWordWrap(true);
  m_tuneProposed->setStyleSheet(
      QStringLiteral("font-family:monospace; font-size:11px;"));
  v->addWidget(m_tuneProposed);

  // AT-2: live current-vs-best gains. `current` jumps as the optimizer explores;
  // `best` improves monotonically and locks in (turns green) on convergence.
  m_tuneGainsTable = new QTableWidget(0, 3, page);
  m_tuneGainsTable->setHorizontalHeaderLabels(
      {tr("Gain"), tr("Current"), tr("Best")});
  m_tuneGainsTable->verticalHeader()->setVisible(false);
  m_tuneGainsTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
  m_tuneGainsTable->setSelectionMode(QAbstractItemView::NoSelection);
  m_tuneGainsTable->setFocusPolicy(Qt::NoFocus);
  m_tuneGainsTable->horizontalHeader()->setStretchLastSection(true);
  m_tuneGainsTable->setMaximumHeight(220);
  v->addWidget(m_tuneGainsTable);

  m_tuneApplyBtn = new QPushButton(tr("Apply Gains"), page);
  m_tuneApplyBtn->setEnabled(false); // enabled once a search proposes gains
  m_tuneApplyBtn->setToolTip(tr(
      "Apply the proposed (best) gains. With a flight controller connected "
      "they go to the board via CMD_SET_PID; otherwise they apply to the "
      "in-app sim and persist to 0:pid.bin (reloaded on the next sim start). "
      "The autotuner never writes them on its own."));
  connect(m_tuneApplyBtn, &QPushButton::clicked, this,
          [this] { applyProposedGains(); });
  v->addWidget(m_tuneApplyBtn);

  m_tuneLog = new QPlainTextEdit(page);
  m_tuneLog->setReadOnly(true);
  m_tuneLog->setMaximumBlockCount(4000);
  m_tuneLog->setStyleSheet("font-family:monospace; font-size:10px;");
  v->addWidget(m_tuneLog, 1);
}
