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
#include "SimulatorWidget.h"
#include "SimPanelsCommon.h"
#include "SimAutotunePanel.h"
#include "SimControlsPanel.h"
#include "SimVehiclePanel.h"
#include "SimViewPanel.h"
#include "SimWorldPanel.h"


// Firmware pokes reached through the loaded module's SITL-only section (see
// vayu_sitl_abi.h): the mixer setter keeps the firmware roll/pitch/yaw->motor
// mix consistent with the vehicle geometry the sim uses, and the flight-mode
// pair drives the mode directly. mode arg: 0=stabilise/angle, 1=acro —
// matching flight_mode_t. Telemetry uses the same values, with source 1 ==
// GCS override.
//
// None of these exist on real hardware, where the GCS asks over NavLink; they
// are a SITL shortcut this file inherited from linking the firmware statically.

namespace {

// Send an airframe preset's gain set to the firmware as CMD_SET_PID, the frames
// the autotuner's Apply sends. Same command gate as the geometry above, so it is
// re-sent with it. Whether the engine persists them to 0:pid.bin is not
// verified (the private store stayed empty in a test), so do not rely on it.
void pushPresetGains(const QString &presetId) {
  const vsim::AirframePreset *p = vsim::findAirframePreset(presetId);
  if (!p)
    return; // a custom vehicle keeps whatever gains the firmware has
  for (const PidSetCmd &c : p->gains)
    SitlModule::instance().send(CommandCodec::encodeSetPid(
        c.controller, c.axis, c.kp, c.ki, c.kd, c.kff));
}

// The roll-mix signs this geometry produces (for logging): -sign(y) per motor.
// The firmware DEFAULT is {-1,-1,+1,+1}; a Y-mirrored airframe (e.g. an S500
// with M1/M2 on -y) needs {+1,+1,-1,-1} — the exact inverse — so flying the
// default mix on it inverts roll and topples. Surfacing this catches a mix that
// didn't get pushed to the firmware before arming.
QString rollMixString(const vsim::GeometryConfig &g) {
  QString s;
  for (int i = 0; i < 4; ++i)
    s += (g.motors[i].pos.y() >= 0.0f ? "-" : "+");
  return s;
}

QString defaultRepoRoot() {
  // 1) Explicit override always wins.
  QByteArray env = qgetenv("VAYU_REPO");
  if (!env.isEmpty())
    return QString::fromUtf8(env);
  // 2) Walk up from the executable to find the repo root, identified by its
  //    marker dirs (tools/ + navlink/). The GCS binary is built somewhere
  //    under the repo, so this resolves on any checkout path / machine
  //    (mirrors SimWorker's app-dir-relative binary lookup).
  QDir d(QCoreApplication::applicationDirPath());
  for (int up = 0; up < 8; ++up) {
    if (d.exists("tools") && d.exists("navlink"))
      return d.absolutePath();
    if (!d.cdUp())
      break;
  }
  // 3) Last-resort dev default (overridable in the UI / via VAYU_REPO).
  return QDir::homePath() + "/Documents/Drone/stack/vayu";
}


// C trampoline handed to the module as its telemetry sink. Runs
// on whichever firmware thread is producing the bytes (telemetry
// task, flush task, ...). Marshals onto the widget's thread via
// queued invocation -- Qt does the heavy lifting; we don't share any
// Qt object across threads here, the QByteArray is copied across the
// connection.
void uart2_to_widget_trampoline(void *user, const uint8_t *data, size_t n) {
  auto *w = static_cast<SimulatorWidget *>(user);
  if (!w || n == 0)
    return;
  // Called once per firmware UART write — PER BYTE in SITL. Coalesce instead of
  // posting a queued GUI event each time (that flooded the event loop and made
  // the whole app sluggish while the sim ran).
  w->queueUartBytes(data, n);
}

} // namespace

// ============================================================================

SimulatorWidget::SimulatorWidget(QWidget *parent) : QWidget(parent) {
  QSettings settings;
  m_repoRoot =
      settings.value(kRepoRootSettingKey, defaultRepoRoot()).toString();
  m_logDir = settings.value(kLogDirSettingKey, vsim::defaultLogDir()).toString();

  // Route firmware UART2 bytes through our queued slot. Once the module boots
  // the telemetry chain, packets begin flowing into onUartBytes() on the GUI
  // thread, which re-emits them as dataReceived(). MainWindow connects that to
  // its existing DroneProtocol parser so the regular telemetry / log panels
  // light up automatically.
  //
  // The sink is a plain callback: the module owns the telemetry plumbing, so
  // this widget holds no firmware struct and nothing here has to match the
  // firmware's layout. Attached to the module rather than to m_sim, which does
  // not exist until the sim is started.
  if (SitlModule::instance().available())
    SitlModule::instance().api()->set_telemetry_sink(
        &uart2_to_widget_trampoline, this);

  buildUi();

  // Stand up the RC bridge NOW, before any vayu_sitl_start: the firmware's
  // RC feeder reads $VAYU_UART_RC_PATH once at boot, so the pty + env must
  // exist beforehand. The bridge runs continuously and streams a neutral
  // frame until enabled; the checkbox toggles joystick influence live.
  m_rc = new RcBridge(this);
  applyRcSource(); // pushes source + device path + baud to the bridge
  m_rc->setEnabled(m_rcEnable->isChecked());
  QString rcErr;
  if (m_rc->openPty(&rcErr)) {
    qputenv("VAYU_UART_RC_PATH", m_rc->slavePath().toLocal8Bit());
    connect(m_rc, &RcBridge::logLine, this,
            [this](const QString &s) { appendLog("rc", s); });
    connect(m_rc, &RcBridge::channelsUpdated, this,
            [this](int r, int pi, int t, int y, int a, int c6) {
              if (m_rcReadout)
                m_rcReadout->setText(
                    QString("RC: R%1 P%2 T%3 Y%4 A%5 C6:%6%7")
                        .arg(r)
                        .arg(pi)
                        .arg(t)
                        .arg(y)
                        .arg(a)
                        .arg(c6)
                        .arg(c6 > 1500 ? " (ACRO)" : " (STAB)"));
            });
    connect(m_rc, &RcBridge::axesUpdated, this,
            [this](QVector<int> au, QVector<int> bu) {
              if (!m_rcAxesLabel)
                return;
              QString s = QStringLiteral("Ax");
              for (int i = 0; i < au.size(); ++i)
                s += QString(" %1:%2").arg(i).arg(au[i]);
              s += QStringLiteral("  Btn");
              for (int i = 0; i < bu.size(); ++i)
                s += QString(" %1:%2").arg(i).arg(bu[i]);
              m_rcAxesLabel->setText(s);
            });
    // Apply the persisted channel→axis mapping to the bridge.
    for (int f = 0; f < 5; ++f)
      if (m_rcAxisCombo[f])
        m_rc->setMapping(f, m_rcAxisCombo[f]->currentData().toInt(),
                         m_rcInvert[f]->isChecked());
    m_rc->start();
  } else {
    delete m_rc;
    m_rc = nullptr;
  }

  // RC UART starts disconnected; reflect that on the button and gate it to the
  // UART source. (The bridge's uartConnected_ already defaults false, so it has
  // not opened any tty.)
  setRcUartConnected(false);
  if (m_rcConnect && m_rcSource)
    m_rcConnect->setEnabled(m_rcSource->currentData().toInt() ==
                            RcBridge::Uart);

  // If another subsystem (board telemetry) claims the RC port, give it up.
  connect(&PortArbiter::instance(), &PortArbiter::revoked, this,
          [this](const QString &, QObject *owner) {
            if (owner == this && m_rcUartConnected) {
              appendLog("rc", tr("RC UART port taken by another connection — "
                                 "disconnected."));
              setRcUartConnected(false);
            }
          });
}

SimulatorWidget::~SimulatorWidget() {
  // Detach the UART2 callback FIRST. The firmware threads outlive this
  // widget (vayu_sitl is one-shot — they run until the process exits), and
  // would otherwise invoke the trampoline on a half-destroyed widget:
  // "QMetaObject::invokeMethod: No such method QWidget::onUartBytes" and a
  // dangling-pointer crash risk.
  if (SitlModule::instance().available())
    SitlModule::instance().api()->shutdown();
  stopInAppSim();
  // A collision build may still be running on the pool: let it finish (<~100 ms)
  // and delete its temp file, so nothing outlives the widget.
  if (m_shipWatcher) {
    m_shipWatcher->disconnect();
    m_shipWatcher->waitForFinished();
    QFile::remove(m_shipRunningTmp);
    m_shipWatcher = nullptr;
  }

  // Tear down the autotune machinery synchronously. Normally a tune is stopped
  // via stopAutotune() -> cancel() -> queued done -> onTuneDone(), but at app
  // exit the event loop is already gone, so that queued signal never arrives.
  // m_tuneThread is a QThread-derived child of this widget; if left running it'd
  // be "destroyed while still running" by ~QObject, which qFatal()s (the abort
  // seen on exit). Join it here.
  if (m_tuneWorker)
    m_tuneWorker->cancel(); // break the blocking engine.run()
  if (m_tuneThread) {
    m_tuneThread->quit();
    if (!m_tuneThread->wait(3000)) {
      m_tuneThread->terminate();
      m_tuneThread->wait(1000);
    }
    delete m_tuneThread; // joined: direct delete is safe (no event loop)
    m_tuneThread = nullptr;
  }
  if (m_tuneWorker) {
    delete m_tuneWorker;
    m_tuneWorker = nullptr;
  }

  closeLogFile(); // belt-and-suspenders: stopInAppSim already does this
  // We do NOT call vayu_sitl_stop()'s teardown completely; the firmware
  // threads keep running until the process exits. See host_lifecycle.c.
  // The module's shutdown() above already dropped the telemetry sink and
  // released its plumbing.
}

// ----------------------------------------------------------------------------
// UI
// ----------------------------------------------------------------------------

void SimulatorWidget::buildUi() {
  auto *root = new QVBoxLayout(this);
  root->setContentsMargins(16, 16, 16, 16);
  root->setSpacing(10);

  // ---- Header: title + mode tabs (Vehicle | World) + back ----
  {
    auto *header = new QHBoxLayout();
    auto *title = new QLabel(tr("Simulator"), this);
    title->setStyleSheet(
        QString("color: %1; font-size: 18px; font-weight: bold;")
            .arg(Theme::hex(Theme::kAccent)));
    header->addWidget(title);
    header->addSpacing(18);

    m_vehicleTab = new QPushButton(tr("Vehicle"), this);
    m_worldTab = new QPushButton(tr("World"), this);
    m_tuneTab = new QPushButton(tr("Autotune"), this);
    auto *grp = new QButtonGroup(this);
    grp->setExclusive(true);
    for (auto *b : {m_vehicleTab, m_worldTab, m_tuneTab}) {
      b->setCheckable(true);
      b->setObjectName("ToggleButton");
      b->setCursor(Qt::PointingHandCursor);
    }
    m_vehicleTab->setToolTip(
        tr("Configure the airframe (locked while simulating)"));
    m_worldTab->setToolTip(tr("Design the world and run the simulation"));
    m_tuneTab->setToolTip(
        tr("Auto-tune the PID gains for the current vehicle"));
    grp->addButton(m_vehicleTab, 0);
    grp->addButton(m_worldTab, 1);
    grp->addButton(m_tuneTab, 2);
    m_vehicleTab->setChecked(true);
    header->addWidget(m_vehicleTab);
    header->addWidget(m_worldTab);
    header->addWidget(m_tuneTab);

    header->addStretch();
    root->addLayout(header);

    connect(grp, &QButtonGroup::idClicked, this,
            [this](int id) { setMode(id); });
  }

  // ---- Split: 3D viewport | stacked properties panel ----
  auto *splitter = new QSplitter(Qt::Horizontal, this);
  root->addWidget(splitter, 1);

  auto *viewPanel = new SimViewPanel(*this, splitter);
  splitter->addWidget(viewPanel);

  m_rightStack = new QStackedWidget(splitter);
  splitter->addWidget(m_rightStack);
  // The panel's minimum width follows its pages (MinWidthScrollArea), so
  // forbid dragging it to nothing instead of cropping its contents.
  splitter->setChildrenCollapsible(false);

  // ===== Vehicle page: airframe geometry + motor editor =====
  m_rightStack->addWidget(new SimVehiclePanel(*this)); // index 0 = Vehicle

  // ===== World page: environment + aerodynamics + simulation =====
  m_rightStack->addWidget(new SimWorldPanel(*this)); // index 1 = World

  // ===== Autotune page: PID gain search against the current vehicle =====
  m_rightStack->addWidget(new SimAutotunePanel(*this)); // index 2 = Autotune

  splitter->setStretchFactor(0, 3);
  splitter->setStretchFactor(1, 2);

  // Restore persisted configs (geometry setConfig also loads the mesh +
  // recomputes, firing previewUpdated -> applyGeometryToRenderer).
  // (Read before setConfig: its previewUpdated persists the geometry.)
  const bool hadSavedGeometry =
      QSettings().contains(QString("%1/mass").arg(kGeomGroup));
  m_geomEditor->setConfig(restoreGeometry());
  {
    // Which preset's gains Start sends: the saved choice, else the 5-inch for a
    // fresh install (that is the default geometry); "" = a custom vehicle.
    m_presetId = QSettings()
                     .value("simulator/presetId",
                            hadSavedGeometry ? QString() : QStringLiteral("5in"))
                     .toString();
    m_geomEditor->setPresetId(m_presetId);
    applyGeometryToRenderer(); // model class follows the restored preset
  }
  m_worldEditor->setConfig(restoreWorld());
  m_worldEditor->setWindConfig(restoreWind());
  if (m_renderer)
    m_renderer->setObstacles(m_worldEditor->config().obstacles);
  if (m_downRenderer)
    m_downRenderer->setObstacles(m_worldEditor->config().obstacles);
  loadWorldMeshToRenderer();
  placeVehicleOnWorld(); // the restored world's ground, not z = 0

  setMode(0); // start in Vehicle (sim stopped)
}

void SimulatorWidget::setMode(int mode) {
  if (!m_rightStack)
    return;
  // Vehicle is config-only and locked while the sim runs.
  if (mode == 0 && m_sim)
    mode = 1;
  m_rightStack->setCurrentIndex(mode);
  if (m_vehicleTab)
    m_vehicleTab->setChecked(mode == 0);
  if (m_worldTab)
    m_worldTab->setChecked(mode == 1);
  if (m_tuneTab)
    m_tuneTab->setChecked(mode == 2);
  // Motor gizmos belong to Vehicle mode; obstacle gizmos to World mode. Both
  // only when stopped (a running sim owns the airframe + the live world).
  if (m_renderer) {
    m_renderer->setMotorsEditable(mode == 0 && !m_sim);
    m_renderer->setObstacleEditMode(mode == 1 && !m_sim);
    // Vehicle = just the airframe; World = the whole world.
    m_renderer->setWorldVisible(mode == 1);
    // Free-roam the world only while stopped; a running sim locks a 3rd-person
    // (orbit) view on the drone, with the FPV toggle for onboard.
    m_renderer->setFreeFly(mode == 1 && !m_sim);
  }
}

void SimulatorWidget::pushFaults() {
  if (!m_sim)
    return;
  m_sim->sendFaults(
      {m_motorKill[0], m_motorKill[1], m_motorKill[2], m_motorKill[3]},
      m_imuDropout);
}

void SimulatorWidget::pushNoise() {
  if (!m_sim)
    return;
  auto &a = m_sensorRow[0];
  auto &g = m_sensorRow[1];
  auto &m = m_sensorRow[2];
  m_sim->sendNoise(a.sigma->value(), a.clip->value(), a.en->isChecked(),
                   g.sigma->value(), g.clip->value(), g.en->isChecked(),
                   m.sigma->value(), m.clip->value(), m.en->isChecked());
}

QString SimulatorWidget::exportVehicleGeometryJson() {
  const vsim::GeometryConfig g = m_geomEditor->physicsConfig();
  QJsonObject root;
  root["mass"] = g.mass;
  QJsonArray inertia;
  for (int i = 0; i < 9; ++i)
    inertia.append(g.inertia[i]);
  root["inertia"] = inertia;
  QJsonArray motors;
  for (const auto &m : g.motors) {
    QJsonObject mo;
    mo["pos"] = QJsonArray{m.pos.x(), m.pos.y(), m.pos.z()};
    mo["axis"] = QJsonArray{m.axis.x(), m.axis.y(), m.axis.z()};
    mo["spin"] = m.spin;
    mo["k_thrust"] = m.k_thrust;
    mo["k_moment"] = m.k_moment;
    mo["max_omega"] = m.max_omega;
    motors.append(mo);
  }
  root["motors"] = motors;
  QDir().mkpath(m_logDir);
  const QString path = QDir(m_logDir).absoluteFilePath("autotune_vehicle.json");
  QFile f(path);
  if (f.open(QIODevice::WriteOnly)) {
    f.write(QJsonDocument(root).toJson(QJsonDocument::Compact));
    f.close();
  }
  return path;
}

QString SimulatorWidget::exportWorldJson() {
  const vsim::WorldConfig w = m_worldEditor->config();
  QJsonObject root;
  root["gravity"] = w.gravity;
  root["ground_z"] = w.ground_z;
  root["restitution"] = w.restitution;
  root["linear_drag"] = w.linear_drag;
  root["angular_drag"] = w.angular_drag; // the one that matters for damping
  root["ground_right_gain"] = w.ground_right_gain;
  root["ground_right_damp"] = w.ground_right_damp;
  QDir().mkpath(m_logDir);
  const QString path = QDir(m_logDir).absoluteFilePath("autotune_world.json");
  QFile f(path);
  if (f.open(QIODevice::WriteOnly)) {
    f.write(QJsonDocument(root).toJson(QJsonDocument::Compact));
    f.close();
  }
  return path;
}

void SimulatorWidget::startAutotune() {
  if (m_tuneThread || !m_geomEditor)
    return;
  const QString root = defaultRepoRoot();

  // The autotune search runs on the single in-process `vayu_sitl_rtos` backend
  // (deterministic, ~70× realtime). System-ID is an analytic design path that
  // runs on the same binary; the optimizer/cost settings are ignored for it.
  AutotuneWorker::Params p;
  p.sysId = m_tuneSysId && m_tuneSysId->isChecked();
  if (m_tuneSysIdBw)
    p.sysIdBwFrac = m_tuneSysIdBw->value();
  p.rtosRateCost = m_tuneCostFn && m_tuneCostFn->currentIndex() == 1;
  for (const QString &cand :
       {root + "/build_sitl_rtos/vayu_sitl_rtos",
        root + "/sim/host/build_sitl_rtos/vayu_sitl_rtos"}) {
    if (QFileInfo::exists(cand)) {
      p.rtosBin = cand;
      break;
    }
  }
  if (p.rtosBin.isEmpty()) {
    m_tuneLog->appendPlainText(
        tr("[error] autotune backend not found. Build it with:\n"
           "  cmake -S sim/host -B build_sitl_rtos -DVAYU_SITL_RTOS_BUILD=ON\n"
           "  cmake --build build_sitl_rtos --target vayu_sitl_rtos"));
    return;
  }

  // Stop the interactive sim and lock Vehicle/World so the airframe can't
  // change mid-search.
  if (m_sim)
    stopInAppSim();
  if (m_vehicleTab)
    m_vehicleTab->setEnabled(false);
  if (m_worldTab)
    m_worldTab->setEnabled(false);

  const QString tuneSuffix =
      QStringLiteral("_attune%1").arg(QCoreApplication::applicationPid());
  p.suffix = tuneSuffix;

  // Tune the actual airframe + environment (mirrors the Python --geometry/
  // --world). Build the vsim_ctl bodies from the editors.
  {
    const vsim::GeometryConfig g = m_geomEditor->physicsConfig();
    vsim_ctl_geometry_t gb{};
    gb.mass = g.mass;
    gb.prop_radius = g.propRadius;
    for (int i = 0; i < 9; ++i)
      gb.inertia[i] = g.inertia[i];
    for (int i = 0; i < 4; ++i) {
      const auto &m = g.motors[i];
      gb.motors[i].pos[0] = m.pos.x();
      gb.motors[i].pos[1] = m.pos.y();
      gb.motors[i].pos[2] = m.pos.z();
      gb.motors[i].axis[0] = m.axis.x();
      gb.motors[i].axis[1] = m.axis.y();
      gb.motors[i].axis[2] = m.axis.z();
      gb.motors[i].spin = float(m.spin);
      gb.motors[i].k_thrust = m.k_thrust;
      gb.motors[i].k_moment = m.k_moment;
      gb.motors[i].max_omega = m.max_omega;
      gb.motors[i].tau = m.tau;
    }
    p.geometry = gb;
    p.hasGeometry = true;
    // Firmware mixer uses the SAME (physics-frame) layout as the in-process
    // physics: the roll/pitch torque is generated in the rotated physics frame,
    // so the mix signs must match it. The backend derives the mix from the
    // geometry blob (no separate mixer layout) after a §10.5 time-sync
    // handshake — without it set_motor_geometry is rejected and the firmware
    // runs its default mix, which is wrong for a rotated vehicle.

    m_tuneLog->clear();
    m_tuneLog->appendPlainText(
        tr("[geometry] mass=%1 kg, inertia diag=[%2, %3, %4] kg·m²")
            .arg(g.mass, 0, 'f', 3)
            .arg(g.inertia[0], 0, 'g', 4)
            .arg(g.inertia[4], 0, 'g', 4)
            .arg(g.inertia[8], 0, 'g', 4));
    m_tuneLog->appendPlainText(
        tr("[mix] firmware roll-mix=%1 (physics frame)").arg(rollMixString(g)));
  }

  p.tuneYaw = m_tuneYaw->isChecked();
  p.optimizer = m_tuneOptimizer->currentText();
  p.budget = m_tuneBudget->value();
  p.repeats =
      m_tuneRepeats->value(); // rollouts averaged per eval (smooths noise)
  p.optSeed = quint64(m_tuneSeed->value());
  p.rollout.stepUs = m_tuneStep->value();
  p.rollout.tetherK = m_tuneTether->value();
  p.rollout.seed = quint32(m_tuneSimSeed->value());
  p.rollout.excite = (m_tuneExcitation->currentIndex() == 1)
                         ? autotune::Excitation::Chirp
                         : autotune::Excitation::Step;
  p.rollout.chirpF0 = m_tuneChirpF0->value();
  p.rollout.chirpF1 = m_tuneChirpF1->value();

  m_tuneChart->reset();
  // Seed the current-vs-best table with this run's param rows (AT-2). The
  // backend tunes a restricted set, so the table must match it.
  {
    const autotune::Space space(p.tuneYaw, /*fastRtos=*/true);
    const auto names = space.names();
    m_tuneGainsTable->setRowCount(int(names.size()));
    for (int i = 0; i < int(names.size()); ++i) {
      m_tuneGainsTable->setItem(
          i, 0, new QTableWidgetItem(QString::fromStdString(names[i])));
      m_tuneGainsTable->setItem(i, 1, new QTableWidgetItem("—"));
      m_tuneGainsTable->setItem(i, 2, new QTableWidgetItem("—"));
    }
  }
  m_tuneResult->setText(tr("running…"));
  m_tuneStart->setEnabled(false);
  m_tuneStop->setEnabled(true);

  // Run the engine on a worker thread; its signals queue back to the GUI.
  m_tuneThread = new QThread(this);
  m_tuneWorker = new AutotuneWorker(p);
  m_tuneWorker->moveToThread(m_tuneThread);
  connect(m_tuneThread, &QThread::started, m_tuneWorker, &AutotuneWorker::run);
  connect(m_tuneWorker, &AutotuneWorker::evaluated, this,
          &SimulatorWidget::onTuneEvaluated);
  connect(m_tuneWorker, &AutotuneWorker::responseWindow, this,
          [this](const QVector<double> &sp, const QVector<double> &meas) {
            if (m_tuneResponse)
              static_cast<ResponsePlot *>(m_tuneResponse)->setData(sp, meas);
          });
  connect(m_tuneWorker, &AutotuneWorker::finished, this,
          &SimulatorWidget::onTuneFinished);
  connect(
      m_tuneWorker, &AutotuneWorker::failed, this,
      [this](const QString &e) { m_tuneLog->appendPlainText("[error] " + e); });
  connect(m_tuneWorker, &AutotuneWorker::log, this,
          [this](const QString &l) { m_tuneLog->appendPlainText(l); });
  connect(m_tuneWorker, &AutotuneWorker::done, this,
          &SimulatorWidget::onTuneDone);

  m_tuneThread->start();
  emit autotuneRunningChanged(true); // drives the source FSM → Autotune state
}

void SimulatorWidget::stopAutotune() {
  if (m_tuneWorker)
    m_tuneWorker->cancel(); // unwinds the optimizer + stack
}

void SimulatorWidget::onTuneEvaluated(const QVector<double> &current,
                                      const QVector<double> &best, double cost,
                                      double bestCost, int n) {
  m_tuneChart->addPoint(cost, bestCost);
  m_tuneResult->setText(tr("eval %1   cost %2   best %3")
                            .arg(n)
                            .arg(cost, 0, 'f', 2)
                            .arg(bestCost, 0, 'f', 2));
  // AT-2: current jumps around as the search explores; best settles.
  const int rows = m_tuneGainsTable->rowCount();
  for (int i = 0; i < rows; ++i) {
    if (i < current.size())
      m_tuneGainsTable->item(i, 1)->setText(
          QString::number(current[i], 'g', 4));
    if (i < best.size())
      m_tuneGainsTable->item(i, 2)->setText(QString::number(best[i], 'g', 4));
  }
}

void SimulatorWidget::onTuneFinished(const QVector<double> &bestX,
                                     const QStringList &names,
                                     double bestCost) {
  // AT-1: surface the best gains as a proposal; applying is the explicit click.
  m_tuneParams = names;
  m_tuneBestX = bestX;
  QStringList parts;
  for (int i = 0; i < names.size() && i < bestX.size(); ++i)
    parts << QStringLiteral("%1=%2").arg(names[i]).arg(bestX[i], 0, 'g', 4);
  m_tuneProposed->setText(tr("Proposed gains (cost %1): %2")
                              .arg(bestCost, 0, 'f', 2)
                              .arg(parts.join(", ")));
  m_tuneApplyBtn->setEnabled(!autotuneGainsToCommands(names, bestX).isEmpty());

  // AT-2: best is final — lock the column in green and show it as current too.
  const QColor ok(Theme::hex(Theme::kOk));
  for (int i = 0; i < bestX.size() && i < m_tuneGainsTable->rowCount(); ++i) {
    const QString v = QString::number(bestX[i], 'g', 4);
    m_tuneGainsTable->item(i, 1)->setText(v);
    QTableWidgetItem *b = m_tuneGainsTable->item(i, 2);
    b->setText(v);
    b->setForeground(ok);
  }
}

void SimulatorWidget::onTuneDone() {
  if (m_tuneThread) {
    m_tuneThread->quit();
    m_tuneThread->wait(3000);
    m_tuneThread->deleteLater();
    m_tuneThread = nullptr;
  }
  // The worker's thread has fully stopped, so direct deletion is safe
  // (deleteLater would never run — that thread's event loop is gone).
  if (m_tuneWorker) {
    delete m_tuneWorker;
    m_tuneWorker = nullptr;
  }
  if (m_tuneStart)
    m_tuneStart->setEnabled(true);
  if (m_tuneStop)
    m_tuneStop->setEnabled(false);
  // Unlock Vehicle/World now the search (which locked them) has ended.
  if (m_vehicleTab)
    m_vehicleTab->setEnabled(true);
  if (m_worldTab)
    m_worldTab->setEnabled(true);
  emit autotuneRunningChanged(false); // source FSM → Idle
}

void SimulatorWidget::parseProposedGains() {
  // AT-1: read the search result (a record on disk) and surface its best gains
  // as a *proposal*. Nothing is applied here — that's the explicit Apply click.
  m_tuneParams.clear();
  m_tuneBestX.clear();
  m_tuneProposed->setText(tr("Proposed gains: —"));
  m_tuneApplyBtn->setEnabled(false);

  QFile f(m_tuneOutJson);
  if (!f.open(QIODevice::ReadOnly))
    return;
  const QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
  f.close();
  if (!doc.isObject())
    return;
  const QJsonObject root = doc.object();
  for (const QJsonValue &n : root.value(QStringLiteral("params")).toArray())
    m_tuneParams << n.toString();
  const QJsonArray results = root.value(QStringLiteral("results")).toArray();
  if (results.isEmpty())
    return;
  // The autotuner sorts results best-first.
  for (const QJsonValue &x :
       results.first().toObject().value(QStringLiteral("best_x")).toArray())
    m_tuneBestX << x.toDouble();
  if (m_tuneParams.isEmpty() || m_tuneBestX.isEmpty())
    return;

  QStringList parts;
  for (int i = 0; i < m_tuneParams.size() && i < m_tuneBestX.size(); ++i)
    parts << QStringLiteral("%1=%2")
                 .arg(m_tuneParams[i])
                 .arg(m_tuneBestX[i], 0, 'g', 4);
  m_tuneProposed->setText(tr("Proposed gains: %1").arg(parts.join(", ")));
  m_tuneApplyBtn->setEnabled(
      !autotuneGainsToCommands(m_tuneParams, m_tuneBestX).isEmpty());
}

void SimulatorWidget::applyProposedGains() {
  const QVector<PidSetCmd> cmds =
      autotuneGainsToCommands(m_tuneParams, m_tuneBestX);
  if (cmds.isEmpty()) {
    m_tuneLog->appendPlainText(tr("[apply] no proposed gains to apply"));
    return;
  }
  // MainWindow turns these into CMD_SET_PID frames and sends them over the
  // live link (and refuses if disconnected or in replay).
  emit applyPidGainsRequested(cmds);
  m_tuneLog->appendPlainText(
      tr("[apply] requested firmware gain update — %1 PID slot(s)")
          .arg(cmds.size()));
}

void SimulatorWidget::notifyPidGainsSent() {
  m_customTune.onGainsApplied();
  updateCustomWarning(false);
}

void SimulatorWidget::updateHud(const vsim::SimSnapshot &s) {
  m_lastDronePos = s.pos_w; // tracked for the lift-onto-terrain logic
  m_lastDroneVel = s.vel_w;
  if (m_hud)
    m_hud->setSnapshot(s);
  if (m_horizon) {
    float roll, pitch, yaw;
    vsim::quatToEulerNED(s.att, &roll, &pitch, &yaw);
    m_horizon->setAttitude(roll, pitch, yaw);
  }
  // Live wind readout (horizontal speed + compass heading the wind blows TOWARD).
  if (m_worldEditor) {
    const float wN = s.wind_w.x(), wE = s.wind_w.y();
    const float spd = std::sqrt(wN * wN + wE * wE);
    float dir = std::atan2(wE, wN) * 180.0f / 3.14159265358979323846f;
    if (dir < 0.0f)
      dir += 360.0f;
    m_worldEditor->setWindReadout(spd, dir);
  }

  // Training course: feed the live pose, advance through gates, refresh the
  // active-gate highlight + guidance arrow in both renderers.
  if (m_training.active()) {
    const bool cleared = m_training.advance(s.pos_w);
    const bool show = !m_training.finished();
    if (m_renderer)
      m_renderer->setTrainingActive(m_training.activeIndex(), show);
    if (m_downRenderer)
      m_downRenderer->setTrainingActive(m_training.activeIndex(), show);
    if (cleared) {
      updateTrainingProgress();
      if (m_training.finished())
        appendLog(
            "train",
            tr("course complete — %1 gates cleared!").arg(m_training.total()));
      else
        appendLog("train", tr("gate %1 / %2 cleared")
                               .arg(m_training.passedCount())
                               .arg(m_training.total()));
    }
  }
}

void SimulatorWidget::setTrainingMode(int difficulty) {
  m_training.generate(
      static_cast<vsim::TrainingCourse::Difficulty>(difficulty));
  pushTrainingGates();
  updateTrainingProgress();
  // Start every run from the floor: re-spawn the airframe at the level pose.
  if (m_training.active() && m_sim)
    m_sim->sendReset(m_runSeed);
  if (m_training.active())
    appendLog("train",
              tr("training course armed (%1 gates)").arg(m_training.total()));
}

void SimulatorWidget::pushTrainingGates() {
  const bool show = m_training.active() && !m_training.finished();
  if (m_renderer) {
    m_renderer->setTrainingGates(m_training.gates());
    m_renderer->setTrainingActive(m_training.activeIndex(), show);
  }
  if (m_downRenderer) {
    m_downRenderer->setTrainingGates(m_training.gates());
    m_downRenderer->setTrainingActive(m_training.activeIndex(), show);
  }
}

void SimulatorWidget::updateTrainingProgress() {
  if (!m_trainingStatus)
    return;
  if (!m_training.active()) {
    m_trainingStatus->setText(tr("Training off"));
  } else if (m_training.finished()) {
    m_trainingStatus->setText(
        tr("Course complete — %1 / %1 gates").arg(m_training.total()));
  } else {
    m_trainingStatus->setText(tr("Next: gate %1 / %2")
                                  .arg(m_training.passedCount() + 1)
                                  .arg(m_training.total()));
  }
}

void SimulatorWidget::loadWorldMeshToRenderer() {
  if (!m_renderer || !m_worldEditor)
    return;
  const vsim::WorldConfig &w = m_worldEditor->config();
  cancelWorldShip(); // a new world config makes any in-flight collision stale

  // The horizon ground belonged to the finite procedural arena, which is gone.
  m_renderer->setHorizonGround(false);
  if (m_downRenderer)
    m_downRenderer->setHorizonGround(false);

  // Every procedural biome streams endlessly from w.field (preset, "endless"
  // and the legacy finite "meadow" of old saves); only imported meshes are
  // finite.
  const bool endless = vsim::isStreamedBiome(w.proceduralBiome);

  // Point the contour minimap at the active terrain's height source. The
  // endless lambda reads the streamer's field lazily (configured below); the
  // finite lambda owns its heightfield via the captured shared_ptr.
  // One terrain-height sampler (world N,E -> elevation [m]) for the active world,
  // shared by the minimap and the "lift drone onto the surface" logic.
  std::function<float(float, float)> heightAt;
  // Raw field height: cheap (~1 us), used where many samples are drawn
  // (minimap, pad slope screening) and metre-level error is harmless.
  std::function<float(float, float)> rawHeightAt;
  float minimapRange = 200.0f;
  if (endless) {
    rawHeightAt = [this](float n, float e) {
      const auto f = m_chunkStreamer.field();
      return f ? f->height(n, e) : 0.0f;
    };
    heightAt = [this](float n, float e) {
      const auto f = m_chunkStreamer.field();
      // The meshed (slope-capped) surface, not the raw field, so spawn and pad
      // heights match what is drawn and collided with.
      const auto &c = m_chunkStreamer.config();
      return f ? vsim::procgen::surfaceHeight(*f, n, e, c.chunkM,
                                              c.collisionResolution)
               : 0.0f;
    };
  }
  if (!rawHeightAt)
    rawHeightAt = heightAt; // imported / no world: null
  m_terrainHeightAt = heightAt; // null for imported / no world
  m_terrainRawHeightAt = rawHeightAt;
  // Scatter helipad pads (and the home pad at origin) for the new world.
  m_lastHelipadCenter = QVector3D(0, 0, 0);
  recomputeHelipads(0.0f, 0.0f);
  if (m_minimap) {
    m_minimap->setSampler(rawHeightAt); // ~12k samples: must stay cheap
    if (heightAt) {
      m_minimap->setRangeM(minimapRange);
      m_minimap->setReferenceHeight(
          endless ? w.field.heightM : 80.0f);
    }
  }

  // Leaving the endless biome: stop the streamer and drop its chunks so a
  // finite mesh / grid shows cleanly.
  if (!endless && m_useGpuGrass) { // leaving the GPU-grass biome
    m_useGpuGrass = false;
    m_renderer->setGpuGrassActive(false);
    if (m_downRenderer)
      m_downRenderer->setGpuGrassActive(false);
  }
  if (!endless && m_chunkStreamer.active()) {
    ++m_streamGen; // invalidate in-flight builds
    m_chunkStreamer.deactivate();
    m_chunkCache.clear();
    m_floraCache.clear();
    m_floraShown.clear();
    m_floraInflight.clear();
    m_collisionPending = false;
    if (m_streamTimer)
      m_streamTimer->stop();
    m_renderer->clearWorldChunks();
    m_renderer->clearChunkFlora();
    if (m_downRenderer) {
      m_downRenderer->clearWorldChunks();
      m_downRenderer->clearChunkFlora();
    }
  }

  // Endless streaming biome: drive the chunk streamer instead of one mesh.
  if (endless) {
    m_renderer->setWorldMesh({}, {});
    if (m_downRenderer)
      m_downRenderer->setWorldMesh({}, {});
    m_renderer->clearWorldChunks();
    m_renderer->clearChunkFlora();
    if (m_downRenderer) {
      m_downRenderer->clearWorldChunks();
      m_downRenderer->clearChunkFlora();
    }

    // New generation: invalidate any in-flight builds from a prior config and
    // drop the mesh + flora caches.
    ++m_streamGen;
    m_chunkCache.clear();
    m_floraCache.clear();
    m_floraShown.clear();
    m_floraInflight.clear();
    m_collisionPending = false;

    vsim::ChunkStreamer::Config sc;
    sc.field = w.field; // user-tuned terrain shape + colour bands
    sc.field.seed = static_cast<uint32_t>(w.proceduralSeed);
    m_chunkStreamer.configure(sc);
    // The pads above were placed before the field existed (heights 0): redo
    // them on the live field so none floats or sinks at world load.
    m_lastHelipadCenter = QVector3D(0, 0, 0);
    recomputeHelipads(0.0f, 0.0f);
    m_floraParams = w.flora; // user-tuned grass density / slope / height
    m_floraParams.seed = static_cast<uint32_t>(w.proceduralSeed);
    // The pads set up above sampled the PREVIOUS field (none on a fresh
    // launch: height 0), so the home pad was at z = 0 whatever the ground is.
    // Re-seat them on the field just configured.
    m_lastHelipadCenter = QVector3D(0, 0, 0);
    recomputeHelipads(0.0f, 0.0f);

#ifdef VAYU_SIM_GRASS
    // Prefer GPU-generated grass when the context supports compute (GL 4.3+):
    // it regenerates around the camera each frame, so we skip the CPU flora.
    vsim::GpuGrass::Params gp;
    gp.seed = static_cast<uint32_t>(w.proceduralSeed);
    gp.heightM = w.field.heightM;
    gp.featureM = w.field.featureM;
    gp.macroM = w.field.macroM;
    gp.octaves = w.field.octaves;
    gp.lacunarity = w.field.lacunarity;
    gp.gain = w.field.gain;
    gp.mountainMix = w.field.mountainMix;
    gp.grassMaxFrac = w.flora.grassMaxFrac;
    gp.slopeLo = w.flora.slopeLo;
    gp.slopeHi = w.flora.slopeHi;
    gp.heightMean = w.flora.heightMean;
    gp.heightStdDev = w.flora.heightStdDev;
    gp.flowerFrac = w.flora.flowerFrac;
    gp.look = w.look; // live grass shading knobs
    gp.field = sc.field; // palette etc. for the ground-cover (grass) mask
    const float bpc = std::max(1.0f, w.flora.bladesPerCell);
    // Near-ring candidate spacing. GpuGrass adds a coarse FAR ring (4x cell) on
    // top for distance, so this only controls near density — push it small.
    gp.cell = std::clamp(w.flora.spacing / std::sqrt(bpc), 0.045f, 1.0f);
    gp.grid = 768;
    // Falloff is computed per-ring inside GpuGrass::render now; these are only a
    // hint for the near ring.
    const float radius = gp.grid * gp.cell * 0.5f;
    gp.falloffEnd = radius * 0.9f;
    gp.falloffStart = gp.falloffEnd * 0.6f;
#endif // VAYU_SIM_GRASS

    // Terrain lighting tracks the same look knobs so ground + grass warm together.
    m_renderer->setTerrainLook(w.look.sunIntensity, w.look.ambientStrength);
    if (m_downRenderer)
      m_downRenderer->setTerrainLook(w.look.sunIntensity,
                                     w.look.ambientStrength);

#ifdef VAYU_SIM_GRASS
    m_useGpuGrass = m_renderer->gpuGrassReady();
    if (m_useGpuGrass) {
      // Params are always pushed so a later toggle-on renders immediately, but
      // the pass only activates when the "Grass" toggle is on.
      m_renderer->setGpuGrassParams(gp);
      m_renderer->setGpuGrassActive(m_grassEnabled);
      m_renderer->clearChunkFlora();
      if (m_downRenderer) {
        m_downRenderer->setGpuGrassParams(gp);
        m_downRenderer->setGpuGrassActive(m_grassEnabled);
        m_downRenderer->clearChunkFlora();
      }
      appendLog("world", m_grassEnabled
                             ? tr("GPU grass active (compute)")
                             : tr("GPU grass ready (toggle 'Grass' to show)"));
    } else {
      m_renderer->setGpuGrassActive(false);
      if (m_downRenderer)
        m_downRenderer->setGpuGrassActive(false);
    }
#endif // VAYU_SIM_GRASS

    if (!m_streamTimer) {
      m_streamTimer = new QTimer(this);
      m_streamTimer->setInterval(100); // 10 Hz poll of the view centre
      connect(m_streamTimer, &QTimer::timeout, this,
              &SimulatorWidget::onStreamTick);
    }
    m_streamTimer->start();
    onStreamTick(); // stream the initial neighbourhood immediately
    appendLog("world", tr("endless procedural terrain (seed %1) streaming")
                           .arg(w.proceduralSeed));
    return;
  }

  if (w.worldMeshPath.isEmpty()) {
    m_renderer->setWorldMesh({}, {});
    if (m_downRenderer)
      m_downRenderer->setWorldMesh({}, {});
    if (m_sim)
      m_sim->clearWorldMesh();
    return;
  }
  // Bake the source up-axis into NED (up = -Z): Z-up needs a 180° flip about
  // X; Y-up a -90° rotation about X. Then a world-space placement offset (NED)
  // so the user can move the world off the drone's spawn. Translate is applied
  // last (M = T·R) so the offset is in final NED metres. The same baked mesh
  // drives both the render and the collision BVH, so they stay in lockstep.
  QMatrix4x4 xform;
  xform.translate(w.worldMeshOffset);
  if (w.worldUpAxis == 1)
    xform.rotate(-90.0f, 1, 0, 0);
  else
    xform.rotate(180.0f, 1, 0, 0);
  QString err;
  const vsim::LoadedMesh m =
      vsim::loadMesh(w.worldMeshPath, w.worldScale, xform, &err);
  if (!m.valid) {
    appendLog("world", tr("world mesh load failed: %1").arg(err));
    m_renderer->setWorldMesh({}, {});
    if (m_downRenderer)
      m_downRenderer->setWorldMesh({}, {});
    if (m_sim)
      m_sim->clearWorldMesh();
    return;
  }
  m_renderer->setWorldMesh(m.positions, m.normals, m.colors);
  if (m_downRenderer)
    m_downRenderer->setWorldMesh(m.positions, m.normals, m.colors);
  appendLog("world", tr("world mesh loaded: %1 tris").arg(m.triangleCount()));

  // Hand the same baked geometry to the physics daemon as a collision BVH.
  // Render and collision therefore share one transform → they never disagree.
  if (m_sim)
    sendWorldMeshToSim(m);
}

void SimulatorWidget::updateMinimap() {
  if (!m_minimap || !m_minimapPip || !m_minimapPip->isVisible())
    return;
  if (!m_renderer || !m_minimap->hasSampler())
    return;
  const QVector3D c = m_renderer->streamCenter();
  m_minimap->setView(c.x(), c.y(), m_renderer->viewHeadingRad());
}

void SimulatorWidget::onStreamTick() {
  if (!m_chunkStreamer.active() || !m_renderer)
    return;
  const QVector3D c = m_renderer->streamCenter();
  // The collision window leads the drone along its velocity (not when the
  // free-fly camera is the centre: that has no physics body).
  const QVector3D v = (m_sim && !m_renderer->freeFly()) ? m_lastDroneVel
                                                        : QVector3D();
  const vsim::StreamPlan p = m_chunkStreamer.plan(c.x(), c.y(), v.x(), v.y());

  for (qint64 key : p.toRemove) {
    m_renderer->removeWorldChunk(key);
    m_renderer->removeChunkFlora(key);
    if (m_downRenderer) {
      m_downRenderer->removeWorldChunk(key);
      m_downRenderer->removeChunkFlora(key);
    }
    m_chunkCache.erase(key);
    m_floraCache.erase(key);
    m_floraShown.erase(key);
    m_floraInflight.erase(key);
    m_chunkStreamer.forget(key);
  }

  if (p.collisionDue) {
    m_collisionPending = true;
    m_colCx = p.colCx;
    m_colCy = p.colCy;
    // CPU grass follows the centre — only when enabled and GPU grass isn't used.
#ifdef VAYU_SIM_GRASS
    if (m_grassEnabled && !m_useGpuGrass)
      streamFlora(p.colCx, p.colCy);
#endif
  }

  // Mesh the requested chunks OFF the UI thread; apply the results on the main
  // thread when each future finishes. The field is const + shared, so parallel
  // sampling is safe; the generation tag drops results from a stale config.
  const auto field = m_chunkStreamer.field();
  const float chunkM = m_chunkStreamer.config().chunkM;
  const int res = m_chunkStreamer.config().resolution;
  const int collRes = m_chunkStreamer.config().collisionResolution;
  const int gen = m_streamGen;
  for (const vsim::ChunkReq &req : p.toBuild) {
    auto *watcher = new QFutureWatcher<BuiltChunk>(this);
    connect(watcher, &QFutureWatcherBase::finished, this,
            [this, watcher, req, gen]() {
              if (gen == m_streamGen)
                onChunkMeshed(req.key, watcher->result());
              else
                m_chunkStreamer.forget(
                    req.key); // stale: config changed mid-build
              watcher->deleteLater();
            });
    watcher->setFuture(QtConcurrent::run([field, req, chunkM, res, collRes]() {
      BuiltChunk b;
      b.mesh =
          vsim::procgen::meshFieldChunk(*field, req.cx, req.cy, chunkM, res);
      b.collMesh = collRes == res
                       ? b.mesh // same tessellation: collide with what is drawn
                       : vsim::procgen::meshFieldChunk(*field, req.cx, req.cy,
                                                       chunkM, collRes);
      return b;
    }));
  }

  tryBuildCollision(); // in case the neighbourhood is already cached

  // Refresh helipads when the view has roamed enough (avoids 10 Hz churn).
  if (std::hypot(c.x() - m_lastHelipadCenter.x(),
                 c.y() - m_lastHelipadCenter.y()) > 35.0f) {
    m_lastHelipadCenter = c;
    recomputeHelipads(c.x(), c.y());
  }
}

void SimulatorWidget::onChunkMeshed(qint64 key, const BuiltChunk &built) {
  m_chunkStreamer.markBuilt(key);
  if (!m_chunkStreamer.wanted(key)) { // drifted out of range while meshing
    m_chunkStreamer.forget(key);
    return;
  }
  // Cache the COARSE mesh for collision reuse; upload the FINE mesh for render.
  // Flora is scattered separately (streamFlora), only for the near chunks.
  m_chunkCache[key] =
      std::make_shared<const vsim::procgen::ProcMesh>(built.collMesh);
  const vsim::ChunkMeshData d = vsim::toChunkMeshData(key, built.mesh);
  m_renderer->setWorldChunk(d.key, d.positions, d.normals, d.colors);
  if (m_downRenderer)
    m_downRenderer->setWorldChunk(d.key, d.positions, d.normals, d.colors);

  tryBuildCollision();
}

#ifdef VAYU_SIM_GRASS
void SimulatorWidget::uploadFloraChunk(qint64 key) {
  auto it = m_floraCache.find(key);
  if (it == m_floraCache.end())
    return;
  const float chunkM = m_chunkStreamer.config().chunkM;
  const float cxw = (vsim::ChunkStreamer::cxOf(key) + 0.5f) * chunkM;
  const float cyw = (vsim::ChunkStreamer::cyOf(key) + 0.5f) * chunkM;
  const float half = chunkM * 0.5f;
  m_renderer->setChunkFlora(key, it->second.first, it->second.second, cxw, cyw,
                            half);
  if (m_downRenderer)
    m_downRenderer->setChunkFlora(key, it->second.first, it->second.second, cxw,
                                  cyw, half);
  m_floraShown.insert(key);
}

void SimulatorWidget::streamFlora(int cx, int cy) {
  // Drop grass that drifted out of the near radius.
  std::vector<qint64> drop;
  for (qint64 key : m_floraShown)
    if (std::abs(vsim::ChunkStreamer::cxOf(key) - cx) > kFloraRadius ||
        std::abs(vsim::ChunkStreamer::cyOf(key) - cy) > kFloraRadius)
      drop.push_back(key);
  for (qint64 key : drop) {
    m_renderer->removeChunkFlora(key);
    if (m_downRenderer)
      m_downRenderer->removeChunkFlora(key);
    m_floraShown.erase(key);
  }
  // Bring grass into range: upload from cache, or scatter it off-thread.
  const auto field = m_chunkStreamer.field();
  if (!field)
    return;
  const float chunkM = m_chunkStreamer.config().chunkM;
  const vsim::procgen::FloraParams fp = m_floraParams; // user-tuned knobs
  const int gen = m_streamGen;
  for (int j = cy - kFloraRadius; j <= cy + kFloraRadius; ++j)
    for (int i = cx - kFloraRadius; i <= cx + kFloraRadius; ++i) {
      const qint64 key = vsim::ChunkStreamer::keyOf(i, j);
      if (m_floraShown.count(key))
        continue;
      if (m_floraCache.count(key)) {
        uploadFloraChunk(key);
        continue;
      }
      if (m_floraInflight.count(key))
        continue;
      m_floraInflight.insert(key);
      const int fcx = i, fcy = j;
      auto *w = new QFutureWatcher<std::vector<float>>(this);
      connect(w, &QFutureWatcherBase::finished, this, [this, w, key, gen]() {
        if (gen == m_streamGen)
          onFloraScattered(key, w->result());
        else
          m_floraInflight.erase(key);
        w->deleteLater();
      });
      w->setFuture(QtConcurrent::run([field, fcx, fcy, chunkM, fp]() {
        const auto blades =
            vsim::procgen::scatterFlora(*field, fcx, fcy, chunkM, fp);
        std::vector<float> d;
        d.reserve(blades.size() * 9);
        for (const vsim::procgen::FloraInstance &g : blades)
          d.insert(d.end(), {g.pos.x, g.pos.y, g.pos.z, g.yaw, g.height,
                             g.flower, g.tint.x, g.tint.y, g.tint.z});
        return d;
      }));
    }
}

void SimulatorWidget::onFloraScattered(qint64 key,
                                       const std::vector<float> &packed) {
  m_floraInflight.erase(key);
  m_floraCache[key] = {packed, static_cast<int>(packed.size() / 9)};
  // Upload only if it's still within the near radius of the current centre.
  if (std::abs(vsim::ChunkStreamer::cxOf(key) - m_colCx) <= kFloraRadius &&
      std::abs(vsim::ChunkStreamer::cyOf(key) - m_colCy) <= kFloraRadius)
    uploadFloraChunk(key);
}
#endif // VAYU_SIM_GRASS

void SimulatorWidget::tryBuildCollision() {
  if (!m_collisionPending)
    return;
  if (!m_sim) {
    m_collisionPending = false;
    return;
  } // no physics -> not needed
  // Build the local collision BVH from the cached chunk meshes (no regen). Wait
  // until the whole collision neighbourhood is cached.
  const std::vector<vsim::ChunkReq> need =
      m_chunkStreamer.collisionChunks(m_colCx, m_colCy);
  std::vector<std::shared_ptr<const vsim::procgen::ProcMesh>> chunks;
  chunks.reserve(need.size());
  for (const vsim::ChunkReq &req : need) {
    auto it = m_chunkCache.find(req.key);
    if (it == m_chunkCache.end())
      return; // not all cached yet; retry on next
    chunks.push_back(it->second);
  }
  m_collisionPending = false;
  // Pads become solid landing surfaces (a few hundred verts: cheap here). The
  // heavy concat + BVH build happen on the worker.
  std::vector<QVector3D> extra;
  appendHelipadCollision(extra);
  // The local terrain just changed under the drone (spawn or a crossing); if
  // that left it buried it is re-dropped above the new surface -- but only once
  // this collision has actually reached the engine (async build), so the lift
  // never targets terrain the engine does not have yet. Self-guarded, so an
  // airborne or resting drone is untouched.
  const float cm = m_chunkStreamer.config().chunkM;
  const int rr = m_chunkStreamer.config().collisionRadius;
  WorldShipReq req;
  req.chunks = std::move(chunks);
  req.extra = std::move(extra);
  req.doubleSided = m_worldEditor->config().worldMeshDoubleSided;
  req.lift = true;
  req.coverage = QRectF((m_colCx - rr) * cm, (m_colCy - rr) * cm,
                        (2 * rr + 1) * cm, (2 * rr + 1) * cm);
  enqueueShip(std::move(req));
}

void SimulatorWidget::recomputeHelipads(float cx, float cy) {
  m_helipads.clear();
  if (!m_terrainHeightAt) {
    m_homePad = QVector3D(0, 0, 0);
    if (m_renderer)
      m_renderer->setHelipads(m_helipads);
    if (m_downRenderer)
      m_downRenderer->setHelipads(m_helipads);
    return;
  }
  // Home pad always at the origin — a deterministic, guaranteed drone spawn.
  m_homePad = QVector3D(0, 0, m_terrainHeightAt(0.0f, 0.0f));
  m_helipads.push_back(m_homePad);

  // Deterministic coarse grid: one jittered candidate per cell, accepted on
  // flat enough ground. Mirrors the flora global-grid idea but far sparser, so
  // the same pads appear regardless of view history. Computed within kRange of
  // the view centre so pads stream in/out as you roam.
  const float kGrid = 70.0f, kRange = 260.0f, kEps = 2.0f, kMaxSlope = 0.09f;
  const int kCandidates = 4; // probe several spots/cell, keep the flattest
  auto hash = [](int gx, int gy) -> uint32_t {
    uint32_t h = uint32_t(gx) * 0x9e3779b1u ^ uint32_t(gy) * 0x85ebca77u;
    h ^= h >> 16;
    h *= 0x7feb352du;
    h ^= h >> 15;
    h *= 0x846ca68bu;
    h ^= h >> 16;
    return h;
  };
  auto slopeAt = [&](float px, float py) {
    const float dhdx =
        (m_terrainRawHeightAt(px + kEps, py) -
         m_terrainRawHeightAt(px - kEps, py)) /
        (2.0f * kEps);
    const float dhdy =
        (m_terrainRawHeightAt(px, py + kEps) -
         m_terrainRawHeightAt(px, py - kEps)) /
        (2.0f * kEps);
    return std::sqrt(dhdx * dhdx + dhdy * dhdy);
  };
  const int gx0 = int(std::floor((cx - kRange) / kGrid));
  const int gx1 = int(std::floor((cx + kRange) / kGrid));
  const int gy0 = int(std::floor((cy - kRange) / kGrid));
  const int gy1 = int(std::floor((cy + kRange) / kGrid));
  for (int gx = gx0; gx <= gx1; ++gx)
    for (int gy = gy0; gy <= gy1; ++gy) {
      if (gx == 0 && gy == 0)
        continue; // origin reserved for the home pad
      if ((hash(gx, gy) & 0xffu) > 180u)
        continue; // ~70% of cells get a pad
      // Probe several jittered spots in the cell, keep the flattest one.
      float bestSlope = 1e9f, bx = 0, by = 0;
      for (int k = 0; k < kCandidates; ++k) {
        const uint32_t h2 =
            hash(gx * 73856093 + k * 19349663, gy * 83492791 - k);
        const float jx = float(h2 & 0xffu) / 255.0f - 0.5f;
        const float jy = float((h2 >> 8) & 0xffu) / 255.0f - 0.5f;
        const float px = (float(gx) + 0.5f + jx * 0.8f) * kGrid;
        const float py = (float(gy) + 0.5f + jy * 0.8f) * kGrid;
        const float s = slopeAt(px, py);
        if (s < bestSlope) {
          bestSlope = s;
          bx = px;
          by = py;
        }
      }
      if (bestSlope > kMaxSlope)
        continue; // no flat-enough spot in this cell
      if ((bx - cx) * (bx - cx) + (by - cy) * (by - cy) > kRange * kRange)
        continue;
      m_helipads.emplace_back(bx, by, m_terrainHeightAt(bx, by));
    }
  if (m_renderer)
    m_renderer->setHelipads(m_helipads);
  if (m_downRenderer)
    m_downRenderer->setHelipads(m_helipads);
}

void SimulatorWidget::appendHelipadCollision(
    std::vector<QVector3D> &pos) const {
  const float R = vsim::SimRendererWidget::kHelipadRadiusM;
  const int N = 20; // collision tessellation (coarser than render is fine)
  // Every triangle is emitted with BOTH windings so it collides regardless of
  // the BVH's single/double-sided flag (the drone must rest on the deck top).
  auto tri2 = [&](const QVector3D &a, const QVector3D &b, const QVector3D &c) {
    pos.push_back(a);
    pos.push_back(b);
    pos.push_back(c);
    pos.push_back(a);
    pos.push_back(c);
    pos.push_back(b);
  };
  for (const QVector3D &pad : m_helipads) {
    const float px = pad.x(), py = pad.y();
    const float zDeck =
        -pad.z() - vsim::SimRendererWidget::kHelipadDeckM; // top
    const float zBase = -pad.z();                          // terrain
    const QVector3D ctr(px, py, zDeck);
    for (int j = 0; j < N; ++j) {
      const double a0 = 2 * M_PI * j / N, a1 = 2 * M_PI * (j + 1) / N;
      const QVector3D r0(px + R * std::cos(a0), py + R * std::sin(a0), zDeck);
      const QVector3D r1(px + R * std::cos(a1), py + R * std::sin(a1), zDeck);
      const QVector3D b0(px + R * std::cos(a0), py + R * std::sin(a0), zBase);
      const QVector3D b1(px + R * std::cos(a1), py + R * std::sin(a1), zBase);
      tri2(ctr, r0, r1); // deck cap (landing surface)
      tri2(r0, b0, b1);  // side wall down to terrain
      tri2(r0, b1, r1);
    }
  }
}

float SimulatorWidget::landingSurfaceZ(float x, float y) const {
  float z =
      m_terrainHeightAt ? -m_terrainHeightAt(x, y) : 0.0f; // terrain surface
  const float r2 = vsim::SimRendererWidget::kHelipadRadiusM *
                   vsim::SimRendererWidget::kHelipadRadiusM;
  for (const QVector3D &pad : m_helipads) {
    const float dx = x - pad.x(), dy = y - pad.y();
    if (dx * dx + dy * dy <= r2) {
      // Deck sits kHelipadDeckM above that pad's terrain (more negative z =
      // higher); rest on the highest surface we're standing over.
      z = std::min(z, -pad.z() - vsim::SimRendererWidget::kHelipadDeckM);
    }
  }
  return z;
}

void SimulatorWidget::sendSpawnReset() {
  if (!m_sim)
    return;
  // On procedural terrain, respawn ON the home helipad (origin) -- a small
  // margin above the pad top so it settles onto the deck, not the slope.
  if (m_terrainHeightAt) {
    const float padTop = landingSurfaceZ(m_homePad.x(), m_homePad.y());
    m_sim->sendResetPose(m_homePad.x(), m_homePad.y(), padTop - 0.4f, m_runSeed);
  } else {
    m_sim->sendReset(m_runSeed);
  }
}

void SimulatorWidget::placeVehicleOnWorld() {
  if (m_attached)
    return; // an external sim owns the pose
  if (m_sim) {
    // Running: what Reset does, held until the new collision is in the engine
    // (the same hold Start uses), so the body is never placed over ground the
    // engine does not have yet.
    if (spawnResetWaitsForCollision())
      m_spawnResetPending = true;
    else
      sendSpawnReset();
    return;
  }
  // Stopped: nothing is simulated, so park the vehicle the 3D view draws (and
  // the camera follows) level, at rest, on the home pad of the new ground. It
  // used to stay where the old world had it, usually inside the new terrain.
  vsim::SimSnapshot s;
  const float padTop = landingSurfaceZ(m_homePad.x(), m_homePad.y());
  s.pos_w = QVector3D(m_homePad.x(), m_homePad.y(), padTop - 0.1f);
  m_lastDronePos = s.pos_w;
  m_lastDroneVel = QVector3D();
  if (m_renderer) {
    m_renderer->setSnapshot(s);
    if (m_renderer->freeFly()) {
      // The free-roam camera (World tab, sim stopped) sat wherever the old
      // world left it, i.e. inside the new terrain: seat it by the vehicle
      // again (setFreeFly seeds it at the orbit eye on the rising edge).
      m_renderer->setFreeFly(false);
      m_renderer->setFreeFly(true);
    }
  }
  if (m_downRenderer)
    m_downRenderer->setSnapshot(s);
}

bool SimulatorWidget::spawnResetWaitsForCollision() const {
  return m_shipWatcher || m_shipPending || m_collisionPending;
}

void SimulatorWidget::liftDroneToSurface() {
  if (!m_sim || !m_terrainHeightAt)
    return;
  const float x = m_lastDronePos.x();
  const float y = m_lastDronePos.y();
  // Terrain is a single-valued height field, so the analytic height under the
  // drone is exactly where a ray cast straight down from far above would hit.
  const float h = m_terrainHeightAt(x, y); // surface elevation [m] above z=0
  // Land on the pad top when over a helipad, otherwise on the terrain.
  const float surfaceZ = landingSurfaceZ(x, y);

  // Only re-drop when the drone is genuinely buried — more than buriedEps below
  // the surface. In NED a larger z is lower, so droneZ > surfaceZ + eps means it
  // has sunk into the terrain. A drone resting on or flying above the surface is
  // left alone (no yanking a landed or airborne drone).
  const float buriedEps = 0.3f;
  if (m_lastDronePos.z() <= surfaceZ + buriedEps)
    return;

  // Re-drop level + stationary a clear margin above the surface, so it falls and
  // settles instead of spawning embedded in a slope (which wedges it tilted).
  const float dropClearance = 1.5f;
  m_sim->sendResetPose(x, y, surfaceZ - dropClearance);
  appendLog("world",
            tr("re-dropped drone above terrain (surface %1 m)")
                .arg(h, 0, 'f', 1));
}

namespace {
// Where the engine mmaps the collision blob (see vsim::worldBlobPath).
QString worldBlobPath() {
  return vsim::worldBlobPath(
      QString::fromLocal8Bit(qgetenv("VSIM_FIFO_SUFFIX")));
}
} // namespace

void SimulatorWidget::cancelWorldShip() {
  // Any build in flight is now stale: its result is dropped (and its temp file
  // removed) when it lands, and nothing queued behind it is started.
  m_shipGen->fetch_add(1);
  m_shipPending.reset();
}

void SimulatorWidget::sendWorldMeshToSim(vsim::LoadedMesh m, bool liftAfter,
                                         std::optional<QRectF> coverage) {
  if (!m_sim || !m.valid)
    return;
  const vsim::WorldConfig &w = m_worldEditor->config();
  WorldShipReq req;
  req.mesh = std::make_shared<const vsim::LoadedMesh>(std::move(m));
  req.doubleSided = w.worldMeshDoubleSided;
  req.lift = liftAfter;
  req.coverage = coverage;
  // The pads drawn on a procedural arena are solid, as they are on the endless
  // terrain; without them the engine only has the ground under the deck and the
  // vehicle sinks through the pad it was spawned on. The call is unconditional:
  // imported worlds have no pads (no terrain height sampler), so it adds nothing.
  appendHelipadCollision(req.extra);
  enqueueShip(std::move(req));
}

void SimulatorWidget::enqueueShip(WorldShipReq req) {
  // A newer request supersedes everything older: bump the generation so a
  // running build aborts at its next checkpoint and its result is discarded.
  m_shipGen->fetch_add(1);
  if (m_shipWatcher) { // one worker at a time; the latest request waits its turn
    if (m_shipPending)
      req.lift = req.lift || m_shipPending->lift;
    m_shipPending = std::move(req);
    return;
  }
  startShipJob(std::move(req));
}

void SimulatorWidget::startShipJob(WorldShipReq req) {
  const int gen = m_shipGen->load();
  const auto genPtr = m_shipGen;
  const QString path = worldBlobPath();
  // Unique per job so a superseded build can never clobber the next one's file.
  const QString tmp = path + QStringLiteral(".") + QString::number(gen) +
                      QStringLiteral(".tmp");
  m_shipRunningTmp = tmp;
  m_shipRunningLift = req.lift;
  m_shipRunningCover = req.coverage;

  auto *watcher = new QFutureWatcher<WorldShipResult>(this);
  m_shipWatcher = watcher;
  connect(watcher, &QFutureWatcherBase::finished, this,
          [this, watcher, gen, path, tmp]() {
            onShipFinished(watcher->result(), gen, path, tmp);
            watcher->deleteLater();
          });
  // Everything heavy (flatten, BVH build, blob write) runs here; it touches only
  // its own captured data, so it is safe even if the widget goes away first.
  watcher->setFuture(QtConcurrent::run(
      [genPtr, gen, tmp, mesh = req.mesh, chunks = std::move(req.chunks),
        extra = std::move(req.extra), ds = req.doubleSided]() {
        WorldShipResult r;
        auto stale = [&] { return genPtr->load() != gen; };
        if (stale())
          return r;
        QElapsedTimer t;
        t.start();
        std::vector<float> verts;
        if (mesh) {
          verts.reserve(mesh->positions.size() * 3);
          for (const QVector3D &p : mesh->positions) {
            verts.push_back(p.x());
            verts.push_back(p.y());
            verts.push_back(p.z());
          }
        } else {
          std::size_t n = extra.size();
          for (const auto &c : chunks)
            n += c->positions.size();
          verts.reserve(n * 3);
          for (const auto &c : chunks)
            for (const vsim::procgen::PgVec3 &p : c->positions) {
              verts.push_back(p.x);
              verts.push_back(p.y);
              verts.push_back(p.z);
            }
        }
        for (const QVector3D &p : extra) {
          verts.push_back(p.x());
          verts.push_back(p.y());
          verts.push_back(p.z());
        }
        const uint32_t nverts = static_cast<uint32_t>(verts.size() / 3);
        std::vector<uint8_t> blob =
            vsim::buildWorldBvh(verts.data(), nverts, ds, r.nodes);
        r.buildMs = t.nsecsElapsed() / 1.0e6;
        if (blob.empty()) {
          r.err = QStringLiteral("world mesh BVH build failed");
          return r;
        }
        if (stale())
          return r;
        QFile f(tmp);
        if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
          r.err = QStringLiteral("world mesh: cannot write %1").arg(tmp);
          return r;
        }
        const qint64 wrote = f.write(reinterpret_cast<const char *>(blob.data()),
                                     static_cast<qint64>(blob.size()));
        f.close();
        if (wrote != static_cast<qint64>(blob.size())) {
          QFile::remove(tmp);
          r.err = QStringLiteral("world mesh: short write to %1").arg(tmp);
          return r;
        }
        r.ok = true;
        r.nverts = nverts;
        r.bytes = static_cast<qint64>(blob.size());
        r.totalMs = t.nsecsElapsed() / 1.0e6;
        return r;
      }));
}

void SimulatorWidget::onShipFinished(const WorldShipResult &r, int gen,
                                     const QString &path, const QString &tmp) {
  const bool lift = m_shipRunningLift;
  m_shipWatcher = nullptr;
  m_shipRunningTmp.clear();
  const bool stale = gen != m_shipGen->load();

  if (!stale && m_sim && !r.err.isEmpty())
    appendLog("world", r.err);

  if (!stale && m_sim && r.ok) {
    QElapsedTimer t; // the only main-thread cost left: rename + one ctl send
    t.start();
    // Publish atomically: rename() over the target so the daemon never mmaps a
    // half-written blob.
#ifdef _WIN32
    // rename() will not replace an existing file on Windows; MoveFileEx will,
    // and atomically on one volume -- which a temp file beside it is.
    const bool moved = MoveFileExW(reinterpret_cast<LPCWSTR>(tmp.utf16()),
                                   reinterpret_cast<LPCWSTR>(path.utf16()),
                                   MOVEFILE_REPLACE_EXISTING);
#else
    const bool moved = ::rename(tmp.toLocal8Bit().constData(),
                                path.toLocal8Bit().constData()) == 0;
#endif
    if (!moved) {
      appendLog("world", tr("world mesh: rename to %1 failed").arg(path));
      QFile::remove(tmp);
    } else {
      const vsim::WorldConfig &w = m_worldEditor->config();
      const uint32_t ntris = r.nverts / 3;
      m_sim->sendWorldMesh(path, r.nverts, ntris, r.nodes,
                           w.worldMeshRestitution, w.worldMeshDoubleSided);
      appendLog("world",
                tr("world mesh → daemon: %1 tris, %2 nodes (%3 KiB); built in "
                   "%4 ms off-thread, %5 ms on the UI thread")
                    .arg(ntris)
                    .arg(r.nodes)
                    .arg(r.bytes / 1024)
                    .arg(r.totalMs, 0, 'f', 1)
                    .arg(t.nsecsElapsed() / 1.0e6, 0, 'f', 2));
      // The collision under the drone is now really in the engine, so it is safe
      // to re-drop a drone that fell through while it was being built.
      // Only if the drone is actually over what shipped (a drone that outran the
      // window is picked up by the next recentre's ship instead).
      if (m_spawnResetPending) {
        // Start's reset: collision is in, place the drone (this supersedes the lift).
        m_spawnResetPending = false;
        sendSpawnReset();
      } else if (lift && (!m_shipRunningCover ||
                   m_shipRunningCover->contains(m_lastDronePos.x(),
                                                m_lastDronePos.y())))
        liftDroneToSurface();
    }
  } else {
    QFile::remove(tmp); // stale, sim gone, or failed
  }

  // Start the newest queued request, if a newer one arrived mid-build. A lift
  // owed by a superseded request is inherited (liftDroneToSurface self-guards).
  if (m_shipPending) {
    WorldShipReq next = std::move(*m_shipPending);
    m_shipPending.reset();
    if (stale)
      next.lift = next.lift || lift;
    startShipJob(std::move(next));
  } else if (m_spawnResetPending && m_sim && !m_collisionPending) {
    // The collision Start was waiting for was dropped or failed and nothing
    // newer is coming: reset anyway rather than leave the stale state.
    m_spawnResetPending = false;
    sendSpawnReset();
  }
}

void SimulatorWidget::pushRatesToSim() {
  if (!m_sim)
    return;
  QSettings s;
  m_sim->sendRates(s.value(kImuHzKey, kDefImuHz).toInt(),
                   s.value(kPhysHzKey, kDefPhysHz).toInt(),
                   s.value(kPoseHzKey, kDefPoseHz).toInt());
}

void SimulatorWidget::hudSetStatus(const QString &s) {
  if (m_hud)
    m_hud->setStatus(s);
}

void SimulatorWidget::setPropAudioDefault(bool on) {
  // Drive the checkbox (its toggled handler enables PropAudio + persists the
  // per-sim key); no-op if it already matches so we don't thrash the device.
  if (m_propAudioChk && m_propAudioChk->isChecked() != on)
    m_propAudioChk->setChecked(on);
}

void SimulatorWidget::setRigControlsEnabled(bool simRunning) {
  // The rig poses the in-app sim (m_sim); with no sim running there's nothing
  // to pose. Disable + uncheck and say so, rather than silently no-op.
  if (m_rigEnable) {
    m_rigEnable->setEnabled(simRunning);
    if (!simRunning) {
      QSignalBlocker block(m_rigEnable);
      m_rigEnable->setChecked(false);
    }
  }
  const bool slidersOn = simRunning && m_rigEnable && m_rigEnable->isChecked();
  for (QSlider *s : {m_rigRoll, m_rigPitch, m_rigYaw})
    if (s)
      s->setEnabled(slidersOn);
  if (m_rigReadout)
    m_rigReadout->setText(simRunning ? tr("rig: off")
                                     : tr("rig: start the simulator to pose"));
}

void SimulatorWidget::applyRigPose() {
  if (!m_rigEnable || !m_rigEnable->isChecked())
    return;
  const int r = m_rigRoll->value(), p = m_rigPitch->value(),
            y = m_rigYaw->value();
  // Always reflect the slider values so dragging is visibly registering, even
  // if the sim isn't connected — that tells us whether the issue is the sliders
  // or the channel.
  if (!m_sim) {
    if (m_rigReadout)
      m_rigReadout->setText(
          tr("rig: r%1 p%2 y%3 — sim not running, start it first")
              .arg(r)
              .arg(p)
              .arg(y));
    return;
  }
  m_sim->sendRigPose(static_cast<float>(r), static_cast<float>(p),
                     static_cast<float>(y));
  if (m_rigReadout)
    m_rigReadout->setText(
        tr("rig: roll %1°  pitch %2°  yaw %3°  (sent)").arg(r).arg(p).arg(y));
}

void SimulatorWidget::setFlightModeStatus(quint8 mode, quint8 source) {
  const bool acro = (mode == kFmAcro);
  const bool gcs = (source == kFmSrcGcs);
  if (m_acroChk && m_acroChk->isChecked() != acro) {
    // Reflect firmware state without re-issuing the command (e.g. an RC switch
    // flip while no override is active).
    QSignalBlocker block(m_acroChk);
    m_acroChk->setChecked(acro);
  }
  const QString modeStr = QString("%1 (%2)").arg(
      acro ? tr("ACRO") : tr("STABILISE"), gcs ? tr("GCS") : tr("RC"));
  if (m_flightModeLabel)
    m_flightModeLabel->setText(tr("mode: ") + modeStr);
  if (m_hud)
    m_hud->setFlightMode(modeStr); // show it in the viewport HUD too
}

void SimulatorWidget::hudSetImu(const float acc[3], const float gyr[3]) {
  if (m_hud)
    m_hud->setImu(acc, gyr);
}

void SimulatorWidget::pushRcMapping(int func) {
  if (func < 0 || func >= 5 || !m_rcAxisCombo[func])
    return;
  const int axis = m_rcAxisCombo[func]->currentData().toInt();
  const bool inv = m_rcInvert[func]->isChecked();
  if (m_rc)
    m_rc->setMapping(func, axis, inv);
  QSettings s;
  s.setValue(QString(kRcMapAxisKey) + QString::number(func), axis);
  s.setValue(QString(kRcMapInvKey) + QString::number(func), inv);
}

void SimulatorWidget::applyRcSource() {
  if (!m_rcSource)
    return;
  const bool uart = m_rcSource->currentData().toInt() == RcBridge::Uart;
  QSettings st;
  st.setValue(kRcSourceKey, uart ? RcBridge::Uart : RcBridge::Joystick);

  // Swap the device field to the saved path for the selected source.
  if (m_rcPath) {
    const QString path =
        uart ? st.value(kRcUartPathKey, kRcUartPathDefault).toString()
             : st.value(kRcPathKey, kRcPathDefault).toString();
    QSignalBlocker block(m_rcPath);
    m_rcPath->clear();
    if (uart) {
      // Enumerate serial ports so the user can pick instead of typing.
      for (const QSerialPortInfo &info : QSerialPortInfo::availablePorts()) {
        const QString desc = info.description();
        m_rcPath->addItem(desc.isEmpty() ? info.portName()
                                         : QStringLiteral("%1 — %2").arg(
                                               info.portName(), desc),
                          info.systemLocation());
      }
    }
    m_rcPath->setEditText(path);
    m_rcPath->setToolTip(
        uart ? tr("Serial device streaming CSV RC frames "
                  "(roll,pitch,throttle,yaw,arm,…), µs per channel.")
             : tr("Joystick device (Linux js API)."));
  }
  if (m_rcBaud)
    m_rcBaud->setEnabled(uart);

  // The explicit Connect button only applies to the UART source. Switching to
  // the joystick (or away from UART) drops any held serial port.
  if (!uart && m_rcUartConnected)
    setRcUartConnected(false);
  if (m_rcConnect)
    m_rcConnect->setEnabled(uart);

  // Axis mapping only applies to a joystick; a CSV stream is already
  // channelised, so grey the mapping out under UART.
  for (int f = 0; f < 5; ++f) {
    if (m_rcAxisCombo[f])
      m_rcAxisCombo[f]->setEnabled(!uart);
    if (m_rcInvert[f])
      m_rcInvert[f]->setEnabled(!uart);
  }

  // Push to the bridge (no-op until it exists; the ctor calls this again).
  if (m_rc) {
    if (uart) {
      if (m_rcPath)
        m_rc->setUartPath(m_rcPath->currentText());
      if (m_rcBaud)
        m_rc->setUartBaud(m_rcBaud->currentData().toInt());
    } else if (m_rcPath) {
      m_rc->setJoystickPath(m_rcPath->currentText());
    }
    m_rc->setSource(uart ? RcBridge::Uart : RcBridge::Joystick);
  }
}

// Persist the current device-field text for the active source and push it to
// the bridge. Shared by the editable combo's pick + edit-finished signals.
void SimulatorWidget::commitRcPath() {
  if (!m_rcPath || !m_rcSource)
    return;
  const bool uart = m_rcSource->currentData().toInt() == RcBridge::Uart;
  const QString path = m_rcPath->currentText();
  QSettings().setValue(uart ? kRcUartPathKey : kRcPathKey, path);
  if (m_rc) {
    if (uart)
      m_rc->setUartPath(path);
    else
      m_rc->setJoystickPath(path);
  }
}

void SimulatorWidget::setRcUartConnected(bool on) {
  m_rcUartConnected = on;
  const QString path = m_rcPath ? m_rcPath->currentText() : QString();
  if (m_rc) {
    if (on) {
      // Claim the port first — this revokes it from the board telemetry (or any
      // other holder), which disconnects in response, so only one owner ever
      // has the tty open.
      PortArbiter::instance().acquire(path, this);
      m_rc->setUartPath(path);
      m_rc->setUartConnected(true);
    } else {
      m_rc->setUartConnected(false);
      PortArbiter::instance().release(path, this);
    }
  }
  if (m_rcConnect) {
    QSignalBlocker b(m_rcConnect);
    m_rcConnect->setChecked(on);
    m_rcConnect->setText(on ? tr("Disconnect") : tr("Connect"));
  }
}

// The rail is only worth its width while at least one pane is showing.
void SimulatorWidget::syncPipRail() {
  if (!m_pipRail)
    return;
  bool any = false;
  for (QWidget *p : {m_horizonPip, m_downPip, m_minimapPip})
    any = any || (p && !p->isHidden());
  m_pipRail->setVisible(any);
}

QImage SimulatorWidget::grabSimView() {
  return m_renderer ? m_renderer->grabFrame() : QImage();
}

// ----------------------------------------------------------------------------
// Sim lifecycle
// ----------------------------------------------------------------------------

static const char *kStartTip = QT_TRANSLATE_NOOP(
    "SimulatorWidget", "Boot firmware (first time) + start the sim");

void SimulatorWidget::blockStartUntilRestart(const QString &reason) {
  m_startGate.block(reason);
  applyStartBlock();
}

void SimulatorWidget::applyStartBlock() {
  if (!m_startGate.blocked() || !m_simStartBtn)
    return;
  m_startGate.apply(m_simStartBtn, false, tr(kStartTip));
  if (m_simStatusLabel && !m_sim) {
    m_simStatusLabel->setText(tr("● Stopped: %1").arg(m_startGate.reason()));
    m_simStatusLabel->setStyleSheet(
        QString("color: %1;").arg(Theme::hex(Theme::kTextMuted)));
  }
}

void SimulatorWidget::updateCustomWarning(bool atStart) {
  if (!m_customWarnLabel)
    return;
  const bool custom = !vsim::findAirframePreset(m_presetId);
  m_customWarnLabel->setVisible(atStart && m_customTune.shouldWarn(custom));
}

void SimulatorWidget::startInAppSim() {
  if (m_sim || startBlocked())
    return;
  updateCustomWarning(true); // informational only; never blocks Start

  // Per-instance path isolation: publish a suffix for this Navigator process in
  // the environment BEFORE the engine boots (the firmware shim caches its
  // pty/advert paths on first use), so a second Navigator can't collide on the
  // UART2 advert (/tmp/vayu_uart2_pty<suffix>). Respect an externally-set value.
  if (qEnvironmentVariableIsEmpty("VSIM_FIFO_SUFFIX")) {
    qputenv("VSIM_FIFO_SUFFIX",
            QByteArrayLiteral("_nav") + QByteArray::number(::getpid()));
  }

  // Open a fresh per-run log file before the firmware starts emitting.
  openNewLogFile();

  // The SINGLE in-process RTOS engine (libvayu_sitl, loaded at runtime) runs
  // BOTH the firmware and the vsim physics in one deterministic stepper — no
  // external daemon, no FIFO. SimWorker owns that engine on its worker thread
  // and boots the firmware once (idempotent across Stop/Re-Start). Telemetry
  // rides the module's callback; RC arrives via the serial feeder reading the
  // RcBridge pty.
  m_sitlStarted = true;

  // (Re)attach the firmware -> GCS telemetry sink BEFORE the worker boots, so
  // IMU / attitude / heartbeat flow to the home-screen panels from frame one.
  // Detached on Stop (the firmware can't truly stop) to silence the LIVE
  // blinker.
  if (SitlModule::instance().available())
    SitlModule::instance().api()->set_telemetry_sink(
        &uart2_to_widget_trampoline, this);

  {
    QSettings st;
    m_runSeed = vsim::SimWorker::resolveResetSeed(
        st.value(kSeedRandomKey, false).toBool(),
        static_cast<uint32_t>(st.value(kSeedKey, kDefaultResetSeed).toInt()),
        QRandomGenerator::global()->generate());
    appendLog("sim", tr("noise seed %1").arg(m_runSeed));
  }
  m_spawnResetPending = false;
  m_sim = new vsim::SimWorker(this);
  m_sim->setTelemetrySink(&uart2_to_widget_trampoline, this);
  connect(m_sim, &vsim::SimWorker::stoppedCleanly, this,
          &SimulatorWidget::onSimWorkerExited);
  connect(m_sim, &vsim::SimWorker::poseUpdated, m_renderer,
          &vsim::SimRendererWidget::setSnapshot);
  if (m_downRenderer)
    connect(m_sim, &vsim::SimWorker::poseUpdated, m_downRenderer,
            &vsim::SimRendererWidget::setSnapshot);
  // Record physics ground truth for this run (gt-*.bin).
  connect(m_sim, &vsim::SimWorker::poseUpdated, this,
          &SimulatorWidget::logGroundTruth);
  connect(m_sim, &vsim::SimWorker::poseUpdated, this,
          [this](vsim::SimSnapshot snap) {
            float pitch, yaw, roll;
            vsim::quatToEulerNED(snap.att, &roll, &pitch, &yaw);
            // Fixed field widths so a leading '-' (or "-0.0") doesn't widen the
            // string and resize the label every frame — that caused the flicker.
            // Monospace + space-padding keeps each number a constant pixel width.
            // Position is a length (NED metres) → convert via the altitude/
            // length unit; rpy via the angle unit. Decimals from Settings ▸
            // Units & Display; field widths stay fixed to avoid label reflow.
            const int dec = Units::decimals();
            m_simPoseLabel->setText(
                QString("pos=(%1, %2, %3) %7   rpy=(%4, %5, %6)%8")
                    .arg(Units::toAltitude(snap.pos_w.x()), 7, 'f', dec)
                    .arg(Units::toAltitude(snap.pos_w.y()), 7, 'f', dec)
                    .arg(Units::toAltitude(snap.pos_w.z()), 7, 'f', dec)
                    .arg(Units::toAngle(roll), 6, 'f', dec)
                    .arg(Units::toAngle(pitch), 6, 'f', dec)
                    .arg(Units::toAngle(yaw), 6, 'f', dec)
                    .arg(Units::altSuffix(), Units::angleSuffix()));
            updateHud(snap);
            m_propAudio.setMotors(snap.motor_omega);
          });
  connect(m_sim, &vsim::SimWorker::logLine, this,
          [this](const QString &s) { appendLog("vsim", s); });
  // Once the daemon's FIFOs are up, push the configured airframe + world
  // so the sim flies the edited params from frame one.
  connect(m_sim, &vsim::SimWorker::online, this, [this] {
    if (!m_sim)
      return;
    const auto cfg = m_geomEditor->physicsConfig();
    pushRatesToSim(); // apply configured loop rates before geometry/world
    // Firmware roll/pitch/yaw mix signs (physics frame) and the flight-mode
    // release. Both are commands, so the firmware drops them until the time
    // sync completes: armed, sent once synced, re-sent until ACKed.
    armMixPush(cfg);
    armStartCommand(
        QStringLiteral("flightmode"), NAVLINK_MSGID_CMD_SET_FLIGHT_MODE, 1,
        tr("flight-mode release"),
        tr("the RC switch may not control the mode"),
        [] { SitlModule::instance().send(CommandCodec::encodeSetFlightMode(2)); });
    pushAirframeGains(true); // the airframe's tuned gains (if a preset)
    m_sim->sendGeometry(cfg); // in-process physics motor layout
    m_sim->sendWorld(m_worldEditor->config());
    m_sim->sendObstacles(m_worldEditor->config().obstacles);
    m_sim->sendWind(m_worldEditor->windConfig()); // restore wind across restart
    pushNoise();  // apply the configured sensor models (σ / enable)
    pushFaults(); // re-assert any latched faults across the restart
    loadWorldMeshToRenderer(); // re-loads + ships the collision BVH now m_sim exists
    // Clean start: reset at the spawn pose with the run's seed, wiping whatever
    // a previous Stop left in the engine. Held until the collision under the
    // spawn has shipped (built off-thread) so the drone is never placed over
    // terrain the engine does not have; with no world to collide with, now.
    if (spawnResetWaitsForCollision())
      m_spawnResetPending = true;
    else
      sendSpawnReset();
    // Restart any armed training course from the first gate, flying from the floor.
    if (m_training.active()) {
      m_training.resetProgress();
      pushTrainingGates();
      updateTrainingProgress();
    }
    appendLog("geom",
              tr("firmware roll-mix %1 (default -+ ; mismatch = inverted "
                 "roll). pushed to firmware + physics.")
                  .arg(rollMixString(cfg)));
    // Belt-and-suspenders: re-assert the physics layout
    // shortly after start. If a restart race left one side on its default while
    // the other had the real (possibly Y-mirrored) geometry, roll inverts and
    // the craft topples at lift-off; this guarantees they agree before arming.
    QTimer::singleShot(800, this, [this] {
      if (!m_sim)
        return;
      // (The firmware mix is no longer re-asserted blindly here: it is retried
      // until the firmware ACKs it, see armMixPush.)
      const auto c = m_geomEditor->physicsConfig();
      m_sim->sendGeometry(c);
      pushRatesToSim(); // re-assert loop rates too: the daemon boots at its
                        // compiled default, so a dropped/raced SET_RATES would
                        // otherwise leave it there (e.g. physics stuck at 8k).
    });
  });
  m_sim->start(QThread::TimeCriticalPriority);

  m_simStartBtn->setEnabled(false);
  m_simStopBtn->setEnabled(true);
  if (m_simResetBtn)
    m_simResetBtn->setEnabled(true);
  if (m_fpvCheck)
    m_fpvCheck->setEnabled(true); // FPV needs the drone-attached camera
  m_simStatusLabel->setText(tr("● Running"));
  m_simStatusLabel->setStyleSheet(
      QString("color: %1;").arg(Theme::hex(Theme::kOk)));
  // Vehicle configuration is locked while simulating; force World mode.
  if (m_vehicleTab)
    m_vehicleTab->setEnabled(false);
  setMode(1);
  // Show the telemetry HUD over the viewport (painted by the renderer).
  m_renderer->setHudVisible(true);
  setRigControlsEnabled(true); // rig pose is now available
  emit simRunningChanged(true);
}

void SimulatorWidget::attachExternalSim() {
  if (m_sim)
    return;
  // Mirror an EXTERNAL sim's pose stream (default /tmp/vsim_pose) — e.g. a
  // headless vayu_sitl_rtos run driving the real firmware. We boot no in-process
  // engine; this view only renders the external pose stream over the
  // already-loaded vehicle + world meshes. Telemetry (dashboards) still comes
  // via the normal serial connection (the harness's --gcs bridge).
  m_attached = true;
  m_sim = new vsim::SimWorker(this);
  connect(m_sim, &vsim::SimWorker::stoppedCleanly, this,
          &SimulatorWidget::onSimWorkerExited);
  connect(m_sim, &vsim::SimWorker::poseUpdated, m_renderer,
          &vsim::SimRendererWidget::setSnapshot);
  if (m_downRenderer)
    connect(m_sim, &vsim::SimWorker::poseUpdated, m_downRenderer,
            &vsim::SimRendererWidget::setSnapshot);
  connect(m_sim, &vsim::SimWorker::poseUpdated, this,
          [this](vsim::SimSnapshot snap) {
            const int dec = Units::decimals();
            float roll, pitch, yaw;
            vsim::quatToEulerNED(snap.att, &roll, &pitch, &yaw);
            if (m_simPoseLabel)
              m_simPoseLabel->setText(
                  QString("pos=(%1, %2, %3) %7   rpy=(%4, %5, %6)%8")
                      .arg(Units::toAltitude(snap.pos_w.x()), 7, 'f', dec)
                      .arg(Units::toAltitude(snap.pos_w.y()), 7, 'f', dec)
                      .arg(Units::toAltitude(snap.pos_w.z()), 7, 'f', dec)
                      .arg(Units::toAngle(roll), 6, 'f', dec)
                      .arg(Units::toAngle(pitch), 6, 'f', dec)
                      .arg(Units::toAngle(yaw), 6, 'f', dec)
                      .arg(Units::altSuffix(), Units::angleSuffix()));
            updateHud(snap);
            m_propAudio.setMotors(snap.motor_omega);
          });
  connect(m_sim, &vsim::SimWorker::logLine, this,
          [this](const QString &s) { appendLog("attach", s); });
  m_sim->startAttach(QStringLiteral("/tmp/vsim_pose"));

  m_simStartBtn->setEnabled(false);
  m_simAttachBtn->setEnabled(false);
  m_simStopBtn->setEnabled(true);
  if (m_fpvCheck)
    m_fpvCheck->setEnabled(true);
  m_simStatusLabel->setText(tr("● Attached (external sim)"));
  m_simStatusLabel->setStyleSheet(
      QString("color: %1;").arg(Theme::hex(Theme::kOk)));
  if (m_vehicleTab)
    m_vehicleTab->setEnabled(false);
  setMode(1);
  m_renderer->setHudVisible(true);
}

void SimulatorWidget::stopInAppSim() {
  if (!m_sim)
    return;
  // Detach the telemetry sink first: the firmware threads keep running
  // (vayu_sitl is one-shot), so without this they'd keep streaming heartbeats
  // and IMU to the home screen after Stop — the LIVE blinker would never stop.
  // Dropping the sink makes the display go quiet, consistent with Stopped.
  // Detach, not shutdown(): those threads still hold the plumbing.
  if (SitlModule::instance().available())
    SitlModule::instance().api()->set_telemetry_sink(nullptr, nullptr);
  cancelWorldShip(); // a build in flight must not ship into a stopped sim
  m_spawnResetPending = false;
  m_sim->requestStop();
  if (!m_sim->wait(2000)) {
    m_sim->terminate();
    m_sim->wait(1000);
  }
  m_sim->deleteLater();
  m_sim = nullptr;
  m_retrier.cancelAll(); // nothing left to ACK
  if (m_retryTimer)
    m_retryTimer->stop();
  m_propAudio.setMotors({0.0f, 0.0f, 0.0f, 0.0f}); // silence once stopped
  m_simStartBtn->setEnabled(!startBlocked());
  if (m_simAttachBtn)
    m_simAttachBtn->setEnabled(true);
  m_attached = false;
  m_simStopBtn->setEnabled(false);
  if (m_simResetBtn)
    m_simResetBtn->setEnabled(false);
  if (m_fpvCheck) { // FPV is drone-only; back to free-roam
    QSignalBlocker block(m_fpvCheck);
    m_fpvCheck->setChecked(false);
    m_fpvCheck->setEnabled(false);
  }
  if (m_renderer)
    m_renderer->setFpv(false);
  m_simStatusLabel->setText(tr("● Stopped (firmware idle)"));
  m_simStatusLabel->setStyleSheet(
      QString("color: %1;").arg(Theme::hex(Theme::kTextMuted)));
  applyStartBlock(); // status text + tooltip, if Start is blocked
  // Vehicle config is editable again; stay in World until the user
  // switches back (clicking the Vehicle tab re-enables motor gizmos).
  if (m_vehicleTab)
    m_vehicleTab->setEnabled(true);
  // Re-apply the current mode now that m_sim is null: re-enables World-mode
  // free-roam + obstacle gizmos (the running sim had locked the camera).
  if (m_rightStack)
    setMode(m_rightStack->currentIndex());
  if (m_renderer)
    m_renderer->setHudVisible(false);
  setRigControlsEnabled(false); // no sim to pose anymore
  // Close the per-run log file so its trailing bytes flush to disk.
  closeLogFile();
  // Note: we don't call vayu_sitl_stop() here on the Stop button.
  // The firmware threads stay alive but receive no fresh IMU samples
  // (SimWorker isn't pushing). Restarting the sim resumes the pipe.
  emit simRunningChanged(false);
}

void SimulatorWidget::onSimWorkerExited() {
  // The worker emitted stoppedCleanly (spawn failure, startup-grace timeout,
  // or the daemon died) without the user clicking Stop. If we haven't already
  // torn down (explicit Stop nulls m_sim before the queued signal arrives),
  // run the normal stop path so the buttons / status reflect the dead worker.
  if (!m_sim)
    return;
  appendLog("vsim", "[worker exited — resetting to Stopped]");
  stopInAppSim();
}

// ----------------------------------------------------------------------------
// Geometry editor <-> renderer / daemon / persistence
// ----------------------------------------------------------------------------

void SimulatorWidget::applyGeometryToRenderer() {
  if (!m_renderer || !m_geomEditor)
    return;
  // CoM frame: mesh is already recentered, motor arms are CoM-relative,
  // and the CoM sits at the body origin.
  const auto cfg = m_geomEditor->physicsConfig();
  if (m_geomEditor->hasMesh()) {
    m_renderer->setDroneMesh(m_geomEditor->meshPositions(),
                             m_geomEditor->meshNormals());
  }
  m_renderer->setComMarker(QVector3D(0, 0, 0)); // CoM == body origin now
  std::array<QVector3D, 4> pos, axis;
  std::array<int, 4> spin;
  for (int i = 0; i < 4; ++i) {
    pos[i] = cfg.motors[i].pos;
    axis[i] = cfg.motors[i].axis;
    spin[i] = cfg.motors[i].spin;
  }
  m_renderer->setMotorLayout(pos, axis, spin);
  m_renderer->setAirframeStyle(m_presetId, cfg.propRadius);

  // Mirror the airframe into the down-cam PiP so its bird's-eye shows the drone.
  if (m_downRenderer) {
    if (m_geomEditor->hasMesh())
      m_downRenderer->setDroneMesh(m_geomEditor->meshPositions(),
                                   m_geomEditor->meshNormals());
    m_downRenderer->setMotorLayout(pos, axis, spin);
    m_downRenderer->setAirframeStyle(m_presetId, cfg.propRadius);
  }
}

// Push the current airframe's gains and, when `announce`, say so in the log:
// the push can overwrite a tune the firmware loaded at boot, so the user should
// be able to see why their tune changed, or that it was deliberately left alone. With the
// sim running the push is a Start command (see armStartCommand): sent once the
// clock is synced and re-sent until the firmware ACKs all of the preset's
// CMD_SET_PID slots.
void SimulatorWidget::pushAirframeGains(bool announce) {
  const vsim::AirframePreset *p = vsim::findAirframePreset(m_presetId);
  if (!p) {
    m_retrier.cancel(QStringLiteral("gains")); // custom: nothing to wait for
  } else if (m_sim) {
    const QString id = m_presetId;
    armStartCommand(QStringLiteral("gains"), NAVLINK_MSGID_CMD_SET_PID,
                    int(p->gains.size()), tr("%1 gains").arg(id),
                    tr("the simulator may fly stock gains"),
                    [id] { pushPresetGains(id); });
  } else {
    pushPresetGains(m_presetId);
  }
  if (!announce)
    return;
  appendLog("geom",
            m_presetId.isEmpty()
                ? tr("custom vehicle: firmware keeps its current gains")
                : tr("sent %1 gains to firmware (waiting for ACK)")
                      .arg(m_presetId));
}

void SimulatorWidget::armStartCommand(const QString &key, quint32 msgid,
                                      int expected, const QString &label,
                                      const QString &consequence,
                                      CommandRetrier::Send send) {
  if (!m_retryClock.isValid())
    m_retryClock.start();
  if (!m_retryTimer) {
    m_retrier.setLog([this](const QString &s) { appendLog("geom", s); });
    m_retryTimer = new QTimer(this);
    m_retryTimer->setInterval(int(PendingCommand::kRetryMs));
    connect(m_retryTimer, &QTimer::timeout, this, [this] { retryTick(); });
  }
  if (m_retrier.arm(key, msgid, expected, label, consequence, std::move(send),
                    m_retryClock.elapsed()))
    m_retryTimer->start();
}

void SimulatorWidget::armMixPush(const vsim::GeometryConfig &g) {
  armStartCommand(QStringLiteral("mix"), NAVLINK_MSGID_CMD_SET_MOTOR_GEOMETRY, 1,
                  tr("firmware motor mix (roll-mix %1)").arg(rollMixString(g)),
                  tr("the simulator may fly the default mix and invert roll"),
                  [g] { pushFirmwareMotorGeometry(g); });
}

void SimulatorWidget::retryTick() {
  if (!m_sim) { // sim stopped under us
    m_retrier.cancelAll();
    m_retryTimer->stop();
    return;
  }
  m_retrier.tick(m_retryClock.elapsed());
  if (!m_retrier.anyPending())
    m_retryTimer->stop();
}

void SimulatorWidget::setTimeSynced(bool synced) {
  if (!m_retryClock.isValid())
    m_retryClock.start();
  m_retrier.setSynced(synced, m_retryClock.elapsed());
}

void SimulatorWidget::onCommandAck(quint32 command, quint8 reqSeq,
                                   quint8 result) {
  Q_UNUSED(reqSeq); // acks are counted per round, not matched per frame
  m_retrier.onAck(command, result);
  if (m_retryTimer && !m_retrier.anyPending())
    m_retryTimer->stop();
}

void SimulatorWidget::persistGeometry(const vsim::GeometryConfig &g) {
  QSettings s;
  s.beginGroup(kGeomGroup);
  s.setValue("meshPath", g.meshPath);
  s.setValue("scale", g.scale);
  s.setValue("mass", g.mass);
  s.setValue("propRadius", g.propRadius);
  s.setValue("tx", g.translate.x());
  s.setValue("ty", g.translate.y());
  s.setValue("tz", g.translate.z());
  s.setValue("rx", g.rotate.x());
  s.setValue("ry", g.rotate.y());
  s.setValue("rz", g.rotate.z());
  s.setValue("comX", g.com.x());
  s.setValue("comY", g.com.y());
  s.setValue("comZ", g.com.z());
  for (int i = 0; i < 9; ++i)
    s.setValue(QString("I%1").arg(i), g.inertia[i]);
  for (int i = 0; i < 4; ++i) {
    const auto &m = g.motors[i];
    const QString p = QString("m%1_").arg(i);
    s.setValue(p + "px", m.pos.x());
    s.setValue(p + "py", m.pos.y());
    s.setValue(p + "pz", m.pos.z());
    s.setValue(p + "ax", m.axis.x());
    s.setValue(p + "ay", m.axis.y());
    s.setValue(p + "az", m.axis.z());
    s.setValue(p + "spin", m.spin);
    s.setValue(p + "kt", m.k_thrust);
    s.setValue(p + "km", m.k_moment);
    s.setValue(p + "wmax", m.max_omega);
    s.setValue(p + "tau", m.tau);
  }
  s.endGroup();
}

vsim::GeometryConfig SimulatorWidget::restoreGeometry() {
  vsim::GeometryConfig g; // sensible defaults if nothing was saved
  QSettings s;
  s.beginGroup(kGeomGroup);
  if (!s.contains("mass")) {
    s.endGroup();
    return g;
  }
  g.meshPath = s.value("meshPath").toString();
  g.scale = s.value("scale", g.scale).toFloat();
  g.mass = s.value("mass", g.mass).toFloat();
  g.propRadius = s.value("propRadius", g.propRadius).toFloat();
  g.translate = QVector3D(s.value("tx").toFloat(), s.value("ty").toFloat(),
                          s.value("tz").toFloat());
  g.rotate = QVector3D(s.value("rx").toFloat(), s.value("ry").toFloat(),
                       s.value("rz").toFloat());
  g.com = QVector3D(s.value("comX").toFloat(), s.value("comY").toFloat(),
                    s.value("comZ").toFloat());
  for (int i = 0; i < 9; ++i)
    g.inertia[i] = s.value(QString("I%1").arg(i), g.inertia[i]).toFloat();
  for (int i = 0; i < 4; ++i) {
    auto &m = g.motors[i];
    const QString p = QString("m%1_").arg(i);
    m.pos = QVector3D(s.value(p + "px", m.pos.x()).toFloat(),
                      s.value(p + "py", m.pos.y()).toFloat(),
                      s.value(p + "pz", m.pos.z()).toFloat());
    m.axis = QVector3D(s.value(p + "ax", m.axis.x()).toFloat(),
                       s.value(p + "ay", m.axis.y()).toFloat(),
                       s.value(p + "az", m.axis.z()).toFloat());
    m.spin = s.value(p + "spin", m.spin).toInt();
    m.k_thrust = s.value(p + "kt", m.k_thrust).toFloat();
    m.k_moment = s.value(p + "km", m.k_moment).toFloat();
    m.max_omega = s.value(p + "wmax", m.max_omega).toFloat();
    m.tau = s.value(p + "tau", m.tau).toFloat();
  }
  s.endGroup();
  return g;
}

void SimulatorWidget::persistWorld(const vsim::WorldConfig &w) {
  QSettings s;
  s.beginGroup(kWorldGroup);
  s.setValue("gravity", w.gravity);
  s.setValue("ground_z", w.ground_z);
  s.setValue("restitution", w.restitution);
  s.setValue("linear_drag", w.linear_drag);
  s.setValue("angular_drag", w.angular_drag);
  s.setValue("ground_right_gain", w.ground_right_gain);
  s.setValue("ground_right_damp", w.ground_right_damp);
  s.setValue("worldMeshPath", w.worldMeshPath);
  s.setValue("worldScale", w.worldScale);
  s.setValue("worldUpAxis", w.worldUpAxis);
  s.setValue("worldMeshRestitution", w.worldMeshRestitution);
  s.setValue("worldMeshDoubleSided", w.worldMeshDoubleSided);
  s.setValue("worldMeshOffX", w.worldMeshOffset.x());
  s.setValue("worldMeshOffY", w.worldMeshOffset.y());
  s.setValue("worldMeshOffZ", w.worldMeshOffset.z());
  // World preset (procedural arena): the id + seed regenerate it exactly.
  s.setValue("worldPreset", w.worldPreset);
  s.setValue("worldPresetSeed", w.proceduralSeed);
  // Full procedural params (field, flora, look...): knob tweaks survive restart.
  s.setValue("proceduralJson", WorldEditorWidget::serializeProcedural(w));
  s.beginWriteArray("obstacles", w.obstacles.size());
  for (int i = 0; i < w.obstacles.size(); ++i) {
    const vsim::Obstacle &o = w.obstacles[i];
    s.setArrayIndex(i);
    s.setValue("type", o.type);
    s.setValue("px", o.pos.x());
    s.setValue("py", o.pos.y());
    s.setValue("pz", o.pos.z());
    s.setValue("sx", o.size.x());
    s.setValue("sy", o.size.y());
    s.setValue("sz", o.size.z());
    s.setValue("rx", o.rotate.x());
    s.setValue("ry", o.rotate.y());
    s.setValue("rz", o.rotate.z());
    s.setValue("rest", o.restitution);
  }
  s.endArray();
  s.endGroup();
}

vsim::WorldConfig SimulatorWidget::restoreWorld() {
  vsim::WorldConfig w; // defaults if nothing saved
  QSettings s;
  s.beginGroup(kWorldGroup);
  if (!s.contains("gravity")) {
    s.endGroup();
    return w;
  }
  w.gravity = s.value("gravity", w.gravity).toFloat();
  w.ground_z = s.value("ground_z", w.ground_z).toFloat();
  w.restitution = s.value("restitution", w.restitution).toFloat();
  w.linear_drag = s.value("linear_drag", w.linear_drag).toFloat();
  w.angular_drag = s.value("angular_drag", w.angular_drag).toFloat();
  w.ground_right_gain =
      s.value("ground_right_gain", w.ground_right_gain).toFloat();
  w.ground_right_damp =
      s.value("ground_right_damp", w.ground_right_damp).toFloat();
  w.worldMeshPath = s.value("worldMeshPath", w.worldMeshPath).toString();
  w.worldScale = s.value("worldScale", w.worldScale).toFloat();
  w.worldUpAxis = s.value("worldUpAxis", w.worldUpAxis).toInt();
  w.worldMeshRestitution =
      s.value("worldMeshRestitution", w.worldMeshRestitution).toFloat();
  w.worldMeshDoubleSided =
      s.value("worldMeshDoubleSided", w.worldMeshDoubleSided).toBool();
  w.worldMeshOffset =
      QVector3D(s.value("worldMeshOffX", w.worldMeshOffset.x()).toFloat(),
                s.value("worldMeshOffY", w.worldMeshOffset.y()).toFloat(),
                s.value("worldMeshOffZ", w.worldMeshOffset.z()).toFloat());
  const int n = s.beginReadArray("obstacles");
  w.obstacles.clear();
  for (int i = 0; i < n; ++i) {
    s.setArrayIndex(i);
    vsim::Obstacle o;
    o.type = s.value("type", 0).toInt();
    o.pos = QVector3D(s.value("px").toFloat(), s.value("py").toFloat(),
                      s.value("pz").toFloat());
    o.size =
        QVector3D(s.value("sx", 1.0).toFloat(), s.value("sy", 1.0).toFloat(),
                  s.value("sz", 1.0).toFloat());
    o.rotate = QVector3D(s.value("rx").toFloat(), s.value("ry").toFloat(),
                         s.value("rz").toFloat());
    o.restitution = s.value("rest", 0.3).toFloat();
    w.obstacles.push_back(o);
  }
  s.endArray();
  const QString preset = s.value("worldPreset").toString();
  const quint32 pseed = s.value("worldPresetSeed", w.proceduralSeed).toUInt();
  const QString procJson = s.value("proceduralJson").toString();
  s.endGroup();
  // Full params saved: restore them verbatim (the preset is NOT re-applied over
  // hand-tuned knobs). Older settings have only preset id + seed: regenerate
  // biome/field from those and keep the saved obstacle list (user edits).
  if (!procJson.isEmpty() &&
      WorldEditorWidget::deserializeProcedural(w, procJson)) {
    // restored verbatim
  } else if (!preset.isEmpty()) {
    const QVector<vsim::Obstacle> saved = w.obstacles;
    if (vsim::applyWorldPreset(w, preset, pseed)) {
      w.worldPreset = preset;
      w.obstacles = saved;
    }
  }
  return w;
}

void SimulatorWidget::persistWind(const vsim::WindConfig &w) {
  QSettings s;
  s.beginGroup(kWorldGroup);
  s.setValue("windN", w.steady.x());
  s.setValue("windE", w.steady.y());
  s.setValue("windD", w.steady.z());
  s.setValue("windGustAmp", w.gustAmp);
  s.setValue("windGustPeriod", w.gustPeriod);
  s.setValue("windTurbSigma", w.turbSigma);
  s.setValue("windTurbTau", w.turbTau);
  s.setValue("windEnabled", w.enabled);
  s.endGroup();
}

vsim::WindConfig SimulatorWidget::restoreWind() {
  vsim::WindConfig w; // defaults (disabled, zero) if nothing saved
  QSettings s;
  s.beginGroup(kWorldGroup);
  w.steady = QVector3D(s.value("windN", w.steady.x()).toFloat(),
                       s.value("windE", w.steady.y()).toFloat(),
                       s.value("windD", w.steady.z()).toFloat());
  w.gustAmp = s.value("windGustAmp", w.gustAmp).toFloat();
  w.gustPeriod = s.value("windGustPeriod", w.gustPeriod).toFloat();
  w.turbSigma = s.value("windTurbSigma", w.turbSigma).toFloat();
  w.turbTau = s.value("windTurbTau", w.turbTau).toFloat();
  w.enabled = s.value("windEnabled", w.enabled).toBool();
  s.endGroup();
  return w;
}

// Worker/firmware thread: append the just-written UART2 bytes to the coalescing
// buffer and, only if a drain isn't already queued, post ONE onUartBytes()
// event to the GUI thread. So no matter how many per-byte writes land between
// event-loop passes, the GUI processes them in a single batch.
void SimulatorWidget::queueUartBytes(const uint8_t *data, size_t n) {
  bool post = false;
  {
    std::lock_guard<std::mutex> lk(m_uartRxMtx);
    m_uartRxBuf.append(reinterpret_cast<const char *>(data),
                       static_cast<int>(n));
    if (!m_uartRxDrainPending) {
      m_uartRxDrainPending = true;
      post = true;
    }
  }
  if (post)
    QMetaObject::invokeMethod(this, "onUartBytes", Qt::QueuedConnection);
}

void SimulatorWidget::onUartBytes() {
  // GUI thread: take the whole accumulated batch and clear the pending flag so a
  // fresh producer post can be queued. Bytes appended after the swap simply
  // trigger the next drain — nothing is lost.
  QByteArray batch;
  {
    std::lock_guard<std::mutex> lk(m_uartRxMtx);
    batch.swap(m_uartRxBuf);
    m_uartRxDrainPending = false;
  }
  if (batch.isEmpty())
    return;

  // Forward the firmware's telemetry downlink to anything connected to
  // dataReceived (MainWindow's DroneProtocol parser typically). The FC view is
  // NOT logged here — that would just duplicate the Export; the per-run log
  // records the physics ground truth instead (see logGroundTruth).
  emit dataReceived(batch);
}

// Ground-truth record size: 2 x u64 (tick, t_us) + 27 x f32 (pose/vel/rates/
// rpy/motors/airspeed/batt). Keep in sync with logGroundTruth + parse_gt.py.
static constexpr int kGtRecordBytes = 2 * 8 + 27 * 4; // 124

void SimulatorWidget::openNewLogFile() {
  closeLogFile();
  QDir dir(m_logDir);
  if (!dir.exists() && !dir.mkpath(".")) {
    appendLog("log", QString("[mkdir failed: %1]").arg(m_logDir));
    return;
  }
  QString stamp = QDateTime::currentDateTime().toString("yyyy-MM-dd_HH-mm-ss");
  // gt- (ground truth), not sim-: this file holds the PHYSICS state, not the FC
  // telemetry. Legacy tools (tools/sim_log_*.py) still parse the old sim-*.bin
  // as FC NavLink, so a distinct name keeps them from mis-reading this format.
  QString path = dir.absoluteFilePath(QString("gt-%1.bin").arg(stamp));
  m_runLog = std::make_unique<QFile>(path);
  if (!m_runLog->open(QIODevice::WriteOnly)) {
    appendLog("log", QString("[open failed: %1]").arg(path));
    m_runLog.reset();
    return;
  }
  m_runLogBytes = 0;
  // Header (little-endian, 20 bytes): magic "VGT1", version, record size,
  // start wall-clock ms. Per-record layout documented in logGroundTruth /
  // firmware/docs/journal/log-analysis/parse_gt.py.
  {
    QDataStream hs(m_runLog.get());
    hs.setByteOrder(QDataStream::LittleEndian);
    hs.writeRawData("VGT1", 4);
    hs << quint32(1) << quint32(kGtRecordBytes)
       << quint64(QDateTime::currentMSecsSinceEpoch());
    m_runLogBytes += 20;
  }
  m_runClock.start(); // monotonic per-record timestamp base
  if (m_logPathLabel) {
    m_logPathLabel->setFullText(QString("Ground-truth log: %1").arg(path));
    m_logPathLabel->setStyleSheet(
        QString("color: %1; font-family: monospace; font-size: 11px;")
            .arg(Theme::hex(Theme::kOk)));
  }
  appendLog("log", QString("[opened %1 — physics ground truth]").arg(path));
}

// One ground-truth record per pose snapshot. Layout (little-endian, packed,
// kGtRecordBytes total) — kept in lock-step with parse_gt.py:
//   u64 tick_count            physics tick (sim virtual-time key)
//   u64 t_us                  microseconds since the log opened (monotonic)
//   f32 pos[3]                NED position [m]
//   f32 quat[4]               body->world quaternion, w first
//   f32 vel[3]                NED velocity [m/s]
//   f32 omega[3]              body angular velocity [rad/s]
//   f32 rpy[3]                NED Tait-Bryan roll/pitch/yaw [deg] (convenience)
//   f32 motor_omega[4]        per-rotor [rad/s]
//   f32 motor_duty[4]         per-rotor [0,1]
//   f32 airspeed              [m/s]
//   f32 batt_voltage          [V]
//   f32 batt_soc              [0,1]
void SimulatorWidget::logGroundTruth(vsim::SimSnapshot snap) {
  if (!m_runLog || !m_runLog->isOpen())
    return;
  float roll, pitch, yaw;
  vsim::quatToEulerNED(snap.att, &roll, &pitch, &yaw);

  QByteArray rec;
  rec.reserve(kGtRecordBytes);
  QDataStream s(&rec, QIODevice::WriteOnly);
  s.setByteOrder(QDataStream::LittleEndian);
  s.setFloatingPointPrecision(QDataStream::SinglePrecision);
  s << quint64(snap.tick_count) << quint64(m_runClock.nsecsElapsed() / 1000);
  s << snap.pos_w.x() << snap.pos_w.y() << snap.pos_w.z();
  s << snap.att.scalar() << snap.att.x() << snap.att.y() << snap.att.z();
  s << snap.vel_w.x() << snap.vel_w.y() << snap.vel_w.z();
  s << snap.omega_b.x() << snap.omega_b.y() << snap.omega_b.z();
  s << roll << pitch << yaw;
  for (float v : snap.motor_omega)
    s << v;
  for (float v : snap.motor_duty)
    s << v;
  s << snap.airspeed << snap.batt_voltage << snap.batt_soc;

  m_runLog->write(rec);
  m_runLogBytes += rec.size();
  // Occasional size readout (every ~5 s at 60 Hz pose).
  static int chatter_div = 0;
  if (++chatter_div % 300 == 0)
    appendLog("gt",
              QString("logged %1 KB ground truth").arg(m_runLogBytes / 1024));
}

void SimulatorWidget::closeLogFile() {
  if (!m_runLog)
    return;
  if (m_runLog->isOpen()) {
    m_runLog->flush();
    appendLog("log", QString("[closed: %1 bytes total]").arg(m_runLogBytes));
    m_runLog->close();
  }
  m_runLog.reset();
}

void SimulatorWidget::appendLog(const QString &tag, const QString &text) {
  QString trimmed = text;
  while (trimmed.endsWith('\n') || trimmed.endsWith('\r')) {
    trimmed.chop(1);
  }
  if (trimmed.isEmpty())
    return;
  for (const QString &line : trimmed.split('\n', Qt::SkipEmptyParts)) {
    m_log->appendPlainText(QString("[%1] %2").arg(tag, line));
  }
  m_log->verticalScrollBar()->setValue(m_log->verticalScrollBar()->maximum());
}
