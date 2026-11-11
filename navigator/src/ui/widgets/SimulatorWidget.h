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

#include "StartGate.h"
#include "CustomVehicleWarning.h"

#include "CommandRetrier.h"
#include "vsim/SimWorker.h"
#include <QElapsedTimer>
#include "vsim/SimRendererWidget.h"
#include "vsim/ChunkStreamer.h"
#include "vsim/procgen/Flora.h"
#include "vsim/RcBridge.h"
#include "GeometryEditorWidget.h"
#include "WorldEditorWidget.h"
#include "SimHudWidget.h"
#include "ContourMinimapWidget.h"
#include "HorizonHud.h"
#include "TuneChart.h"
#include "PanelWidgets.h"
#include "AutotuneGains.h"
#include "../../audio/PropAudio.h"

extern "C" {}

#include <QElapsedTimer>
#include <QFutureWatcher>
#include <QFile>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QRectF>
#include <QString>
#include <QVector3D>
#include <QWidget>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <unordered_map>

#include <QByteArray>

class QGridLayout;
class QStackedWidget;
class QPushButton;
class QCheckBox;
class QSlider;
class QComboBox;
class QSpinBox;
class QPlainTextEdit;
class QProcess;
class SimViewPanel;
class SimVehiclePanel;
class SimWorldPanel;
class SimControlsPanel;
class SimAutotunePanel;

namespace vsim {
struct LoadedMesh;
}

/**
 * SimulatorWidget - control + monitor page for the SITL.
 *
 * In-process architecture: this widget owns
 *   - a telemetry sink (the UART2 callback that carries firmware
 *     telemetry to the GUI)
 *   - a vsim::SimWorker, which boots the REAL vaios firmware + in-process
 *     vsim physics in one deterministic stepper (no external daemon, no
 *     FIFO) and emits pose snapshots to the renderer
 *   - a vsim::SimRendererWidget (OpenGL view of the airframe)
 *
 * The engine's firmware side boots at most ONCE per Navigator process (see
 * host_lifecycle.c comments); the Stop button only halts the worker's step
 * loop (the firmware stays booted, so Re-Start resumes). A separate
 * viewer-only "Attach Ext" mode renders a pose FIFO published by an external
 * headless run — see attachExternalSim().
 */
// Result of an off-thread terrain-chunk build: the collision/render mesh and the
// grass/flower instances scattered on it. Carried back to the UI thread via a
// QFuture.
struct BuiltChunk {
  vsim::procgen::ProcMesh mesh;     // fine, for rendering
  vsim::procgen::ProcMesh collMesh; // coarse, for the collision BVH
};

class SimulatorWidget : public QWidget {
  Q_OBJECT
  // The simulator panels (sim.view, sim.vehicle, sim.world, sim.controls,
  // sim.autotune) build their widget trees through the private builders below:
  // the controls they create are this widget's m_* state.
  friend class SimViewPanel;
  friend class SimVehiclePanel;
  friend class SimWorldPanel;
  friend class SimControlsPanel;
  friend class SimAutotunePanel;
  friend class SimulatorWidgetTestAccess; // tests/tst_world_change_floor.cpp

public:
  explicit SimulatorWidget(QWidget *parent = nullptr);
  ~SimulatorWidget() override;

  /* True once vayu_sitl_start has booted the in-process firmware this session.
   * vayu_sitl_start runs at most once per process and the firmware threads then
   * live for the whole process (Stop pauses the feed, it does NOT tear the
   * firmware down), so this stays true after the first sim start — including the
   * Idle window after an autotune run. MainWindow uses it to allow applying
   * tuned gains to the in-process firmware (which persists them to 0:pid.bin). */
  bool sitlCoreStarted() const { return m_sitlStarted; }

  /* The 3D view as an image (SimRendererWidget::grabFrame, built on
   * QOpenGLWindow::grabFramebuffer; a native GL window cannot be grabbed with
   * QWidget::grab). Includes the HUD. */
  QImage grabSimView();

signals:
  void backToHomeRequested();

  /* Mirror of SerialManager::dataReceived. Emitted on the GUI thread
   * whenever the in-process firmware writes to UART2 - DroneProtocol
   * packets, vayu_log() text, telemetry. MainWindow connects this to
   * the same DroneProtocol parser the real serial path feeds, so the
   * existing telemetry panels light up without further plumbing. */
  void dataReceived(const QByteArray &bytes);

  /* Emitted when the in-app sim starts (true) / stops (false), so the
   * MainWindow can reflect "Connected: SIM" in the status bar. */
  void simRunningChanged(bool running);

  /* Emitted when an autotune run starts (true) / finishes or is cancelled
   * (false). The tuner runs an isolated SITL and does NOT feed the GCS engine,
   * so MainWindow drives the source FSM into its own Autotune state (read-only,
   * no feed). gcs-source-state-machine.md */
  void autotuneRunningChanged(bool running);

  /* AT-1: the operator clicked "Apply Gains". MainWindow routes each PidSetCmd
   * to the live FC over the link (CMD_SET_PID) when connected, or otherwise to
   * the in-process sim firmware (which persists them to 0:pid.bin). The
   * autotuner never writes to firmware on its own. */
  void applyPidGainsRequested(const QVector<PidSetCmd> &cmds);

public slots:
  // Drains the coalesced UART2 RX buffer on the GUI thread and re-emits it as
  // dataReceived. Invoked via QMetaObject::invokeMethod(Qt::QueuedConnection)
  // from queueUartBytes — AT MOST ONE drain is ever queued at a time, so a
  // high-rate byte-by-byte firmware telemetry flush collapses into one GUI
  // event per event-loop pass instead of thousands (the sim-running slowdown).
  // Public + no-arg so the static C trampoline can post it through the
  // meta-object system.
  void onUartBytes();

  // The firmware's COMMAND_ACK stream (MainWindow wires DroneProtocol's
  // commandAckReceived here). Acks end the Start-command retries.
  void onCommandAck(quint32 command, quint8 reqSeq, quint8 result);
  // Time-sync state from MainWindow (true once the clock is locked + synced).
  // Pending Start commands are sent the moment it becomes true.
  void setTimeSynced(bool synced);

  // Append firmware UART2 bytes to the coalescing buffer (called from the C
  // trampoline on the sim worker thread) and queue at most one onUartBytes()
  // drain. Public so the trampoline can reach it.
  void queueUartBytes(const uint8_t *data, size_t n);

  // Forward telemetry-decoded values (parsed in MainWindow) into the HUD
  // overlay: the flight-state name and the latest IMU accel/gyro sample.
  void hudSetStatus(const QString &s);
  void hudSetImu(const float acc[3], const float gyr[3]);
  // Reflect the firmware's reported flight mode (SYSTEM_ORIGIN_FLIGHT_MODE):
  // syncs the Acro checkbox without re-issuing a command, and shows the source
  // (RC switch vs GCS override).
  void setFlightModeStatus(quint8 mode, quint8 source);

  // Set the prop-audio default (Settings ▸ Alerts ▸ Sim prop audio). Drives the
  // in-panel "Prop audio" checkbox, which in turn enables PropAudio.
  void setPropAudioDefault(bool on);

public:
  // Stop the sim / autotune. Public so the source state machine's teardown can
  // enforce strict single source (gcs-source-state-machine.md). Both are
  // idempotent; stopAutotune requests cancel (completes via onTuneDone).
  void stopInAppSim();
  void stopAutotune();

  // MainWindow calls this after it actually sent the proposed gains (live FC
  // or running sim). A refused Apply (no link, sim stopped, replay) does not,
  // so the Custom-vehicle notice stays.
  void notifyPidGainsSent();

  // Keep Start disabled until the app is restarted, showing `reason` as its
  // tooltip and in the status line. For after "Reset Firmware Settings": the
  // engine module caches the old settings in RAM and is never unloaded, so a
  // Start before a restart would write them back. Call stopInAppSim() first.
  void blockStartUntilRestart(const QString &reason);
  bool startBlocked() const { return m_startGate.blocked(); }

private:
  StartGate m_startGate; // blocked: Start stays disabled this process
  void applyStartBlock();

  void buildUi();
  // Panel builders (defined next to their panel class, Sim*Panel.cpp).
  void buildViewPanel(QWidget *viewHost);
  void buildVehiclePage(ui::MinWidthScrollArea *scroll);
  void buildWorldPage(ui::MinWidthScrollArea *scroll);
  void buildSimControls(QWidget *host);
  void appendLog(const QString &tag, const QString &text);

  void startInAppSim();
  // Attach the 3D view to an EXTERNAL pose stream (/tmp/vsim_pose) — e.g. a
  // headless vayu_sitl_rtos run — without booting the in-process engine.
  void attachExternalSim();
  void pushRatesToSim(); // read persisted rates → m_sim->sendRates
  // Load the configured world mesh (baking up-axis/scale into NED) and push
  // it to the renderer; empty path clears it. When the sim is running, also
  // builds the collision BVH and ships it to the daemon (sendWorldMeshToSim).
  void loadWorldMeshToRenderer();
  // Endless terrain: poll the streamer at the current view centre (drone or
  // free-fly camera), drop out-of-range chunks, and farm new-chunk meshing out
  // to background threads.
  void onStreamTick();
  // A background chunk build finished: cache the mesh, upload mesh + flora, and
  // (re)try the local collision build. Runs on the UI thread.
  void onChunkMeshed(qint64 key, const BuiltChunk &built);
  // Flora is dense, so only the chunks within kFloraRadius of the view centre
  // are uploaded (the rest stay cached). streamFlora adds/removes as you move.
  void streamFlora(int cx, int cy);
  void uploadFloraChunk(qint64 key);
  void onFloraScattered(qint64 key, const std::vector<float> &packed);
  // Build + ship the local collision BVH from cached chunk meshes once the whole
  // collision neighbourhood is present (no terrain regeneration).
  void tryBuildCollision();
  // Contour minimap: re-centre it on the current view centre + heading (~8 Hz).
  void updateMinimap();
  // Training course: (re)generate gates for the selected difficulty, push the
  // layout to both renderers, reset the drone to the floor, and refresh the
  // progress readout. setTrainingMode is the dropdown handler.
  void setTrainingMode(int difficulty);
  void pushTrainingGates();
  void updateTrainingProgress();
  // Build a serialized BVH from an already-loaded mesh, write it atomically to
  // an mmap file, and point the daemon at it (VSIM_CTL_SET_WORLD_MESH).
  // The BVH build and blob write run on a worker (never the UI thread); only
  // the rename + ctl send happen here once it lands. Newest request wins: an
  // older in-flight one is dropped. liftAfter re-drops a buried drone, but only
  // after this collision has been shipped.
  // coverage (world N,E rect the mesh spans) lets the lift check that the drone
  // is really over shipped collision; unset = covers everything (finite world).
  void sendWorldMeshToSim(vsim::LoadedMesh m, bool liftAfter = false,
                          std::optional<QRectF> coverage = std::nullopt);
  struct WorldShipReq {
    // Either a finished soup (finite worlds) or the cached terrain chunks, plus
    // a small extra soup (helipads) in both cases; the worker flattens to
    // float[3n].
    std::shared_ptr<const vsim::LoadedMesh> mesh;
    std::vector<std::shared_ptr<const vsim::procgen::ProcMesh>> chunks;
    std::vector<QVector3D> extra;
    bool doubleSided = false;
    bool lift = false;
    std::optional<QRectF> coverage;
  };
  struct WorldShipResult {
    bool ok = false;
    uint32_t nodes = 0, nverts = 0;
    qint64 bytes = 0;
    double buildMs = 0, totalMs = 0;
    QString err;
  };
  void enqueueShip(WorldShipReq req);
  // Clean reset at the spawn pose (home pad on procedural terrain) carrying the
  // run's noise seed. Start, the Reset button and training all use it.
  void sendSpawnReset();
  // Start owes a spawn reset, but only once the collision under the spawn has
  // shipped (otherwise the drone is placed over terrain the engine lacks).
  bool spawnResetWaitsForCollision() const;
  // After the world changed: respawn on the home pad of the new ground (sim
  // running: like Reset, once the collision is in) or park the view there.
  void placeVehicleOnWorld();
  void startShipJob(WorldShipReq req);
  void onShipFinished(const WorldShipResult &r, int gen, const QString &path,
                      const QString &tmp);
  void cancelWorldShip(); // drop in-flight + queued collision ships
  // Invoked when the SimWorker exits on its own (spawn failure, startup-grace
  // timeout, daemon death) so the UI doesn't get stuck in the Running state.
  void onSimWorkerExited();

  // Per-run log records the PHYSICS ground truth (vsim SimSnapshot), NOT the FC
  // telemetry — that is captured separately via Export. So a run yields both the
  // FC's estimated view (export *.bin, VREC) and the true physics state
  // (gt-*.bin), making estimate-vs-truth an offline overlay. Format: see
  // firmware/docs/journal/log-analysis/parse_gt.py.
  void openNewLogFile();
  void closeLogFile();
  // Append one ground-truth record for this pose snapshot (no-op if no log open).
  void logGroundTruth(vsim::SimSnapshot snap);

  // Push the editor's motor layout + CoM (and mesh, if loaded) into the
  // renderer so the 3D preview matches the configured airframe.
  void applyGeometryToRenderer();
  void persistGeometry(const vsim::GeometryConfig &g);
  vsim::GeometryConfig restoreGeometry();
  void pushAirframeGains(bool announce);
  QString m_presetId; // airframe preset whose gains Start sends ("" = custom)
  // Register a firmware push that the pre-sync command gate can drop: it is
  // sent once synced and re-sent every second until the firmware ACKs it (see
  // CommandRetrier). Other pushes (e.g. an airframe gain set) register here.
  void armStartCommand(const QString &key, quint32 msgid, int expected,
                       const QString &label, const QString &consequence,
                       CommandRetrier::Send send);
  void armMixPush(const vsim::GeometryConfig &g);
  void retryTick();
  CommandRetrier m_retrier;
  class QTimer *m_retryTimer = nullptr;
  QElapsedTimer m_retryClock;
  void persistWorld(const vsim::WorldConfig &w);
  vsim::WorldConfig restoreWorld();
  void persistWind(const vsim::WindConfig &w);
  vsim::WindConfig restoreWind();

  // Switch the right-hand properties panel: 0 = Vehicle, 1 = World.
  // Vehicle is locked while the sim is running.
  void setMode(int mode);

  // Push the current rig sliders' orientation to the sim (test-rig pose).
  void applyRigPose();
  // Enable/disable the rig pose controls — they need a running in-app sim to
  // pose against. Called when the sim starts/stops.
  void setRigControlsEnabled(bool simRunning);

  // Refresh the viewport HUD overlay from a pose snapshot.
  void updateHud(const vsim::SimSnapshot &s);

  // ---- repo root (kept for legacy widget consistency) ----
  QString m_repoRoot;
  QLineEdit *m_repoRootEdit = nullptr;

  // ---- per-run physics ground-truth log (gt-*.bin) ----
  QString m_logDir; // editable in UI
  QLineEdit *m_logDirEdit = nullptr;
  ui::ElidedLabel *m_logPathLabel = nullptr; // shows current run's file
  std::unique_ptr<QFile> m_runLog;  // open while sim running
  qint64 m_runLogBytes = 0;
  QElapsedTimer m_runClock; // monotonic per-record t_us base

  // ---- shared iface + in-app sim ----

  // Coalescing buffer for the firmware's UART2 telemetry. The in-process
  // callback fires once per hal_uart_write_char — i.e. PER BYTE in SITL (the
  // channel flushes UART2 without DMA) — so instead of posting a queued GUI
  // event per byte we accumulate here and post a single drain event when none
  // is already pending. Filled on the sim worker thread, drained on the GUI
  // thread (onUartBytes).
  std::mutex m_uartRxMtx;
  QByteArray m_uartRxBuf;
  bool m_uartRxDrainPending = false;
  bool m_sitlStarted = false;
  vsim::SimWorker *m_sim = nullptr;

  // Endless procedural terrain streaming (active only for the "endless" biome).
  vsim::ChunkStreamer m_chunkStreamer;
  class QTimer *m_streamTimer = nullptr; // polls the view centre (~10 Hz)
  // Mesh cache for loaded chunks (keyed like the streamer) — lets local
  // collision reuse meshes instead of regenerating them on each crossing.
  std::unordered_map<qint64, std::shared_ptr<const vsim::procgen::ProcMesh>>
      m_chunkCache; // shared so the collision worker can read without a copy
  int m_streamGen = 0; // bumped on (re)configure to drop stale builds
  bool m_collisionPending = false; // a crossing asked for a collision rebuild
  int m_colCx = 0, m_colCy = 0;    // cell that collision should cover
  // Async collision ship: generation counter shared with the worker (bumped by
  // every new request / cancel), the one running job, and the latest queued one.
  std::shared_ptr<std::atomic<int>> m_shipGen =
      std::make_shared<std::atomic<int>>(0);
  QFutureWatcher<WorldShipResult> *m_shipWatcher = nullptr;
  QString m_shipRunningTmp;
  bool m_shipRunningLift = false;
  std::optional<QRectF> m_shipRunningCover;
  std::optional<WorldShipReq> m_shipPending;
  bool m_spawnResetPending = false; // Start's reset, held for the collision
  uint32_t m_runSeed = 1;           // noise seed for this run's resets

  // Flora: packed instance data per built chunk (cache), and which chunks are
  // currently uploaded to the renderer (only the near ones, for perf).
  std::unordered_map<qint64, std::pair<std::vector<float>, int>> m_floraCache;
  std::set<qint64> m_floraShown;
  std::set<qint64> m_floraInflight;         // flora scatters in progress
  static constexpr int kFloraRadius = 1;    // chunks each side kept grassed
  vsim::procgen::FloraParams m_floraParams; // active grass tuning (from config)
  bool m_useGpuGrass =
      false; // GPU grass active -> skip the CPU flora streaming
  bool m_grassEnabled =
      false; // "Grass" toggle (off by default); gates rendering

  // Lift-onto-terrain: a height sampler for the active procedural world (null
  // for imported / no world), the last known drone position, and a request to
  // lift once the endless biome's local collision has shipped.
  std::function<float(float, float)> m_terrainHeightAt;
  // Raw field height for bulk, display-only sampling (minimap, slope screen);
  // m_terrainHeightAt is the exact meshed surface and costs ~0.3 ms a call.
  std::function<float(float, float)> m_terrainRawHeightAt;
  QVector3D m_lastDronePos{0, 0, 0};
  QVector3D m_lastDroneVel; // NED [m/s]; feeds the collision look-ahead
  void liftDroneToSurface(); // re-drop the drone above the surface if buried

  // Helipad landing platforms scattered deterministically on flat ground. Each
  // entry is (worldX, worldY, terrainHeight). The home pad (drone spawn) is at
  // the origin. Recomputed as the view roams; pushed to the renderer to draw.
  std::vector<QVector3D> m_helipads;
  QVector3D m_homePad{0, 0, 0};
  QVector3D m_lastHelipadCenter{1e9f, 1e9f,
                                0}; // recompute only when view moves
  void recomputeHelipads(float cx, float cy);
  // Append each helipad's solid cylinder (deck cap + side wall, double-sided) to
  // a collision triangle-soup so the drone physically lands on the pads.
  void appendHelipadCollision(std::vector<QVector3D> &positions) const;
  // NED z of the landing surface at (x,y): pad top when over a helipad, else
  // the terrain surface. Used by spawn/reset and the lift-onto-surface logic.
  float landingSurfaceZ(float x, float y) const;
  RcBridge *m_rc = nullptr; // RC transmitter → firmware RC feeder
  QCheckBox *m_rcEnable = nullptr;
  QComboBox *m_rcSource = nullptr;    // USB joystick vs UART (CSV)
  QComboBox *m_rcBaud = nullptr;      // UART baud (UART source only)
  QComboBox *m_rcPath = nullptr;      // editable: device path or picked port
  QPushButton *m_rcConnect = nullptr; // explicit connect for the RC UART
  bool m_rcUartConnected = false;     // is the RC UART device opened?
  QLabel *m_rcReadout = nullptr;
  QLabel *m_rcAxesLabel = nullptr;     // live per-axis µs (identify)
  QCheckBox *m_acroChk = nullptr;      // acro toggle; synced from telemetry
  QLabel *m_flightModeLabel = nullptr; // shows effective mode + source
  QComboBox *m_rcAxisCombo[5] = {nullptr, nullptr, nullptr, nullptr, nullptr};
  QCheckBox *m_rcInvert[5] = {nullptr, nullptr, nullptr, nullptr, nullptr};
  QCheckBox *m_swArm = nullptr; // software arm (no hardware switch)
  void pushRcMapping(int func); // combo/invert → bridge + persist
  // Apply the selected RC source (joystick vs UART CSV) to the bridge and the
  // UI: swap the device field, enable/disable baud + axis mapping.
  void applyRcSource();
  void commitRcPath(); // device field → bridge + persist
  // Open/close the RC UART device via the bridge, claiming/releasing the port
  // through PortArbiter so the board telemetry and the sim never collide.
  void setRcUartConnected(bool on);
  vsim::SimRendererWidget *m_renderer = nullptr; // QOpenGLWindow (native)
  QWidget *m_rendererBox = nullptr; // its window container, in the layout
  QWidget *m_pipRail = nullptr;     // instrument panes beside the view
  static constexpr int kPipRailW = 220;
  void syncPipRail();
  SimHudWidget *m_hud = nullptr;   // FPV telemetry HUD, painted by m_renderer
  HorizonHud *m_horizon = nullptr; // compact attitude indicator (Horizon PiP)
  vsim::SimRendererWidget *m_downRenderer = nullptr; // Down-Cam PiP renderer
  QWidget *m_horizonPip = nullptr; // pane hosting m_horizon
  QWidget *m_downPip = nullptr; // pane hosting m_downRenderer
  ContourMinimapWidget *m_minimap = nullptr; // top-down contour minimap
  QWidget *m_minimapPip = nullptr;           // pane hosting it
  class QTimer *m_minimapTimer = nullptr;    // re-centres the minimap (~8 Hz)
  PropAudio m_propAudio;                     // rpm-driven propeller sound
  QCheckBox *m_propAudioChk =
      nullptr; // "Prop audio" toggle (default via Settings)
  GeometryEditorWidget *m_geomEditor = nullptr;
  WorldEditorWidget *m_worldEditor = nullptr;
  QComboBox *m_trainingMode = nullptr;    // Off / Easy / Medium / Hard course
  QLabel *m_trainingStatus = nullptr;     // "Gate 2 / 7" progress readout
  vsim::TrainingCourse m_training;        // halo-gate course state + progress
  QStackedWidget *m_rightStack = nullptr; // 0 = Vehicle, 1 = World
  QPushButton *m_vehicleTab = nullptr;
  QPushButton *m_worldTab = nullptr;
  QPushButton *m_tuneTab = nullptr;

  // ---- Fault injection (mockup Vehicle ▸ Fault Injection) ----
  bool m_motorKill[4] = {false, false, false, false};
  bool m_imuDropout = false; // driven by the Sensor Models IMU enable
  QPushButton *m_killBtn[4] = {nullptr, nullptr, nullptr, nullptr};
  QWidget *buildFaultPanel(); // kill-motor / RC-loss / GPS-glitch group
  void pushFaults();          // current fault flags → SimWorker

  // ---- Sensor models (mockup Vehicle ▸ Sensor Models) ----
  struct SensorRow {
    class QCheckBox *en = nullptr;
    class QDoubleSpinBox *sigma = nullptr;
    class QDoubleSpinBox *clip = nullptr;
  };
  SensorRow m_sensorRow[3];    // 0=accel, 1=gyro, 2=mag
  QWidget *buildSensorPanel(); // per-sensor noise σ / bias-clip / enable
  void pushNoise();            // current sensor model → SimWorker

  // ---- Autotune section (drives tools/autotune against the current vehicle) ----
  QComboBox *m_tuneOptimizer = nullptr;
  QSpinBox *m_tuneBudget = nullptr;
  QSpinBox *m_tuneStep = nullptr;        // excitation amplitude (doublet µs)
  QComboBox *m_tuneExcitation = nullptr; // Step vs Chirp waveform
  QDoubleSpinBox *m_tuneChirpF0 = nullptr;
  QDoubleSpinBox *m_tuneChirpF1 = nullptr;
  QWidget *m_chirpRow = nullptr; // freq-range row, shown only for chirp
  QWidget *m_tuneResponse =
      nullptr; // live step/chirp response plot (ResponsePlot)
  QSpinBox *m_tuneRepeats = nullptr; // rollouts averaged per eval (--repeats)
  QDoubleSpinBox *m_tuneTether = nullptr; // soft-rig stiffness (--rig-tether)
  QSpinBox *m_tuneSeed = nullptr;         // optimizer RNG seed (--seed)
  QSpinBox *m_tuneSimSeed = nullptr;      // sensor-noise base seed (--sim-seed)
  QCheckBox *m_tuneYaw = nullptr;
  QCheckBox *m_tuneSysId = nullptr; // analytic plant-fit design
  QDoubleSpinBox *m_tuneSysIdBw =
      nullptr; // sys-ID crossover as a fraction of actuator BW
  QComboBox *m_tuneCostFn = nullptr; // fast-backend cost: angle vs angle+rate
  QCheckBox *m_tunePlot = nullptr;
  QCheckBox *m_tuneCompare = nullptr; // run every optimizer (--compare)
  QCheckBox *m_tuneValidate =
      nullptr; // free-flight validation (--no-validate if off)
  QCheckBox *m_tuneBuzz =
      nullptr; // throttle buzz check (--no-buzz-check if off)
  QCheckBox *m_tuneVerbose = nullptr; // print every eval (--verbose)
  QPushButton *m_tuneStart = nullptr;
  QPushButton *m_tuneStop = nullptr;
  QPlainTextEdit *m_tuneLog = nullptr;
  QLabel *m_tuneResult = nullptr;
  TuneChart *m_tuneChart = nullptr;
  // AT-1: autotune proposes; applying to firmware is an explicit click.
  QLabel *m_tuneProposed = nullptr;      // human-readable best gains
  QPushButton *m_tuneApplyBtn = nullptr; // "Apply Gains" (FC link or sim)
  class QTableWidget *m_tuneGainsTable = nullptr; // AT-2: current vs best
  QString m_tuneOutJson;       // --out path for the running search
  QStringList m_tuneParams;    // param names from the result
  QVector<double> m_tuneBestX; // best gain vector (the proposal)
  void parseProposedGains();   // read m_tuneOutJson into the above
  void applyProposedGains();   // emit applyPidGainsRequested
  // Test-rig pose controls: pin the airframe and tilt it on the stand.
  QCheckBox *m_rigEnable = nullptr;
  QSlider *m_rigRoll = nullptr;
  QSlider *m_rigPitch = nullptr;
  QSlider *m_rigYaw = nullptr;
  QLabel *m_rigReadout = nullptr;
  // C++ autotune engine on a worker thread (replaces the python3 subprocess).
  class QThread *m_tuneThread = nullptr;
  class AutotuneWorker *m_tuneWorker = nullptr;
  void onTuneEvaluated(const QVector<double> &current,
                       const QVector<double> &best, double cost,
                       double bestCost, int n);
  void onTuneFinished(const QVector<double> &bestX, const QStringList &names,
                      double bestCost);
  void onTuneDone();
  void buildAutotunePage(QWidget *page);
  void startAutotune();
  QString exportVehicleGeometryJson();
  QString
  exportWorldJson(); // env (gravity/drag) so the tuner flies the same plant
  QPushButton *m_simStartBtn = nullptr;
  QPushButton *m_simStopBtn = nullptr;
  QPushButton *m_simResetBtn = nullptr;
  QPushButton *m_simAttachBtn =
      nullptr;                     // attach 3D view to an external pose stream
  bool m_attached = false;         // true while mirroring an external sim
  QCheckBox *m_fpvCheck = nullptr; // onboard FPV (only meaningful running)
  QLabel *m_simStatusLabel = nullptr;
  QLabel *m_customWarnLabel = nullptr; // Custom + untuned notice (non-modal)
  CustomTuneState m_customTune;
  void updateCustomWarning(bool atStart);
  QLabel *m_simPoseLabel = nullptr;

  // ---- live readouts derived from snapshot ----
  QProgressBar *m_motorBars[4] = {nullptr, nullptr, nullptr, nullptr};
  QLabel *m_motorLabels[4] = {nullptr, nullptr, nullptr, nullptr};

  // ---- log ----
  QPlainTextEdit *m_log = nullptr;
};
