# Chapter 5 — The flight screens

This chapter walks through the pages of the normal (non-docked) window and says, for each one, what it shows and which data path feeds it. The simulator page is Chapter 6. Record and replay are Chapter 7.

MainWindow lays every page out as a dock panel (`DockShell`). There is no View menu: the old pages are built-in workspaces in the Workspaces menu, with the old keys: Ctrl+1 Fly (the old Flight Dashboard), Ctrl+2 RC channels, Ctrl+3 Motors, Ctrl+4 Control loop, Ctrl+5 Packets, Ctrl+6 Sim, Ctrl+8 Kernel perf. The Tools menu has Calibration (Ctrl+7) and Gyro Notch (Ctrl+9). Settings has its own menu-bar entry. Fullscreen (F11) is in the Window menu.

## Where the data comes from

Two paths feed these pages (Chapter 2 has the detail):

- **Snapshot path.** High-rate telemetry (IMU, attitude, RC, motors, baro, battery, vertical state, flight mode, vehicle state) is stored by `TelemetryEngine` in `VehicleState`. `MainWindow::onUiTimer` runs every 33 ms (about 30 Hz), takes one `snapshot()`, and pushes it to the widgets.
- **Signal path.** Low-rate or event data (log lines, faults, perf reports, calibration updates, control-loop traces, notch status) arrives as `DroneProtocol` signals, which are queued across to the GUI thread.

Most pages treat telemetry as stale after a timeout (`kTelemetryStaleMs`). When it is stale, numeric fields show "-" instead of the last value.

## Flight Dashboard (home page)

The home page is built in `MainWindow::buildUi`. It has an attitude group, an IMU group, and a Log/Faults tab area at the bottom.

- **Attitude (`AttitudePanel`).** A 2D / 3D toggle. 2D is `AttitudeWidget`, an artificial horizon with a pitch ladder, roll arc and heading tape. 3D is `Drone3DWidget`, which despite the name is a stylised, QPainter-drawn X-frame quad that banks, pitches and yaws with the attitude; it is not the simulator's 3D world. Both are fed from the snapshot (`s.attitude`), and both ease toward the latest angle so low-rate data looks smooth. Pills show the firmware state and the flight mode. `NumbersPanel` adds roll, pitch and yaw with a rolling standard deviation each, refreshed every fourth tick.
- **IMU (`ImuPanel`).** Graphs for accelerometer, gyroscope, magnetometer and baro altitude (MSL and AGL), a fused-versus-raw vertical estimate chart, and temperature and battery gauges. Battery comes from the `BATTERY` message; the gauge says "no battery telemetry" when none has arrived, and says why when the reading is not a pack.
- **Log and Faults (`LogsPanel`).** `LogPanel` shows text from `logReceived` (firmware `STATUSTEXT`, command acks, unrecognised frames) with a timestamp style and line cap set in Settings. `FaultsPanel` shows `faultReceived` events with severity, acknowledge and clear; Clear is disabled while the vehicle is armed.

## RC Channels

`RcChannelsWidget`: two stick gimbals, four aux switches, eight named channel bars and an eight-trace history. Fed from the snapshot (`s.rc`). The router keeps the first 14 of the 18 NavLink channels. The Link Health box (RSSI, link quality, frame rate, protocol, channels, failsafe) is a placeholder: every field shows "-" because no link-quality telemetry is wired.

## Motors

`MotorStatusWidget`: a quad-X schematic with a speed ring and percentage per motor, plus a 10-second history graph. Fed from the snapshot (`s.motors`); the router reads the first 4 of 8 values from `MOTOR_TELEMETRY`. The Power group shows average throttle and maximum spread (computed from the speeds) and the battery text. Current, ESC temperature and mAh used stay "-" (Gap: no source for them).

## Control Loop

`ControlLoopPlot` has five graphs: outer-loop angle setpoint versus current, inner-loop rate setpoint versus current, controller outputs including throttle, outer and inner loop timing, and estimator latency (peak and mean). It connects to `DroneProtocol` itself, for `controlLoopDataReceived`, `imuReceived` and `estPerfReceived`, rather than using the snapshot. It has Pause and Export CSV buttons.

## Packet Analyzer

`PacketAnalyzerWidget` logs every inbound frame (`packetReceived`, and unrecognised bytes through `unknownPacket`) and every outbound frame (`TelemetryEngine::txPacket`). It has direction, device and text filters, a display-filter expression box (placeholder example: `type==attitude && dev==42 && size>50`), Save Table to CSV, and Start Streaming to CSV. Selecting a row opens a field tree and hex view in `PacketDetailWidget`, built by `PacketDissector`. `FrequencyRibbon` shows a per-type rate, and `LinkStatsPanel` shows link quality (good bytes over all bytes) and up/down throughput.

## Calibration

`CalibrationWidget` has four cards: Accelerometer, Gyroscope, Magnetometer and Board Level, with a guided step checklist and a progress bar. Start sends `CommandCodec::encodeCalibrate(which)`; Cancel sends `encodeCalibrate(0xFF)`. Progress and results come back as `calibrationUpdateReceived` and `statusReceived`. The controls are disabled when there is no connection.

## Gyro Notch and Kernel Perf

`GyroNotchWidget` sends `encodeSetGyroNotch` and shows `NOTCH_STATUS` (tracked centre frequencies, three per axis). `PerfWidget` shows `PerfReport` data: CPU load, heap peak, context switches, SysTick maximum, IPC blocked, uptime, and task and FIFO tables. It asks for task names through `sendTaskNameRequest`.

## Settings

`SettingsWidget` has panes for Telemetry Streams, Link & Connection, Units & Display, Plots & Graphs, Logging & Recording, Alerts & Audio and Advanced. Edits are staged until Apply. The Telemetry Streams pane is a disabled table marked "soon", so stream rates cannot be changed from it today.

## Gaps and disagreements

Gap: the repository does not document what each firmware state pill means beyond the names in `DroneProtocol`.

`docs/reference/mockup-to-app.md` lists `vsim_d` for the simulator page; that daemon is not in the current code (Chapter 6). It also lists the settings panes without noting that Telemetry Streams is disabled. `feature-matrix.md` marks stream-rate configuration "enhance", which matches the code. Its "ARM / Disarm ... enhance" row is also out of date for the app side: `MainWindow` already sends `encodeArm` and `encodeDisarm`.

## Sources

- `navigator/src/ui/main/MainWindow.cpp` (`buildUi`, `buildMenuBar`, `onUiTimer`)
- `navigator/src/ui/widgets/` (`AttitudePanel`, `AttitudeWidget`, `Drone3DWidget`, `NumbersPanel`, `ImuPanel`, `LogsPanel`, `LogPanel`, `FaultsPanel`, `RcChannelsWidget`, `MotorStatusWidget`, `ControlLoopPlot`, `PacketAnalyzerWidget`, `LinkStatsPanel`, `CalibrationWidget`, `GyroNotchWidget`, `PerfWidget`)
- `navigator/src/ui/main/SettingsWidget.cpp`
- `navigator/src/protocol/NavlinkRouter.cpp`
- `navigator/docs/reference/feature-matrix.md`, `mockup-to-app.md`

## Checked against the code

- Pages are added to one `QStackedWidget` unless docking is on; docking is off by default -> `MainWindow.cpp` `addPage`, `DockShell::enabled`
- Snapshot pushed every 33 ms; stale data shows "-" -> `MainWindow::onUiTimer`
- `Drone3DWidget` is a painted X-frame quad -> `Drone3DWidget.h` comment
- RC Link Health fields are "-"; motor current, ESC temp and mAh are "-" -> `RcChannelsWidget.cpp`, `MotorStatusWidget.h`
- Control Loop has five graphs and subscribes to three `DroneProtocol` signals -> `ControlLoopPlot.cpp`
- Calibration sends `encodeCalibrate` -> `CalibrationWidget.cpp`
- Telemetry Streams table is disabled -> `SettingsWidget.cpp` (`tbl->setEnabled(false)`)
- Shortcuts Ctrl+1 to Ctrl+9 -> `MainWindow::buildMenuBar`
