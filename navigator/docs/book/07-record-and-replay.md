# Chapter 7 — Record and replay, logs and ground truth

Navigator writes four kinds of files. This chapter says what each is, where it goes and how to read it.

| File | Written by | Format |
|---|---|---|
| `rec_<yyyyMMdd-hhmmss>.bin` | automatic recording on connect | VREC, raw inbound bytes |
| `export-<yyyyMMdd-HHmmss>.bin` | File > Export Log (Ctrl+E) | VREC, whole packets, filtered by type |
| `gt-<yyyy-MM-dd_HH-mm-ss>.bin` | in-app simulator, one per run | VGT1, physics ground truth |
| `navigator-<UTC date>.log`, `session_<stamp>.log` | the text loggers | plain text |

## The VREC recording format

`RecordSink` writes and `RecordReader` reads the format that `replay/RecordFormat.h` defines. Everything is little-endian.

```
header   magic u32 = 0x56524543 ("VREC")
         formatVersion u32 = 1
         protocolVersion u32
         startWallClockMs u64
records  repeated: t_us u64 | len u32 | len raw bytes
```

`t_us` is a monotonic microsecond timestamp taken from the telemetry engine's elapsed timer. A record holds raw byte chunks as they arrived, not decoded packets, so replay goes through the same parser as a live link. The reader checks only the magic; it does not check `formatVersion`. The engine always writes `protocolVersion` as 1 (the source comments it as a forward-compatibility field).

Two things produce VREC files, and they differ in what they store:

- **Recording** (`startRecording`) tees every inbound chunk. It starts on connect when "Record telemetry" is on in Settings, Logging & Recording (`recordOnConnect`, default off), and stops on disconnect or when a replay starts.
- **Export** (`startExport`) writes each decoded wire packet whole, and only for the packet types you tick in the "Export telemetry" dialog: IMU, attitude, RC channels, motors, system status, log messages, heartbeat, perf. All are ticked by default. The file is still VREC and still replayable.

## Where files go

- Recordings and session logs use the directory from Settings (`logDirectory`). If it is empty, the default is `~/vayu-logs`, created on demand. The export dialog also pre-fills `~/vayu-logs`, and the Open Flight Log dialog starts there.
- With "export on disconnect" enabled, the System Log buffer is dumped to `session_<stamp>.log` in the same directory.
- The persistent text log (`Logger`) writes `navigator-<UTC date>.log` under `<AppDataLocation>/logs`, rotating at about 5 MB and keeping 10 files (the `maxBytes` and `maxFiles` defaults in `Logger.h`). If Qt reports no AppDataLocation it falls back to the system temp directory under `vayu-gcs/logs`.
- Simulator ground-truth logs go to the simulator's own log directory, a setting editable on the Simulator page (settings key `simulator/logDir`). Its default is `$VAYU_REPO/logs` if `VAYU_REPO` is set, otherwise `<AppDataLocation>/logs`, otherwise the temp-directory fallback.

## Replay

Open a log with File > Open Flight Log (Ctrl+O), File > Open Recent Log, or File > Open Last Log (Ctrl+Shift+O). The main window asks the source controller to enter Replay; `ReplaySource` loads the whole file into memory, subtracts the first timestamp so the playhead starts at zero, and emits frames from a 20 ms timer (50 Hz) as their virtual time falls due. Opening fails, with an error message, if the header is bad or the file holds no frames.

The `ReplayBar` has play and pause, 5-second step buttons, a scrub bar, a crop range with two handles, a Play once / Loop selector, and a speed selector (0.25x, 0.5x, 1x, 2x, 4x). Replay is read-only: Replay is not a state in which commands may be sent, and entering it stops any live recording so replayed frames are never recorded again.

The recent list holds at most 10 paths in the `replay/recentLogs` setting. A path is added when you open it, when a recording starts, and when an export opens. Missing files are pruned each time the menu is shown, except the file currently being recorded.

## The ground-truth log (`gt-*.bin`)

The simulator writes this file, not the telemetry path. It records the physics state the simulator knows, so you can overlay what the flight controller estimated (the VREC data) against what actually happened. It is opened when the in-app simulator starts and closed when it stops. The `gt-` prefix is deliberate: older tools that expect flight-controller telemetry in `sim-*.bin` would misread it. Attach mode (viewing an external simulator's pose) does not open one.

All fields are little-endian. The header is 20 bytes:

| Offset | Type | Field |
|---|---|---|
| 0 | 4 bytes | magic `VGT1` (raw ASCII) |
| 4 | u32 | version, written as 1 |
| 8 | u32 | record size in bytes, 124 |
| 12 | u64 | start wall-clock time, ms since the Unix epoch |

One 124-byte record follows for each pose update the simulator emits:

| Type | Field |
|---|---|
| u64 | `tick_count`, the physics tick |
| u64 | `t_us`, microseconds since the log opened |
| f32 x3 | position, NED, m |
| f32 x4 | attitude quaternion, w first |
| f32 x3 | velocity, NED, m/s |
| f32 x3 | body angular velocity, rad/s |
| f32 x3 | roll, pitch, yaw, degrees |
| f32 x4 | motor angular speed, rad/s |
| f32 x4 | motor duty, 0 to 1 |
| f32 | airspeed, m/s |
| f32 | battery voltage, V |
| f32 | battery state of charge, 0 to 1 |

That is 2 x 8 + 27 x 4 = 124 bytes. The widget's comments say the layout is kept in step with a `parse_gt.py` reader under the firmware repository's journal; that file is not in this repository.

Gap: no parser for `gt-*.bin` ships here, and the pose update rate is not documented (one code comment assumes about 60 Hz).

Gap: the log viewer is not a separate tool; the only log widgets are the System Log panel and the Faults tab, which show live text, not recorded files.

## Sources

- navigator/src/replay/RecordFormat.h, RecordSink.h, RecordSink.cpp, ReplaySource.h, ReplaySource.cpp
- navigator/src/protocol/TelemetryEngine.cpp
- navigator/src/ui/main/MainWindow.cpp
- navigator/src/ui/widgets/SimulatorWidget.cpp, SimulatorWidget.h, ReplayBar.cpp, LogsPanel.h
- navigator/src/core/Logger.cpp, Logger.h, SettingsManager.h
- navigator/src/vsim/SimPaths.cpp
- navigator/docs/reference/gcs-architecture.md (section 6)

## Checked against the code

- VREC header and record layout -> navigator/src/replay/RecordFormat.h, RecordSink.cpp
- Reader validates only the magic -> navigator/src/replay/RecordSink.cpp (`RecordReader::open`)
- Export filters by packet type and writes whole packets -> navigator/src/protocol/TelemetryEngine.cpp, MainWindow.cpp (`onExportLogToggle`)
- Default directory `~/vayu-logs` -> navigator/src/ui/main/MainWindow.cpp (`resolveLogDir`)
- VGT1 header (20 bytes) and 124-byte record -> navigator/src/ui/widgets/SimulatorWidget.cpp (`openNewLogFile`, `logGroundTruth`)
- Replay timer 20 ms, speeds, recent-list limit of 10 -> ReplaySource.cpp, ReplayBar.cpp, MainWindow.cpp
- Simulator log directory default -> navigator/src/vsim/SimPaths.cpp
