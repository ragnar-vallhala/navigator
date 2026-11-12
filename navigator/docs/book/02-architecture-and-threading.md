# Chapter 2 — Architecture and threading

Navigator has two jobs that want different rhythms. Telemetry arrives in bursts at whatever rate the vehicle sends it. The screen should redraw at a steady rate no matter how fast or slow packets come. The design keeps these apart with a worker thread, a mutex-guarded state object, and a render clock.

## The one-paragraph version

Inbound bytes are read and parsed on a worker thread called `telemetry`. The parser updates a `VehicleState` under a mutex. The GUI thread runs a 33 ms timer (about 30 Hz); on each tick it takes a `snapshot()` copy of `VehicleState` and pushes it to the widgets. Packet rate and render rate are decoupled.

```
 FC / link --> SerialManager / UdpManager        (worker thread "telemetry")
                    |
              TelemetryEngine::onLiveBytes  --(tee)--> RecordSink
                    |
              DroneProtocol -> NavlinkRouter   (NavLink decode)
                    |  lambdas lock + update
                    v
              VehicleState  [QMutex]
                    |  snapshot() = copy under the lock
 ------------------ | -------------------------------------------
                    v                               (GUI thread)
   uiTimer 33 ms -> onUiTimer -> widgets
   low-rate events (log, heartbeat, time-sync, ...) -> queued signals -> GUI slots
```

## The engine and its thread

`MainWindow` creates a `TelemetryEngine` with no parent, creates a `QThread`, names it `telemetry`, moves the engine to it and starts it. The engine owns the `DroneProtocol` parser, a `SerialManager`, a `UdpManager`, the recorder, and the `VehicleState`. Because the transports are children of the engine, they move with it. Qt therefore delivers serial and socket reads on the worker thread, and the socket drains as soon as data arrives, independent of how busy the GUI or the 3D view is.

The GUI never calls into the transports directly. It uses `QMetaObject::invokeMethod(..., Qt::QueuedConnection)` to run engine slots such as `openSerial`, `bindUdp`, `closeLinks` and `send` on the worker. Results come back as signals, for example `linkOpened`, which the GUI receives on its own thread.

## Why `VehicleState` is behind a mutex

High-rate signals from the parser (IMU, attitude, RC, motors, barometer, battery, vertical state) are connected to lambdas inside the engine. Each lambda locks the mutex, writes the new value and a last-received timestamp, and unlocks. These signals are not wired to widgets. `snapshot()` locks the mutex and returns a copy of the struct, which the header describes as a cheap value copy.

Low-rate, event-like signals (log lines, faults, unknown packets, heartbeat, time-sync, HSL status) are connected straight to GUI slots. Because the parser lives on the worker, Qt turns those connections into queued ones, and the engine registers the payload types with `qRegisterMetaType` so the copies marshal correctly.

## Three byte sources, one input

All bytes reach the parser through `TelemetryEngine::feedBytes`. The live transports call it internally. The in-app simulator and replay arrive through objects implementing `ITelemetrySource` (`SimSource` and `ReplaySource`); `MainWindow` connects the active source's `bytesReceived` signal to `feedBytes`. Those two sources live on the GUI thread, so their bytes cross to the worker as queued calls. Chapter 3 covers how exactly one source is selected.

## Module map

Directories under `navigator/src/`:

| Directory | Contents |
|---|---|
| `app` | `main.cpp`: OpenGL 4.6 core request with a fallback, 4x MSAA, then the main window |
| `audio` | `PropAudio` (propeller sound from motor speeds) and `ChimeAudio` (alert tones), each on its own thread; silent stubs without PulseAudio |
| `autotune` | Native C++ autotuner: optimizer, cost, system identification, evaluation worker |
| `comm` | `SerialManager`, `UdpManager`, `PortArbiter`, `ITelemetrySource`, `SimSource` |
| `core` | `SourceController` and `SourceState`, settings, logger, command registry, shortcuts, theme, units, CSV export, calibration wizard logic, battery estimator |
| `protocol` | `TelemetryEngine`, `VehicleState`, `DroneProtocol`, `NavlinkRouter`, `CommandCodec`, packet dissector, time-sync estimator |
| `replay` | `RecordSink`, `ReplaySource`, `RecordFormat` |
| `ui` | `main` (window, toolbar, status bar, settings), `widgets` (pages and panels), and `workspace` (experimental dockable layout, covered later) |
| `vsim` | Simulator host side: `SitlModule` loader, `SimWorker`, renderer, world and mesh code, RC bridge |

## Code and doc disagreements

- `gcs-architecture.md` section 5 describes the simulator as a child process `vsim_d` spawned with `posix_spawnp`, talking over FIFOs. The current source has no `vsim_d` or `posix_spawnp` in `navigator/src` apart from a mention in `autotune/Rollout.h`. `vsim/SitlModule.h` and the CMake comments describe a firmware module `libvayu_sitl` loaded at runtime. This book follows the code; the simulator gets its own chapter.
- A comment above `MainWindow::onUiTimer` says "10 Hz". The timer is started at 33 ms.
- `gcs-architecture.md` says a `DroneProtocol.h` comment describing ASCII packets is stale. In this checkout that comment already says "No ASCII framing", so the note is itself out of date.

## Sources

- `navigator/docs/reference/gcs-architecture.md`, sections 1 and 3, and the notes
- `navigator/src/protocol/TelemetryEngine.h`, `TelemetryEngine.cpp`, `VehicleState.h`
- `navigator/src/ui/main/MainWindow.cpp`
- `navigator/src/comm/ITelemetrySource.h`, `SimSource.h`
- `navigator/src/app/main.cpp`, `navigator/src/vsim/SitlModule.h`, `navigator/CMakeLists.txt`
- Directory listing of `navigator/src/`

## Checked against the code

- The engine runs on a `QThread` named `telemetry` -> `MainWindow.cpp` (`m_worker->setObjectName("telemetry")`, `moveToThread`)
- The render timer is 33 ms -> `MainWindow.cpp` (`m_uiTimer->start(33)`)
- `snapshot()` copies `m_state` under a `QMutexLocker` -> `TelemetryEngine.cpp`
- Per-packet state updates are lambdas that lock the mutex and are not wired to widgets -> `TelemetryEngine.cpp` (constructor)
- The transports are children of the engine -> `TelemetryEngine.cpp` (`new SerialManager(this)`, `new UdpManager(this)`)
- GUI drives the link with queued invokes -> `MainWindow.cpp` (`openSerial`, `bindUdp`, `closeLinks`, `send`)
- Active source's `bytesReceived` is connected to `feedBytes` -> `MainWindow.cpp` `applyFeed`
- Stale `10 Hz` comment and no `vsim_d` spawn in the source -> `MainWindow.cpp` above `onUiTimer`; grep of `navigator/src`
