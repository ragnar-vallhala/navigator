# Chapter 6 — The in-app simulator

Navigator can fly the Vayu firmware without hardware. The firmware is compiled for the host and loaded into Navigator's process as a shared library. Navigator provides the window, the 3D world, the RC input and the telemetry screens. This chapter describes what happens, in order, from build to first frame.

## Build time: the `NAVIGATOR_SITL` option

`NAVIGATOR_SITL` is a CMake option, default ON. When it is ON and the SDK headers are found, `navigator/CMakeLists.txt` adds the simulator sources (`src/vsim/*` and `SimulatorWidget`), links assimp, defines `NAVIGATOR_HAS_SITL`, and prints "SITL ON". With `-DNAVIGATOR_SITL=OFF` you get a GCS-only binary and no assimp dependency. If the option is ON but no SDK is present, CMake prints a warning and builds without the simulator.

Navigator links no firmware. It compiles against the SDK headers (`vayu_sitl_abi.h`, `vsim_proto.h` and others) from a firmware release. CMake looks for them in `sim/host/sdk` (monorepo layout) and otherwise in `vayu-sitl-sdk/include` at the repository root; override with `-DVAYU_SITL_SDK=<include dir>`.

## What the firmware release provides

The README section "The simulator needs a firmware release" says nothing here builds firmware. The `vayu` repository ships a release asset (CI builds against v0.2.1; a rolling `sitl-sdk-latest` also exists). The README example downloads `vayu-sitl-sdk-v0.2.1-linux-x86_64.tar.gz`, which unpacks to `./vayu-sitl-sdk/`. The repository is private, so the download needs a GitHub login with access. The README names two runtime files:

- `lib/libvayu_sitl.so`, loaded by the app.
- `bin/vayu_sitl_rtos`, used by the headless SDK and the autotuner (Chapter 8), found through `$VAYU_SITL_RTOS_BIN`.

Without the SDK the app still runs; the simulator reports itself unavailable and lists where it looked.

## Loading the module: `SitlModule`

`SitlModule::instance()` is a singleton that loads on first use and never retries. Loading goes like this:

1. It sets up the firmware's virtual filesystem directory (`vsim::applyVfsEnvironment`), because the firmware reads `VAYU_VFS_DIR` from the environment.
2. It tries candidate paths in order: `$VAYU_SITL_MODULE`; `lib<name>` beside the executable; `../lib/lib<name>`; then sibling build directories `../../build_sitl_rtos/` and `../../build_sitl/`. `<name>` is `vayu_sitl` (`VAYU_SITL_MODULE_NAME`); `QLibrary` adds the platform's suffix.
3. For each candidate it loads with `QLibrary` and `ResolveAllSymbolsHint`, so a module with missing symbols fails at load.
4. It resolves the entry symbol (`VAYU_SITL_ENTRY_SYMBOL`) and calls it with `VAYU_SITL_ABI_VERSION`. A module built for another ABI returns null, and the failure text says "ABI mismatch (this build needs vN)". The number itself is defined in the SDK header, which this repository does not contain.
5. On success it keeps the vtable (`vayu_sitl_api_t`), the absolute path, and the module's own `build_id`. On Unix it re-opens the library with `RTLD_NODELETE`, because the firmware's tasks never stop and unmapping them would crash the app at exit.

If every candidate fails, `available()` stays false and `error()` lists each path with the reason (not found, no such directory, wrong ABI or architecture, missing dependency). The ABI check proves only that the layout matches, so the build id is logged when the sim starts and shown with the sim; a stale module and a current one otherwise look identical.

## Pressing Start: `SimulatorWidget` and `SimWorker`

`SimulatorWidget::startInAppSim` sets a per-process `VSIM_FIFO_SUFFIX` (`_nav<pid>`) unless one is set, opens a run log, attaches the telemetry sink, picks the noise seed, and creates a `vsim::SimWorker`. The worker is a `QThread`, started at time-critical priority.

`SimWorker::runEngine` then, on the worker thread:

1. Checks `SitlModule::available()`; if not, logs the loader error and stops.
2. Calls `boot`. The firmware and physics run in one stepper inside the module. Boot is idempotent, so Stop then Start resumes the same engine.
3. Starts the serial RC feeder once per process. RC reaches the firmware through a pty that `RcBridge` fills from a joystick or a serial transmitter (path in `VAYU_UART_RC_PATH`).
4. Emits `online`, which makes `SimulatorWidget` push rates, firmware mix and gains, geometry, world, obstacles, wind, noise and faults, load the world mesh, and reset the vehicle at the spawn pose.
5. Loops until stopped: apply queued config calls, `run_step` (one 1 ms step, paced to wall-clock), and every 16th step `get_pose` and emit `poseUpdated` (about 62.5 Hz). The GUI thread never calls the engine directly; every `sendX()` is queued and applied between steps.

Two streams leave the module:

- **Pose** goes to `SimRendererWidget` (the 3D view), the HUD and the ground-truth log.
- **Telemetry** is the firmware's UART2 output. The module calls a sink callback, `SimulatorWidget` batches the bytes and re-emits them on the GUI thread, and `MainWindow` routes them through `SimSource` into the same `DroneProtocol` parser a real link uses (Chapter 4). Commands go the other way: `sendToFc` calls `SitlModule::send`, which passes frames to the module's `uart2_rx`.

Stop detaches the telemetry sink and stops the loop, but the firmware threads stay alive.

## The 3D world, terrain and collision

The renderer is `vsim::SimRendererWidget`, an OpenGL window. The world comes from `WorldEditorWidget` settings. Procedural biomes ("endless", the presets, and the legacy "meadow") stream endlessly. Imported meshes (loaded with assimp through `vsim::loadMesh`) are finite.

For streamed terrain, a 100 ms timer calls `SimulatorWidget::onStreamTick`. It asks `ChunkStreamer::plan` which chunks to build around the view centre. Defaults in `ChunkStreamer::Config`: 192 m chunks, render radius 3, collision radius 1, at most 6 builds in flight. Meshing runs off the UI thread with `QtConcurrent`; each finished chunk goes to the renderer and its collision mesh is cached.

Collision follows the same chunks. The collision window leads the drone along its velocity, with hysteresis so a hovering drone does not flip windows. When all chunks for the window are cached, `tryBuildCollision` queues a job (`startShipJob`). Off the UI thread it flattens the meshes, builds a BVH (`vsim::buildWorldBvh`) and writes a blob to `vsim_world<suffix>.bin` in the temp directory. Back on the UI thread the blob is renamed into place and `SimWorker::sendWorldMesh` tells the engine to map it. A newer request supersedes an older build.

Placement waits for collision. A reset or a "place vehicle" request is held (`m_heldPose`, `m_spawnResetPending`) until the covering collision has shipped, so the drone is never dropped over ground the engine does not have. A drone found buried after a window change is re-dropped above the surface.

`SimWorker::startAttach` is a second, viewer-only mode: it reads an existing pose FIFO from an external sim and draws it, with no engine.

## Gaps and disagreements

Gap: the ABI version number and the contents of the release (beyond the files the README names) are defined by the firmware repository, not this one.

Gap: the README shows only a Linux x86_64 asset; the repository does not say which release assets exist for Windows.

`gcs-architecture.md` section 5 is out of date. It describes a linked `libvayu_sitl_core` started with `vayu_sitl_start`, a child process `vsim_d` started with `posix_spawnp`, and a pose FIFO at 60 Hz. The code has no `posix_spawn` call and no `vsim_d` launch. The engine is loaded with `QLibrary`, and pose comes from `get_pose`; the FIFO exists only in attach mode. The CMake option text says "needs a POSIX host", but the SITL block links `winmm` on Windows and `SimWorker` has a Windows branch.

## Sources

- `README.md` ("The simulator needs a firmware release")
- `navigator/CMakeLists.txt` (SITL block, `VAYU_SITL_SDK`)
- `navigator/src/vsim/SitlModule.{h,cpp}`, `SimWorker.{h,cpp}`, `ChunkStreamer.h`, `SimPaths.cpp`, `ProceduralWorld.{h,cpp}`
- `navigator/src/ui/widgets/SimulatorWidget.cpp`
- `navigator/src/ui/main/MainWindow.cpp` (`sendToFc`, `applyFeed`, `SimSource` wiring)
- `navigator/docs/reference/gcs-architecture.md` section 5

## Checked against the code

- Module search order and `VAYU_SITL_MODULE` override -> `SitlModule.cpp` `candidatePaths`
- ABI mismatch is reported and the module is unloaded -> `SitlModule.cpp` constructor
- `RTLD_NODELETE` pin on Unix -> `SitlModule.cpp`
- Engine loop: 1 ms steps, pose every 16th step, queued config -> `SimWorker.cpp` `runEngine`
- No `vsim_d` launch and no `posix_spawn` -> `grep` under `navigator/src`
- Telemetry via sink, `SimSource` and `feedBytes`; commands via `SitlModule::send` -> `SimulatorWidget.cpp`, `MainWindow.cpp`
- Stream and collision defaults -> `ChunkStreamer.h`; blob path -> `SimPaths.cpp`
- `NAVIGATOR_SITL` default ON and the no-SDK warning -> `CMakeLists.txt`
