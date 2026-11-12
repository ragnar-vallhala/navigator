# Chapter 1 — What Navigator is

Navigator is the ground control station (GCS) of the Vayu stack. It is a Qt6 desktop application that talks to a Vayu flight controller over the NavLink wire protocol, shows what the vehicle is reporting, and sends it commands. The repository is called `navigator`; the window title and application name in the code are "Vayu GCS".

## Where it sits in the stack

The repository `README.md` splits the project into four parts:

- `navigator/` is the Qt6 application.
- `navigator/headless-sdk/` is the installable `vayu_headless` Python package and its tests, which drives the flight stack from Python.
- `tools/autotune/` is a set of PID autotuning scripts that run against the simulator.
- `navlink/` is the wire protocol, included as a git submodule.

The flight controller firmware is a separate, private repository. What is public here is the ground station and the protocol it speaks. NavLink is Apache-2.0 licensed, so a third party can build against it. Navigator builds against the version of NavLink pinned by the `navlink/` submodule (commit `ee2a9f6`, which the NavLink repository tags v2.4.0).

Navigator's build generates its NavLink C codec from `navlink/dialect.json` at CMake configure time, so the codec is never checked in.

## What it does

From the source tree, Navigator provides:

- **Live telemetry.** Attitude (2D and 3D views), IMU, RC channels, motors, barometer, battery voltage, control-loop data, faults, and a log panel. Chapter 2 explains how this stays smooth under load.
- **Links.** A serial port, or a UDP socket for a WiFi telemetry bridge. Chapter 3 covers both.
- **Commands.** Arm and disarm, PID gain upload, time-sync, flight mode and calibration, all built by `CommandCodec` and sent through a single guarded function.
- **Packet analysis.** A packet analyzer page with a filter expression, a per-type frequency ribbon, and link statistics.
- **Calibration.** A calibration wizard.
- **Record and replay.** Raw inbound bytes can be recorded to a `.bin` file and replayed later through the same decoder, read-only.
- **An in-app simulator.** The firmware compiled for the host, loaded at runtime from a separate release asset, so you can fly the real control logic with no hardware.
- **Autotune.** A native C++ autotuner launched from the simulator page.
- **Extras.** An editable shortcut and command registry, a command palette, and optional propeller and alert audio.

There is also an experimental dockable layout, covered later.

## Who it is for

Pilots and bench engineers who need to connect to a board, watch it, calibrate it, tune it, and replay a log afterwards. Firmware developers who want to exercise the control code in the simulator before touching hardware. Integrators who want to script the same stack through the headless SDK.

## What it is not

- It is not the firmware. Nothing in this repository builds firmware. The simulator needs a firmware release (the README names the vayu releases and says CI builds against v0.2.1). Without it, Navigator still builds and runs and reports the simulator as unavailable.
- It is not a mission planner or a map. `requirements.md` lists mission upload, a parameter editor, and a map page as missing; a whole-word, case-insensitive search of `navigator/src` for "mission" and "waypoint" finds only one code comment mentioning another product's UI style. Gap: that requirements file is a living document with parts that look older than the code (for example, it says arming is blocked on firmware, while the code builds arm and disarm frames), so treat its status columns with care.
- It is not a protocol library. NavLink is a separate repository.

## Building, briefly

Per the README: initialise submodules, then `cmake -S navigator -B build` and `cmake --build build -j`. The CMake files require C++17, CMake 3.20 or newer, and Qt6 with the components Core, Widgets, SerialPort, Network, OpenGL, OpenGLWidgets and Concurrent. They set no minimum Qt6 version. The simulator needs assimp, and `-DNAVIGATOR_SITL=OFF` builds the GCS without it.

Gap: the README lists libpulse as required. The CMake file treats PulseAudio as optional and builds a silent audio stub when it is absent.

Gap: supported platforms are not stated authoritatively. The README says CI builds Linux with the simulator and a GCS-only Windows build; `requirements.md` has a tier table that this chapter does not rely on.

## Sources

- `README.md` (layout, building, simulator, branches, license)
- `navigator/docs/README.md`
- `navigator/docs/reference/requirements.md` (scope: sections 1 and 2)
- `navigator/docs/reference/feature-matrix.md`
- `navigator/CMakeLists.txt`
- `navigator/src/app/main.cpp`, `navigator/src/ui/main/MainWindow.cpp`
- `.gitmodules`

## Checked against the code

- Required Qt6 components are Core, Widgets, SerialPort, Network, OpenGL, OpenGLWidgets, Concurrent, with no version floor -> `navigator/CMakeLists.txt` (`find_package(Qt6 REQUIRED COMPONENTS ...)`)
- C++17 and CMake 3.20 are required -> `navigator/CMakeLists.txt` lines 1-5
- The NavLink codec is generated at configure time from `navlink/dialect.json` -> `navigator/CMakeLists.txt` (`generate.py --lang c`)
- PulseAudio is optional, with a silent stub fallback -> `navigator/CMakeLists.txt` (`find_library(PULSE_LIB pulse)` block)
- assimp is required only on the simulator path -> `navigator/CMakeLists.txt` (`find_package(assimp REQUIRED)` inside the `NAVIGATOR_SITL` gate)
- Arm and disarm frames are built and sent through the guarded function -> `navigator/src/ui/main/MainWindow.cpp` (`sendToFc(... encodeArm ...)`)
- The application name is "Vayu GCS" -> `navigator/src/app/main.cpp`
- The navlink submodule is pinned at commit `ee2a9f6`, tagged v2.4.0 in the NavLink repository -> `git ls-tree HEAD navlink`, `git describe --tags` in navlink/
