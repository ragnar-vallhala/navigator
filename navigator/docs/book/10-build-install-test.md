# Chapter 10 — Build, install, test

## Release status

There is no packaged release. The repository has no git tags, locally or on the remote. `project(Navigator VERSION 1.0.0)` in `navigator/CMakeLists.txt` sets the compiled-in version string, but no installer, archive or binary is published from this repository. You build from source. The `stable` branch exists and is meant to carry releases (see Branches below), but a branch is not a release artifact.

## What you need

From `navigator/CMakeLists.txt`:

- CMake 3.20 or later, a C++17 compiler, and `python3` on the path. Configure runs `navlink/generate.py` to produce the C codec, and fails if that fails. Nothing generated is committed.
- Qt6 with the modules Core, Widgets, SerialPort, Network, OpenGL, OpenGLWidgets and Concurrent. No minimum Qt version is set. Tests add the Test module.
- assimp, only when the simulator is built (it imports the airframe mesh).
- PulseAudio (`pulse` and `pulse-simple`) is optional. Without it the propeller audio is a silent stub.
- The `navlink` submodule: `git submodule update --init --recursive`.

```
git submodule update --init --recursive
cmake -S navigator -B build
cmake --build build -j
```

## CMake options

| Option | Default | Effect |
|---|---|---|
| `NAVIGATOR_SITL` | ON | Build the in-app simulator and autotune. OFF gives a GCS-only binary. |
| `NAVIGATOR_SIM_GRASS` | OFF | Compile the grass and flora renderer. CI turns it on so static analysis sees that code. |
| `NAVIGATOR_BUILD_TESTS` | ON | Build the QtTest targets and register them with ctest. |
| `NAVIGATOR_WORKSPACE_MOCK` | ON | Add the docking mock app. It configures nothing unless Qt Advanced Docking System is found. |
| `NAVIGATOR_FETCH_ADS` | OFF | Fetch and build that library from a pinned commit. Defined in `navigator/tools/workspace_mock/CMakeLists.txt`. |
| `VAYU_SITL_SDK` | see below | Cache path to the SDK include directory. |

Even with `NAVIGATOR_SITL=ON`, a missing SDK is only a CMake warning ("SITL features disabled"). The app still builds. CMake looks for the SDK at `sim/host/sdk` under the repository root if present, otherwise `vayu-sitl-sdk/include` there.

## The simulator needs a firmware release

Navigator contains no firmware and no physics. The simulator is a shared library, `libvayu_sitl`, built in the separate `vayu` repository and loaded at run time. Navigator compiles against two of its headers. The root `README.md` gives the fetch command; CI pins v0.2.1.

```
gh release download v0.2.1 --repo ragnar-vallhala/vayu \
    --pattern 'vayu-sitl-sdk-v0.2.1-linux-x86_64.tar.gz'
tar xzf vayu-sitl-sdk-v0.2.1-linux-x86_64.tar.gz
```

The `vayu` repository is private, so the download needs a GitHub login with access. At run time the module is searched for in this order: `$VAYU_SITL_MODULE`, beside the executable, `../lib` from the executable, then sibling `build_sitl_rtos` and `build_sitl` directories. A module with a different ABI version is refused at load.

The headless SDK reads `$VAYU_SITL_RTOS_BIN`. The in-app autotune does not: it looks under `$VAYU_REPO` (or the repository root found by walking up from the executable) in `build_sitl_rtos/` and `sim/host/build_sitl_rtos/`. A binary unpacked to `vayu-sitl-sdk/bin/` is not on that list, so for in-app autotune place or link it at `build_sitl_rtos/vayu_sitl_rtos` under the repository root. The root README says the autotuner finds it via `$VAYU_SITL_RTOS_BIN`; that holds for the Python tool and the headless SDK, not the in-app tuner.

## Windows (GCS-only)

CI builds Windows with `-DNAVIGATOR_SITL=OFF` on MSYS2 UCRT64 (toolchain, cmake, ninja, python, qt6-base, qt6-serialport), runs ctest, and keeps the SITL tabs compiled out. `tools/windows/deploy-navigator-msys.sh <build-dir>` bundles a standalone folder: it runs `windeployqt6`, copies the MinGW runtime DLLs and resolves the remaining DLLs with `ldd`, writing `dist/` in the build directory. The CMake file also contains Windows branches for the simulator (it links `winmm`), and one commit message describes it passing on Windows 10, but no CI job builds that configuration.

## Tests

`navigator/tests/CMakeLists.txt` registers 44 test targets through `navigator_test()`, one per `tst_*.cpp` file; ctest runs each with `QT_QPA_PLATFORM=offscreen`. The README's "20 QtTest cases" and a comment in `ci.yml` are out of date. How the 44 split, derived from the CMake conditions:

- 32 build in every configuration, including a Windows GCS-only build.
- 8 need `NAVIGATOR_SITL=ON` (7 of them also need the SDK headers).
- 1 needs `NAVIGATOR_SIM_GRASS=ON`.
- 3 need Qt Advanced Docking System.

Some tests skip, and ctest reports a skip as a pass. Those that load the engine (`tst_sitl_engine`, `tst_airframe_flight`, `tst_sim_restart`) skip without a module, and the GL tests skip on the offscreen platform. CI therefore runs `tst_sim_gl_context` and `tst_world_change_floor` directly under Xvfb with software GL and fails if their totals show skips.

The Python suite is separate: `cd navigator/headless-sdk && python3 -m pytest tests`. Its integration tests need the engine binary and the generated Python codec (`python3 navlink/generate.py --lang both`). CI does not run it.

## CI

Workflows are in `.github/workflows/`. `ci.yml` runs on pushes and pull requests to `main` and `stable`.

- **changes**: marks a change as docs-only when every file is Markdown or under `docs/`, so build jobs skip.
- **commit-lint** (pull requests): runs `tools/dev/lint_commits.sh` over the new commits.
- **navigator**: installs Qt6, assimp and tools with apt; fetches the pinned SDK with a repository secret (so fork PRs fail with a message); builds with SITL and grass on; fails if SITL turned itself off; runs ctest, the GL test and the world-change test; runs `clang-tidy-18` through `tools/dev/run_clang_tidy.sh`.
- **workspace**: builds the docking mock and two of its tests with SITL off.
- **windows**: the GCS-only build above.
- **ci-required**: passes when every job succeeded or was skipped. This is the one required check.

`semver-gate.yml` runs only on pull requests into `stable`. It reads the version from `navigator/CMakeLists.txt` on both sides and fails unless the PR's version is strictly greater.

## Commit messages and branches

`lint_commits.sh` requires `<type>[(<scope>)][!]: <subject>` with a type from feat, fix, docs, style, refactor, perf, test, build, ci, chore, revert; a subject of at most 72 characters; a blank second line; and no `Co-Authored-By` or "generated with" trailer naming an AI. Merge commits and commits already on `main` or `stable` are skipped. Enable the same check locally with `git config core.hooksPath .githooks`.

Changes reach `main` by pull request. A release is a pull request from `main` to `stable` that raises the version, merged with a merge commit.

Gap: no document says which Qt6 versions are supported; CI uses whatever the Ubuntu runner provides.

Gap: nothing describes an install step (`cmake --install`) or packaging; none is defined in the CMake files read.

## Sources

- README.md, .github/workflows/ci.yml, .github/workflows/semver-gate.yml, .githooks/commit-msg
- navigator/CMakeLists.txt, navigator/tests/CMakeLists.txt, navigator/tools/workspace_mock/CMakeLists.txt
- navigator/src/vsim/SitlModule.cpp, navigator/src/ui/widgets/SimulatorWidget.cpp
- tools/dev/lint_commits.sh, tools/windows/deploy-navigator-msys.sh

## Checked against the code

- No tags locally or on the remote -> `git tag -l` and `git ls-remote --tags origin` both empty
- Requirements and options -> navigator/CMakeLists.txt
- `NAVIGATOR_FETCH_ADS` default OFF -> navigator/tools/workspace_mock/CMakeLists.txt
- 44 test targets, with conditions -> navigator/tests/CMakeLists.txt
- Module search order -> navigator/src/vsim/SitlModule.cpp, comment in navigator/CMakeLists.txt
- CI jobs, pinned v0.2.1 -> .github/workflows/ci.yml
- Commit rules -> tools/dev/lint_commits.sh
- Windows job uses `-DNAVIGATOR_SITL=OFF` -> .github/workflows/ci.yml
