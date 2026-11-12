# Chapter 8 — Autotune and the headless SDK

Both features run the real flight-controller logic against the simulator without a person at the controls. They share one engine binary, `vayu_sitl_rtos`, which this repository does not build (Chapter 10 explains where it comes from). This chapter covers PID autotuning, in two implementations, and the `vayu_headless` Python package.

## PID autotuning, two implementations

| | Python tool | Native GCS autotune |
|---|---|---|
| Location | `tools/autotune/` | `navigator/src/autotune/` |
| Runs from | command line | Simulator page, Tuning tab |
| Engine use | long-running stack over pseudo-terminals, real-time pace | one short subprocess per gain set |
| Status | original; the C++ code is a port of it | the one the GCS uses |

The C++ headers say so directly: `Optimizer.h` is "a faithful C++ port of tools/autotune/optimizers.py", `Cost.h` ports the scoring in `autotune.py`, `Space.h` ports its parameter space, and `RtosEval.h` ports `rtos_eval.py`. The point of the port was to run the search inside the GCS with no `python3` subprocess.

### The native autotune

`AutotuneWorker` runs on a worker thread and has two modes.

**Optimizer search** (the default). `AutotuneEngine` drives an optimizer from `Optimizer.h` over a cost function. For each candidate gain vector, `RtosEval` launches `vayu_sitl_rtos` once as a subprocess with the gains passed as environment variables (`VAYU_RATE_KP` and similar, plus `VAYU_RTOS_SCENARIO=doublet`, a seed, and a private `VSIM_FIFO_SUFFIX`). The binary flies a seeded step doublet and writes the response window; `Cost.cpp` scores it. The code comments and the Python tool's docstring describe a rollout as about 0.04 s of wall time and bit-deterministic for a given seed and gains; those are the source's claims, not measurements made for this book.

The optimizer names are `random`, `spsa`, `fdgd`, `coordinate`, `nelder-mead`, `structured`, `hybrid` and `portfolio`. The worker's defaults are `structured`, a budget of 30 evaluations and 5 repeats per evaluation. The cost combines tracking error, overshoot and a chatter penalty, and returns a large penalty (1e6) when the angle passes 80 degrees. The fast backend can only set the rate loop's kp, ki and kd, the angle kp and, with yaw enabled, the yaw rate kp; `Space` drops the other gains in that mode because the backend has no hook for them.

**System identification.** With `sysId` set, the worker flies a single chirp, fits a first-order actuator lag per axis (`SysId.cpp`) and computes `rate_kp`, `rate_ki`, `rate_kd` and `angle_kp` by loop shaping instead of searching. `SysId.h` gives the reason: a cost-based search can settle on a "do not move" minimum.

The in-app tuner finds the binary at `build_sitl_rtos/vayu_sitl_rtos` or `sim/host/build_sitl_rtos/vayu_sitl_rtos` under `$VAYU_REPO` (or the repository root found from the executable); it does not read `VAYU_SITL_RTOS_BIN`.

Applying the result sends `CMD_SET_PID` frames from the main window, to a connected flight controller or to the running in-app simulator. If neither is live the request is ignored with a log message.

The Python tool's README warns to validate tuned gains on a tethered bench and never to tune on hardware unattended.

### The Python tool

`tools/autotune/autotune.py` takes `--optimizer` (the same eight names), `--compare` to rank every optimizer, `--budget`, `--repeats`, `--yaw`, `--apply` to push and persist the best gains, `--plot` for a live dashboard and `--out` (default `autotune_result.json`). `sitl.py` spawns the binary once and drives it over a pty. `rtos_eval.py` is the fast backend and is usable as a library. The rest are analysis helpers: `plant_sweep.py`, `validate_lockstep_determinism.py`, `verify_flakiness.py`, `verify_loops_mode.py`, and figure scripts.

## The headless SDK

`navigator/headless-sdk/` is an installable package, `vayu_headless` 0.0.1, with no runtime dependencies and Python 3.9 or later. It uses `pty` and POSIX file descriptors, so it targets Linux.

```
SitlSession --spawns--> vayu_sitl_rtos  (VAYU_RTOS_SCENARIO=driver)
   |  RC frames over a pty (VAYU_UART_RC_PATH)
   |  pose and control over FIFOs  /tmp/vsim_pose<suffix>, /tmp/vsim_ctl<suffix>
   |  UART2 telemetry over a pty advertised at /tmp/vayu_uart2_pty<suffix>
Pilot --writes RC sticks only--> SitlSession
```

`SitlSession` spawns the binary with `VSIM_FIFO_SUFFIX` set to `_lab<pid>`, so many sessions can coexist, and it waits for the FIFOs to appear. It reads the GCS settings file (default `~/.config/Vayu/Vayu GCS.conf`) so the simulated vehicle and world match what the GCS has loaded. With `gcs=True` it rebroadcasts pose to `/tmp/vsim_pose` and bridges UART2 so the GCS can use "Attach Ext" and the "SITL UART2" port. `Pilot` is a guidance loop that writes only RC sticks, so position stays pure physics apart from takeoff and land resets.

Environment variables read by the package: `VAYU_SITL_RTOS_BIN` (the binary), `VAYU_REPO_ROOT` (repo root), `VSIM_WORLDMESH_BIN` (mesh tool), `VAYU_GCS_CONF` (settings file) and `SITL_LAB_DEBUG` (send the engine's stderr to `/tmp/sitl.err`). Without `VAYU_SITL_RTOS_BIN` the package looks for `sim/host/build_sitl_rtos/vayu_sitl_rtos` under the repo root, which does not exist in this repository, so set the variable.

The command `vayu-headless` has `serve`, `do` and `run`. `serve` keeps a session open behind the socket `/tmp/sitl_lab.sock`; `do` sends it a text or JSON command (`takeoff`, `goto`, `fly`, `alt`, `wait`, `rc`, `land`, `status`, `quit`); `run` flies one course and exits. `examples/` has step, chirp and doublet system-ID scripts that log ground truth to CSV, plus `figure8.py` and `box_mission.py`. `fidelity/` holds a separate scorecard suite with its own README.

Tests: `tests/unit` defines 45 test functions and `tests/integration` 12, counted by `def test_`. Integration tests skip when the binary is missing. CI does not run this suite (Chapter 10).

Gap: the `PLAN.md` progress note ("51 tests", 6 integration flights) is out of date, and the `docs/FINDINGS.md` file the README points to is not in the tree.

Gap: no document states how long a real autotune takes on a given machine.

## Sources

- tools/autotune/README.md, autotune.py, sitl.py, rtos_eval.py, optimizers.py
- navigator/src/autotune/ (all headers; AutotuneWorker.cpp, RtosEval.cpp, Space.cpp, Optimizer.cpp)
- navigator/src/ui/widgets/SimulatorWidget.cpp
- navigator/headless-sdk/README.md, PLAN.md, pyproject.toml, fidelity/README.md
- navigator/headless-sdk/vayu_headless/ (session.py, paths.py, _repo.py, cli.py, server.py), tests/conftest.py, examples/

## Checked against the code

- Native autotune is a port of the Python tool -> navigator/src/autotune/Optimizer.h, Cost.h, Space.h, RtosEval.h
- Eight optimizer names -> navigator/src/autotune/Optimizer.cpp, tools/autotune/optimizers.py
- Defaults structured / 30 / 5 -> navigator/src/autotune/AutotuneWorker.h
- Subprocess per rollout with env gains -> navigator/src/autotune/RtosEval.cpp
- Apply targets a live flight controller or the running sim -> navigator/src/ui/main/MainWindow.cpp
- Session spawn, env vars, FIFO names -> navigator/headless-sdk/vayu_headless/session.py, paths.py
- Version, Python floor, no dependencies -> navigator/headless-sdk/pyproject.toml
- Test function counts 45 and 12 -> navigator/headless-sdk/tests/
