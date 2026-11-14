# Flight event log (`ev-*.jsonl`)

Every simulator run writes two files into the log directory (Simulator panel,
**Log dir**):

| file | content |
| --- | --- |
| `gt-<stamp>.bin` | physics ground truth, binary (unchanged; see `parse_gt.py`) |
| `ev-<stamp>.jsonl` | what happened during the run: RC input, mode, arm, injected faults, markers |

`<stamp>` is the local start time, `yyyy-MM-dd_HH-mm-ss`, and is identical for
the two files of a run. The writer is `vsim::FlightEventLog`
(`src/vsim/FlightEventLog.{h,cpp}`), opened when the gt log opens and closed
when it closes. Each line is flushed as it is written, so a crash leaves a
readable prefix (a half-written last line is possible; readers should skip a
line that does not parse).

## Format

JSON Lines: one JSON object per line, UTF-8. Line 1 is the header; every other
line is an event.

### Header (line 1)

Keys are always present; an unknown value is `null`, never omitted.

| key | type | meaning |
| --- | --- | --- |
| `schema` | string | `"nav-flight-events/1"` |
| `run_id` | string | the run's `<stamp>` |
| `app_version` | string | Navigator version (`"dev"` for an unversioned build) |
| `sdk_version` | int or null | the simulator module's ABI version (`abi_version` of the loaded `libvayu_sitl`) |
| `module_build_id` | string or null | the module's own build id string (extra to the ABI number: the ABI only proves the layout) |
| `vehicle_sha256` | string or null | hex SHA-256 of the **canonical JSON serialisation** of the vehicle geometry the run uses (`geometryToJson`, compact, keys sorted). The app keeps the geometry in its settings, not in a file, so there is no file content to hash. Equal hash = same mass, inertia, motors, mixer inputs |
| `world_preset` | string or null | id of the world preset that filled the world config; `null` for a custom world |
| `world_seed` | number or null | the world's procedural seed |
| `gt_file` | string | file name of the paired ground-truth log |
| `t0_unix_ms` | int | wall-clock Unix time in ms when the log opened (the instant `t_us` = 0) |

### Time base

`t_us` is **microseconds since the log opened, from a monotonic clock**
(`QElapsedTimer`, the same `m_runClock` that stamps the `t_us` field of every
`gt-*.bin` record). The two files therefore share one time axis: join on
`t_us`. Wall time of an event is `t0_unix_ms * 1000 + t_us` (µs, subject to
clock slew). Within a file `t_us` never decreases.

### Events

Every event has `t_us` (int, µs) and `kind`.

| kind | extra fields | when |
| --- | --- | --- |
| `start` | none | right after the header (`t_us` 0) |
| `rc` | `channels` (int array, µs, order roll, pitch, throttle, yaw, arm, ch6), `source` (`"joystick"`, `"uart"`, or `"disabled"` when RC is switched off in the UI) | when any channel changes, and at least every 100 ms (10 Hz) otherwise while the run is live |
| `mode` | `from` (string or null for the first report), `to`, `source` (`"rc"` or `"gcs"`) | the firmware reports a different flight mode (`STABILISE`, `ACRO`) or mode source |
| `arm` | `armed` (bool) | the armed state seen in telemetry changes |
| `fault` | `id` (string), `active` (bool), `detail` (string or null) | a simulator fault is injected (`active` true) or cleared |
| `marker` | `text` (string) | the user pressed **Add marker** (or called `SimulatorWidget::addMarker`) |
| `stop` | `reason` (`"user"`, `"fault"`, `"exit"`) | the run ends: Stop pressed, the engine worker died, or Navigator closing |

Fault ids today: `motor1_kill` ... `motor4_kill` and `rc_link_loss`.

What is **not** recorded, because the app has no such source: keyboard RC input
(only joystick and UART sources exist), an IMU-dropout fault (the flag exists
but nothing in the UI sets it), GPS faults (no GPS model), and flight mode
changes other than STABILISE/ACRO.

## Example

From a real 3.4 s run (the `rc` stream is cut to its first sample):

```json
{"app_version":"1.0.0","gt_file":"gt-2026-11-15_00-24-26.bin","module_build_id":"v0.2.1 2026-11-10","run_id":"2026-11-15_00-24-26","schema":"nav-flight-events/1","sdk_version":4,"t0_unix_ms":1794682466897,"vehicle_sha256":"83fe75e2815ce9413ffecaa42b106e11d49efc802aa47877dacb8a6f285f55fc","world_preset":null,"world_seed":1337}
{"kind":"start","t_us":0}
{"channels":[1500,1500,1000,1500,1500,1000],"kind":"rc","source":"joystick","t_us":14611}
{"from":null,"kind":"mode","source":"rc","t_us":1506179,"to":"STABILISE"}
{"armed":true,"kind":"arm","t_us":1808770}
{"active":true,"detail":"motor 1 killed","id":"motor1_kill","kind":"fault","t_us":2214234}
{"kind":"marker","t_us":2619358,"text":"gust test"}
{"from":"STABILISE","kind":"mode","source":"gcs","t_us":2619408,"to":"ACRO"}
{"active":false,"detail":"motor 1 killed","id":"motor1_kill","kind":"fault","t_us":3124204}
{"kind":"stop","reason":"user","t_us":3428001}
```

## Reading it

pandas (the snippet was **not run**: pandas is not installed on the machine
this was written on; the same parse was checked with the standard `json`
module):

```python
import json, pandas as pd

path = "ev-2026-11-15_00-24-26.jsonl"
header = json.loads(open(path).readline())
ev = pd.read_json(path, lines=True).iloc[1:]      # row 0 is the header
ev["t_s"] = ev["t_us"] / 1e6
print(ev[ev["kind"] != "rc"][["t_s", "kind"]])
```

Align with the ground truth by `t_us` (both are µs since the log opened); for
example `pd.merge_asof(gt.sort_values("t_us"), ev_rc.sort_values("t_us"), on="t_us")`.

PlotJuggler (not run here): *Data* > *Load data* > pick the `.jsonl` file; the
JSON loader reads one message per line, so `t_us` can be chosen as the time
axis and `channels[0]` ... `channels[5]` plot as series. The header line
appears as one extra sample; delete or ignore it.

## Keeping and listing runs

Setting `simulator/logKeepRuns` (QSettings), edited as **Keep last runs** next to
**Log dir** in the Simulator panel: default 50; `0` keeps everything. When a
run starts the oldest runs beyond the limit are deleted, `gt-*.bin` and
`ev-*.jsonl` of a run together (a lone file of either kind counts as one run).
Only files named exactly `gt-*.bin` or `ev-*.jsonl` in the log directory are
ever touched, and never the run that is opening. The limit counts the new run.

The **Flights** list under the log path shows the runs found in the log
directory, newest first, with the size of both files and the event count, and
buttons **Open folder**, **Copy path** and **Delete** (asks first; deletes both
files of the run; the run being recorded cannot be deleted).
