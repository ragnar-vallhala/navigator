# Chapter 3 — Links and the telemetry source state machine

Navigator is fed from a serial port, a UDP socket, the in-app simulator, or a recorded file, never a mix. A small state machine, `SourceController`, decides which is active and whether commands may go out.

## The five states

`SourceState` has five values:

| State | Meaning |
|---|---|
| `Idle` | App start, or disconnected. No feed. |
| `Fc` | A live serial or UDP link to a flight controller. |
| `Sim` | The in-app simulator. |
| `Autotune` | An isolated tuner run. It does not feed the telemetry engine. |
| `Replay` | A recorded `.bin` file played through the parser. |

`txAllowed()` is true only in `Fc` and `Sim`. `feedActive()` is true in `Fc`, `Sim` and `Replay`.

## How a transition works

The graph is fully connected: any state can move to any other. Each `request...()` call does the same three steps. It tears down the current source, runs the setup for the new one, then calls `applyFeed` so the engine listens to the right place. If setup fails, the machine lands in `Idle`. The controller calls hooks that `MainWindow` supplies and holds no widgets or engine, so it can be tested headless (`navigator/tests/tst_source_controller.cpp`).

`MainWindow::applyFeed` does the actual swap. It disconnects the previous `ITelemetrySource` from the engine's `feedBytes`, connects the new one (`ReplaySource` for `Replay`, `SimSource` for `Sim`, none otherwise), and queues `setLiveFeed(true)` on the engine only for `Fc`. There is no `FcSource` class: the live serial and UDP feed is internal to the engine, and `setLiveFeed` switches it on and off.

## Serial

`SerialManager` opens the port as 8 data bits, no parity, one stop bit, no flow control.

Before opening, `MainWindow` claims the port from `PortArbiter`. The arbiter's policy is last acquire wins. If the in-sim RC bridge takes the same tty, `MainWindow` logs it and closes the board link, so two readers never split one byte stream.

### Auto-reconnect

Auto-reconnect is a setting, off by default. When on, and the port closes on an error (not a deliberate Disconnect), `SerialManager` retries the last port and baud. The first wait is the base delay (default 1000 ms, adjustable, floor 100 ms); each failed attempt doubles it, capped at 8000 ms. After 5 attempts it gives up and says so. A successful open resets the counter.

## Network (UDP)

`UdpManager` binds a local UDP port on any IPv4 address. Typing `udp` as the port selects UDP with the default port 14555; `udp:<port>` selects another. A code comment names an ESP8266 WiFi telemetry bridge as the peer.

It announces itself with a `GCS-HELLO` datagram: broadcast at bind, then once a second. Once it has heard telemetry it learns the sender's address and unicasts both the hello and outgoing commands there; until then commands are broadcast. When both links are open, `TelemetryEngine::send` prefers UDP. UDP has no reconnect logic; the hello timer lets a rebooted bridge find the GCS again.

## Replay

`ReplaySource` loads a `.bin` and emits recorded frames on a 50 Hz tick, scaled by a speed factor, with an optional crop range and loop. Entering `Replay` stops any live recording first, so replayed frames are not re-recorded. Because `Replay` is not `txAllowed`, every command path hits the guard in `MainWindow::sendToFc`, which shows "Read-only — not connected". If the file cannot be opened, setup fails and the state becomes `Idle`.

## Sim and Autotune

Simulator bytes reach the engine through `SimSource::feed`. In `Sim`, `sendToFc` hands commands to the loaded simulator module instead of a transport. `Autotune` feeds nothing and cannot send. Entering any other state stops the sim or cancels the tuner.

## Connect, lose, recover

1. **Connect.** The toolbar calls `requestFc(port, baud)`. Setup queues `openSerial` or `bindUdp` on the worker and returns true at once, since the open is asynchronous. The state is `Fc` immediately.
2. **Open fails.** The engine's `linkOpened(false, ...)` signal makes `MainWindow` call `forceIdle()`, which tears down and returns to `Idle`.
3. **Disconnect.** `requestIdle()` closes the links. Recording stops.
4. **Transport error.** `SerialManager` closes the port and emits the error. `MainWindow` shows it in the log and status bar and marks the UI disconnected. The code does not change the state machine here: it stays `Fc`. A successful auto-reconnect restores the UI.
5. **Heartbeat loss.** While in `Fc`, a watchdog compares time since the last heartbeat with a timeout (default 1500 ms, a setting). On timeout the status bar shows "Link lost — no heartbeat for N ms". The transport stays open, and the pill is restored when heartbeats resume.

## Code and doc disagreements

- `gcs-architecture.md` draws `Fc --> Idle : forceIdle (link loss)` and shows only some transitions. In the code, `forceIdle()` is called from two places: the failed-open handler and `MainWindow` shutdown. Link loss and serial errors do not call it. The graph is fully connected, per the header comment and `transition()`.
- The doc cites `autotune/SitlStack`; no such file exists in `navigator/src/autotune`.

Gap: the app was not run; step 4 is read from code, not observed.

## Sources

- `navigator/docs/reference/gcs-architecture.md`, section 2 and notes
- `navigator/src/core/` (`SourceState.h`, `SourceController.{h,cpp}`, `SettingsManager.h`)
- `navigator/src/comm/`, `navigator/src/replay/ReplaySource.{h,cpp}`
- `navigator/src/protocol/TelemetryEngine.{h,cpp}`, `navigator/src/ui/main/MainWindow.cpp`

## Checked against the code

- Five states, `txAllowed` is Fc or Sim, `feedActive` is Fc, Sim or Replay -> `core/SourceState.h`, `core/SourceController.h`
- Failed setup lands in `Idle`; teardown precedes setup -> `core/SourceController.cpp` `transition()`
- `forceIdle()` callers are the failed-open handler and shutdown only -> `ui/main/MainWindow.cpp` (grep `forceIdle`)
- Reconnect: base 1000 ms, cap 8000 ms, 5 attempts, off by default -> `comm/SerialManager.h`, `SerialManager.cpp`, `core/SettingsManager.h`
- Default UDP port 14555 and `GCS-HELLO` every 1000 ms -> `ui/main/MainWindow.cpp` `setupFc`, `comm/UdpManager.cpp`
- Heartbeat watchdog applies only in `Fc`, default 1500 ms, leaves the transport open -> `MainWindow.cpp` `updateLiveBlinker`, `SettingsManager.h`
- Replay tick is 20 ms (50 Hz); replay is read-only via the `sendToFc` guard -> `replay/ReplaySource.cpp`, `MainWindow.cpp` `sendToFc`
- Last acquire wins; a revoked board link is closed -> `comm/PortArbiter.h`, `MainWindow.cpp` constructor
