# Chapter 4 — NavLink on the GCS side

This chapter covers how Navigator turns NavLink bytes into values a screen can show, and how a command goes back out. It does not describe NavLink itself (the frame layout, the CRC, the message catalog). The NavLink book covers that. Navigator builds against the NavLink submodule pin (on the branch this book was written from, commit `ee2a9f6`, which the NavLink repository tags `v2.4.0`), and its CMake generates the C codec from `navlink/dialect.json` when you configure the build.

## One file sees the decoder

The generated codec header, `navlink_msgs.h`, is included in exactly two source files under `navigator/src/protocol/`:

- `NavlinkRouter.cpp`, the receive side.
- `CommandCodec.cpp`, the send side.

Nothing else in the GCS sees a `navlink_*` type. Decoded data leaves the router as the GCS's own plain structs (`AttitudeData`, `ImuData`, `RcData` and so on, from `core/Types.h`).

## The receive path

```
bytes -> DroneProtocol::parseBuffer -> NavlinkRouter::feed
      -> generated parser -> thunk -> hook (std::function)
      -> DroneProtocol signal -> TelemetryEngine / widget slot
```

**Slicing.** `DroneProtocol::processData` appends bytes to a buffer. `parseBuffer` looks for the sync byte `0x56`, checks that the next byte is `0x02` (the v2 version byte), reads the payload length from byte 2, and waits until a whole frame (10-byte header, payload, 2-byte CRC) is present. Bytes that are not part of a v2 frame are emitted as `unknownPacket`. Each whole frame is emitted as `packetReceived`, which is what the packet analyzer and the engine's packet counter listen to. Then the frame goes to `NavlinkRouter::feed`.

**Dispatch.** `NavlinkRouter` owns a `navlink_parser_t` and a `navlink_handlers_t`. The constructor zeroes every handler slot, then fills in the ones the GCS cares about (24 message slots plus `on_default` in `NavlinkRouter.cpp`). The generated parser checks the CRC and calls the matching slot. Each slot is a file-local function, a thunk, that converts the NavLink struct into a GCS struct and calls a public hook on the router. Some thunks do real work:

- Attitude is converted from radians to degrees.
- `IMU_COMPRESSED` carries half-float deltas, so the router keeps `lastImu` and rebuilds the absolute sample. Deltas that arrive before any `IMU_RAW` are dropped.
- RC keeps the first 14 of 18 channels. Motors keep the first 4 of 8. (`NavlinkRouter.cpp`: the RC thunk comment reads "RcData holds 14 channels; v2 carries 18 — take the first 14", with a 14-iteration copy loop; the motor thunk likewise: "MotorData holds 4 motors; v2 carries 8 — take the first 4".)
- `PERF_GLOBAL`, `PERF_TASK` and `PERF_FIFO` are reassembled into one `PerfReport`, emitted when the expected rows have arrived.
- `FAULT_STATUS` gets its fault name from the generated fault table.
- `TIME_SYNC` is passed on only for the RESPONSE role, and the router stamps the receive time (t4) itself.

**Hooks.** The hooks are public `std::function` members of `NavlinkRouter` (`onAttitude`, `onImu`, `onRc`, `onMotor`, `onHeartbeat`, `onCommandAck`, and so on). `DroneProtocol`'s constructor assigns each one a lambda that emits a Qt signal (`attitudeReceived`, `imuReceived`, ...). Everything runs inline on the thread that called `feed`. In a live session that is the telemetry worker thread (Chapter 2).

**Where the signals go.** High-rate signals are consumed inside `TelemetryEngine`, which stores them in the mutex-guarded `VehicleState`; the GUI reads a snapshot (Chapter 2). Low-rate and event signals (log, fault, heartbeat, time-sync, perf, command ack) are connected to GUI slots in `MainWindow`. Some panels connect themselves: `ControlLoopPlot`, `CalibrationWidget` and `GyroNotchWidget` each take a `DroneProtocol*` through `setProtocol`.

## Adding a message

The header comment in `NavlinkRouter.h` says a new message takes one thunk, one hook and one constructor line. That is correct for the router. To get the data to a screen you also touch `DroneProtocol`:

1. In `NavlinkRouter.h`, add a `std::function` hook.
2. In `NavlinkRouter.cpp`, add a thunk that fills a GCS struct and calls the hook.
3. In the `NavlinkRouter` constructor, assign the thunk to the generated slot (`d_->handlers.on_<message> = thunk...`).
4. In `DroneProtocol.h/.cpp`, declare a signal and assign the hook a lambda that emits it.
5. Connect the signal to a consumer (an engine lambda for state, or a widget slot).

Two behaviours are worth knowing. A message with no thunk reaches `on_default`, which calls `onDefault`; `DroneProtocol` sets that to write a "[navlink] v2 msgid N (L B)" line to the log. But a message that has a thunk and an unset hook is dropped silently, because the thunk checks the hook and returns. `SYSTEM_HEALTH` works this way today: the hook exists, `DroneProtocol` leaves it unset, and the header says so. The comment in `NavlinkRouter.h` ("any decoded leaf without a hook falls to onDefault") is therefore only true for messages that have no thunk. Also, `on_crc_error` is not wired, so a frame that fails its CRC produces no log line.

## The send path

`CommandCodec` is a namespace of `encode*` functions, one per command: `encodeSetPid`, `encodeSetGyroLpf`, `encodeSetGyroNotch`, `encodeSetFlightMode`, `encodeSetMotorGeometry`, `encodeArm`, `encodeDisarm`, `encodeCalibrate`, `encodeTimeSyncRequest`, `encodeTimeSyncRequestWide` and `encodeTaskNameRequest`. Each fills a generated command struct (target system from the `devId` argument, component 1, a rolling `req_seq` from a static counter), calls the generated `navlink_*_encode`, and returns a `QByteArray` holding the whole frame. The frame is sent with system id `0xFF`, component 1. The `tsMs` argument is unused; the header says it is kept for call-site compatibility.

The frame then goes through `MainWindow::sendToFc`. It refuses when the source state does not allow transmit (for example during replay). In the in-app simulator it hands the frame to `SitlModule::send`, so the simulated firmware parses it as it would on a wire. For a real link it posts `TelemetryEngine::send` to the worker thread, which writes to UDP if a UDP socket is open and otherwise to the serial port.

The firmware answers with `COMMAND_ACK`. `NavlinkRouter` hands it to `onCommandAck`, and `DroneProtocol` emits `commandAckReceived(command, reqSeq, result)` and logs a line with the result name. The simulator page uses these acks to retry its start-up commands.

## Gaps

Gap: the repository does not say how many handler slots the pinned codec offers in total, because the generated header is not checked in. The count of wired slots above comes from the constructor.

## Sources

- `navigator/src/protocol/NavlinkRouter.{h,cpp}`
- `navigator/src/protocol/DroneProtocol.{h,cpp}`
- `navigator/src/protocol/CommandCodec.{h,cpp}`
- `navigator/src/protocol/TelemetryEngine.cpp`
- `navigator/src/ui/main/MainWindow.cpp` (`sendToFc`, `applyFeed`, signal wiring)
- `navlink/ABI.md` sections 3 (generated API surface) and 6 (Navigator / GCS C++ integration)
- `navlink/generate.py` (handler dispatch to `on_default`)
- `navigator/docs/reference/gcs-architecture.md` sections 3 and 4
- `navigator/CMakeLists.txt` (codegen step)

## Checked against the code

- The codec header is included only in `NavlinkRouter.cpp` and `CommandCodec.cpp` -> `grep navlink_msgs.h` under `navigator/src`
- Frame slicing: sync `0x56`, version `0x02`, payload length at byte 2, whole frame emitted as `packetReceived` before `feed` -> `DroneProtocol.cpp` `parseBuffer`
- A new router message needs a hook, a thunk and a constructor line; reaching a screen also needs a `DroneProtocol` signal and lambda -> `NavlinkRouter.h/.cpp`, `DroneProtocol.cpp`
- A thunk with an unset hook drops the message; no thunk falls to `onDefault` -> thunk bodies in `NavlinkRouter.cpp`, and `navlink/generate.py` (`if (h->on_default)` branch)
- `on_crc_error` is never assigned -> `NavlinkRouter` constructor (`handlers = {}` then only `on_*` message slots)
- Commands use a static rolling `req_seq`, sysid `0xFF`, component 1 -> `CommandCodec.cpp`
- Transmit routing (refuse / SITL send / engine send) -> `MainWindow::sendToFc`; UDP-else-serial -> `TelemetryEngine::send`
- `ABI.md` sections 3 and 6 match the pinned submodule commit (same text as the older tag; only sections 4, 5 and 8 differ) -> `git diff` of `ABI.md` between the two NavLink revisions
