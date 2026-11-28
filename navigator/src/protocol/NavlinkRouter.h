/*
 * Copyright (C) 2026 NAVRobotec Pvt Ltd
 * Author: Ragnar Vallhala
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

#include "core/Types.h" // AttitudeData, ...
#include "ParamValueMsg.h"
#include <QByteArray>
#include <QString>
#include <cstdint>
#include <functional>

// The single home for the GCS's NavLink v2 *receive* handler table. This is the
// only translation unit that includes the generated codec for decoding, so the
// rest of the GCS never sees a navlink_* type — decoded leaves arrive as plain
// GCS structs through the std::function hooks below. Assign a hook from whatever
// module owns that data; any decoded leaf without a hook falls to onDefault
// (which just logs). Adding a message = one thunk + one hook + one ctor line in
// NavlinkRouter.cpp. See navlink/INTEGRATION.md.
class NavlinkRouter {
public:
  NavlinkRouter();
  ~NavlinkRouter();
  NavlinkRouter(const NavlinkRouter &) = delete;
  NavlinkRouter &operator=(const NavlinkRouter &) = delete;

  // Push one whole v2 frame (sync..CRC) through the parser; hooks fire inline.
  void feed(const QByteArray &v2Frame);

  // Where the parser sends the frames it emits itself (COMMAND_ACK for a
  // command addressed to the GCS). Unset by default: nothing is emitted and no
  // frame seq is consumed. The ACK's frame seq is taken from the GCS tx counter
  // (TxSeq.h), shared with every frame CommandCodec builds. Called on the
  // thread that calls feed().
  void setSendFrame(std::function<void(const QByteArray &)> f);
  std::function<void(const QByteArray &)> sendFrame; // public for the thunk

  // Per-leaf hooks — set by the owning GCS module (default: unset -> onDefault).
  std::function<void(const AttitudeData &)> onAttitude;
  std::function<void(const ImuData &)> onImu;
  std::function<void(const RcData &)> onRc;
  std::function<void(const MotorData &)> onMotor;
  std::function<void(const ControlLoopData &)> onControlLoop;
  std::function<void(const EstPerfData &)> onEstPerf;
  std::function<void(const BaroData &)> onBaro;
  std::function<void(const BatteryData &)> onBattery;
  std::function<void(const FailsafeStatusData &)> onFailsafeStatus;
  std::function<void(const GpsRawData &)> onGpsRaw;
  std::function<void(const VerticalStateData &)> onVerticalState;
  std::function<void(const NotchStatusData &)> onNotchStatus;
  std::function<void(const HslStatusData &)> onHslStatus;
  std::function<void(uint8_t mode, uint8_t source)> onFlightMode;
  // HEARTBEAT's nav_state and capabilities word (onHeartbeat keeps its own shape).
  std::function<void(uint8_t navState, uint32_t capabilities)> onHeartbeatCaps;
  // PARAM_VALUE, decoded (the parameter service, spec 12.2.1).
  std::function<void(const params::ParamCodec::ValueMsg &)> onParamValue;
  // HEARTBEAT: nav_state is the flight-state enum index; timestamp + device id
  // (frame sysid) drive link liveness. The owner maps nav_state -> state name.
  std::function<void(uint8_t navState, uint64_t timestamp, uint8_t deviceId)>
      onHeartbeat;
  // STATUSTEXT, after chunk reassembly. onStatusText gets the severity (the
  // navlink `severity` enum), the group id and the whole text; onLog is fed the
  // same message as one display line prefixed with the severity, "[ERR] text".
  std::function<void(uint8_t severity, uint16_t id, const QString &text)>
      onStatusText;
  std::function<void(const QString &line)> onLog;
  // The id only groups the chunks of one message; it is not a fault id.
  // Syslog-style label for a severity value ("ERR", "WARN", ...; "SEV n" if
  // unknown).
  static QString severityLabel(uint8_t severity);

  // STATUSTEXT reassembly (see NavlinkRouter.cpp for the assumed convention).
  // Called by the decoder thunk; public so the file-scope thunk can reach it.
  void handleStatusText(uint8_t severity, uint16_t id, uint8_t chunkSeq,
                        const char *text, size_t textCap);
  // Emit every pending group whose first chunk is older than kStatusTextStaleMs
  // as an incomplete message. feed() calls this on every frame; DroneProtocol
  // also calls it from a 500 ms timer on the router's thread, so a message cut
  // off by a quiet link still reaches the log.
  void flushStale(qint64 nowMs);
  void flushStale(); // same, using clockMs (default: monotonic clock)
  // Emit every pending group now, oldest first, as incomplete. Call on link
  // loss / close, before any state is dropped, so no text is lost.
  void flushAll();
  // Time source in ms (default: steady clock); tests inject a fake one.
  std::function<qint64()> clockMs;

  static constexpr qint64 kStatusTextStaleMs = 2000;
  static constexpr int kStatusTextMaxGroups = 8;
  static constexpr int kStatusTextMaxBytes = 1024; // pending text, all groups
  // FAULT_STATUS: one raised/cleared fault, name resolved from faults.json.
  std::function<void(const FaultEvent &)> onFault;
  std::function<void(const CalibrationUpdate &)> onCalibration;
  std::function<void(const PerfReport &)> onPerf; // reassembled report
  std::function<void(int taskId, const QString &name)> onTaskName;
  // TIME_SYNC RESPONSE: the four NTP stamps (t4 captured here, on receipt).
  std::function<void(uint8_t seq, uint64_t t1, uint64_t t2, uint64_t t3,
                     uint64_t t4)>
      onTimeSync;
  // COMMAND_ACK: the FC's response to a command, correlated by (command,reqSeq).
  std::function<void(uint32_t command, uint8_t reqSeq, uint8_t result)>
      onCommandAck;
  // XFER_INFO / XFER_ACK from the vehicle on an upload session (see
  // core/XferUploader.h). XFER_DATA / XFER_SACK are download-side: unused.
  std::function<void(uint8_t session, uint8_t result, uint16_t chunkSize,
                     uint32_t totalSize)>
      onXferInfo;
  std::function<void(uint8_t session, uint8_t flags, uint8_t result,
                     uint32_t nextOffset)>
      onXferAck;
  // SYSTEM_HEALTH: decoded but has no GCS consumer today (parity with v1, which
  // never surfaced it). A hook is provided for when one is added.
  std::function<void(uint32_t txOverflow, uint32_t imuDrop, uint32_t logWrap,
                     uint8_t cpuLoad)>
      onSystemHealth;
  // Fallback for every decoded leaf without a specific hook above.
  std::function<void(uint32_t msgid, int payloadLen)> onDefault;

  // IMU delta state: IMU_COMPRESSED carries half-float deltas vs the last
  // IMU_RAW, so the router reconstructs the absolute sample (as v1's
  // PacketDecoder did). Public so the file-scope thunks can reach it.
  ImuData lastImu;
  bool hasLastImu = false;

  // PERF reassembly: PERF_GLOBAL opens a report (carrying the expected task/fifo
  // counts), then PERF_TASK/PERF_FIFO rows (sharing its seq) fill it; onPerf
  // fires once both row sets have arrived. Public for the file-scope thunks.
  PerfReport perfAccum;
  bool perfHaveGlobal = false;
  int perfPendingTasks = 0;
  int perfPendingFifos = 0;

private:
  struct Impl;
  Impl *d_;
};
