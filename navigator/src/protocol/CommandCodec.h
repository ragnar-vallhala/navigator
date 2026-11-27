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

#include <QByteArray>
#include <QtGlobal>

// Builds NavLink v2 command frames (GCS -> FC). Every command is a typed v2
// message (10-byte header + truncated payload + CRC-16) built by the generated
// codec; the firmware decodes them in navlink_router.c and correlates the
// COMMAND_ACK by req_seq. `devId` becomes target_sys; `tsMs` is vestigial (v2
// frames carry no header timestamp) and kept only for call-site compatibility.
namespace CommandCodec {

// The vehicle the commands are addressed to: 42 until a heartbeat arrives, then
// the sysid of the FIRST heartbeat after connect (observeHeartbeat), kept until
// resetTarget(). Every encoder below defaults its `devId` to it, so a vehicle
// with another sysid is addressed correctly instead of being sent frames it
// ignores. It never moves to a second sysid while connected: a heartbeat from
// another craft must not redirect an ARM or DISARM. Thread-safe.
quint8 targetSys();

enum class TargetEvent {
  Ignored,  // sysid 0 (unset) or 0xFF (broadcast): not a vehicle
  Latched,  // first vehicle seen since the reset: now the target
  Same,     // the latched vehicle again
  Conflict, // another vehicle's heartbeat: the target is NOT changed
};
TargetEvent observeHeartbeat(quint8 sysid);

// Forget the latched vehicle (the link went down or came up): back to 42 and
// the next heartbeat latches.
void resetTarget();

// CMD_SET_PID: controller 0=angle / 1=rate, axis 0=roll / 1=pitch / 2=yaw.
QByteArray encodeSetPid(int controller, int axis, float kp, float ki, float kd,
                        float kff, quint8 devId = targetSys(), quint32 tsMs = 0);

// CMD_SET_GYRO_LPF: rate-loop gyro low-pass time constant for one axis
// (rc seconds; <= 0 disables).
QByteArray encodeSetGyroLpf(int axis, float rc, quint8 devId = targetSys(),
                            quint32 tsMs = 0);

// CMD_SET_GYRO_NOTCH: master enable + FFT dynamic-notch detection params applied
// live to every axis. Each of q / fmin / fmax / minRatio is <= 0 => leave that
// field unchanged on the FC (partial tune). autoband arms a one-shot auto-band
// learn that tightens [fmin,fmax] around the observed hover spectrum.
QByteArray encodeSetGyroNotch(bool enabled, float q, float fminHz, float fmaxHz,
                              float minRatio, bool autoband = false,
                              quint8 devId = targetSys(), quint32 tsMs = 0);

// CMD_NOTCH_AUTOBAND: arm the one-shot auto-band learn of the dynamic gyro notch
// (live only, nothing is stored). The settings themselves are NOTCH_* parameters.
QByteArray encodeNotchAutoband(quint8 devId = targetSys(), quint8 *reqSeq = nullptr);

// CMD_SET_FLIGHT_MODE: 0=angle, 1=acro, 2=release to RC (source = GCS).
QByteArray encodeSetFlightMode(int mode, quint8 devId = targetSys(), quint32 tsMs = 0);

// CMD_SET_MOTOR_GEOMETRY: per-motor body x/y [m] and spin (+1 CCW / -1 CW).
// Returns an empty frame (send nothing) if a position is not finite or a spin
// is not exactly +1 or -1.
QByteArray encodeSetMotorGeometry(const float x[4], const float y[4],
                                  const float spin[4], quint8 devId = targetSys(),
                                  quint32 tsMs = 0);

// CMD_ARM / CMD_DISARM: set/clear the software-arm latch (force = 0).
QByteArray encodeArm(quint8 devId = targetSys(), quint32 tsMs = 0);
QByteArray encodeDisarm(quint8 devId = targetSys(), quint32 tsMs = 0);

// CMD_CALIBRATE_IMU: `which` selects the routine; 0xFF cancels.
QByteArray encodeCalibrate(quint8 which, quint8 devId = targetSys());

// TIME_SYNC REQUEST: role=REQUEST, t1_gcs_tx stamped by the caller. `seq` is the
// sync round id carried in the payload (the response echoes it); the frame seq
// comes from the shared GCS counter (TxSeq.h) like every other frame.
// commandedOffsetMs = INT32_MIN means "no correction this round".
QByteArray encodeTimeSyncRequest(quint8 seq, quint64 t1,
                                 qint32 commandedOffsetMs, quint8 devId = targetSys());

// TIME_SYNC REQUEST_WIDE: full 64-bit correction split across the lo/hi words,
// for deviations beyond int32 ms (e.g. FC uptime clock vs GCS epoch on cold
// start). The hi word truncates off the wire on the normal int32 path.
QByteArray encodeTimeSyncRequestWide(quint8 seq, quint64 t1,
                                     qint64 commandedOffsetMs,
                                     quint8 devId = targetSys());

// PERF_TASKNAME_REQUEST: resolve one task id to its name.
QByteArray encodeTaskNameRequest(int taskId, quint8 devId = targetSys(),
                                 quint32 tsMs = 0);

// XFER (bulk transfer) upload frames, GCS -> FC. File mode, service 0 (SD).
// `path` is truncated to the 31 characters XFER_OPEN.arg can carry.
QByteArray encodeXferOpen(quint8 session, quint8 reqSeq, const QByteArray &path,
                          quint8 devId = targetSys());
// One XFER_DATA chunk (<= 247 bytes; extra is dropped). flags: see xfer::kFlag*.
QByteArray encodeXferData(quint8 session, quint8 flags, quint32 offset,
                          const QByteArray &payload);
QByteArray encodeXferClose(quint8 session, quint8 reqSeq, quint8 result,
                           quint8 devId = targetSys());

} // namespace CommandCodec
