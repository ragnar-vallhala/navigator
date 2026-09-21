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

// CMD_SET_PID: controller 0=angle / 1=rate, axis 0=roll / 1=pitch / 2=yaw.
QByteArray encodeSetPid(int controller, int axis, float kp, float ki, float kd,
                        float kff, quint8 devId = 42, quint32 tsMs = 0);

// CMD_SET_GYRO_LPF: rate-loop gyro low-pass time constant for one axis
// (rc seconds; <= 0 disables).
QByteArray encodeSetGyroLpf(int axis, float rc, quint8 devId = 42,
                            quint32 tsMs = 0);

// CMD_SET_GYRO_NOTCH: master enable + FFT dynamic-notch detection params applied
// live to every axis. Each of q / fmin / fmax / minRatio is <= 0 => leave that
// field unchanged on the FC (partial tune). autoband arms a one-shot auto-band
// learn that tightens [fmin,fmax] around the observed hover spectrum.
QByteArray encodeSetGyroNotch(bool enabled, float q, float fminHz, float fmaxHz,
                              float minRatio, bool autoband = false,
                              quint8 devId = 42, quint32 tsMs = 0);

// CMD_SET_FLIGHT_MODE: 0=angle, 1=acro, 2=release to RC (source = GCS).
QByteArray encodeSetFlightMode(int mode, quint8 devId = 42, quint32 tsMs = 0);

// CMD_SET_MOTOR_GEOMETRY: per-motor body x/y [m] and spin (+1 CCW / -1 CW).
QByteArray encodeSetMotorGeometry(const float x[4], const float y[4],
                                  const float spin[4], quint8 devId = 42,
                                  quint32 tsMs = 0);

// CMD_ARM / CMD_DISARM: set/clear the software-arm latch (force = 0).
QByteArray encodeArm(quint8 devId = 42, quint32 tsMs = 0);
QByteArray encodeDisarm(quint8 devId = 42, quint32 tsMs = 0);

// CMD_CALIBRATE_IMU: `which` selects the routine; 0xFF cancels.
QByteArray encodeCalibrate(quint8 which, quint8 devId = 42);

// TIME_SYNC REQUEST: role=REQUEST, t1_gcs_tx stamped by the caller.
// commandedOffsetMs = INT32_MIN means "no correction this round".
QByteArray encodeTimeSyncRequest(quint8 seq, quint64 t1,
                                 qint32 commandedOffsetMs, quint8 devId = 42);

// TIME_SYNC REQUEST_WIDE: full 64-bit correction split across the lo/hi words,
// for deviations beyond int32 ms (e.g. FC uptime clock vs GCS epoch on cold
// start). The hi word truncates off the wire on the normal int32 path.
QByteArray encodeTimeSyncRequestWide(quint8 seq, quint64 t1,
                                     qint64 commandedOffsetMs,
                                     quint8 devId = 42);

// PERF_TASKNAME_REQUEST: resolve one task id to its name.
QByteArray encodeTaskNameRequest(int taskId, quint8 devId = 42,
                                 quint32 tsMs = 0);

} // namespace CommandCodec
