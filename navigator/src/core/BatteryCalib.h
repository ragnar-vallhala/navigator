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
#include <QString>
#include <QVector>

// Battery divider calibration: the two-point fit, the firmware's sanity band,
// and the on-card file format. A port of vayu tools/calib/battery_calib.py,
// whose constants mirror firmware storage/battery_calib.h (the loader refuses
// anything outside the band, so this refuses it first).
//
//   volts = (counts - offset_counts) * volts_per_count
//
// File 0:batcal.bin: struct '<IHHff' = magic 'BCAL' (0x4C414342), version 1,
// 2 pad bytes, volts_per_count, offset_counts -- 16 bytes, little-endian.
namespace batcal {

constexpr quint32 kMagic = 0x4C414342u;
constexpr quint16 kVersion = 1;
constexpr int kBlobSize = 16;
constexpr double kVpcMin = 0.001;
constexpr double kVpcMax = 0.050;
constexpr double kOffsetAbsMax = 200.0;
// Below this the two states are too close for the intercept to mean anything.
constexpr double kMinCountSpan = 50.0;
// The card path the firmware reads (8.3 name: its FS has no long names).
inline const char *kCardPath = "0:batcal.bin";

struct Point {
  double counts = 0.0;
  double volts = 0.0;
};

struct Fit {
  bool ok = false;
  QString error; // why not ok (fit failed or out of band)
  double vpc = 0.0;
  double offset = 0.0;
  QVector<double> residualsMv; // fit - measured, per input point, in mV
};

// Least squares through (counts, volts). Not ok when under two points, no
// count baseline, a non-positive scale, or the result is outside the band.
Fit fit(const QVector<Point> &pts);

// Empty when in band; otherwise one reason per violated bound. Written as "is
// inside" so a NaN is rejected (every NaN comparison is false).
QStringList bandErrors(double vpc, double offset);

// The 16-byte file. Empty when out of band (never persist what the loader
// would refuse). Float narrowing matches the Python struct.pack('f').
QByteArray pack(double vpc, double offset);

} // namespace batcal
