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
#include "BatteryCalib.h"

#include <QStringList>
#include <QtEndian>

#include <cmath>
#include <cstring>

namespace batcal {

QStringList bandErrors(double vpc, double offset) {
  QStringList bad;
  // The file stores floats and the firmware compares the stored floats against
  // float limits, so judge the value as it will be stored: 0.04999999999 rounds
  // to 0.05f and is refused on the vehicle. NaN still fails every comparison.
  const float fv = float(vpc);
  const float fo = float(offset);
  if (!(fv > float(kVpcMin) && fv < float(kVpcMax)))
    bad << QStringLiteral("volts per count %1 is outside [%2, %3]")
               .arg(vpc, 0, 'g', 6)
               .arg(kVpcMin)
               .arg(kVpcMax);
  if (!(fo > -float(kOffsetAbsMax) && fo < float(kOffsetAbsMax)))
    bad << QStringLiteral("offset %1 counts is outside +/-%2")
               .arg(offset, 0, 'f', 1)
               .arg(kOffsetAbsMax);
  return bad;
}

Fit fit(const QVector<Point> &pts) {
  Fit f;
  const int n = int(pts.size());
  if (n < 2) {
    f.error = QStringLiteral("need two points: a scale AND an offset");
    return f;
  }
  double mx = 0.0;
  double my = 0.0;
  for (const Point &p : pts) {
    mx += p.counts;
    my += p.volts;
  }
  mx /= n;
  my /= n;
  double sxx = 0.0;
  double sxy = 0.0;
  for (const Point &p : pts) {
    sxx += (p.counts - mx) * (p.counts - mx);
    sxy += (p.counts - mx) * (p.volts - my);
  }
  if (sxx == 0.0) {
    f.error = QStringLiteral("both points have the same count: no baseline");
    return f;
  }
  const double span = [&] {
    double lo = pts[0].counts;
    double hi = lo;
    for (const Point &p : pts) {
      lo = std::min(lo, p.counts);
      hi = std::max(hi, p.counts);
    }
    return hi - lo;
  }();
  if (span < kMinCountSpan) {
    f.error = QStringLiteral("points are only %1 counts apart: use pack "
                             "states further apart (need %2)")
                  .arg(span, 0, 'f', 0)
                  .arg(kMinCountSpan, 0, 'f', 0);
    return f;
  }
  const double vpc = sxy / sxx;
  if (!(vpc > 0.0)) {
    f.error = QStringLiteral("fitted scale %1 is not positive: the points "
                             "are inconsistent")
                  .arg(vpc, 0, 'g', 6);
    return f;
  }
  f.vpc = vpc;
  f.offset = mx - my / vpc; // y = vpc * (x - offset)
  for (const Point &p : pts)
    f.residualsMv.push_back(((p.counts - f.offset) * vpc - p.volts) * 1000.0);
  const QStringList bad = bandErrors(f.vpc, f.offset);
  if (!bad.isEmpty()) {
    f.error = bad.join(QStringLiteral("; ")) +
              QStringLiteral(" (the firmware would refuse this)");
    return f;
  }
  f.ok = true;
  return f;
}

QByteArray pack(double vpc, double offset) {
  if (!bandErrors(vpc, offset).isEmpty())
    return {};
  QByteArray b(kBlobSize, '\0');
  auto *d = reinterpret_cast<uchar *>(b.data());
  qToLittleEndian<quint32>(kMagic, d);
  qToLittleEndian<quint16>(kVersion, d + 4);
  qToLittleEndian<quint16>(0, d + 6);
  const float fv = float(vpc);
  const float fo = float(offset);
  quint32 bits = 0;
  std::memcpy(&bits, &fv, sizeof bits);
  qToLittleEndian<quint32>(bits, d + 8);
  std::memcpy(&bits, &fo, sizeof bits);
  qToLittleEndian<quint32>(bits, d + 12);
  return b;
}

} // namespace batcal
