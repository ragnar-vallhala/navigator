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
// batcal: the two-point fit, the firmware band, the 16-byte file, and the
// dialog that produces it (constructed, never shown).

#include "BatteryCalib.h"
#include "ui/widgets/BatteryCalibDialog.h"

#include <QTemporaryDir>
#include <QFile>
#include <QtEndian>
#include <QtTest>

#include <cmath>
#include <cstring>

namespace {
// The bench numbers from the tool's docstring: 12.46 V at 1091, 7.95 V at 698.
QVector<batcal::Point> bench() { return {{1091.0, 12.46}, {698.0, 7.95}}; }
} // namespace

class TstBatteryCalib : public QObject {
  Q_OBJECT
private slots:
  void exactFitThroughTwoPoints();
  void residualsOfThreePoints();
  void refusesDegenerateInput();
  void refusesOutOfBand();
  void nanIsRejected();
  void blobLayout();
  void blobEmptyWhenOutOfBand();
  void dialogProducesBlob();
  void dialogRefusesCloseStates();
  void dialogNeedsUsableLiveCount();
};

void TstBatteryCalib::exactFitThroughTwoPoints() {
  const batcal::Fit f = batcal::fit(bench());
  QVERIFY2(f.ok, qPrintable(f.error));
  QCOMPARE(f.residualsMv.size(), 2);
  QVERIFY(std::abs(f.residualsMv[0]) < 1e-6);
  QVERIFY(std::abs(f.residualsMv[1]) < 1e-6);
  QVERIFY(f.vpc > batcal::kVpcMin);
  // volts = (counts - offset) * vpc reproduces both inputs.
  QVERIFY(std::abs((1091.0 - f.offset) * f.vpc - 12.46) < 1e-9);
  QVERIFY(std::abs((698.0 - f.offset) * f.vpc - 7.95) < 1e-9);
  QVERIFY(std::abs(f.vpc - (12.46 - 7.95) / (1091.0 - 698.0)) < 1e-12);
}

void TstBatteryCalib::residualsOfThreePoints() {
  QVector<batcal::Point> p = bench();
  p.push_back({900.0, 10.30}); // 40 mV or so off the line
  const batcal::Fit f = batcal::fit(p);
  QVERIFY2(f.ok, qPrintable(f.error));
  QCOMPARE(f.residualsMv.size(), 3);
  double sum = 0.0;
  for (double r : f.residualsMv)
    sum += r;
  QVERIFY(std::abs(sum) < 1e-6); // least squares with an intercept
  QVERIFY(std::abs(f.residualsMv[2]) > 1.0);
}

void TstBatteryCalib::refusesDegenerateInput() {
  QVERIFY(!batcal::fit({}).ok);
  QVERIFY(!batcal::fit({{1000.0, 12.0}}).ok);
  QVERIFY(!batcal::fit({{1000.0, 12.0}, {1000.0, 8.0}}).ok); // no baseline
  QVERIFY(!batcal::fit({{1000.0, 12.0}, {1010.0, 11.9}}).ok); // short baseline
  // Higher counts at lower volts: negative scale.
  const batcal::Fit neg = batcal::fit({{700.0, 12.0}, {1100.0, 8.0}});
  QVERIFY(!neg.ok);
  QVERIFY(neg.error.contains("not positive"));
}

void TstBatteryCalib::refusesOutOfBand() {
  // 0.1 V/count: far above the 0.05 band.
  const batcal::Fit hi = batcal::fit({{100.0, 10.0}, {200.0, 20.0}});
  QVERIFY(!hi.ok);
  QVERIFY(hi.error.contains("refuse"));
  // 0.0005 V/count: below the band.
  QVERIFY(!batcal::fit({{1000.0, 1.0}, {2000.0, 1.5}}).ok);
  // In-band scale, but a zero error of 300 counts: volts = (c - 300) * 0.01.
  const batcal::Fit off = batcal::fit({{500.0, 2.0}, {1000.0, 7.0}});
  QVERIFY(!off.ok);
  QVERIFY(off.error.contains("offset"));
  QCOMPARE(batcal::bandErrors(0.0114, 5.0).size(), 0);
  QCOMPARE(batcal::bandErrors(0.0114, 200.0).size(), 1); // bound is exclusive
}

void TstBatteryCalib::nanIsRejected() {
  const double nan = std::nan("");
  QCOMPARE(batcal::bandErrors(nan, 0.0).size(), 1);
  QCOMPARE(batcal::bandErrors(0.01, nan).size(), 1);
  QVERIFY(batcal::pack(nan, 0.0).isEmpty());
}

void TstBatteryCalib::blobLayout() {
  const QByteArray b = batcal::pack(0.0114, -3.5);
  QCOMPARE(b.size(), 16); // '<IHHff'
  const auto *d = reinterpret_cast<const uchar *>(b.constData());
  QCOMPARE(qFromLittleEndian<quint32>(d), 0x4C414342u);
  QCOMPARE(b.left(4), QByteArray("BCAL"));
  QCOMPARE(qFromLittleEndian<quint16>(d + 4), quint16(1));
  QCOMPARE(qFromLittleEndian<quint16>(d + 6), quint16(0));
  quint32 bits = qFromLittleEndian<quint32>(d + 8);
  float vpc = 0.0f;
  std::memcpy(&vpc, &bits, sizeof vpc);
  QCOMPARE(vpc, 0.0114f);
  bits = qFromLittleEndian<quint32>(d + 12);
  float off = 0.0f;
  std::memcpy(&off, &bits, sizeof off);
  QCOMPARE(off, -3.5f);
}

void TstBatteryCalib::blobEmptyWhenOutOfBand() {
  QVERIFY(batcal::pack(0.5, 0.0).isEmpty());
  QVERIFY(batcal::pack(0.01, 250.0).isEmpty());
}

void TstBatteryCalib::dialogProducesBlob() {
  BatteryCalibDialog dlg;
  QVERIFY(dlg.blob().isEmpty());
  dlg.setLive(1091, true);
  dlg.setVolts(0, 12.46);
  QVERIFY(dlg.capture(0));
  QVERIFY(dlg.blob().isEmpty()); // one point is not a fit
  dlg.setLive(698, true);
  dlg.setVolts(1, 7.95);
  QVERIFY(dlg.capture(1));
  QVERIFY2(dlg.fit().ok, qPrintable(dlg.fit().error));
  QCOMPARE(dlg.blob(), batcal::pack(dlg.fit().vpc, dlg.fit().offset));

  QTemporaryDir dir;
  QVERIFY(dir.isValid());
  const QString path = dir.filePath("batcal.bin");
  QVERIFY(dlg.saveTo(path));
  QFile f(path);
  QVERIFY(f.open(QIODevice::ReadOnly));
  QCOMPARE(f.readAll(), dlg.blob());
}

void TstBatteryCalib::dialogRefusesCloseStates() {
  BatteryCalibDialog dlg;
  dlg.setLive(1000, true);
  dlg.setVolts(0, 12.0);
  dlg.capture(0);
  dlg.setLive(1010, true);
  dlg.setVolts(1, 11.9);
  dlg.capture(1);
  QVERIFY(!dlg.fit().ok);
  QVERIFY(dlg.blob().isEmpty());
  QString err;
  QTemporaryDir dir;
  QVERIFY(!dlg.saveTo(dir.filePath("x.bin"), &err));
  QVERIFY(!err.isEmpty());
  QVERIFY(!QFile::exists(dir.filePath("x.bin")));
}

void TstBatteryCalib::dialogNeedsUsableLiveCount() {
  BatteryCalibDialog dlg;
  dlg.setLive(1091, false);
  QVERIFY(!dlg.capture(0));
  dlg.setLive(1091, true);
  QVERIFY(dlg.capture(0));
  QVERIFY(!dlg.capture(2));
}

QTEST_MAIN(TstBatteryCalib)
#include "tst_battery_calib.moc"
