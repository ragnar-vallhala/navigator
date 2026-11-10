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
// The built-in airframe presets as data: the table, the vehicle-file JSON round
// trip, the motor order/spin the firmware mixer fixes, and the derived T/W and
// hover duty. No engine; tst_airframe_flight flies them.

#include "AirframePresets.h"
#include "VehicleJson.h"

#include <QFile>
#include <QJsonDocument>
#include <QtTest>

#include <cmath>

namespace {

// The numbers from the research note, typed independently of the source table.
struct Expect {
  const char *id;
  float mass, propRadius, p;
  float ixx, iyy, izz;
  float kt, km, wmax, tau;
  double twLo, twHi, hoverLo, hoverHi; // expected T/W and hover duty bands
};
const Expect kExpect[] = {
    {"5in", 0.682f, 0.0622f, 0.0866f, 1.54e-3f, 1.80e-3f, 3.06e-3f, 1.234e-6f,
     1.59e-8f, 3365, 0.0125f, 7.5, 9.0, 0.30, 0.40},
    {"3in", 0.180f, 0.0381f, 0.0502f, 1.38e-4f, 1.57e-4f, 2.70e-4f, 3.46e-7f,
     2.73e-9f, 2753, 0.010f, 5.0, 7.0, 0.35, 0.46},
    {"7in", 1.165f, 0.0889f, 0.1156f, 3.78e-3f, 4.62e-3f, 7.70e-3f, 5.49e-6f,
     9.32e-8f, 1942, 0.020f, 6.5, 8.0, 0.32, 0.42},
    {"10in", 1.520f, 0.127f, 0.1485f, 1.12e-2f, 1.30e-2f, 2.29e-2f, 1.076e-5f,
     2.41e-7f, 901, 0.030f, 2.0, 2.7, 0.60, 0.70},
    {"whoop65", 0.029f, 0.0155f, 0.0230f, 6.2e-6f, 7.1e-6f, 1.27e-5f, 8.15e-9f,
     2.71e-11f, 5668, 0.020f, 3.3, 4.1, 0.47, 0.57},
};

// Firmware motor order: [0] front-right, [1] rear-right, [2] rear-left,
// [3] front-left; body NED, x forward, y right.
constexpr int kSignX[4] = {+1, -1, -1, +1};
constexpr int kSignY[4] = {+1, +1, -1, -1};
constexpr int kSpin[4] = {+1, -1, +1, -1};

bool near(double a, double b) {
  return std::fabs(a - b) <= 1e-6 * std::max(std::fabs(a), std::fabs(b));
}

void expectGeometryEqualsTable(const vsim::GeometryConfig &g,
                               const Expect &e) {
  QVERIFY(near(g.mass, e.mass));
  QVERIFY(near(g.propRadius, e.propRadius));
  QVERIFY(near(g.inertia[0], e.ixx));
  QVERIFY(near(g.inertia[4], e.iyy));
  QVERIFY(near(g.inertia[8], e.izz));
  for (int i : {1, 2, 3, 5, 6, 7})
    QCOMPARE(g.inertia[i], 0.0f);
  QVERIFY(g.com.isNull());
  for (int i = 0; i < 4; ++i) {
    const vsim::MotorConfig &m = g.motors[i];
    QVERIFY(near(std::fabs(m.pos.x()), e.p));
    QVERIFY(near(std::fabs(m.pos.y()), e.p));
    QCOMPARE(m.pos.z(), 0.0f);
    QCOMPARE(m.axis, QVector3D(0, 0, -1));
    QVERIFY(near(m.k_thrust, e.kt));
    QVERIFY(near(m.k_moment, e.km));
    QVERIFY(near(m.max_omega, e.wmax));
    QVERIFY(near(m.tau, e.tau));
  }
}

} // namespace

class TstAirframePresets : public QObject {
  Q_OBJECT
private slots:
  void data_data() {
    QTest::addColumn<int>("row");
    for (int i = 0; i < int(sizeof(kExpect) / sizeof(kExpect[0])); ++i)
      QTest::newRow(kExpect[i].id) << i;
  }
  void tableMatchesAndRoundTrips_data() { data_data(); }
  void tableMatchesAndRoundTrips();
  void vehicleFileMatchesPreset_data() { data_data(); }
  void vehicleFileMatchesPreset();
  void motorOrderAndSpinMatchFirmware_data() { data_data(); }
  void motorOrderAndSpinMatchFirmware();
  void thrustToWeightAndHoverDuty_data() { data_data(); }
  void thrustToWeightAndHoverDuty();
  void gainsAreSane_data() { data_data(); }
  void gainsAreSane();
  void allFivePresetsAndDefault();
};

void TstAirframePresets::tableMatchesAndRoundTrips() {
  QFETCH(int, row);
  const Expect &e = kExpect[row];
  const vsim::AirframePreset *p = vsim::findAirframePreset(e.id);
  QVERIFY(p);
  expectGeometryEqualsTable(p->geometry, e);
  if (QTest::currentTestFailed())
    return;
  // Editor Save -> Load: JSON text out and back.
  const QByteArray text =
      QJsonDocument(vsim::geometryToJson(p->geometry)).toJson();
  const vsim::GeometryConfig back =
      vsim::geometryFromJson(QJsonDocument::fromJson(text).object());
  expectGeometryEqualsTable(back, e);
  QCOMPARE(back.propRadius, p->geometry.propRadius);
  for (int i = 0; i < 4; ++i) {
    QCOMPARE(back.motors[i].pos, p->geometry.motors[i].pos);
    QCOMPARE(back.motors[i].spin, p->geometry.motors[i].spin);
    QCOMPARE(back.motors[i].k_moment, p->geometry.motors[i].k_moment);
  }
}

// docs/vehicles/<id>.vveh are the presets as standalone files (hand them to a
// headless harness, or Load Vehicle in the editor); they must not drift from
// the built-in table.
void TstAirframePresets::vehicleFileMatchesPreset() {
  QFETCH(int, row);
  const Expect &e = kExpect[row];
  QFile f(QStringLiteral(AIRFRAME_VEHICLE_DIR "/") + e.id +
          QStringLiteral(".vveh"));
  QVERIFY2(f.open(QIODevice::ReadOnly), qPrintable(f.fileName()));
  QJsonParseError pe;
  const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &pe);
  QVERIFY2(pe.error == QJsonParseError::NoError, qPrintable(pe.errorString()));
  QCOMPARE(doc.object().value("format").toString(), QString("vayu-vehicle"));
  const vsim::GeometryConfig g = vsim::geometryFromJson(doc.object());
  expectGeometryEqualsTable(g, e);
  const vsim::GeometryConfig &t = vsim::findAirframePreset(e.id)->geometry;
  for (int i = 0; i < 4; ++i) {
    QCOMPARE(g.motors[i].pos, t.motors[i].pos);
    QCOMPARE(g.motors[i].spin, t.motors[i].spin);
  }
}

void TstAirframePresets::motorOrderAndSpinMatchFirmware() {
  QFETCH(int, row);
  const vsim::GeometryConfig &g = vsim::findAirframePreset(kExpect[row].id)->geometry;
  for (int i = 0; i < 4; ++i) {
    QCOMPARE(g.motors[i].pos.x() > 0 ? +1 : -1, kSignX[i]);
    QCOMPARE(g.motors[i].pos.y() > 0 ? +1 : -1, kSignY[i]);
    QCOMPARE(g.motors[i].spin, kSpin[i]);
  }
  // Diagonal pairs spin together, adjacent pairs oppose: zero net yaw torque
  // at equal speeds.
  QCOMPARE(g.motors[0].spin, g.motors[2].spin);
  QCOMPARE(g.motors[1].spin, g.motors[3].spin);
  QCOMPARE(g.motors[0].spin, -g.motors[1].spin);
}

void TstAirframePresets::thrustToWeightAndHoverDuty() {
  QFETCH(int, row);
  const Expect &e = kExpect[row];
  const vsim::AirframeDerived d =
      vsim::deriveAirframe(vsim::findAirframePreset(e.id)->geometry);
  qInfo("%s: T/W %.2f, hover duty %.3f", e.id, d.thrustToWeight, d.hoverDuty);
  QVERIFY2(d.thrustToWeight >= e.twLo && d.thrustToWeight <= e.twHi,
           qPrintable(QString::number(d.thrustToWeight)));
  QVERIFY2(d.hoverDuty >= e.hoverLo && d.hoverDuty <= e.hoverHi,
           qPrintable(QString::number(d.hoverDuty)));
  // Hover duty is exactly the T/W reciprocal's square root: thrust ~ omega^2.
  QVERIFY(near(d.hoverDuty, std::sqrt(1.0 / d.thrustToWeight)));
}

void TstAirframePresets::gainsAreSane() {
  QFETCH(int, row);
  const vsim::AirframePreset *p = vsim::findAirframePreset(kExpect[row].id);
  QCOMPARE(p->gains.size(), 6);
  bool seen[2][3] = {};
  for (const PidSetCmd &c : p->gains) {
    QVERIFY(c.controller == 0 || c.controller == 1);
    QVERIFY(c.axis >= 0 && c.axis <= 2);
    QVERIFY(std::isfinite(c.kp) && c.kp > 0.0f);
    QVERIFY(std::isfinite(c.ki) && c.ki >= 0.0f);
    QVERIFY(std::isfinite(c.kd) && c.kd >= 0.0f);
    QVERIFY(!seen[c.controller][c.axis]); // no slot twice
    seen[c.controller][c.axis] = true;
  }
}

void TstAirframePresets::allFivePresetsAndDefault() {
  QCOMPARE(vsim::airframePresets().size(), 5);
  QCOMPARE(vsim::airframePresets().first().id, QString("5in"));
  QVERIFY(!vsim::findAirframePreset("nope"));
  // The editor's initial airframe is the 5-inch preset.
  const vsim::GeometryConfig def;
  const vsim::GeometryConfig &five = vsim::findAirframePreset("5in")->geometry;
  QCOMPARE(def.mass, five.mass);
  QCOMPARE(def.propRadius, five.propRadius);
  QCOMPARE(def.inertia, five.inertia);
  for (int i = 0; i < 4; ++i) {
    QCOMPARE(def.motors[i].pos, five.motors[i].pos);
    QCOMPARE(def.motors[i].spin, five.motors[i].spin);
    QCOMPARE(def.motors[i].k_thrust, five.motors[i].k_thrust);
    QCOMPARE(def.motors[i].k_moment, five.motors[i].k_moment);
    QCOMPARE(def.motors[i].max_omega, five.motors[i].max_omega);
    QCOMPARE(def.motors[i].tau, five.motors[i].tau);
  }
}

QTEST_MAIN(TstAirframePresets)
#include "tst_airframe_presets.moc"
