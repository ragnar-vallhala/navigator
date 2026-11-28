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
// GPS_RAW (msgid 1052) on the GCS: a real frame built with the generated
// encoder goes through NavlinkRouter and arrives with every field, and the
// status text says what the receiver reported, never more.

#include "GpsStatus.h"
#include "NavlinkRouter.h"

#include <QtTest>

extern "C" {
#include "navlink_msgs.h"
}

namespace {
QByteArray gpsFrame(uint8_t fix, uint8_t sats, uint8_t flags,
                    uint32_t hAcc = 1500) {
  navlink_gps_raw_t m{};
  m.time_us = 123456789012ull;
  m.lat_e7 = 311234567;
  m.lon_e7 = -1223456789;
  m.alt_mm = -1500;
  m.vel_ned_mm_s[0] = 1000;
  m.vel_ned_mm_s[1] = -2000;
  m.vel_ned_mm_s[2] = 300;
  m.h_acc_mm = hAcc;
  m.v_acc_mm = 2500;
  m.s_acc_mm_s = 0xFFFFFFFFu;
  m.pdop_x100 = 187;
  m.fix_type = fix;
  m.sats = sats;
  m.nav_flags = flags;
  uint8_t buf[NAVLINK_MAX_FRAME];
  const size_t n = navlink_gps_raw_encode(buf, &m, 0, 1, 1);
  return QByteArray(reinterpret_cast<const char *>(buf), int(n));
}
} // namespace

class TstGpsRaw : public QObject {
  Q_OBJECT
private slots:
  void everyFieldArrives();
  void onlyItsOwnCallbackFires();
  void noHandlerIsHarmless();
  void noGpsUntilOneArrives();
  void fixAndSatsAreShown();
  void aFixTheReceiverDoesNotClaimIsNotShownAsOne();
  void theFcVerdictIsAddedAndAlarms();
  void unknownFixTypeIsItsNumber();
};

void TstGpsRaw::everyFieldArrives() {
  NavlinkRouter r;
  int calls = 0;
  GpsRawData g;
  r.onGpsRaw = [&](const GpsRawData &d) {
    ++calls;
    g = d;
  };
  r.feed(gpsFrame(NAVLINK_GPS_FIX_TYPE_FIX_3D, 14, 1));
  QCOMPARE(calls, 1);
  QCOMPARE(quint64(g.timeUs), quint64(123456789012ull));
  QCOMPARE(int(g.latE7), 311234567);
  QCOMPARE(int(g.lonE7), -1223456789);
  QCOMPARE(int(g.altMm), -1500);
  QCOMPARE(int(g.velNedMmS[0]), 1000);
  QCOMPARE(int(g.velNedMmS[1]), -2000);
  QCOMPARE(int(g.velNedMmS[2]), 300);
  QCOMPARE(quint32(g.hAccMm), quint32(1500));
  QCOMPARE(quint32(g.vAccMm), quint32(2500));
  QCOMPARE(quint32(g.sAccMmS), quint32(0xFFFFFFFFu));
  QCOMPARE(int(g.pdopX100), 187);
  QCOMPARE(int(g.fixType), int(NAVLINK_GPS_FIX_TYPE_FIX_3D));
  QCOMPARE(int(g.sats), 14);
  QCOMPARE(int(g.navFlags), 1);
}

void TstGpsRaw::onlyItsOwnCallbackFires() {
  NavlinkRouter r;
  int gps = 0, failsafe = 0;
  r.onGpsRaw = [&](const GpsRawData &) { ++gps; };
  r.onFailsafeStatus = [&](const FailsafeStatusData &) { ++failsafe; };
  r.feed(gpsFrame(3, 10, 1));
  QCOMPARE(gps, 1);
  QCOMPARE(failsafe, 0);
}

void TstGpsRaw::noHandlerIsHarmless() {
  NavlinkRouter r; // onGpsRaw left empty
  r.feed(gpsFrame(3, 10, 1));
  QVERIFY(true);
}

void TstGpsRaw::noGpsUntilOneArrives() {
  const GpsRawData none;
  QCOMPARE(gpsstatus::statusLine(false, none, 0), QStringLiteral("no GPS"));
  QVERIFY(!gpsstatus::isAlarm(false, none, 0)); // a craft with no receiver is not an alarm
  // Not even a stale 3D fix is shown as one.
  GpsRawData stale;
  stale.fixType = NAVLINK_GPS_FIX_TYPE_FIX_3D;
  stale.sats = 14;
  stale.navFlags = 1;
  QCOMPARE(gpsstatus::statusLine(false, stale, 0), QStringLiteral("no GPS"));
}

void TstGpsRaw::fixAndSatsAreShown() {
  GpsRawData d;
  d.fixType = NAVLINK_GPS_FIX_TYPE_FIX_3D;
  d.sats = 14;
  d.navFlags = 1;
  d.hAccMm = 1500;
  QCOMPARE(gpsstatus::statusLine(true, d, 0), QStringLiteral("3D fix, 14 sats, ±1.5 m"));
  QVERIFY(!gpsstatus::isAlarm(true, d, 0));
  d.hAccMm = 0xFFFFFFFFu; // the receiver gave no accuracy: none is shown
  QCOMPARE(gpsstatus::statusLine(true, d, 0), QStringLiteral("3D fix, 14 sats"));
  d.fixType = NAVLINK_GPS_FIX_TYPE_NO_FIX;
  d.sats = 3;
  d.navFlags = 0;
  QCOMPARE(gpsstatus::statusLine(true, d, 0), QStringLiteral("no fix, 3 sats"));
  QVERIFY(gpsstatus::isAlarm(true, d, 0));
}

void TstGpsRaw::aFixTheReceiverDoesNotClaimIsNotShownAsOne() {
  GpsRawData d;
  d.fixType = NAVLINK_GPS_FIX_TYPE_FIX_3D;
  d.sats = 9;
  d.navFlags = 0; // gnssFixOK clear
  d.hAccMm = 800;
  const QString s = gpsstatus::statusLine(true, d, 0);
  QVERIFY(s.contains(QStringLiteral("fix not OK")));
  QVERIFY(!s.contains(QStringLiteral("±"))); // no accuracy claimed for a fix it does not claim
  QVERIFY(gpsstatus::isAlarm(true, d, 0));
}

void TstGpsRaw::theFcVerdictIsAddedAndAlarms() {
  GpsRawData d;
  d.fixType = NAVLINK_GPS_FIX_TYPE_FIX_3D;
  d.sats = 12;
  d.navFlags = 1;
  const QString s =
      gpsstatus::statusLine(true, d, NAVLINK_FAILSAFE_FLAGS_GPS_UNTRUSTED);
  QVERIFY(s.contains(QStringLiteral("FC does not trust it")));
  QVERIFY(gpsstatus::isAlarm(true, d, NAVLINK_FAILSAFE_FLAGS_GPS_UNTRUSTED));
  QVERIFY(!gpsstatus::isAlarm(true, d, NAVLINK_FAILSAFE_FLAGS_LANDING)); // other flags
}

void TstGpsRaw::unknownFixTypeIsItsNumber() {
  QCOMPARE(gpsstatus::fixText(200), QStringLiteral("fix type 200"));
  GpsRawData d;
  d.fixType = 200;
  d.sats = 5;
  d.navFlags = 1;
  QVERIFY(!gpsstatus::hasPosition(d)); // never taken as a fix
  QVERIFY(gpsstatus::isAlarm(true, d, 0));
}

QTEST_MAIN(TstGpsRaw)
#include "tst_gps_raw.moc"
