// One rule for "armed" and the state name, from the HEARTBEAT nav_state number
// through the protocol and the telemetry engine; an unknown state is shown as
// UNKNOWN and counts as armed (it used to be dropped, freezing the UI on the old
// state while the parameters already treated it as armed).
#include "DroneProtocol.h"
#include "NavState.h"
#include "TelemetryEngine.h"
#include <QByteArray>
#include <QtTest>
extern "C" {
#include "navlink_msgs.h"
}

namespace {
QByteArray heartbeat(uint8_t navState) {
  navlink_heartbeat_t m{};
  m.nav_state = navState;
  m.timestamp = 1000;
  uint8_t buf[NAVLINK_MAX_FRAME];
  const size_t n = navlink_heartbeat_encode(buf, &m, 1, 42, 1);
  return QByteArray(reinterpret_cast<const char *>(buf), int(n));
}
} // namespace

class TstNavState : public QObject {
  Q_OBJECT
private slots:
  void tableMatchesTheFirmwareEnum() {
    const char *want[] = {"UNINITIALIZED", "INIT",     "STANDBY",    "PREARM",     "ARMED",
                          "IN_AIR",        "FAILSAFE", "TERMINATED", "CALIBRATING"};
    for (int i = 0; i < 9; ++i)
      QCOMPARE(QString::fromLatin1(navstate::name(uint8_t(i))),
               QString::fromLatin1(want[i]));
  }
  void armedStates() {
    for (int i = 0; i < 256; ++i) {
      const bool expect = (i == 4 || i == 5 || i == 6 || i >= 9);
      QCOMPARE(navstate::armed(uint8_t(i)), expect);
    }
  }
  void unknownIsNamedAndArmed() {
    QCOMPARE(QString::fromLatin1(navstate::name(9)), QStringLiteral("UNKNOWN"));
    QCOMPARE(QString::fromLatin1(navstate::name(255)), QStringLiteral("UNKNOWN"));
    bool ok = false;
    QVERIFY(navstate::armedByName("UNKNOWN", &ok));
    QVERIFY(ok);
  }
  void nameAndNumberAgree() {
    for (int i = 0; i < 9; ++i) {
      bool ok = false;
      const bool byName =
          navstate::armedByName(QString::fromLatin1(navstate::name(uint8_t(i))), &ok);
      QVERIFY(ok);
      QCOMPARE(byName, navstate::armed(uint8_t(i)));
    }
  }
  void textThatIsNotAStateIsIgnored() {
    bool ok = true;
    QVERIFY(!navstate::armedByName("\x01\x02garbage", &ok));
    QVERIFY(!ok);
  }

  // Through the real protocol and engine: a heartbeat with a state number the
  // build does not know must reach the snapshot as UNKNOWN + armed, and a
  // known disarmed state afterwards must clear it.
  void heartbeatDrivesTheSnapshot() {
    TelemetryEngine eng;
    DroneProtocol *p = eng.protocol();
    p->processData(heartbeat(2)); // STANDBY
    QCOMPARE(eng.snapshot().vehicleState, QStringLiteral("STANDBY"));
    QVERIFY(!eng.snapshot().armed);
    p->processData(heartbeat(5)); // IN_AIR
    QCOMPARE(eng.snapshot().vehicleState, QStringLiteral("IN_AIR"));
    QVERIFY(eng.snapshot().armed);
    p->processData(heartbeat(77)); // not a state this build knows
    QCOMPARE(eng.snapshot().vehicleState, QStringLiteral("UNKNOWN"));
    QVERIFY(eng.snapshot().armed);
    p->processData(heartbeat(2));
    QCOMPARE(eng.snapshot().vehicleState, QStringLiteral("STANDBY"));
    QVERIFY(!eng.snapshot().armed);
  }
};

QTEST_MAIN(TstNavState)
#include "tst_nav_state.moc"
