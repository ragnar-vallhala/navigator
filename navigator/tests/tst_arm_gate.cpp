#include "ArmGate.h"
#include <QtTest>

using namespace armgate;

class TstArmGate : public QObject {
  Q_OBJECT
private slots:
  void disarmNeedsConfirmWhenFlyingOrUnknown() {
    for (const char *s : {"IN_AIR", "FAILSAFE", "", "NOPE", "in_air"})
      QCOMPARE(int(disarmPolicy(QString::fromLatin1(s))), int(DisarmPolicy::Confirm));
  }
  void disarmSendsAtOnceOnTheGround() {
    for (const char *s : {"UNINITIALIZED", "INIT", "STANDBY", "PREARM", "ARMED",
                          "TERMINATED", "CALIBRATING"})
      QCOMPARE(int(disarmPolicy(QString::fromLatin1(s))), int(DisarmPolicy::Send));
  }
  void calibrationStartOnlyWhenIdle() {
    QVERIFY(calibrationStartAllowed("STANDBY"));
    QVERIFY(calibrationStartAllowed("PREARM"));
    for (const char *s : {"ARMED", "IN_AIR", "FAILSAFE", "CALIBRATING", "INIT",
                          "TERMINATED", "UNINITIALIZED", "", "NOPE"})
      QVERIFY2(!calibrationStartAllowed(QString::fromLatin1(s)), s);
  }
  void calibrationCancelNeverWhileArmedOrFlying() {
    for (const char *s : {"STANDBY", "PREARM", "CALIBRATING"})
      QVERIFY2(calibrationCancelAllowed(QString::fromLatin1(s)), s);
    for (const char *s : {"ARMED", "IN_AIR", "FAILSAFE", "INIT", "", "NOPE"})
      QVERIFY2(!calibrationCancelAllowed(QString::fromLatin1(s)), s);
  }
};

QTEST_APPLESS_MAIN(TstArmGate)
#include "tst_arm_gate.moc"
