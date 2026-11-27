#include "ArmGate.h"
#include <QtTest>
#include <limits>

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
  void pidGainsMustBeFiniteAndNonNegative() {
    QVERIFY(pidGainsValid(0.0f, 0.0f, 0.0f, 0.0f));
    QVERIFY(pidGainsValid(1.5f, 0.2f, 0.01f, 0.0f));
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    for (float bad : {nan, inf, -inf, -0.001f}) {
      QVERIFY(!pidGainsValid(bad, 1, 1, 1));
      QVERIFY(!pidGainsValid(1, bad, 1, 1));
      QVERIFY(!pidGainsValid(1, 1, bad, 1));
      QVERIFY(!pidGainsValid(1, 1, 1, bad));
    }
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
