// CalibrationWidget gating: Start only in STANDBY/PREARM, Cancel never while
// armed or flying, and "started"/"canceled" shown from the FC's COMMAND_ACK.
#include "CalibrationWidget.h"
#include "CommandCodec.h"
#include "DroneProtocol.h"
#include "navlink_msgs.h"
#include <QPushButton>
#include <QSignalSpy>
#include <QtTest>

class TstCalibrationWidget : public QObject {
  Q_OBJECT
  static QPushButton *button(CalibrationWidget &w, const QString &text) {
    for (QPushButton *b : w.findChildren<QPushButton *>())
      if (b->text() == text)
        return b;
    return nullptr;
  }
  static QLabel *statusLabel(CalibrationWidget &w, const QString &startsWith) {
    for (QLabel *l : w.findChildren<QLabel *>())
      if (l->text().startsWith(startsWith))
        return l;
    return nullptr;
  }

private slots:
  void startOnlyInStandbyOrPrearm() {
    DroneProtocol proto;
    CalibrationWidget w;
    w.setProtocol(&proto);
    w.setConnected(true);
    QPushButton *start = button(w, "START CALIBRATION");
    QVERIFY(start);
    for (const char *s : {"ARMED", "IN_AIR", "FAILSAFE", "INIT", "CALIBRATING", ""}) {
      w.setVehicleState(QString::fromLatin1(s));
      QVERIFY2(!start->isEnabled(), s);
    }
    for (const char *s : {"STANDBY", "PREARM"}) {
      w.setVehicleState(QString::fromLatin1(s));
      QVERIFY2(start->isEnabled(), s);
    }
    w.setConnected(false); // a lost link disables it again
    QVERIFY(!start->isEnabled());
  }

  void aClickWhileArmedSendsNothing() {
    DroneProtocol proto;
    CalibrationWidget w;
    w.setProtocol(&proto);
    w.setConnected(true);
    w.setVehicleState("ARMED");
    QSignalSpy sent(&w, &CalibrationWidget::commandRequested);
    button(w, "START CALIBRATION")->click(); // disabled: no effect
    QCOMPARE(sent.count(), 0);
  }

  void startIsConfirmedByTheAckNotTheClick() {
    DroneProtocol proto;
    CalibrationWidget w;
    w.setProtocol(&proto);
    w.setConnected(true);
    w.setVehicleState("STANDBY");
    QSignalSpy sent(&w, &CalibrationWidget::commandRequested);
    QPushButton *start = button(w, "START CALIBRATION");
    start->click();
    QCOMPARE(sent.count(), 1);
    QVERIFY(!start->isEnabled());
    QVERIFY(statusLabel(w, "START SENT")); // not "CALIBRATING" yet
    proto.commandAckReceived(NAVLINK_MSGID_CMD_CALIBRATE_IMU, 0,
                             NAVLINK_COMMAND_RESULT_ACCEPTED);
    QVERIFY(statusLabel(w, "CALIBRATING"));
  }

  void aRefusedStartRestoresTheButtonAndSaysWhy() {
    DroneProtocol proto;
    CalibrationWidget w;
    w.setProtocol(&proto);
    w.setConnected(true);
    w.setVehicleState("STANDBY");
    QPushButton *start = button(w, "START CALIBRATION");
    start->click();
    proto.commandAckReceived(NAVLINK_MSGID_CMD_CALIBRATE_IMU, 0,
                             NAVLINK_COMMAND_RESULT_DENIED);
    QVERIFY(statusLabel(w, "REFUSED"));
    QVERIFY(start->isEnabled());
  }

  void anAckForAnotherCommandIsIgnored() {
    DroneProtocol proto;
    CalibrationWidget w;
    w.setProtocol(&proto);
    w.setConnected(true);
    w.setVehicleState("STANDBY");
    button(w, "START CALIBRATION")->click();
    proto.commandAckReceived(NAVLINK_MSGID_CMD_ARM, 0, NAVLINK_COMMAND_RESULT_DENIED);
    QVERIFY(statusLabel(w, "START SENT"));
  }

  void cancelShowsCanceledOnlyOnAck() {
    DroneProtocol proto;
    CalibrationWidget w;
    w.setProtocol(&proto);
    w.setConnected(true);
    w.setVehicleState("STANDBY");
    button(w, "START CALIBRATION")->click();
    proto.commandAckReceived(NAVLINK_MSGID_CMD_CALIBRATE_IMU, 0,
                             NAVLINK_COMMAND_RESULT_ACCEPTED);
    w.setVehicleState("CALIBRATING");
    QSignalSpy sent(&w, &CalibrationWidget::commandRequested);
    QPushButton *cancel = button(w, "CANCEL");
    QVERIFY(cancel->isEnabled());
    cancel->click();
    QCOMPARE(sent.count(), 1);
    QVERIFY(statusLabel(w, "CANCEL SENT"));
    proto.commandAckReceived(NAVLINK_MSGID_CMD_CALIBRATE_IMU, 0,
                             NAVLINK_COMMAND_RESULT_ACCEPTED);
    QVERIFY(statusLabel(w, "READY"));
  }

  void cancelIsDisabledOnceArmed() {
    DroneProtocol proto;
    CalibrationWidget w;
    w.setProtocol(&proto);
    w.setConnected(true);
    w.setVehicleState("STANDBY");
    button(w, "START CALIBRATION")->click();
    w.setVehicleState("ARMED"); // armed mid-routine: no stop frame may go out
    QPushButton *cancel = button(w, "CANCEL");
    QVERIFY(!cancel->isEnabled());
    QSignalSpy sent(&w, &CalibrationWidget::commandRequested);
    cancel->click();
    QCOMPARE(sent.count(), 0);
  }
};

QTEST_MAIN(TstCalibrationWidget)
#include "tst_calibration_widget.moc"
