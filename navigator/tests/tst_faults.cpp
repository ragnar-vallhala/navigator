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
// FaultsModel (dedup / retention / ack / clear / unknown id), the router's
// FAULT_STATUS decode (a real frame, built with the generated encoder, through
// NavlinkRouter) and the FaultsPanel badge.
#include "FaultsModel.h"
#include "FaultsPanel.h"
#include "NavlinkRouter.h"

#include <QPushButton>
#include <QSignalSpy>
#include <QtTest>

extern "C" {
#include "navlink_msgs.h"
}

namespace {
FaultEvent ev(uint16_t id, uint8_t sev, bool active, const char *name = "",
              const char *text = "") {
  FaultEvent e;
  e.id = id;
  e.severity = sev;
  e.active = active;
  e.name = QString::fromLatin1(name);
  e.text = QString::fromLatin1(text);
  return e;
}

QByteArray faultFrame(uint16_t id, uint8_t sev, uint8_t active,
                      const char *text) {
  navlink_fault_status_t m{};
  m.id = id;
  m.severity = sev;
  m.active = active;
  strncpy(m.text, text, sizeof(m.text) - 1);
  uint8_t buf[NAVLINK_MAX_FRAME];
  const size_t n = navlink_fault_status_encode(buf, &m, 0, 1, 1);
  return QByteArray(reinterpret_cast<const char *>(buf), static_cast<int>(n));
}
} // namespace

class TstFaults : public QObject {
  Q_OBJECT

private slots:
  void resendDoesNotAddRow() {
    FaultsModel m;
    m.apply(ev(256, 2, true, "IMU_INIT_FAILED", "IMU init failed"), 1000);
    m.apply(ev(256, 2, true, "IMU_INIT_FAILED", "IMU init failed"), 6000);
    m.apply(ev(256, 2, true, "IMU_INIT_FAILED", "IMU init failed"), 11000);
    QCOMPARE(m.rowCount(), 1);
    const QModelIndex ix = m.index(0);
    QCOMPARE(ix.data(FaultsModel::CountRole).toInt(), 3);
    QCOMPARE(ix.data(FaultsModel::FirstSeenRole).toLongLong(), 1000LL);
    QCOMPARE(ix.data(FaultsModel::LastSeenRole).toLongLong(), 11000LL);
    QCOMPARE(m.unackedCount(), 1);
  }

  void distinctIdsGetDistinctRows() {
    FaultsModel m;
    m.apply(ev(256, 2, true));
    m.apply(ev(257, 2, true));
    m.apply(ev(272, 4, true));
    QCOMPARE(m.rowCount(), 3);
    QCOMPARE(m.index(2).data(FaultsModel::IdRole).toInt(), 272);
  }

  void seriousFaultStaysUntilFirmwareClears() {
    FaultsModel m;
    m.apply(ev(258, 3, true, "MAG_INIT_FAILED"));
    // Unrelated traffic and resends never drop it.
    m.apply(ev(272, 4, true));
    m.apply(ev(258, 3, true, "MAG_INIT_FAILED"));
    QCOMPARE(m.rowCount(), 2);
    QCOMPARE(m.unackedSeriousCount(), 1);
    // A clear for a different id leaves it alone.
    m.apply(ev(272, 4, false));
    QCOMPARE(m.rowCount(), 1);
    QCOMPARE(m.index(0).data(FaultsModel::IdRole).toInt(), 258);
    // Its own clear removes it.
    m.apply(ev(258, 3, false));
    QCOMPARE(m.rowCount(), 0);
    // Clearing an id that is not listed is a no-op.
    m.apply(ev(999, 3, false));
    QCOMPARE(m.rowCount(), 0);
  }

  void acknowledgeKeepsRowAndSurvivesResend() {
    FaultsModel m;
    m.apply(ev(256, 2, true));
    m.apply(ev(272, 4, true));
    QSignalSpy spy(&m, &FaultsModel::countsChanged);
    m.acknowledgeAll();
    QCOMPARE(m.rowCount(), 2);
    QCOMPARE(m.unackedCount(), 0);
    QCOMPARE(m.worstUnackedSeverity(), -1);
    QCOMPARE(spy.count(), 1);
    QCOMPARE(spy.at(0).at(0).toInt(), 0);
    // The FC's 5 s resend must not un-acknowledge it.
    m.apply(ev(256, 2, true));
    QCOMPARE(m.unackedCount(), 0);
    QCOMPARE(m.index(0).data(FaultsModel::CountRole).toInt(), 2);
    // Ack again with nothing new: no signal.
    spy.clear();
    m.acknowledgeAll();
    QCOMPARE(spy.count(), 0);
  }

  void clearDropsRowsAndResendRaisesAgain() {
    FaultsModel m;
    m.apply(ev(256, 2, true));
    m.acknowledgeAll();
    m.clear();
    QCOMPARE(m.rowCount(), 0);
    QCOMPARE(m.unackedCount(), 0);
    m.apply(ev(256, 2, true));
    QCOMPARE(m.rowCount(), 1);
    QCOMPARE(m.index(0).data(FaultsModel::AckedRole).toBool(), false);
    QCOMPARE(m.index(0).data(FaultsModel::CountRole).toInt(), 1);
  }

  void unknownIdShownAsNumber() {
    FaultsModel m;
    m.apply(ev(0x1234, 3, true));
    QCOMPARE(m.rowCount(), 1);
    QCOMPARE(m.index(0).data().toString(),
             QStringLiteral("Fault 4660 (0x1234)"));
    m.apply(ev(0x1235, 3, true, "", "vendor thing"));
    QCOMPARE(m.index(1).data().toString(),
             QStringLiteral("Fault 4661 (0x1235): vendor thing"));
  }

  void countsAndWorstSeverity() {
    FaultsModel m;
    m.apply(ev(272, 4, true));
    m.apply(ev(256, 2, true));
    QCOMPARE(m.unackedCount(), 2);
    QCOMPARE(m.unackedSeriousCount(), 1);
    QCOMPARE(m.worstUnackedSeverity(), 2);
  }

  void routerDecodesFaultStatusFrame() {
    NavlinkRouter r;
    QVector<FaultEvent> got;
    r.onFault = [&](const FaultEvent &e) { got.push_back(e); };
    r.feed(faultFrame(NAVLINK_FAULT_IMU_INIT_FAILED, 2, 1, "IMU init failed"));
    r.feed(faultFrame(0x1234, 3, 1, "vendor"));
    r.feed(faultFrame(NAVLINK_FAULT_IMU_INIT_FAILED, 2, 0, ""));
    QCOMPARE(got.size(), 3);
    QCOMPARE(int(got[0].id), 256);
    QCOMPARE(got[0].name, QStringLiteral("IMU_INIT_FAILED"));
    QCOMPARE(got[0].text, QStringLiteral("IMU init failed"));
    QCOMPARE(int(got[0].severity), 2);
    QVERIFY(got[0].active);
    QVERIFY(got[1].name.isEmpty()); // not in faults.json
    QCOMPARE(got[1].text, QStringLiteral("vendor"));
    QVERIFY(!got[2].active);
    // Empty wire text falls back to the faults.json text for a known id.
    QCOMPARE(got[2].text, QStringLiteral("IMU init failed"));

    // Fed into the model, the three frames net out to one unknown-id row.
    FaultsModel m;
    for (const FaultEvent &e : got)
      m.apply(e);
    QCOMPARE(m.rowCount(), 1);
    QCOMPARE(m.index(0).data(FaultsModel::IdRole).toInt(), 0x1234);
  }

  void statustextStillReachesOnLog() {
    NavlinkRouter r;
    QStringList lines;
    r.onLog = [&](const QString &l) { lines << l; };
    navlink_statustext_t m{};
    m.severity = 3;
    strncpy(m.text, "hello", sizeof(m.text) - 1);
    uint8_t buf[NAVLINK_MAX_FRAME];
    const size_t n = navlink_statustext_encode(buf, &m, 0, 1, 1);
    r.feed(QByteArray(reinterpret_cast<const char *>(buf), int(n)));
    QCOMPARE(lines, QStringList{QStringLiteral("hello")});
  }

  void panelBadgeAndButtons() {
    FaultsPanel p;
    QCOMPARE(p.objectName(), QStringLiteral("faults"));
    QCOMPARE(p.badgeText(), QString());
    QCOMPARE(p.title(), QStringLiteral("Faults"));
    QSignalSpy spy(&p, &FaultsPanel::countsChanged);
    p.model()->apply(ev(256, 2, true));
    p.model()->apply(ev(272, 4, true));
    QCOMPARE(p.badgeText(), QStringLiteral("2"));
    QCOMPARE(p.title(), QStringLiteral("Faults (2)"));
    QCOMPARE(p.badgeColor(), FaultsPanel::severityColor(2));
    QVERIFY(spy.count() >= 2);
    auto *ack = p.findChild<QPushButton *>("faultsAck");
    auto *clr = p.findChild<QPushButton *>("faultsClear");
    QVERIFY(ack && clr);
    QTest::mouseClick(ack, Qt::LeftButton);
    QCOMPARE(p.badgeText(), QString());
    QCOMPARE(p.model()->rowCount(), 2);
    QTest::mouseClick(clr, Qt::LeftButton);
    QCOMPARE(p.model()->rowCount(), 0);
  }
};

QTEST_MAIN(TstFaults)
#include "tst_faults.moc"
