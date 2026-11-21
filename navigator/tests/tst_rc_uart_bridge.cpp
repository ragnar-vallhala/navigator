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
// RcBridge's UART reader end to end over a pty standing in for the Arduino
// bridge: a corrupt line must never become a stick movement, and the raw-line
// log must show what arrived. POSIX only (needs a pty).
#include <QtTest>

#ifndef _WIN32
#include <fcntl.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include "RcBridge.h"

class TstRcUartBridge : public QObject {
  Q_OBJECT
  int master_ = -1;
  int keepSlave_ = -1; // keeps the line open so the master never sees a hangup
  QString slave_;

  void send(const char *s) {
    const ssize_t w = ::write(master_, s, strlen(s));
    QVERIFY2(w == (ssize_t)strlen(s), "pty write");
  }

private slots:
  void initTestCase() {
    QStandardPaths::setTestModeEnabled(true); // log goes to a test location
    master_ = posix_openpt(O_RDWR | O_NOCTTY);
    QVERIFY(master_ >= 0);
    QVERIFY(grantpt(master_) == 0 && unlockpt(master_) == 0);
    slave_ = QString::fromLocal8Bit(ptsname(master_));
    keepSlave_ = ::open(slave_.toLocal8Bit().constData(), O_RDWR | O_NOCTTY);
    QVERIFY(keepSlave_ >= 0);
    termios t;
    QVERIFY(tcgetattr(keepSlave_, &t) == 0);
    cfmakeraw(&t);
    tcsetattr(keepSlave_, TCSANOW, &t);
  }

  void cleanupTestCase() {
    if (keepSlave_ >= 0)
      ::close(keepSlave_);
    if (master_ >= 0)
      ::close(master_);
  }

  void corruptLinesNeverMoveTheSticksAndLogShowsThem() {
    QString logPath;
    QVector<QVector<int>> seen;
    {
      RcBridge b;
      QObject::connect(&b, &RcBridge::channelsUpdated, &b,
                       [&](int r, int p, int t, int y, int a, int c) {
                         seen.push_back({r, p, t, y, a, c});
                       });
      QObject::connect(&b, &RcBridge::logLine, &b, [&](QString l) {
        const int i = l.indexOf(QStringLiteral("raw log -> "));
        if (i >= 0)
          logPath = l.mid(i + 11);
      });
      b.setEnabled(true);
      b.setSource(RcBridge::Uart);
      b.setUartPath(slave_);
      b.setUartBaud(115200);
      b.setUartLog(true);
      b.setUartConnected(true);
      b.start();

      const char *good = "1500,1500,1000,1500,2000,1000\n"; // armed, throttle min
      // Warm up until the armed frame is live.
      for (int i = 0; i < 40; ++i) {
        send(good);
        QTest::qWait(20);
      }
      QVERIFY(!seen.isEmpty());
      QCOMPARE(seen.last()[4], 2000);

      // Each of these used to be clamped into a real command (arm or throttle
      // forced to 1000, or a frame mixing old and new channels).
      seen.clear();
      const char *bad[] = {
          "1500,1500,1800,1500,10\n",         // cut off: arm would clamp to 1000
          "00,1500,1800,1500,1000,1000\n",    // started mid-line: roll -> 1000
          "1500,1500,1\n",                    // cut off: throttle would clamp
          "15001500,1800,1500,1000\n",        // lost a comma
          "1500,1500,1800,1500,200\n",        // lost digits
      };
      for (int rep = 0; rep < 3; ++rep)
        for (const char *l : bad) {
          send(l);
          QTest::qWait(35); // a full read tick sees the bad line on its own
          send(good);
          QTest::qWait(20);
        }
      // Over-long junk, then good frames resume.
      send(QByteArray(300, '7').append('\n').constData());
      for (int i = 0; i < 15; ++i) {
        send(good);
        QTest::qWait(20);
      }
      QVERIFY(seen.size() > 20);
      for (const auto &c : std::as_const(seen)) {
        QCOMPARE(c[2], 1000); // throttle never left min
        QCOMPARE(c[4], 2000); // arm never dropped or glitched
        QCOMPARE(c[0], 1500); // roll never took a half-parsed value
      }
    } // bridge stopped: log flushed and closed

    QVERIFY2(!logPath.isEmpty(), "bridge announced its log path");
    QFile f(logPath);
    QVERIFY(f.open(QIODevice::ReadOnly));
    const QString log = QString::fromUtf8(f.readAll());
    QVERIFY(log.contains(QStringLiteral("\tOK\t1500,1500,1000,1500,2000,1000")));
    QVERIFY(log.contains(QStringLiteral("\tREJECT\t1500,1500,1800,1500,10")));
    QVERIFY(log.contains(QStringLiteral("\tREJECT\t00,1500,1800,1500,1000,1000")));
    QVERIFY(log.contains(QStringLiteral("\tDROP\t")));
    QVERIFY(log.contains(QStringLiteral("\tARM\t")));
    QVERIFY(log.contains(QStringLiteral("\tSTAT\tok=")));
    QVERIFY(log.contains(QStringLiteral("fwd=")));
    QFile::remove(logPath);
  }
};

QTEST_MAIN(TstRcUartBridge)
#include "tst_rc_uart_bridge.moc"
#else
int main() { return 0; }
#endif
