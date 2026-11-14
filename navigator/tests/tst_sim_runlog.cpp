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
// Per-run log files of the real SimulatorWidget (gt-<stamp>.bin + ev-<stamp>.jsonl).
//
//  - missingModuleLeavesNoLogs: a Start whose engine module cannot load must
//    leave no gt/ev files (run with VAYU_SITL_MODULE pointing at a missing
//    file).
//  - rapidStartStopOneRunPerStart: many Start/Stop cycles, far shorter than a
//    second apart, produce one gt+ev pair per Start, none overwritten (needs
//    the real engine module; skips without one).
//
// The module loads once per process, so ctest runs each case in its own
// process with its own VAYU_SITL_MODULE. The widget owns GL views, so it needs
// a real platform plugin (xcb on an Xvfb); on "offscreen" both cases skip.
// Give it private VAYU_VFS_DIR / VSIM_FIFO_SUFFIX / XDG_* when running it.

#include "SimulatorWidget.h"
#include "SitlModule.h"

#include <QApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QGuiApplication>
#include <QTemporaryDir>
#include <QtTest>

// Test seam: SimulatorWidget befriends this name.
class SimulatorWidgetTestAccess {
public:
  explicit SimulatorWidgetTestAccess(SimulatorWidget &w) : w_(w) {}
  void setLogDir(const QString &d) { w_.m_logDir = d; }
  void setKeepRuns(int n) { w_.m_logKeepRuns = n; }
  void start() { w_.startInAppSim(); }
  void stop() { w_.stopInAppSim(); }
  bool running() const { return w_.m_sim != nullptr; }
  QString logText() const { return w_.m_log->toPlainText(); }

private:
  SimulatorWidget &w_;
};

namespace {
void pump(int ms) {
  QElapsedTimer t;
  t.start();
  while (t.elapsed() < ms) {
    QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    QThread::msleep(5);
  }
}
} // namespace

class TstSimRunLog : public QObject {
  Q_OBJECT
  bool gl_ = false;

private slots:
  void initTestCase() {
    gl_ = QGuiApplication::platformName() != QLatin1String("offscreen");
  }

  void missingModuleLeavesNoLogs() {
    if (!gl_)
      QSKIP("needs a GL platform plugin (QT_QPA_PLATFORM=xcb on Xvfb)");
    if (SitlModule::instance().available())
      QSKIP("an engine module loaded; run with VAYU_SITL_MODULE=<missing file>");
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    SimulatorWidget w;
    SimulatorWidgetTestAccess a(w);
    a.setLogDir(dir.path());
    w.resize(1200, 800);
    w.show();
    QVERIFY(QTest::qWaitForWindowExposed(&w));
    for (int i = 0; i < 3; ++i) {
      a.start();
      // The worker reports the missing module and the widget returns to Stopped.
      QElapsedTimer t;
      t.start();
      while (a.running() && t.elapsed() < 5000)
        pump(20);
      QVERIFY2(!a.running(), "the failed start never returned to Stopped");
      pump(100);
    }
    QVERIFY2(a.logText().contains("rtos engine:"), "no engine failure shown");
    const QStringList left = QDir(dir.path()).entryList(QDir::Files);
    QVERIFY2(left.isEmpty(), qPrintable(left.join(", ")));
  }

  void rapidStartStopOneRunPerStart() {
    if (!gl_)
      QSKIP("needs a GL platform plugin (QT_QPA_PLATFORM=xcb on Xvfb)");
    if (!SitlModule::instance().available())
      QSKIP("no engine module (set VAYU_SITL_MODULE)");
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    SimulatorWidget w;
    SimulatorWidgetTestAccess a(w);
    a.setLogDir(dir.path());
    a.setKeepRuns(0); // keep everything: count every run
    w.resize(1200, 800);
    w.show();
    QVERIFY(QTest::qWaitForWindowExposed(&w));
    constexpr int kCycles = 20;
    QElapsedTimer total;
    total.start();
    for (int i = 0; i < kCycles; ++i) {
      a.start();
      QVERIFY(a.running());
      pump(100); // far below one second: many runs share a wall-clock second
      a.stop();
    }
    qInfo("%d cycles in %lld ms", kCycles, total.elapsed());
    QCOMPARE(QDir(dir.path()).entryList({"gt-*.bin"}).size(), kCycles);
    QCOMPARE(QDir(dir.path()).entryList({"ev-*.jsonl"}).size(), kCycles);
    // Every gt has its ev under the same stamp.
    for (const QString &g : QDir(dir.path()).entryList({"gt-*.bin"})) {
      const QString stamp = g.mid(3, g.size() - 3 - 4);
      QVERIFY2(QFile::exists(dir.filePath("ev-" + stamp + ".jsonl")),
               qPrintable(stamp));
    }
  }
};

QTEST_MAIN(TstSimRunLog)
#include "tst_sim_runlog.moc"
