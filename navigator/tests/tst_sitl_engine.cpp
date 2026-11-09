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
// The in-app simulator end to end, headless, through Navigator's own classes
// in the order SimulatorWidget wires them: RcBridge's pty (a named pipe on
// Windows) exported as VAYU_UART_RC_PATH, a telemetry sink, then a SimWorker
// that loads and drives the engine module. No widget, no display.
//
// It proves three things a unit test of any one class cannot: the module
// loads, boots and steps (poses arrive with an advancing tick), firmware
// telemetry reaches the sink, and RC synthesised by RcBridge reaches the
// firmware through the pty/pipe -- read back from the firmware's own
// RC_CHANNELS telemetry.
//
// Skips, saying why, when no engine module can be found: it needs a firmware
// build or release beside the binaries (CI copies one), not just headers.

#include "RcBridge.h"
#include "SimWorker.h"
#include "SitlModule.h"

#include "navlink_msgs.h"

#include <QSignalSpy>
#include <QtTest>

#include <atomic>

namespace {

// Written on the engine's thread (the sink), read on the test's.
std::atomic<int> g_telemetryBytes{0};
std::atomic<int> g_rcFromBridge{0};
navlink_parser_t g_parser;
navlink_handlers_t g_handlers;

void onRcChannels(void *, const navlink_frame_hdr_t *,
                  const navlink_rc_channels_t *m) {
  // The bridge's frame below: throttle at idle (1000), arm forced off (1000),
  // acro on (ch6 2000). The firmware's own fallback when no RC arrives is a
  // hover frame -- throttle 1300, arm 2000, ch6 1500 -- so this cannot be it.
  if (m->chan[2] == 1000 && m->chan[4] == 1000 && m->chan[5] == 2000)
    g_rcFromBridge.fetch_add(1);
}

void onUart2(void *, const uint8_t *data, size_t n) {
  g_telemetryBytes.fetch_add(static_cast<int>(n));
  navlink_parser_push(&g_parser, &g_handlers, data, n);
}

} // namespace

class TstSitlEngine : public QObject {
  Q_OBJECT
private slots:
  void resetBodyCarriesPoseAndSeed();
  void seedResolution();
  void engineRunsWithRcFromTheBridge();
};

// The reset message the GCS sends on Start / Reset: pose, level attitude, zero
// velocity, and the noise seed in the new tail field (no engine needed).
void TstSitlEngine::resetBodyCarriesPoseAndSeed() {
  const vsim_ctl_reset_t b = vsim::SimWorker::resetBody(1.5f, -2.0f, -0.4f, 1337u);
  QCOMPARE(b.pos_w[0], 1.5f);
  QCOMPARE(b.pos_w[1], -2.0f);
  QCOMPARE(b.pos_w[2], -0.4f);
  QCOMPARE(b.quat_wxyz[0], 1.0f);
  QCOMPARE(b.quat_wxyz[1], 0.0f);
  for (int i = 0; i < 3; ++i) {
    QCOMPARE(b.vel_w[i], 0.0f);
    QCOMPARE(b.omega_b[i], 0.0f);
  }
  QCOMPARE(b.seed, 1337u);
  // Legacy callers still send seed 0 (free-running noise).
  QCOMPARE(vsim::SimWorker::resetBody(0, 0, 0, 0).seed, 0u);
}

void TstSitlEngine::seedResolution() {
  using vsim::SimWorker;
  QCOMPARE(SimWorker::resolveResetSeed(false, 42u, 7u), 42u); // fixed wins
  QCOMPARE(SimWorker::resolveResetSeed(true, 42u, 7u), 7u);   // random per start
  // Never 0: that would mean "do not seed".
  QCOMPARE(SimWorker::resolveResetSeed(false, 0u, 7u), 1u);
  QCOMPARE(SimWorker::resolveResetSeed(true, 42u, 0u), 1u);
}

void TstSitlEngine::engineRunsWithRcFromTheBridge() {
  if (!SitlModule::instance().available())
    QSKIP(qPrintable("no SITL engine module: " + SitlModule::instance().error()));

  navlink_parser_init(&g_parser);
  g_handlers = navlink_handlers_t{};
  g_handlers.on_rc_channels = &onRcChannels;

  // RC first: the firmware's feeder reads VAYU_UART_RC_PATH once, at boot.
  RcBridge rc;
  rc.setJoystickPath(QStringLiteral("no-such-joystick")); // -> neutral axes
  rc.setEnabled(true);
  rc.setArmOverride(0); // arm 1000
  rc.setAcro(true);     // ch6 2000
  QString err;
  QVERIFY2(rc.openPty(&err), qPrintable(err));
  qputenv("VAYU_UART_RC_PATH", rc.slavePath().toLocal8Bit());
  rc.start();

  vsim::SimWorker sim;
  sim.setTelemetrySink(&onUart2, nullptr);
  QSignalSpy online(&sim, &vsim::SimWorker::online);
  quint64 firstTick = 0, lastTick = 0;
  int poses = 0;
  connect(&sim, &vsim::SimWorker::poseUpdated, this,
          [&](const vsim::SimSnapshot &s) {
            if (poses++ == 0)
              firstTick = s.tick_count;
            lastTick = s.tick_count;
          });
  sim.start();

  QVERIFY2(online.wait(10000), "engine never came online");
  QTRY_VERIFY_WITH_TIMEOUT(poses > 20 && lastTick > firstTick, 10000);
  QTRY_VERIFY_WITH_TIMEOUT(g_telemetryBytes.load() > 0, 10000);
  QTRY_VERIFY2_WITH_TIMEOUT(g_rcFromBridge.load() > 0,
                            "RcBridge's frames never reached the firmware", 10000);

  sim.requestStop();
  QVERIFY(sim.wait(5000));
  rc.requestStop();
  QVERIFY(rc.wait(2000));
  // The firmware's tasks outlive the worker; stop them calling into a sink
  // whose parser this test is about to stop owning.
  SitlModule::instance().api()->set_telemetry_sink(nullptr, nullptr);
}

QTEST_MAIN(TstSitlEngine)
#include "tst_sitl_engine.moc"
