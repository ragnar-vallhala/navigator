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
// Simulator Stop -> Start, twice, against the real engine module.
//
// The engine boots once per process, so a Stop only ends the worker thread and
// the next Start runs a fresh SimWorker against the same running firmware.
// What a Start does is reproduced here in the order SimulatorWidget does it:
// new worker, wait for `online`, send the spawn reset, run the time-sync
// handshake, push the firmware mix and flight mode through CommandRetrier.
// RC arrives through a pty, as from RcBridge.
//
// For each of the three runs it asserts
//   - the body is at the spawn pose: on the spawn point, level, at rest, with
//     the motors not driven;
//   - the Start commands (mix, flight mode) end ACK OK;
//   - ONE RC feeder reads the pty: every Start used to add another, and the
//     feeders split the byte stream between them, so the arm switch
//     flickered the firmware between armed and disarmed;
//   - holding the arm switch up arms once and stays armed.
// Plus a pure check of the once-per-process gate that keeps the feeder single.
//
// Wall-clock paced (about 40 s). The engine part skips, saying why, when no
// engine module can be loaded; set VAYU_VFS_DIR / VSIM_FIFO_SUFFIX to private
// values when running it so it cannot touch another sim's state.

#include "CommandCodec.h"
#include "CommandRetrier.h"
#include "SimWorker.h"
#include "SitlModule.h"

#include "navlink_msgs.h"

#include <QDir>
#include <QSignalSpy>
#include <QTemporaryDir>
#include <QtTest>

#include <atomic>
#include <cmath>
#include <fcntl.h>
#include <mutex>
#include <stdlib.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

// ---- firmware telemetry: command acks and the time-sync handshake ---------
std::mutex g_mtx;
std::vector<std::pair<quint32, int>> g_acks; // (command id, result)
struct TsResp {
  bool have = false;
  quint64 t1 = 0, t2 = 0, t3 = 0, t4 = 0;
} g_ts;
navlink_parser_t g_parser;
navlink_handlers_t g_handlers;

void onAck(void *, const navlink_frame_hdr_t *, const navlink_command_ack_t *m) {
  std::lock_guard<std::mutex> lk(g_mtx);
  g_acks.emplace_back(m->command, m->result);
}
void onTimeSync(void *, const navlink_frame_hdr_t *,
                const navlink_time_sync_t *m) {
  if (m->role != 1)
    return;
  std::lock_guard<std::mutex> lk(g_mtx);
  g_ts = {true, m->t1_gcs_tx, m->t2_fc_rx, m->t3_fc_tx,
          quint64(QDateTime::currentMSecsSinceEpoch())};
}
std::mutex g_parserMtx; // not g_mtx: the handlers above take that one
void onUart2(void *, const uint8_t *data, size_t n) {
  std::lock_guard<std::mutex> lk(g_parserMtx);
  navlink_parser_push(&g_parser, &g_handlers, data, n);
}

// ---- RC through a pty, the way RcBridge feeds the firmware ----------------
class RcPty {
public:
  bool open(QString *slave) {
    fd_ = ::posix_openpt(O_RDWR | O_NOCTTY);
    if (fd_ < 0 || ::grantpt(fd_) != 0 || ::unlockpt(fd_) != 0)
      return false;
    *slave = QString::fromLocal8Bit(::ptsname(fd_));
    th_ = std::thread([this] {
      while (!stop_.load()) {
        char buf[96];
        const int n = std::snprintf(buf, sizeof buf, "%d,%d,%d,%d,%d,%d\n",
                                    roll.load(), pitch.load(), thr.load(),
                                    yaw.load(), arm.load(), ch6.load());
        if (::write(fd_, buf, size_t(n)) < 0) { /* nobody reading yet */ }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    });
    return true;
  }
  ~RcPty() {
    stop_.store(true);
    if (th_.joinable())
      th_.join();
    if (fd_ >= 0)
      ::close(fd_);
  }
  std::atomic<int> roll{1500}, pitch{1500}, thr{1000}, yaw{1500}, arm{1000},
      ch6{1500};

private:
  int fd_ = -1;
  std::atomic<bool> stop_{false};
  std::thread th_;
};

struct Sample {
  quint64 tick;
  double x, y, z; // NED [m]
  double speed;   // |v| [m/s]
  double tilt;    // max(|roll|, |pitch|) [deg]
  double duty;    // max motor duty
};

// How many descriptors this process holds open on the RC pty's slave side: one
// per feeder thread the engine started. -1 where /proc is not available.
int openersOf(const QString &slave) {
#ifdef __linux__
  int n = 0;
  const QDir fds("/proc/self/fd");
  for (const QString &e : fds.entryList(QDir::NoDotAndDotDot)) {
    char buf[256];
    const QByteArray p = fds.filePath(e).toLocal8Bit();
    const ssize_t k = ::readlink(p.constData(), buf, sizeof buf - 1);
    if (k > 0 && QString::fromLocal8Bit(buf, int(k)) == slave)
      ++n;
  }
  return n;
#else
  Q_UNUSED(slave);
  return -1;
#endif
}

} // namespace

class TstSimRestart : public QObject {
  Q_OBJECT
private slots:
  void initTestCase();
  void cleanupTestCase();
  void serialRcStartsOncePerProcess();
  void stopStartTwice();

private:
  bool ready_ = false;
  QTemporaryDir vfs_;
  RcPty rc_;
  QString slave_;
  std::mutex smtx_;
  std::vector<Sample> samples_;

  void startRun(vsim::SimWorker **out);
  void stopRun(vsim::SimWorker *sim);
  bool handshake();
  void send(const QByteArray &frame) { SitlModule::instance().send(frame); }
  std::vector<Sample> window(double seconds);
  bool lastDutyOn() {
    std::lock_guard<std::mutex> lk(smtx_);
    return !samples_.empty() && samples_.back().duty > 0.01;
  }
};

void TstSimRestart::serialRcStartsOncePerProcess() {
  std::atomic<bool> gate{false};
  QVERIFY(vsim::SimWorker::claimSerialRcStart(gate));  // first Start
  QVERIFY(!vsim::SimWorker::claimSerialRcStart(gate)); // Stop -> Start
  QVERIFY(!vsim::SimWorker::claimSerialRcStart(gate)); // and again
}

void TstSimRestart::initTestCase() {
  QString vfs = qEnvironmentVariable("VAYU_VFS_DIR");
  if (vfs.isEmpty()) {
    QVERIFY(vfs_.isValid());
    vfs = vfs_.path();
    qputenv("VAYU_VFS_DIR", vfs.toLocal8Bit());
  }
  QVERIFY2(QDir(vfs).isEmpty(), qPrintable("VAYU_VFS_DIR not empty: " + vfs));
  if (qEnvironmentVariableIsEmpty("VSIM_FIFO_SUFFIX"))
    qputenv("VSIM_FIFO_SUFFIX", "_tstsr" + QByteArray::number(::getpid()));
  if (!SitlModule::instance().available())
    return; // stopStartTwice skips with the loader's reason
  QVERIFY(rc_.open(&slave_));
  qputenv("VAYU_UART_RC_PATH", slave_.toLocal8Bit());

  navlink_parser_init(&g_parser);
  g_handlers = navlink_handlers_t{};
  g_handlers.on_command_ack = &onAck;
  g_handlers.on_time_sync = &onTimeSync;
  ready_ = true;
}

void TstSimRestart::cleanupTestCase() {
  if (SitlModule::instance().available())
    SitlModule::instance().api()->set_telemetry_sink(nullptr, nullptr);
}

// What SimulatorWidget::startInAppSim does up to the spawn reset.
void TstSimRestart::startRun(vsim::SimWorker **out) {
  {
    std::lock_guard<std::mutex> lk(smtx_);
    samples_.clear();
  }
  SitlModule::instance().api()->set_telemetry_sink(&onUart2, nullptr);
  auto *sim = new vsim::SimWorker;
  sim->setTelemetrySink(&onUart2, nullptr);
  connect(
      sim, &vsim::SimWorker::poseUpdated, this,
      [this](const vsim::SimSnapshot &s) {
        float r, p, y;
        vsim::quatToEulerNED(s.att, &r, &p, &y);
        Sample m{};
        m.tick = s.tick_count;
        m.x = s.pos_w.x();
        m.y = s.pos_w.y();
        m.z = s.pos_w.z();
        m.speed = std::sqrt(double(s.vel_w.x()) * s.vel_w.x() +
                            double(s.vel_w.y()) * s.vel_w.y() +
                            double(s.vel_w.z()) * s.vel_w.z());
        m.tilt = std::max(std::fabs(r), std::fabs(p));
        m.duty = *std::max_element(s.motor_duty.begin(), s.motor_duty.end());
        std::lock_guard<std::mutex> lk(smtx_);
        samples_.push_back(m);
      },
      Qt::DirectConnection);
  QSignalSpy online(sim, &vsim::SimWorker::online);
  sim->start();
  QVERIFY2(online.wait(15000), "engine never came online");
  sim->sendResetPose(0.0f, 0.0f, -0.05f); // Start's spawn reset
  *out = sim;
}

void TstSimRestart::stopRun(vsim::SimWorker *sim) {
  // stopInAppSim: detach the sink, stop the worker; the firmware keeps running.
  SitlModule::instance().api()->set_telemetry_sink(nullptr, nullptr);
  sim->requestStop();
  QVERIFY(sim->wait(5000));
  delete sim;
}

// MainWindow's time-sync loop, until the offset is inside the 50 ms deadband.
bool TstSimRestart::handshake() {
  quint8 seq = 1;
  qint32 cmd = std::numeric_limits<qint32>::min();
  bool wide = false;
  qint64 wideOff = 0;
  for (int i = 0; i < 40; ++i) {
    {
      std::lock_guard<std::mutex> lk(g_mtx);
      g_ts.have = false;
    }
    const quint64 t1 = quint64(QDateTime::currentMSecsSinceEpoch());
    send(wide ? CommandCodec::encodeTimeSyncRequestWide(seq++, t1, wideOff)
              : CommandCodec::encodeTimeSyncRequest(seq++, t1, cmd));
    TsResp r;
    for (int w = 0; w < 100; ++w) {
      QTest::qWait(5);
      std::lock_guard<std::mutex> lk(g_mtx);
      if (g_ts.have) {
        r = g_ts;
        break;
      }
    }
    if (!r.have)
      continue;
    const qint64 off = ((qint64(r.t2) - qint64(r.t1)) +
                        (qint64(r.t3) - qint64(r.t4))) / 2;
    const qint64 corr = -off;
    if (std::llabs(corr) < 50)
      return true;
    if (corr > INT32_MIN && corr <= INT32_MAX) {
      wide = false;
      cmd = qint32(corr);
    } else {
      wide = true;
      wideOff = corr;
    }
  }
  return false;
}

std::vector<Sample> TstSimRestart::window(double seconds) {
  size_t i0;
  {
    std::lock_guard<std::mutex> lk(smtx_);
    i0 = samples_.size();
  }
  QTest::qWait(int(seconds * 1000));
  std::lock_guard<std::mutex> lk(smtx_);
  return std::vector<Sample>(samples_.begin() + long(i0), samples_.end());
}

void TstSimRestart::stopStartTwice() {
  if (!ready_)
    QSKIP(qPrintable("no SITL engine module: " + SitlModule::instance().error()));

  for (int run = 1; run <= 3; ++run) { // Start, then Stop -> Start twice
    const QByteArray tag = "run " + QByteArray::number(run);
    rc_.arm.store(1000); // switch down at every Start
    rc_.thr.store(1000);
    {
      std::lock_guard<std::mutex> lk(g_mtx);
      g_acks.clear();
    }
    vsim::SimWorker *sim = nullptr;
    startRun(&sim);
    QVERIFY2(sim, tag.constData());

    // ---- spawn pose: sampled right after the reset
    // The reset is queued behind `online`, and every Start's tick count starts
    // at 0 (run_begin), so the poses at tick <= 400 are the first 0.4 s of this
    // Start. Wait until they are all in.
    QTRY_VERIFY2_WITH_TIMEOUT(
        [&] {
          std::lock_guard<std::mutex> lk(smtx_);
          return !samples_.empty() && samples_.back().tick > 420;
        }(),
        qPrintable(tag + ": the engine stopped posing after Start"), 5000);
    std::vector<Sample> spawn;
    {
      std::lock_guard<std::mutex> lk(smtx_);
      for (const Sample &s : samples_)
        if (s.tick <= 400)
          spawn.push_back(s);
    }
    QVERIFY2(!spawn.empty(), tag.constData());
    double xyMax = 0, tiltMax = 0, dutyMax = 0, vMax = 0, zMin = 1e9, zMax = -1e9;
    for (const Sample &s : spawn) {
      xyMax = std::max(xyMax, std::hypot(s.x, s.y));
      tiltMax = std::max(tiltMax, s.tilt);
      dutyMax = std::max(dutyMax, s.duty);
      vMax = std::max(vMax, s.speed);
      zMin = std::min(zMin, s.z);
      zMax = std::max(zMax, s.z);
    }
    qInfo("%s spawn: n=%zu tick %llu..%llu |xy|max=%.4f z=[%.3f,%.3f] "
          "vmax=%.3f tiltmax=%.2f dutymax=%.3f",
          tag.constData(), spawn.size(), (unsigned long long)spawn.front().tick,
          (unsigned long long)spawn.back().tick, xyMax, zMin, zMax, vMax,
          tiltMax, dutyMax);
    QVERIFY2(xyMax < 0.05, qPrintable(tag + ": not on the spawn point"));
    QVERIFY2(zMin > -0.2 && zMax < 0.05,
             qPrintable(tag + ": spawn height off the ground (z " +
                        QString::number(zMin) + ".." + QString::number(zMax) + ")"));
    QVERIFY2(tiltMax < 1.0, qPrintable(tag + ": not upright"));
    QVERIFY2(vMax < 1.5, qPrintable(tag + ": not at rest (v " +
                                    QString::number(vMax) + ")"));
    QVERIFY2(dutyMax < 0.01, qPrintable(tag + ": motors driven at spawn"));

    // ---- Start commands: mix + flight mode end ACK OK
    CommandRetrier retr;
    float mx[4] = {0.0866f, -0.0866f, -0.0866f, 0.0866f};
    float my[4] = {0.0866f, 0.0866f, -0.0866f, -0.0866f};
    float spin[4] = {1, -1, 1, -1};
    retr.setSynced(false, 0);
    retr.arm("mix", NAVLINK_MSGID_CMD_SET_MOTOR_GEOMETRY, 1, "mix", "", [&] {
      send(CommandCodec::encodeSetMotorGeometry(mx, my, spin));
    }, 0);
    retr.arm("flightmode", NAVLINK_MSGID_CMD_SET_FLIGHT_MODE, 1, "mode", "",
             [&] { send(CommandCodec::encodeSetFlightMode(2)); }, 0);
    QVERIFY2(handshake(), qPrintable(tag + ": time sync never locked"));
    qint64 now = 0;
    retr.setSynced(true, now);
    for (int i = 0; i < 80 && retr.anyPending(); ++i) {
      QTest::qWait(100);
      now += 100;
      std::vector<std::pair<quint32, int>> acks;
      {
        std::lock_guard<std::mutex> lk(g_mtx);
        acks.swap(g_acks);
      }
      for (const auto &a : acks)
        retr.onAck(a.first, a.second);
      retr.tick(now);
    }
    QVERIFY2(retr.state("mix") && retr.state("mix")->accepted(),
             qPrintable(tag + ": mix command did not end ACK OK"));
    QVERIFY2(retr.state("flightmode") && retr.state("flightmode")->accepted(),
             qPrintable(tag + ": flight-mode command did not end ACK OK"));

    const int feeders = openersOf(slave_); // checked after the arm window
    // ---- arm: switch up with the throttle low arms once and stays armed
    rc_.arm.store(2000);
    QTest::qWait(500); // let the firmware take it
    const std::vector<Sample> armed = window(2.0);
    int flips = 0, on = 0;
    bool prev = false;
    for (const Sample &s : armed) {
      const bool d = s.duty > 0.01;
      on += d;
      flips += d != prev;
      prev = d;
    }
    qInfo("%s armed window: n=%zu duty>0 in %d samples, %d on/off edges",
          tag.constData(), armed.size(), on, flips);
    QVERIFY2(!armed.empty() && on > int(armed.size()) * 9 / 10 && flips <= 1,
             qPrintable(tag + ": arm state flickered (" + QString::number(flips) +
                        " edges, " + QString::number(on) + "/" +
                        QString::number(armed.size()) + " samples driven)"));

    qInfo("%s: RC feeders on the pty: %d", tag.constData(), feeders);
    if (feeders >= 0)
      QVERIFY2(feeders == 1, qPrintable(tag + ": " + QString::number(feeders) +
                                        " RC feeders read the pty, want 1"));

    rc_.arm.store(1000); // disarm, then Stop
    QTest::qWait(300);
    stopRun(sim);
    QTest::qWait(2000); // a person takes a moment to press Start again
  }
}

QTEST_MAIN(TstSimRestart)
#include "tst_sim_restart.moc"
