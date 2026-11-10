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
// Each airframe preset, flown in the real firmware through the same path the
// Simulator page uses (SimWorker + SitlModule): geometry, the firmware mix and
// the preset's gains pushed as commands, RC fed through a pty like RcBridge's.
//
//   hover      arm, climb, hold ~5 s sim time under a simple altitude pilot;
//              altitude, attitude and drift must stay bounded
//   yaw sign   command a yaw stick and check the sign of the body yaw rate (a
//              wrong spin sign is yaw positive feedback), then release and
//              check it does not run away
//
// Wall-clock paced (the engine steps 1 ms per ms), so a full run takes about a
// minute. Skips, saying why, when no engine module can be loaded.

#include "AirframePresets.h"
#include "CommandCodec.h"
#include "SimWorker.h"
#include "SitlModule.h"

#include "navlink_msgs.h"

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
std::atomic<int> g_acks{0};
std::atomic<int> g_ackLast{-1};
std::mutex g_tsMtx;
struct TsResp {
  bool have = false;
  quint64 t1 = 0, t2 = 0, t3 = 0, t4 = 0;
} g_ts;
navlink_parser_t g_parser;
navlink_handlers_t g_handlers;

void onAck(void *, const navlink_frame_hdr_t *, const navlink_command_ack_t *m) {
  g_ackLast.store(m->result);
  g_acks.fetch_add(1);
}
void onTimeSync(void *, const navlink_frame_hdr_t *,
                const navlink_time_sync_t *m) {
  if (m->role != 1)
    return;
  std::lock_guard<std::mutex> lk(g_tsMtx);
  g_ts = {true, m->t1_gcs_tx, m->t2_fc_rx, m->t3_fc_tx,
          quint64(QDateTime::currentMSecsSinceEpoch())};
}
void onUart2(void *, const uint8_t *data, size_t n) {
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
  double t; // sim time [s]
  double x, y, alt;
  double roll, pitch, yaw; // deg
  double wz;               // body yaw rate [rad/s]
  double vz;               // climb rate [m/s]
  double duty;             // mean motor duty
  double dspread = 0;      // max-min motor duty
  double wx = 0, wy = 0;
  double stick = 0;        // pilot collective stick at this sample
};

constexpr double kTick = 1e-3; // engine step

} // namespace

class TstAirframeFlight : public QObject {
  Q_OBJECT
private slots:
  void initTestCase();
  void cleanupTestCase();
  void hover_data();
  void hover();
  void yawSign_data() { hover_data(); }
  void yawSign();

private:
  bool ready_ = false;
  QTemporaryDir vfs_;
  RcPty rc_;
  vsim::SimWorker *sim_ = nullptr;
  std::mutex mtx_;
  std::vector<Sample> log_;
  quint64 tick0_ = 0;
  bool haveTick0_ = false;

  Sample latest();
  void waitSim(double seconds); // wall-clock paced: sim seconds == wall seconds
  void handshake();
  void send(const QByteArray &frame) { SitlModule::instance().send(frame); }
  void configure(const vsim::AirframePreset &p);
  bool takeoff(const vsim::AirframePreset &p);
  void pilotAltitude(double target, double seconds, std::vector<Sample> *out);
  void land();

  double stick_ = 0.5;
  double iterm_ = 0.0;
};

Sample TstAirframeFlight::latest() {
  std::lock_guard<std::mutex> lk(mtx_);
  return log_.empty() ? Sample{} : log_.back();
}

void TstAirframeFlight::waitSim(double seconds) {
  const double t0 = latest().t;
  QElapsedTimer wall;
  wall.start();
  while (latest().t - t0 < seconds && wall.elapsed() < seconds * 3000 + 3000)
    QTest::qWait(5);
}

void TstAirframeFlight::initTestCase() {
  if (!SitlModule::instance().available())
    QSKIP(qPrintable("no SITL engine module: " + SitlModule::instance().error()));
  QVERIFY(vfs_.isValid());
  // Keep the firmware's persisted gains (0:pid.bin) out of the user's /tmp.
  qputenv("VAYU_VFS_DIR", vfs_.path().toLocal8Bit());
  qputenv("VSIM_FIFO_SUFFIX", "_tstaf" + QByteArray::number(::getpid()));
  QString slave;
  QVERIFY(rc_.open(&slave));
  qputenv("VAYU_UART_RC_PATH", slave.toLocal8Bit());

  navlink_parser_init(&g_parser);
  g_handlers = navlink_handlers_t{};
  g_handlers.on_command_ack = &onAck;
  g_handlers.on_time_sync = &onTimeSync;

  sim_ = new vsim::SimWorker;
  sim_->setTelemetrySink(&onUart2, nullptr);
  connect(
      sim_, &vsim::SimWorker::poseUpdated, this,
      [this](const vsim::SimSnapshot &s) {
        if (!haveTick0_) {
          tick0_ = s.tick_count;
          haveTick0_ = true;
        }
        float r, p, y;
        vsim::quatToEulerNED(s.att, &r, &p, &y);
        Sample m{};
        m.t = double(s.tick_count - tick0_) * kTick;
        m.x = s.pos_w.x();
        m.y = s.pos_w.y();
        m.alt = -s.pos_w.z();
        m.roll = r;
        m.pitch = p;
        m.yaw = y;
        m.wz = s.omega_b.z();
        m.vz = -s.vel_w.z();
        m.duty = (s.motor_duty[0] + s.motor_duty[1] + s.motor_duty[2] +
                  s.motor_duty[3]) / 4.0;
        m.dspread = *std::max_element(s.motor_duty.begin(), s.motor_duty.end()) - *std::min_element(s.motor_duty.begin(), s.motor_duty.end());
        m.wx = s.omega_b.x(); m.wy = s.omega_b.y();
        std::lock_guard<std::mutex> lk(mtx_);
        log_.push_back(m);
      },
      Qt::DirectConnection);
  QSignalSpy online(sim_, &vsim::SimWorker::online);
  sim_->start();
  QVERIFY2(online.wait(15000), "engine never came online");
  ready_ = true;
  waitSim(1.0);
  handshake();
}

void TstAirframeFlight::cleanupTestCase() {
  if (sim_) {
    sim_->requestStop();
    sim_->wait(5000);
    delete sim_;
    sim_ = nullptr;
  }
  if (SitlModule::instance().available())
    SitlModule::instance().api()->set_telemetry_sink(nullptr, nullptr);
}

// The firmware rejects every command until its clock is disciplined (the same
// handshake MainWindow runs against the in-app sim).
void TstAirframeFlight::handshake() {
  quint8 seq = 1;
  qint32 cmd = std::numeric_limits<qint32>::min();
  bool wide = false;
  qint64 wideOff = 0;
  for (int i = 0; i < 40; ++i) {
    {
      std::lock_guard<std::mutex> lk(g_tsMtx);
      g_ts.have = false;
    }
    const quint64 t1 = quint64(QDateTime::currentMSecsSinceEpoch());
    send(wide ? CommandCodec::encodeTimeSyncRequestWide(seq++, t1, wideOff)
              : CommandCodec::encodeTimeSyncRequest(seq++, t1, cmd));
    for (int w = 0; w < 100; ++w) {
      QTest::qWait(5);
      std::lock_guard<std::mutex> lk(g_tsMtx);
      if (g_ts.have)
        break;
    }
    TsResp r;
    {
      std::lock_guard<std::mutex> lk(g_tsMtx);
      r = g_ts;
    }
    if (!r.have)
      continue;
    const qint64 off = ((qint64(r.t2) - qint64(r.t1)) +
                        (qint64(r.t3) - qint64(r.t4))) / 2;
    const qint64 corr = -off;
    if (std::llabs(corr) < 50) {
      cmd = std::numeric_limits<qint32>::min();
      wide = false;
      if (i > 3)
        break;
    } else if (corr > INT32_MIN && corr <= INT32_MAX) {
      wide = false;
      cmd = qint32(corr);
    } else {
      wide = true;
      wideOff = corr;
    }
  }
}

void TstAirframeFlight::configure(const vsim::AirframePreset &p) {
  // Same order Start uses: mix signs to the firmware, layout to the physics,
  // then the preset's gains.
  float x[4], y[4], spin[4];
  for (int i = 0; i < 4; ++i) {
    x[i] = p.geometry.motors[i].pos.x();
    y[i] = p.geometry.motors[i].pos.y();
    spin[i] = float(p.geometry.motors[i].spin);
  }
  send(CommandCodec::encodeSetMotorGeometry(x, y, spin));
  sim_->sendGeometry(p.geometry);
  for (const PidSetCmd &c : p.gains)
    send(CommandCodec::encodeSetPid(c.controller, c.axis, c.kp, c.ki, c.kd,
                                    c.kff));
  waitSim(0.3);
}

void TstAirframeFlight::hover_data() {
  QTest::addColumn<QString>("id");
  for (const auto &p : vsim::airframePresets())
    QTest::newRow(qPrintable(p.id)) << p.id;
}

void TstAirframeFlight::hover() {
  if (!ready_)
    QSKIP("engine not running");
  QFETCH(QString, id);
  const vsim::AirframePreset *p = vsim::findAirframePreset(id);
  QVERIFY(p);
  QVERIFY(takeoff(*p));

  // Hold ~5 s sim time at 1 m.
  std::vector<Sample> hold;
  pilotAltitude(1.0, 5.0, &hold);
  double altMin = 1e9, altMax = -1e9, rpMax = 0, drift = 0, spread = 0;
  for (const Sample &s : hold) {
    altMin = std::min(altMin, s.alt);
    altMax = std::max(altMax, s.alt);
    rpMax = std::max({rpMax, std::fabs(s.roll), std::fabs(s.pitch)});
    drift = std::max(drift, std::hypot(s.x, s.y));
    spread = std::max(spread, s.dspread);
  }
  // A roll step in angle mode: the attitude loop must track it and settle (a
  // gain set too hot for the airframe limit-cycles instead).
  rc_.roll = 1800;
  std::vector<Sample> step;
  pilotAltitude(1.0, 0.6, &step);
  rc_.roll = 1500;
  pilotAltitude(1.0, 2.0, &step);
  double stepPeak = 0, tail = 0, tPeak = 0;
  for (const Sample &s : step)
    if (std::fabs(s.roll) > stepPeak) {
      stepPeak = std::fabs(s.roll);
      tPeak = s.t - step.front().t;
    }
  for (const Sample &s : step)
    if (s.t > step.back().t - 0.5)
      tail = std::max(tail, std::fabs(s.roll));
  const double dr = vsim::deriveAirframe(p->geometry).hoverDuty;
  qInfo("RESULT hover %s: alt %.2f..%.2f m, max|roll,pitch| %.2f deg, xy drift "
        "%.2f m, motor duty spread %.2f, hover stick %.2f, roll step peak "
        "%.1f deg at %.2f s (tail %.2f deg), predicted hover duty %.2f",
        qPrintable(id), altMin, altMax, rpMax, drift, spread, stick_, stepPeak,
        tPeak, tail, dr);
  land();

  QVERIFY2(altMin > 0.4 && altMax < 1.6, "altitude left the 1 m band");
  QVERIFY2(rpMax < 10.0, "attitude not bounded while hovering");
  QVERIFY2(drift < 5.0, "horizontal drift while hovering");
  QVERIFY2(spread < 0.5, "rate loop is limit-cycling (motor duty spread)");
  QVERIFY2(stepPeak > 3.0 && stepPeak < 40.0, "roll step response out of range");
  QVERIFY2(tail < 0.35 * stepPeak, "roll did not recover after the step");
}

void TstAirframeFlight::yawSign() {
  if (!ready_)
    QSKIP("engine not running");
  QFETCH(QString, id);
  const vsim::AirframePreset *p = vsim::findAirframePreset(id);
  QVERIFY(p);
  QVERIFY(takeoff(*p));
  std::vector<Sample> scratch;
  pilotAltitude(1.0, 1.0, &scratch);

  double mean[2] = {0, 0}, peak = 0;
  const int stickUs[2] = {1500 + 250, 1500 - 250};
  for (int k = 0; k < 2; ++k) {
    rc_.yaw = stickUs[k];
    std::vector<Sample> run;
    pilotAltitude(1.0, 1.8, &run);
    int n = 0;
    for (const Sample &s : run) {
      peak = std::max(peak, std::fabs(s.wz));
      if (s.t > run.back().t - 0.8) {
        mean[k] += s.wz;
        ++n;
      }
    }
    mean[k] /= std::max(n, 1);
    // Release and let it settle before the other direction.
    rc_.yaw = 1500;
    std::vector<Sample> rel;
    pilotAltitude(1.0, 1.5, &rel);
    double tail = 0;
    for (const Sample &s : rel)
      if (s.t > rel.back().t - 0.5)
        tail = std::max(tail, std::fabs(s.wz));
    QVERIFY2(tail < 1.5, "yaw rate did not die out after the stick was released");
  }
  qInfo("RESULT yaw %s: yaw rate with stick right %+.2f rad/s, stick left "
        "%+.2f rad/s, peak %.2f rad/s",
        qPrintable(id), mean[0], mean[1], peak);
  land();
  // Body z is down, so right (clockwise from above) is positive.
  QVERIFY2(mean[0] > 0.03, "yaw stick right did not give a positive yaw rate");
  QVERIFY2(mean[1] < -0.03, "yaw stick left did not give a negative yaw rate");
  QVERIFY2(peak < 12.0, "yaw rate ran away (positive feedback?)");
}

bool TstAirframeFlight::takeoff(const vsim::AirframePreset &p) {
  rc_.arm = 1000;
  rc_.thr = 1000;
  rc_.roll = rc_.pitch = rc_.yaw = 1500;
  sim_->sendResetPose(0, 0, -0.05f);
  configure(p);
  rc_.arm = 2000;
  waitSim(0.5);
  std::vector<Sample> climb;
  stick_ = 0.5;
  iterm_ = 0.0;
  pilotAltitude(1.0, 4.0, &climb);
  const Sample s = latest();
  return s.alt > 0.5 && std::isfinite(s.alt);
}

void TstAirframeFlight::pilotAltitude(double target, double seconds,
                                      std::vector<Sample> *out) {
  // A climb-rate pilot: altitude error -> a bounded climb-rate demand, then a PI
  // on the rate error around mid-stick (the firmware's hover point). The
  // integral finds each airframe's own hover stick; it carries across calls.
  const double t0 = latest().t;
  double tPrev = t0;
  while (latest().t - t0 < seconds) {
    const Sample s = latest();
    const double dt = std::clamp(s.t - tPrev, 0.0, 0.1);
    tPrev = s.t;
    const double vzCmd = std::clamp(0.8 * (target - s.alt), -1.0, 1.2);
    const double verr = vzCmd - s.vz;
    iterm_ = std::clamp(iterm_ + 0.35 * verr * dt, -0.3, 0.45);
    stick_ = std::clamp(0.5 + 0.12 * verr + iterm_, 0.05, 0.95);
    rc_.thr = int(1000 + 1000 * stick_);
    out->push_back(s);
    QTest::qWait(10);
  }
}

void TstAirframeFlight::land() {
  rc_.thr = 1000;
  rc_.roll = rc_.pitch = rc_.yaw = 1500;
  waitSim(1.5);
  rc_.arm = 1000;
  waitSim(0.5);
}

QTEST_MAIN(TstAirframeFlight)
#include "tst_airframe_flight.moc"
