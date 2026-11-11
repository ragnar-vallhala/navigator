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
#include "PerfHarness.h"

#include "../core/PerfProbe.h"
#include "MainWindow.h"
#ifdef NAVIGATOR_HAS_SITL
#include "SimulatorWidget.h"
#endif

#include <QAbstractEventDispatcher>
#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDir>
#include <QGuiApplication>
#include <QEvent>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaEnum>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QPushButton>
#include <QScreen>
#include <QThread>
#include <QTimer>

#include <algorithm>
#include <atomic>
#include <csignal>
#include <cstdlib>
#include <dlfcn.h>
#include <execinfo.h>
#include <ctime>
#include <sys/syscall.h>
#include <cmath>
#include <fcntl.h>
#include <functional>
#include <map>
#include <sys/stat.h>
#include <thread>
#include <tuple>
#include <unistd.h>
#include <unordered_map>

namespace {

constexpr double kSlowIterMs = 25.0; // event-loop iteration logged as a hitch
constexpr int kLagTickMs = 5;        // scheduled tick of the loop-lag meter

QString envStr(const char *k, const QString &def = QString()) {
  const QByteArray v = qgetenv(k);
  return v.isEmpty() ? def : QString::fromLocal8Bit(v);
}

// ---- /proc readers ---------------------------------------------------------
struct ThreadCpu {
  QString comm;
  double ticks = 0;
};
std::map<int, ThreadCpu> readThreads() {
  std::map<int, ThreadCpu> m;
  const QString base = QStringLiteral("/proc/self/task/");
  const QStringList tids = QDir(base).entryList(QDir::Dirs | QDir::NoDotAndDotDot);
  for (const QString &t : tids) {
    QFile f(base + t + "/stat");
    if (!f.open(QIODevice::ReadOnly))
      continue;
    const QByteArray s = f.readAll();
    const int lp = s.indexOf('('), rp = s.lastIndexOf(')');
    if (lp < 0 || rp < 0)
      continue;
    const QList<QByteArray> rest = s.mid(rp + 2).split(' ');
    if (rest.size() < 13)
      continue;
    ThreadCpu c;
    c.comm = QString::fromLocal8Bit(s.mid(lp + 1, rp - lp - 1));
    c.ticks = rest[11].toDouble() + rest[12].toDouble(); // utime + stime
    m[t.toInt()] = c;
  }
  return m;
}
double rssMb() {
  QFile f("/proc/self/statm");
  if (!f.open(QIODevice::ReadOnly))
    return 0;
  const QList<QByteArray> p = f.readAll().split(' ');
  return p.size() > 1 ? p[1].toDouble() * double(sysconf(_SC_PAGESIZE)) / 1048576.0
                      : 0.0;
}

// ---- stats -----------------------------------------------------------------
QJsonObject stats(std::vector<float> v, double durS) {
  QJsonObject o;
  if (v.empty())
    return o;
  std::sort(v.begin(), v.end());
  const size_t n = v.size();
  auto pct = [&](double p) {
    size_t i = size_t(std::ceil(p * double(n))) ;
    return double(v[std::min(n - 1, i ? i - 1 : 0)]);
  };
  double sum = 0;
  int g33 = 0, g50 = 0;
  for (float x : v) {
    sum += x;
    g33 += x > 33.0f;
    g50 += x > 50.0f;
  }
  o["n"] = double(n);
  o["rate_hz"] = durS > 0 ? double(n) / durS : 0.0;
  o["mean"] = sum / double(n);
  o["p50"] = pct(0.50);
  o["p95"] = pct(0.95);
  o["p99"] = pct(0.99);
  o["max"] = double(v.back());
  o["gt33"] = g33;
  o["gt50"] = g50;
  o["sum"] = sum;
  return o;
}


// ---- in-process stack sampler (NAVIGATOR_PROFILE_SAMPLE=<hz>) -------------
// perf is unusable unprivileged here (perf_event_paranoid=4) and ptrace is
// scoped, so each thread gets a CPU-time POSIX timer that signals that thread;
// the handler stores a backtrace(). Samples are therefore proportional to the
// thread's CPU use. Raw frames are written (module + offset) to <out>.stacks
// and symbolized offline by tools/perf/stacks.py.
namespace sampler {
struct Rec {
  int tid;
  int n;
  long long tNs;
  void *pc[28];
};
Rec *g_buf = nullptr;
std::atomic<size_t> g_n{0};
constexpr size_t kCap = 400000;
std::map<int, timer_t> g_timers;
std::map<int, QString> g_comm;
long long g_baseNs = 0;
int g_hz = 0;

long long monoNs() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}
void handler(int, siginfo_t *, void *) {
  const size_t i = g_n.fetch_add(1, std::memory_order_relaxed);
  if (i >= kCap)
    return;
  Rec &r = g_buf[i];
  r.tid = int(syscall(SYS_gettid));
  r.tNs = monoNs() - g_baseNs;
  r.n = backtrace(r.pc, 28);
}
void start(int hz) {
  g_hz = hz;
  perfprobe::installAltStack(); // GUI thread
  g_buf = static_cast<Rec *>(calloc(kCap, sizeof(Rec)));
  g_baseNs = monoNs();
  void *warm[4];
  backtrace(warm, 4); // load libgcc now, not inside the handler
  struct sigaction sa{};
  sa.sa_sigaction = handler;
  sa.sa_flags = SA_SIGINFO | SA_RESTART | SA_ONSTACK;
  sigemptyset(&sa.sa_mask);
  sigaction(SIGPROF, &sa, nullptr);
}
void addThreads(const std::map<int, ThreadCpu> &now) {
  if (!g_hz)
    return;
  for (auto &kv : now) {
    if (g_timers.count(kv.first))
      continue;
    const int tid = kv.first;
    // Only the GUI thread, the GL/pool threads named like it and the engine's
    // SimWorker thread: the firmware's own fibre threads do not unwind safely.
    const bool simWorker = tid == perfprobe::g_simWorkerTid.load();
    const bool all = envStr("NAVIGATOR_PROFILE_SAMPLE_ALL") == "1";
    if (!all && !simWorker && kv.second.comm != QStringLiteral("Navigator"))
      continue;
    const clockid_t cid = (~clockid_t(tid) << 3) | 6; // per-thread, SCHED
    sigevent sev{};
    sev.sigev_notify = SIGEV_THREAD_ID;
    sev.sigev_signo = SIGPROF;
    sev._sigev_un._tid = tid;
    timer_t t;
    if (timer_create(cid, &sev, &t) != 0)
      continue;
    itimerspec its{};
    its.it_interval.tv_nsec = 1000000000L / g_hz;
    its.it_value = its.it_interval;
    timer_settime(t, 0, &its, nullptr);
    g_timers[tid] = t;
    g_comm[tid] = kv.second.comm;
  }
}
void stopAndDump(const QString &path) {
  if (!g_hz)
    return;
  for (auto &kv : g_timers)
    timer_delete(kv.second);
  g_timers.clear();
  usleep(20000);
  const size_t n = std::min(g_n.load(), kCap);
  QFile f(path);
  if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate))
    return;
  for (size_t i = 0; i < n; ++i) {
    const Rec &r = g_buf[i];
    QByteArray line = QByteArray("S ") + QByteArray::number(r.tid) + ' ' +
                      QByteArray::number(r.tNs / 1000000) + ' ' +
                      (g_comm.count(r.tid) ? g_comm[r.tid] : QString("?")).toLocal8Bit().replace(' ', '_');
    for (int k = 2; k < r.n; ++k) { // 0 = handler, 1 = signal trampoline
      Dl_info di;
      // Frame 2 is the interrupted pc; deeper frames are return addresses, so
      // look one byte back to stay inside the calling function.
      char *pc = (char *)r.pc[k] - (k > 2 ? 1 : 0);
      if (dladdr(pc, &di) && di.dli_fname) {
        line += ' ';
        line += QByteArray(di.dli_fname) + ':' +
                QByteArray::number((qulonglong)(pc - (char *)di.dli_fbase), 16) +
                ':' + (di.dli_sname ? di.dli_sname : "");
      } else {
        line += " ?:0:";
      }
    }
    f.write(line + "\n");
  }
}
} // namespace sampler
} // namespace

// ---------------------------------------------------------------------------
// Event-loop observer: per-iteration busy time (first event after the loop
// wakes -> aboutToBlock), and a
// per-(class, event type) time table. An event's time is the gap to the next
// filter call (or to aboutToBlock); nested events make this approximate, so
// treat it as a lead for the PERF_SCOPE series and the stack samples.
class LoopObserver : public QObject {
public:
  struct Key {
    const char *cls, *parent;
    int type;
    bool operator<(const Key &o) const {
      return std::tie(cls, parent, type) < std::tie(o.cls, o.parent, o.type);
    }
  };
  struct Acc {
    double ms = 0;
    long n = 0;
  };

  bool eventFilter(QObject *o, QEvent *e) override {
    if (!perfprobe::on())
      return false;
    const double t = perfprobe::nowMs();
    closeCur(t);
    if (!inIter_) { // first event after the loop woke: the busy period starts
      iterStart_ = t;
      iter_.clear();
      inIter_ = true;
    }
    cur_.cls = o->metaObject()->className();
    cur_.parent = (o->parent() && o->inherits("QTimer"))
                      ? o->parent()->metaObject()->className()
                      : "";
    cur_.type = int(e->type());
    curStart_ = t;
    have_ = true;
    return false;
  }
  void onBlock() {
    if (!inIter_ || !perfprobe::on())
      return;
    const double t = perfprobe::nowMs();
    closeCur(t);
    const double busy = t - iterStart_;
    inIter_ = false;
    perfprobe::record("evloop.iter_ms", busy);
    if (busy > kSlowIterMs && hitches_ < 400) {
      ++hitches_;
      std::sort(iter_.begin(), iter_.end(),
                [](auto &a, auto &b) { return a.second > b.second; });
      QJsonArray top;
      for (size_t i = 0; i < iter_.size() && i < 4; ++i) {
        QJsonObject x;
        x["ev"] = name(iter_[i].first);
        x["ms"] = iter_[i].second;
        top.append(x);
      }
      QJsonObject h;
      h["type"] = "hitch";
      h["t_ms"] = iterStart_;
      h["iter_ms"] = busy;
      h["top"] = top;
      pending.push_back(h);
    }
  }
  QString name(const Key &k) const {
    static const QMetaEnum me = QMetaEnum::fromType<QEvent::Type>();
    const char *tn = me.valueToKey(k.type);
    QString s = QString("%1:%2").arg(k.cls, tn ? tn : QString::number(k.type));
    if (k.parent[0])
      s += QString("(parent=%1)").arg(k.parent);
    return s;
  }
  void closeCur(double t) {
    if (!have_)
      return;
    const double d = t - curStart_;
    Acc &a = acc[cur_];
    a.ms += d;
    ++a.n;
    if (inIter_)
      iter_.push_back({cur_, d});
    have_ = false;
  }
  std::map<Key, Acc> acc;
  std::vector<QJsonObject> pending; // hitch lines awaiting the next flush

private:
  Key cur_{"", "", 0};
  double curStart_ = 0, iterStart_ = 0;
  bool have_ = false, inIter_ = false;
  int hitches_ = 0;
  std::vector<std::pair<Key, double>> iter_;
};

struct PerfHarness::Impl {
  MainWindow *w = nullptr;
  QString scenario, tag, outPath, preset;
  bool grass = false;
  int secs = 0; // 0 = scenario default
  QFile out;
  LoopObserver obs;
  QTimer lagTimer, secTimer;
  double lastLagTick = 0;
  // phase state
  QString phase;
  double phaseStart = 0;
  std::map<int, ThreadCpu> thr0;
  std::vector<QJsonObject> marks;
  QJsonObject gl;
  // flight
  int jsFd = -1;
  bool piloting = false;
  double pilotTarget = 3.0, pilotPitchUs = 1500;
  double stick = 0.5, iterm = 0, z0 = 0, tPrev = 0;
  bool haveZ0 = false;
  uint64_t lastTick = 0;
  double lastPoseMs = 0;
  std::vector<std::function<void()>> steps;

  void emitLine(QJsonObject o) {
    o["tag"] = tag;
    o["scenario"] = scenario;
    out.write(QJsonDocument(o).toJson(QJsonDocument::Compact) + "\n");
    out.flush();
  }
  void mark(const QString &what) {
    QJsonObject m;
    m["type"] = "mark";
    m["t_ms"] = perfprobe::nowMs();
    m["what"] = what;
    emitLine(m);
  }

  void beginPhase(const QString &name) {
    {
      std::lock_guard<std::mutex> lk(perfprobe::state().mtx);
      perfprobe::state().series.clear();
    }
    obs.acc.clear();
    thr0 = readThreads();
    phase = name;
    phaseStart = perfprobe::nowMs();
    secSamples.clear();
  }
  std::vector<std::pair<double, double>> secSamples; // (main%, proc%) per second
  std::map<int, ThreadCpu> secPrev;
  double secPrevT = 0;

  void endPhase() {
    const double now = perfprobe::nowMs();
    const double dur = (now - phaseStart) / 1000.0;
    const double hz = double(sysconf(_SC_CLK_TCK));
    QJsonObject o;
    o["type"] = "phase";
    o["phase"] = phase;
    o["dur_s"] = dur;
    QJsonObject ser;
    {
      std::lock_guard<std::mutex> lk(perfprobe::state().mtx);
      for (auto &kv : perfprobe::state().series) {
        std::vector<float> v;
        v.reserve(kv.second.size());
        for (auto &s : kv.second)
          v.push_back(s.v);
        ser[QString::fromStdString(kv.first)] = stats(std::move(v), dur);
      }
    }
    // loop lag is recorded as tick-interval minus the scheduled tick
    o["series"] = ser;
    // threads
    const auto thr1 = readThreads();
    QJsonArray ta;
    double procTicks = 0;
    const int mainTid = int(getpid());
    double mainPct = 0;
    for (auto &kv : thr1) {
      const double t0 = thr0.count(kv.first) ? thr0[kv.first].ticks : 0;
      const double pct = 100.0 * (kv.second.ticks - t0) / hz / dur;
      procTicks += kv.second.ticks - t0;
      if (kv.first == mainTid)
        mainPct = pct;
      if (pct >= 0.5) {
        QJsonObject t;
        t["tid"] = kv.first;
        t["comm"] = kv.second.comm;
        t["pct"] = pct;
        ta.append(t);
      }
    }
    o["threads"] = ta;
    o["main_cpu_pct"] = mainPct;
    o["proc_cpu_pct"] = 100.0 * procTicks / hz / dur;
    o["rss_mb"] = rssMb();
    {
      std::vector<float> a, b;
      for (auto &s : secSamples) {
        a.push_back(float(s.first));
        b.push_back(float(s.second));
      }
      o["main_cpu_pct_1s"] = stats(a, dur);
      o["proc_cpu_pct_1s"] = stats(b, dur);
    }
    // event-time table, top 14
    std::vector<std::pair<LoopObserver::Key, LoopObserver::Acc>> ev(
        obs.acc.begin(), obs.acc.end());
    std::sort(ev.begin(), ev.end(),
              [](auto &a, auto &b) { return a.second.ms > b.second.ms; });
    QJsonArray ea;
    for (size_t i = 0; i < ev.size() && i < 14; ++i) {
      QJsonObject e;
      e["ev"] = obs.name(ev[i].first);
      e["n"] = double(ev[i].second.n);
      e["ms"] = ev[i].second.ms;
      e["ms_per_s"] = ev[i].second.ms / dur;
      ea.append(e);
    }
    o["events_top"] = ea;
    emitLine(o);
    for (auto &h : obs.pending)
      emitLine(h);
    obs.pending.clear();
  }

  void sampleSecond() {
    const auto now = readThreads();
    sampler::addThreads(now);
    const double t = perfprobe::nowMs();
    if (secPrevT > 0 && !phase.isEmpty()) {
      const double hz = double(sysconf(_SC_CLK_TCK));
      const double dt = (t - secPrevT) / 1000.0;
      double procD = 0, mainD = 0;
      for (auto &kv : now) {
        const double p = secPrev.count(kv.first) ? secPrev[kv.first].ticks : 0;
        procD += kv.second.ticks - p;
        if (kv.first == int(getpid()))
          mainD = kv.second.ticks - p;
      }
      secSamples.push_back({100.0 * mainD / hz / dt, 100.0 * procD / hz / dt});
    }
    secPrev = now;
    secPrevT = t;
  }

  void queryGl() {
    QOffscreenSurface s;
    s.create();
    QOpenGLContext c;
    if (!c.create() || !s.isValid() || !c.makeCurrent(&s)) {
      gl["error"] = "no offscreen GL context";
      return;
    }
    auto *f = c.functions();
    gl["vendor"] = QString::fromLatin1((const char *)f->glGetString(GL_VENDOR));
    gl["renderer"] =
        QString::fromLatin1((const char *)f->glGetString(GL_RENDERER));
    gl["version"] =
        QString::fromLatin1((const char *)f->glGetString(GL_VERSION));
    c.doneCurrent();
  }

  // ---- step runner -------------------------------------------------------
  void after(int ms, std::function<void()> fn) {
    QTimer::singleShot(ms, w, std::move(fn));
  }
  int measureSecs(int def) const { return secs > 0 ? secs : def; }
  void finish() {
    mark("done");
    perfprobe::state().on.store(false);
    sampler::stopAndDump(outPath + ".stacks");
    out.close();
#ifdef NAVIGATOR_HAS_SITL
    if (w->m_simulatorWidget)
      w->m_simulatorWidget->stopInAppSim();
#endif
    if (jsFd >= 0)
      ::close(jsFd);
    // The engine's firmware threads never exit; do not let a clean quit hang.
    std::thread([] {
      std::this_thread::sleep_for(std::chrono::seconds(12));
      _exit(0);
    }).detach();
    QTimer::singleShot(300, w, [] { QCoreApplication::quit(); });
  }
  void fail(const QString &why) {
    QJsonObject o;
    o["type"] = "error";
    o["why"] = why;
    emitLine(o);
    finish();
  }
  // Measure a phase of `s` seconds, then run next.
  void measure(const QString &name, int s, std::function<void()> next) {
    beginPhase(name);
    after(s * 1000, [this, next = std::move(next)] {
      endPhase();
      next();
    });
  }

#ifdef NAVIGATOR_HAS_SITL
  SimulatorWidget *sim() const { return w->m_simulatorWidget; }

  void setPreset(const QString &id) {
    if (!sim() || !sim()->m_worldEditor)
      return;
    const auto combos = sim()->m_worldEditor->findChildren<QComboBox *>();
    // "endless" is a biome (streaming terrain), not a preset: it lives in the
    // Biome combo whose first entry is "None ...".
    const bool biome = id == QStringLiteral("endless");
    for (QComboBox *c : combos) {
      if (c->count() < 2)
        continue;
      if (biome ? !c->itemText(0).startsWith(QStringLiteral("None"))
                : c->itemText(0) != QStringLiteral("Custom"))
        continue;
      const int i = c->findData(id);
      if (i >= 0) {
        mark("preset:" + id);
        c->setCurrentIndex(i);
        return;
      }
    }
    mark("preset-not-found:" + id);
  }
  void tickGrass() {
    for (QCheckBox *c : sim()->findChildren<QCheckBox *>())
      if (c->text() == QStringLiteral("Grass")) {
        c->setChecked(true);
        mark("grass-on");
        return;
      }
    mark("grass-box-missing (build without NAVIGATOR_SIM_GRASS)");
  }
  void setupRcFifo() {
    const QString path =
        envStr("NAVIGATOR_PROFILE_OUT", "/tmp/navprof") + ".js.fifo";
    ::unlink(path.toLocal8Bit().constData());
    if (::mkfifo(path.toLocal8Bit().constData(), 0600) != 0)
      return;
    // O_RDWR keeps a writer open so the bridge's reads see EAGAIN, not EOF.
    jsFd = ::open(path.toLocal8Bit().constData(), O_RDWR | O_NONBLOCK);
    sim()->m_rc->setJoystickPath(path);
    for (int f = 0; f < 4; ++f)
      sim()->m_rc->setMapping(f, f, false);
    sim()->m_rc->setArmOverride(1);
    pilotAxes(1500, 1500, 1000, 1500);
    QObject::connect(sim()->m_rc, &RcBridge::channelsUpdated, w,
                     [](int r, int p, int t, int y, int a, int c) {
                       (void)r;
                       (void)y;
                       (void)c;
                       perfprobe::record("rc.pitch_us", p);
                       perfprobe::record("rc.thr_us", t);
                       perfprobe::record("rc.arm_us", a);
                     });
  }
  void pilotAxes(int roll, int pitch, int thr, int yaw) {
    if (jsFd < 0)
      return;
    const int us[4] = {roll, pitch, thr, yaw};
    for (int i = 0; i < 4; ++i) {
      struct {
        uint32_t time;
        int16_t value;
        uint8_t type, number;
      } e{0, int16_t(std::clamp((us[i] - 1500) * 32767 / 500, -32767, 32767)),
          0x02, uint8_t(i)};
      const ssize_t r = ::write(jsFd, &e, sizeof e);
      (void)r;
    }
  }
  void onPose(const vsim::SimSnapshot &s) {
    const double t = perfprobe::nowMs();
    if (lastPoseMs > 0)
      perfprobe::record("pose.interval_ms", t - lastPoseMs);
    lastPoseMs = t;
    if (lastTick && s.tick_count >= lastTick)
      perfprobe::record("pose.engine_ticks_per_pose",
                        double(s.tick_count - lastTick));
    lastTick = s.tick_count;
    if (!haveZ0) {
      z0 = s.pos_w.z();
      haveZ0 = true;
    }
    const double alt = z0 - s.pos_w.z();
    perfprobe::record("flight.alt_m", alt);
    perfprobe::record("flight.speed_ms", std::hypot(s.vel_w.x(), s.vel_w.y()));
    if (!piloting)
      return;
    const double dt = std::clamp((t - tPrev) / 1000.0, 0.0, 0.1);
    tPrev = t;
    const double vz = -s.vel_w.z();
    const double vzCmd = std::clamp(0.8 * (pilotTarget - alt), -1.0, 1.2);
    const double verr = vzCmd - vz;
    iterm = std::clamp(iterm + 0.35 * verr * dt, -0.3, 0.45);
    stick = std::clamp(0.5 + 0.12 * verr + iterm, 0.05, 0.95);
    pilotAxes(1500, int(pilotPitchUs), int(1000 + 1000 * stick), 1500);
  }
#endif

  // ---- scenarios ---------------------------------------------------------
  void run() {
    if (scenario == "home") {
      after(3000, [this] {
        measure("idle", measureSecs(20), [this] { finish(); });
      });
      return;
    }
#ifdef NAVIGATOR_HAS_SITL
    if (!sim()) {
      fail("built without the SITL simulator");
      return;
    }
    if (scenario == "flight")
      setupRcFifo();
    if (scenario != "homesim")
      w->showSimulator();
    if (!preset.isEmpty() && scenario != "worldchange")
      setPreset(preset);
    if (grass)
      tickGrass();
    QTimer::singleShot(800, w, [this] { startSim(); });
#else
    fail("built without the SITL simulator");
#endif
  }

#ifdef NAVIGATOR_HAS_SITL
  void startSim() {
    mark("click-start");
    sim()->m_simStartBtn->click();
    if (!sim()->m_sim) {
      fail("Start did not create the SimWorker (gated?)");
      return;
    }
    auto *timeout = new QTimer(w);
    timeout->setSingleShot(true);
    connect_online(timeout);
    timeout->start(40000);
    connect(timeout, &QTimer::timeout, w, [this] { fail("engine never online"); });
  }
  void connect_online(QTimer *timeout) {
    QObject::connect(
        sim()->m_sim, &vsim::SimWorker::online, w,
        [this, timeout] {
          timeout->stop();
          mark("online");
          after(600, [this] { afterOnline(); });
        },
        Qt::SingleShotConnection);
    QObject::connect(sim()->m_sim, &vsim::SimWorker::poseUpdated, w,
                     [this](vsim::SimSnapshot s) { onPose(s); });
  }
  void afterOnline() {
    const int settle = envStr("NAVIGATOR_PROFILE_SETTLE", "8").toInt();
    after(settle * 1000, [this] { scenarioBody(); });
  }
  void scenarioBody() {
    if (scenario == "sim" || scenario == "world" || scenario == "homesim") {
      measure("run", measureSecs(20), [this] { finish(); });
    } else if (scenario == "long") {
      const int per = measureSecs(10);
      chain({"w0", "w1", "w2", "w3", "w4", "w5"}, 0, per);
    } else if (scenario == "worldchange") {
      measure("steady", measureSecs(8), [this] {
        changeSeq(envStr("NAVIGATOR_PROFILE_SEQ", "alpine,hills,desert,meadow,alpine")
                      .split(','),
                  0);
      });
    } else if (scenario == "flight") {
      flightBody();
    } else {
      fail("unknown scenario " + scenario);
    }
  }
  void chain(QStringList names, int i, int per) {
    if (i >= names.size()) {
      finish();
      return;
    }
    measure(names[i], per, [=, this] { chain(names, i + 1, per); });
  }
  void changeSeq(const QStringList &ids, int i) {
    if (i >= ids.size()) {
      finish();
      return;
    }
    beginPhase("change_" + ids[i] + "_" + QString::number(i));
    setPreset(ids[i]); // synchronous: the stall, if any, lands in this phase
    after(measureSecs(8) * 1000, [=, this] {
      endPhase();
      changeSeq(ids, i + 1);
    });
  }
  void flightBody() {
    tPrev = perfprobe::nowMs();
    haveZ0 = false; // altitude is relative to the spawn after the settle
    piloting = true;
    pilotTarget = 3.0;
    measure("climb", 8, [this] {
      measure("hover", measureSecs(15), [this] {
        pilotTarget = 8.0;
        pilotPitchUs = envStr("NAVIGATOR_PROFILE_PITCH_US", "1200").toInt(); // forward stick; the speed series shows what it gave
        measure("fast", measureSecs(15), [this] {
          piloting = false;
          pilotAxes(1500, 1500, 1000, 1500);
          finish();
        });
      });
    });
  }
#endif
};

PerfHarness::PerfHarness(MainWindow *w) : QObject(w), d(new Impl) {
  d->w = w;
  d->scenario = envStr("NAVIGATOR_PROFILE");
  d->tag = envStr("NAVIGATOR_PROFILE_TAG");
  d->preset = envStr("NAVIGATOR_PROFILE_PRESET");
  d->grass = envStr("NAVIGATOR_PROFILE_GRASS") == "1";
  d->secs = envStr("NAVIGATOR_PROFILE_SECS", "0").toInt();
  d->outPath = envStr("NAVIGATOR_PROFILE_OUT", "/dev/stderr");
  d->out.setFileName(d->outPath);
  if (!d->out.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
    qWarning("PerfHarness: cannot open %s", qPrintable(d->outPath));
    d->out.open(stderr, QIODevice::WriteOnly);
  }

  const QStringList sz = envStr("NAVIGATOR_PROFILE_SIZE", "1600x900").split('x');
  if (sz.size() == 2)
    w->resize(sz[0].toInt(), sz[1].toInt());

  d->queryGl();
  if (const int hz = envStr("NAVIGATOR_PROFILE_SAMPLE", "0").toInt())
    sampler::start(hz);
  perfprobe::state().clock.start();
  perfprobe::state().on.store(true);

  QJsonObject m;
  m["type"] = "meta";
  m["gl"] = d->gl;
  m["pid"] = int(getpid());
  m["preset"] = d->preset;
  m["seq"] = envStr("NAVIGATOR_PROFILE_SEQ");
  m["grass_cpu"] = !qEnvironmentVariableIsEmpty("NAVIGATOR_GRASS_CPU");
  m["grass"] = d->grass;
  m["platform"] = QGuiApplication::platformName();
  if (QScreen *s = QGuiApplication::primaryScreen()) {
    m["screen_hz"] = s->refreshRate();
    m["screen"] = QString("%1x%2").arg(s->size().width()).arg(s->size().height());
  }
  m["window"] = QString("%1x%2").arg(w->width()).arg(w->height());
#ifdef NAVIGATOR_HAS_SITL
  m["sitl"] = true;
#else
  m["sitl"] = false;
#endif
  m["build"] =
#ifdef QT_DEBUG
      "debug";
#else
      "release";
#endif
  d->emitLine(m);

  qApp->installEventFilter(&d->obs);
  if (auto *disp = QAbstractEventDispatcher::instance()) {
    QObject::connect(disp, &QAbstractEventDispatcher::aboutToBlock, this,
                     [this] { d->obs.onBlock(); });
  }
  // Loop-lag meter: how late a 5 ms precise timer actually fires.
  d->lagTimer.setTimerType(Qt::PreciseTimer);
  d->lagTimer.setInterval(kLagTickMs);
  QObject::connect(&d->lagTimer, &QTimer::timeout, this, [this] {
    const double t = perfprobe::nowMs();
    if (d->lastLagTick > 0)
      perfprobe::record("evloop.lag_ms", (t - d->lastLagTick) - kLagTickMs);
    d->lastLagTick = t;
  });
  d->lagTimer.start();
  d->secTimer.setInterval(1000);
  QObject::connect(&d->secTimer, &QTimer::timeout, this,
                   [this] { d->sampleSecond(); });
  d->secTimer.start();

  QTimer::singleShot(500, this, [this] { d->run(); });
}

bool PerfHarness::maybeStart(MainWindow *w) {
  if (qEnvironmentVariableIsEmpty("NAVIGATOR_PROFILE"))
    return false;
  new PerfHarness(w);
  return true;
}
