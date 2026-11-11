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
#pragma once

// Measurement hooks for the profiling harness (see app/PerfHarness.h).
//
// Everything here is header-only and inert unless the harness switches it on
// (env NAVIGATOR_PROFILE): every hook is one relaxed atomic load when off.
// Samples are named series of (time, value) pairs; the harness windows them
// into phases and reports mean / percentiles / worst case per series.

#include <QElapsedTimer>
#include <QOpenGLContext>

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace perfprobe {

struct Sample {
  double tMs; // since the harness clock started
  float v;
};

struct State {
  std::atomic<bool> on{false};
  std::mutex mtx;
  QElapsedTimer clock; // started by the harness
  std::map<std::string, std::vector<Sample>> series;
  std::map<std::string, double> lastPaintStart; // per paint name
  std::map<std::string, double> lastPaintEnd;
};

inline State &state() {
  static State s;
  return s;
}
// tid of the SimWorker thread that steps the engine (set by SimWorker::run so
// the stack sampler can pick it out from the firmware's own threads).
inline std::atomic<int> g_simWorkerTid{0};
// Give the calling thread an alternate signal stack, so the stack sampler's
// SIGPROF handler never runs on a tiny fibre stack (the firmware's RTOS tasks
// run on small makecontext stacks on the SimWorker thread).
inline void installAltStack() {
  const size_t n = 256 * 1024;
  stack_t ss;
  ss.ss_sp = std::malloc(n);
  ss.ss_size = n;
  ss.ss_flags = 0;
  if (ss.ss_sp)
    sigaltstack(&ss, nullptr);
}
inline bool on() { return state().on.load(std::memory_order_relaxed); }
inline double nowMs() {
  State &s = state();
  return s.clock.isValid() ? double(s.clock.nsecsElapsed()) / 1e6 : 0.0;
}

// Record one sample of series `name` (any thread).
inline void record(const std::string &name, double v) {
  State &s = state();
  const double t = nowMs();
  std::lock_guard<std::mutex> lk(s.mtx);
  s.series[name].push_back({t, float(v)});
}

// Paint-level frame timing for one widget: call paintBegin at the top of its
// paint and paintEnd at the bottom (GUI thread). Records
//   <name>.interval_ms     start-to-start gap between consecutive paints
//   <name>.paint_cpu_ms    time spent inside the paint call
inline void paintBegin(const std::string &name) {
  State &s = state();
  const double t = nowMs();
  std::lock_guard<std::mutex> lk(s.mtx);
  auto it = s.lastPaintStart.find(name);
  if (it != s.lastPaintStart.end())
    s.series[name + ".interval_ms"].push_back({t, float(t - it->second)});
  s.lastPaintStart[name] = t;
}
inline void paintEnd(const std::string &name) {
  State &s = state();
  const double t = nowMs();
  std::lock_guard<std::mutex> lk(s.mtx);
  s.lastPaintEnd[name] = t;
  s.series[name + ".paint_cpu_ms"].push_back(
      {t, float(t - s.lastPaintStart[name])});
}
// Called from frameSwapped: how long after the paint returned the swap
// completed (includes any blocking on vsync / the GPU queue).
inline void swapped(const std::string &name) {
  State &s = state();
  const double t = nowMs();
  std::lock_guard<std::mutex> lk(s.mtx);
  auto it = s.lastPaintEnd.find(name);
  if (it != s.lastPaintEnd.end())
    s.series[name + ".paint_to_swap_ms"].push_back({t, float(t - it->second)});
}

// RAII CPU timer for a named section: records elapsed wall ms on scope exit.
class Scope {
public:
  explicit Scope(const char *name) : name_(name), active_(on()) {
    if (active_)
      t_.start();
  }
  ~Scope() {
    if (active_)
      record(name_, double(t_.nsecsElapsed()) / 1e6);
  }
  Scope(const Scope &) = delete;
  Scope &operator=(const Scope &) = delete;

private:
  const char *name_;
  bool active_;
  QElapsedTimer t_;
};

// GL_TIME_ELAPSED timer: begin()/end() bracket GPU work on the current
// context; poll() later (a few frames on) reads the finished results without
// stalling. Silently does nothing when the driver lacks timer queries.
class GlTimer {
public:
  void begin() {
    if (!init())
      return;
    if (open_ || pending_ >= kRing)
      return; // ring full: drop this frame's sample rather than stall
    const int slot = (head_ + pending_) % kRing;
    glBeginQuery_(kTimeElapsed, q_[slot]);
    open_ = true;
  }
  void end() {
    if (!ok_ || !open_)
      return;
    glEndQuery_(kTimeElapsed);
    open_ = false;
    ++pending_;
  }
  // Collect finished queries (oldest first) into series `name`.
  void poll(const char *name) {
    if (!ok_)
      return;
    while (pending_ > 0) {
      unsigned avail = 0;
      getQi_(q_[head_], kResultAvailable, &avail);
      if (!avail)
        break;
      unsigned long long ns = 0;
      getQ64_(q_[head_], kResult, &ns);
      record(name, double(ns) / 1e6);
      head_ = (head_ + 1) % kRing;
      --pending_;
    }
  }
  bool supported() const { return ok_; }

private:
  static constexpr int kRing = 8;
  static constexpr unsigned kTimeElapsed = 0x88BF;
  static constexpr unsigned kResult = 0x8866;
  static constexpr unsigned kResultAvailable = 0x8867;
  using GenQ = void (*)(int, unsigned *);
  using BeginQ = void (*)(unsigned, unsigned);
  using EndQ = void (*)(unsigned);
  using GetQi = void (*)(unsigned, unsigned, unsigned *);
  using GetQ64 = void (*)(unsigned, unsigned, unsigned long long *);

  bool init() {
    if (tried_)
      return ok_;
    tried_ = true;
    QOpenGLContext *c = QOpenGLContext::currentContext();
    if (!c)
      return false;
    auto g = [&](const char *n) { return c->getProcAddress(n); };
    genQ_ = reinterpret_cast<GenQ>(g("glGenQueries"));
    glBeginQuery_ = reinterpret_cast<BeginQ>(g("glBeginQuery"));
    glEndQuery_ = reinterpret_cast<EndQ>(g("glEndQuery"));
    getQi_ = reinterpret_cast<GetQi>(g("glGetQueryObjectuiv"));
    getQ64_ = reinterpret_cast<GetQ64>(g("glGetQueryObjectui64v"));
    if (!genQ_ || !glBeginQuery_ || !glEndQuery_ || !getQi_ || !getQ64_)
      return false;
    genQ_(kRing, q_);
    ok_ = true;
    return true;
  }

  bool tried_ = false, ok_ = false, open_ = false;
  int head_ = 0, pending_ = 0;
  unsigned q_[kRing] = {};
  GenQ genQ_ = nullptr;
  BeginQ glBeginQuery_ = nullptr;
  EndQ glEndQuery_ = nullptr;
  GetQi getQi_ = nullptr;
  GetQ64 getQ64_ = nullptr;
};

} // namespace perfprobe

#define PERF_CAT2(a, b) a##b
#define PERF_CAT(a, b) PERF_CAT2(a, b)
#define PERF_SCOPE(name) perfprobe::Scope PERF_CAT(perfScope_, __LINE__)(name)
