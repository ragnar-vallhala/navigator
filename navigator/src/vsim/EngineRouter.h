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

// Routes a host -> firmware action to the thread that steps the engine.
//
// The SITL engine is single-threaded: SimWorker's loop runs run_step() and
// applies queued config on one thread. A host call that touches engine state
// (a frame into the firmware's UART) from the GUI thread races that stepper.
// While the engine loop runs, SimWorker installs a dispatcher (its enqueue) so
// the action runs between steps on the engine thread; with no loop running
// nothing steps the engine, so the action runs at once on the caller.
//
// run() holds the router's lock while it dispatches, and set({}) takes the same
// lock: once set({}) has returned, no dispatch is in flight, so the loop's last
// drain sees every action that will ever be queued for this run.
#include <functional>
#include <mutex>
#include <utility>

namespace vsim {

class EngineRouter {
public:
  using Op = std::function<void()>;
  using Dispatcher = std::function<void(Op)>;

  void set(Dispatcher d) {
    std::lock_guard<std::mutex> lk(m_);
    d_ = std::move(d);
  }

  // true: handed to the dispatcher (runs later on the engine thread);
  // false: no dispatcher, ran now on the calling thread.
  bool run(Op op) {
    std::lock_guard<std::mutex> lk(m_);
    if (d_) {
      d_(std::move(op));
      return true;
    }
    op();
    return false;
  }

private:
  std::mutex m_;
  Dispatcher d_;
};

} // namespace vsim
