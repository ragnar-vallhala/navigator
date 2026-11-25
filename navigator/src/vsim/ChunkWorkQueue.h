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

#include "ChunkStreamer.h" // ChunkReq

#include <algorithm>
#include <cstdlib>
#include <mutex>
#include <vector>

namespace vsim {

// The chunks that still need meshing, handed to worker threads that PULL the
// nearest one themselves. The GUI thread only publishes what is needed and
// where the view is; it is not in the loop between one chunk and the next, so
// a GUI frame that takes seconds (software GL on a starved machine) no longer
// leaves the mesh workers idle with work outstanding.
//
// Thread safety: every member is safe from any thread (one mutex). Nothing
// here touches Qt objects, so a worker never needs the GUI thread's state.
class ChunkWorkQueue {
public:
  // Drop everything pending and start generation `gen`. Workers still holding
  // an older generation get nothing from pop() and end.
  void reset(int gen) {
    std::lock_guard<std::mutex> g(m_);
    pending_.clear();
    dropped_.clear();
    gen_ = gen;
  }

  // The view moved: pop() serves the chunk nearest (cx, cy), and a pending
  // chunk further than `radius` cells (Chebyshev) is dropped, not built.
  void setWindow(int cx, int cy, int radius) {
    std::lock_guard<std::mutex> g(m_);
    cx_ = cx;
    cy_ = cy;
    radius_ = radius;
    haveWindow_ = true;
    dropOutsideLocked();
  }

  // Queue chunks for generation `gen` (ignored if it is not the current one).
  void push(const std::vector<ChunkReq> &reqs, int gen) {
    std::lock_guard<std::mutex> g(m_);
    if (gen != gen_)
      return;
    for (const ChunkReq &r : reqs)
      pending_.push_back(r);
  }

  // Worker side: take the nearest pending chunk. False when nothing is pending
  // or `gen` is stale.
  bool pop(int gen, ChunkReq &out) {
    std::lock_guard<std::mutex> g(m_);
    if (gen != gen_)
      return false;
    dropOutsideLocked();
    if (pending_.empty())
      return false;
    auto best = pending_.begin();
    long bestD = distLocked(*best);
    for (auto it = pending_.begin() + 1; it != pending_.end(); ++it) {
      const long d = distLocked(*it);
      if (d < bestD) {
        best = it;
        bestD = d;
      }
    }
    out = *best;
    pending_.erase(best);
    return true;
  }

  // Keys dropped (never built) since the last call, so the owner can forget
  // them and ask for them again if they come back into range.
  std::vector<qint64> takeDropped() {
    std::lock_guard<std::mutex> g(m_);
    std::vector<qint64> out;
    out.swap(dropped_);
    return out;
  }

  std::size_t pending() const {
    std::lock_guard<std::mutex> g(m_);
    return pending_.size();
  }

  // Runner accounting: at most `max` tasks pull at once, so the global pool
  // keeps room for other jobs. A task that finds the queue empty releases.
  bool acquireRunner(int max) {
    std::lock_guard<std::mutex> g(m_);
    if (runners_ >= max)
      return false;
    ++runners_;
    return true;
  }
  void releaseRunner() {
    std::lock_guard<std::mutex> g(m_);
    if (runners_ > 0)
      --runners_;
  }
  int runners() const {
    std::lock_guard<std::mutex> g(m_);
    return runners_;
  }

private:
  // Chebyshev distance first, Euclidean as the tie-break (squared, scaled).
  long distLocked(const ChunkReq &r) const {
    if (!haveWindow_)
      return 0; // no view yet: first come, first served
    const long dx = std::labs(static_cast<long>(r.cx) - cx_);
    const long dy = std::labs(static_cast<long>(r.cy) - cy_);
    return std::max(dx, dy) * 100000L + (dx * dx + dy * dy);
  }
  void dropOutsideLocked() {
    if (!haveWindow_)
      return;
    auto out = std::remove_if(
        pending_.begin(), pending_.end(), [&](const ChunkReq &r) {
          const long dx = std::labs(static_cast<long>(r.cx) - cx_);
          const long dy = std::labs(static_cast<long>(r.cy) - cy_);
          if (std::max(dx, dy) <= radius_)
            return false;
          dropped_.push_back(r.key);
          return true;
        });
    pending_.erase(out, pending_.end());
  }

  mutable std::mutex m_;
  std::vector<ChunkReq> pending_;
  std::vector<qint64> dropped_;
  int gen_ = 0;
  int cx_ = 0, cy_ = 0, radius_ = 0;
  bool haveWindow_ = false;
  int runners_ = 0;
};

} // namespace vsim
