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

#include <atomic>
#include <cstdint>

// THE per-(sysid, compid) frame sequence counter of the GCS (NavLink spec 3.5).
// Every frame the GCS transmits takes its `seq` from here: the command and
// request encoders (CommandCodec) and the COMMAND_ACK frames the NavLink parser
// emits through handlers.next_seq (NavlinkRouter). One counter means the
// receiver's loss estimate sees no gaps.
//
// Thread safety: next_seq runs on whichever thread calls the parser push, while
// commands are encoded on the GUI thread, the worker thread and the SITL tune
// thread. The counter is atomic, so no two frames get the same value and none
// is skipped. A uint8_t fetch_add wraps 255 -> 0.
namespace TxSeq {
inline std::atomic<uint8_t> &counter() {
  static std::atomic<uint8_t> c{0};
  return c;
}
// The next frame seq (post-increment, wraps at 256).
inline uint8_t next() { return counter().fetch_add(1, std::memory_order_relaxed); }
// Tests only: the value the next call to next() returns.
inline void setNext(uint8_t v) { counter().store(v, std::memory_order_relaxed); }
} // namespace TxSeq
