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
// Stop a worker thread without ever killing it.
#pragma once

#include <QObject>
#include <QThread>

namespace vsim {

// Wait up to \p ms for \p t to finish. Returns true when it did; the caller
// then owns an ordinary finished thread (deleteLater() is fine).
//
// When it did not, the thread is stuck, in practice inside the firmware engine
// (a semaphore wait that run_step never leaves). QThread::terminate() on it
// kills it mid-engine call and takes the process down, and destroying a
// QThread that is still running aborts, so neither is allowed: the object is
// taken out of its parent (so no parent's destructor deletes it) and left
// alone for the rest of the process. That leaks one thread object; the caller
// must also drop its own connections to it and stop using it. Returns false.
inline bool joinOrAbandon(QThread *t, unsigned long ms) {
  if (!t || t->wait(ms))
    return true;
  t->setParent(nullptr);
  return false;
}

} // namespace vsim
