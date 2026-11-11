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

#include <QPushButton>
#include <QString>

// Latch that keeps a Start button disabled for the rest of the process, with a
// reason shown as its tooltip. Used after "Reset Firmware Settings": the engine
// caches the old settings in RAM and is never unloaded, so only a restart picks
// the reset up. Header-only so it can be tested without the simulator widget.
class StartGate {
public:
  void block(const QString &reason) { reason_ = reason; }
  bool blocked() const { return !reason_.isEmpty(); }
  const QString &reason() const { return reason_; }

  // Set the button for the state the caller wants: enabled only if wanted and
  // not blocked; the tooltip is the reason while blocked, else normalTip.
  void apply(QPushButton *btn, bool wantEnabled,
             const QString &normalTip) const {
    if (!btn)
      return;
    btn->setEnabled(wantEnabled && !blocked());
    btn->setToolTip(blocked() ? reason_ : normalTip);
  }

private:
  QString reason_;
};
