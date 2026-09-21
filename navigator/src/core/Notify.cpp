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
#include "Notify.h"

#include "Theme.h"
#include "ToastOverlay.h"

#include <QMainWindow>
#include <QWidget>

namespace Notify {

namespace {
// Global toast switch (Settings ▸ Alerts). Toasts are status-bar feedback, so
// suppressing them is purely cosmetic — the log panel still records events.
bool g_enabled = true;

// Walk up the parent chain to the QMainWindow that hosts the toast overlay.
// Returns nullptr if we never hit one — caller handles silently in that case.
QWidget *findHost(QWidget *anchor) {
  for (QWidget *w = anchor; w; w = w->parentWidget()) {
    if (auto *mw = qobject_cast<QMainWindow *>(w))
      return mw;
  }
  return nullptr;
}

int defaultTimeout(Kind k) {
  switch (k) {
  case Kind::Info:
  case Kind::Ok:
    return 3000;
  case Kind::Warn:
    return 5000;
  case Kind::Error:
    return 7000;
  }
  return 3000;
}

QColor color(Kind k) {
  switch (k) {
  case Kind::Info:
    return Theme::kAccent;
  case Kind::Ok:
    return Theme::kOk;
  case Kind::Warn:
    return Theme::kWarn;
  case Kind::Error:
    return Theme::kDanger;
  }
  return Theme::kAccent;
}

const char *prefix(Kind k) {
  switch (k) {
  case Kind::Info:
    return "ℹ ";
  case Kind::Ok:
    return "✓ ";
  case Kind::Warn:
    return "⚠ ";
  case Kind::Error:
    return "✕ ";
  }
  return "";
}

} // namespace

void setEnabled(bool on) { g_enabled = on; }

void send(QWidget *anchor, Kind kind, const QString &text, int timeout_ms) {
  if (!g_enabled)
    return;
  QWidget *host = findHost(anchor);
  if (!host)
    return;
  const int t = timeout_ms > 0 ? timeout_ms : defaultTimeout(kind);
  ToastOverlay::forHost(host)->addToast(color(kind), QString(prefix(kind)),
                                        text, t);
}

} // namespace Notify
