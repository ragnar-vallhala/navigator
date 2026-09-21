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

#include <QList>
#include <QObject>

// Most-recently-used stack of view ids for the recent-views switcher
// (FR-UX-21). visit() is called on every page change; the front is the current
// view, the next entry is the toggle-to-previous target, and the first `depth`
// entries feed the hold-to-cycle overlay. Pure logic — unit-tested headless.
class ViewHistory : public QObject {
  Q_OBJECT

public:
  explicit ViewHistory(QObject *parent = nullptr);

  // Configurable cycle depth (mockup: Settings ▸ Units & Display). Clamped to
  // [kMinDepth, kMaxDepth]; shrinking truncates the kept history.
  static constexpr int kMinDepth = 2;
  static constexpr int kMaxDepth = 9;
  void setDepth(int n);
  int depth() const { return m_depth; }

  // Record a navigation to `viewId`. Re-visiting the current view is a no-op;
  // otherwise the id moves/inserts at the front and the list is capped to
  // depth.
  void visit(int viewId);

  QList<int> mru() const { return m_mru; } // front = most recent
  int count() const { return int(m_mru.size()); }

  // The view a quick toggle should jump to (second entry), or -1 if there is
  // no distinct previous view yet.
  int previous() const;

private:
  QList<int> m_mru; // front = most recent
  int m_depth = 5;
};
