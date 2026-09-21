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
#include "ViewHistory.h"

#include <algorithm>

ViewHistory::ViewHistory(QObject *parent) : QObject(parent) {}

void ViewHistory::setDepth(int n) {
  m_depth = std::clamp(n, kMinDepth, kMaxDepth);
  while (m_mru.size() > m_depth)
    m_mru.removeLast();
}

void ViewHistory::visit(int viewId) {
  if (!m_mru.isEmpty() && m_mru.front() == viewId)
    return; // re-visiting the current view collapses
  m_mru.removeAll(viewId);
  m_mru.prepend(viewId);
  while (m_mru.size() > m_depth)
    m_mru.removeLast();
}

int ViewHistory::previous() const {
  return m_mru.size() >= 2 ? m_mru.at(1) : -1;
}
