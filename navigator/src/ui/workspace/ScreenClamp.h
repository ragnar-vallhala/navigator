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
#include <QRect>
#include <algorithm>

namespace ws {

// Is `r` reachable? Its title strip (top 30 px) must lie fully inside some
// screen for at least min(80 px, width): otherwise the user cannot grab it.
inline bool rectReachable(const QRect &r, const QList<QRect> &screens) {
  if (screens.isEmpty())
    return true; // nothing to compare against: leave it alone
  const QRect strip(r.left(), r.top(), r.width(), 30);
  const int needW = std::min(80, std::max(1, r.width()));
  for (const QRect &s : screens) {
    const QRect i = strip.intersected(s);
    if (i.height() >= 30 && i.width() >= needW)
      return true;
  }
  return false;
}

// Pull an unreachable rect (e.g. saved on a monitor that is gone) onto the
// screen it overlaps most, or the nearest one. Reachable rects are unchanged.
inline QRect clampRectToScreens(const QRect &r, const QList<QRect> &screens) {
  if (screens.isEmpty() || rectReachable(r, screens))
    return r;
  const QRect *best = &screens.first();
  qint64 bestScore = -1;
  for (const QRect &s : screens) {
    const QRect i = r.intersected(s);
    qint64 score = i.isValid() ? qint64(i.width()) * i.height() : 0;
    if (score == 0) { // no overlap: prefer the nearest centre
      const QPoint d = r.center() - s.center();
      score = -(qint64(d.x()) * d.x() + qint64(d.y()) * d.y()) - 1;
      score = std::min<qint64>(score, -1);
    }
    if (bestScore == -1 || score > bestScore) {
      bestScore = score;
      best = &s;
    }
  }
  const QRect s = *best;
  const int w = std::min(r.width(), s.width());
  const int h = std::min(r.height(), s.height());
  const int x = std::clamp(r.left(), s.left(), s.right() - w + 1);
  const int y = std::clamp(r.top(), s.top(), s.bottom() - h + 1);
  return QRect(x, y, w, h);
}

} // namespace ws
