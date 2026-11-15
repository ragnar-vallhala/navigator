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
// Fit a main-window size to the screen it opens on.
#pragma once

#include <QRect>
#include <QSize>

namespace windowfit {

// Height kept for the title bar and window borders, which the size a widget
// reports does not include and which are not known before the window is shown.
constexpr int kFrameAllowance = 40;

// The size to open at. A size that fits the available screen area is returned
// unchanged. One that does not is cut down to it (less the frame allowance),
// but never below the window's own minimum: a screen smaller than the minimum
// size leaves the window at that minimum. A saved size from a bigger screen, or
// the default size on a small one, is the case this is for.
inline QSize fitToScreen(const QSize &wanted, const QSize &available,
                         const QSize &minimum) {
  const QSize room(available.width(), available.height() - kFrameAllowance);
  if (wanted.width() <= room.width() && wanted.height() <= room.height())
    return wanted;
  return wanted.boundedTo(room).expandedTo(minimum);
}

// Where to open: fitToScreen's size, with the frame's top-left corner moved
// inside the available area. A resize keeps the top-left corner, so a window at
// x=200 on a 1280 px screen would still be cut off on the right after being
// narrowed to 1280, and one above a top panel would stay under it. \p wanted is
// the frame's top-left corner and the client size. When the window (plus the
// frame allowance) is bigger than the area, which only the minimum size can
// cause, it is pinned to the area's top-left.
inline QRect fitRect(const QRect &wanted, const QRect &available,
                     const QSize &minimum) {
  const QSize size = fitToScreen(wanted.size(), available.size(), minimum);
  const int maxX = available.right() - size.width() + 1;
  const int maxY = available.bottom() - size.height() - kFrameAllowance + 1;
  const int x = qMax(available.left(), qMin(wanted.x(), maxX));
  const int y = qMax(available.top(), qMin(wanted.y(), maxY));
  return QRect(QPoint(x, y), size);
}

} // namespace windowfit
