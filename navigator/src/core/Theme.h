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

#include <QColor>
#include <QString>

// Single source of truth for the Navigator dark theme. The bulk of
// styling lives in resources/styles/dark.qss; this header exposes the
// same colour tokens to C++ for the cases where QSS isn't enough
// (QPalette, QPainter, dynamic per-event recolouring of widgets like
// the LIVE heartbeat blinker or the status pill).
//
// Keep the hex codes here in sync with the variables at the top of
// dark.qss. If you find yourself defining a new colour, add it here
// AND there. Never inline a raw hex literal in a widget .cpp.
namespace Theme {

// ---- Surfaces ---------------------------------------------------------------
inline const QColor kBg = QColor("#1A1D27");         // window background
inline const QColor kSurface = QColor("#21252B");    // panels / inputs
inline const QColor kSurfaceAlt = QColor("#2C313A"); // raised cards / hovers
inline const QColor kBase = QColor("#13141B");       // deepest pit (log bg)
inline const QColor kBorder = QColor("#2A3347");     // hairlines
inline const QColor kBorderStrong = QColor("#3E4452");

// ---- Text -------------------------------------------------------------------
inline const QColor kText = QColor("#DCDFE4");      // primary text
inline const QColor kTextMuted = QColor("#ABB2BF"); // secondary text
inline const QColor kTextDim = QColor("#5C6370");   // tertiary / disabled

// ---- Semantic accents -------------------------------------------------------
inline const QColor kAccent = QColor("#61AFEF"); // info / focus / brand
inline const QColor kOk = QColor("#98C379");     // success / connected
inline const QColor kWarn = QColor("#D19A66");   // caution
inline const QColor kDanger = QColor("#E06C75"); // error / armed / disconnect

// ---- Attitude axes (chart-friendly distinct hues) ---------------------------
inline const QColor kAxisRoll = QColor("#FF6B6B");
inline const QColor kAxisPitch = QColor("#4ECDC4");
inline const QColor kAxisYaw = QColor("#FFE66D");

// Hex helpers — handy when building QSS fragments inline.
inline QString hex(const QColor &c) { return c.name(QColor::HexRgb).toUpper(); }

// Load the bundled dark.qss into a string. Returns empty on failure;
// caller should fall back to whatever defaults Qt provides.
QString loadStyleSheet();

// Apply the dark palette + stylesheet to qApp. Call once from main()
// before constructing windows.
void apply();

} // namespace Theme
