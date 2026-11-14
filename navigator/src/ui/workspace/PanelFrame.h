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
// Content-minimum frame for a docked / floating panel.
//
// ADS reports 60x40 as a dock widget's minimum by default, so a panel could be
// squeezed (docked in a short strip) or floated far below what its layout
// needs, and its rows overlapped. A panel is hosted in a PanelFrame instead: a
// scroll area (resizable widget) whose minimumSizeHint is the panel's own
// minimumSizeHint, capped at kMaxMinW x kMaxMinH. Below the content minimum the
// panel keeps its minimum size and the frame scrolls, so rows never overlap;
// the cap keeps tall/wide panels (packet table, simulator page) from forcing
// the main window past a small screen.
#include <QScrollArea>
#include <QSize>

namespace ws {

constexpr int kMaxMinW = 360;
constexpr int kMaxMinH = 240;

// The minimum size a frame reports for content that needs `content`.
inline QSize cappedMinimum(const QSize &content) {
  return {qMin(content.width(), kMaxMinW), qMin(content.height(), kMaxMinH)};
}

class PanelFrame : public QScrollArea {
public:
  explicit PanelFrame(QWidget *content, QWidget *parent = nullptr)
      : QScrollArea(parent) {
    setFrameShape(QFrame::NoFrame);
    setWidgetResizable(true);
    setWidget(content);
  }

  // What a layout would give the content as its minimum: an explicit
  // setMinimumSize wins, per axis, over the minimumSizeHint.
  static QSize minimumOf(const QWidget *w) {
    const QSize ms = w->minimumSize(), mh = w->minimumSizeHint();
    return {ms.width() > 0 ? ms.width() : mh.width(),
            ms.height() > 0 ? ms.height() : mh.height()};
  }
  QSize contentMinimum() const {
    return widget() ? minimumOf(widget()) : QSize();
  }
  QSize minimumSizeHint() const override {
    return cappedMinimum(contentMinimum());
  }
  QSize sizeHint() const override {
    return widget() ? widget()->sizeHint() : QScrollArea::sizeHint();
  }
};

} // namespace ws
