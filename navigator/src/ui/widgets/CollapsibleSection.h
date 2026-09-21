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

#include <QWidget>

class QToolButton;
class QVBoxLayout;

// CollapsibleSection — a Blender-style property panel: a clickable header
// with a disclosure triangle that expands/collapses a content widget.
// Multiple sections stack in a scroll area to form the categorized
// right-hand properties panel.
class CollapsibleSection : public QWidget {
  Q_OBJECT
public:
  explicit CollapsibleSection(const QString &title, QWidget *parent = nullptr,
                              bool expanded = true);

  // Reparents `content` into the section body.
  void setContentWidget(QWidget *content);
  void setExpanded(bool on);
  bool isExpanded() const;

private:
  void updateArrow();

  QToolButton *header_ = nullptr;
  QWidget *body_ = nullptr;
  QVBoxLayout *bodyLayout_ = nullptr;
};
