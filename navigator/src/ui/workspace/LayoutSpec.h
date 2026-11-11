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
#include <QString>
#include <QStringList>
#include <Qt>
#include <vector>

namespace ws {

// A library-neutral description of a window layout. Built-in workspaces are
// written as these trees, so the docking library behind DockBackend can change
// without touching the layouts.
//
//   Tabs  : the listed panels share one tab group (`current` is on top).
//   Split : children laid out side by side (Horizontal) or stacked (Vertical).
//           `weights` (optional, one per child) set the relative sizes.
//
// Panels not named anywhere are left hidden. Nesting is exact for a root split
// whose children are tab groups or one level of further splits.
struct LayoutNode {
  enum Kind { Tabs, Split };
  Kind kind = Tabs;
  Qt::Orientation orientation = Qt::Horizontal;
  QStringList panels;
  QString current;
  std::vector<LayoutNode> children;
  QList<int> weights;

  static LayoutNode tabs(const QStringList &panels,
                         const QString &current = QString()) {
    LayoutNode n;
    n.kind = Tabs;
    n.panels = panels;
    n.current = current;
    return n;
  }
  static LayoutNode split(Qt::Orientation o,
                          const std::vector<LayoutNode> &children,
                          const QList<int> &weights = {}) {
    LayoutNode n;
    n.kind = Split;
    n.orientation = o;
    n.children = children;
    n.weights = weights;
    return n;
  }
};

// A floating window holding one or more tabbed panels.
struct FloatSpec {
  QStringList panels;
  QRect geometry; // screen coordinates; null = let the library choose
};

struct LayoutSpec {
  LayoutNode root;
  std::vector<FloatSpec> floats;
};

} // namespace ws
