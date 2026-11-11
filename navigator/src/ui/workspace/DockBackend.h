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

#include "LayoutSpec.h"

#include <QAction>
#include <QByteArray>
#include <QRect>
#include <QString>
#include <QStringList>

class QWidget;

namespace ws {

struct PanelState {
  bool known = false;
  bool visible = false;  // placed and open
  bool floating = false; // in a window of its own
  QStringList tabGroup;  // ids of the open panels sharing its tab group
  QRect floatGeometry;   // only meaningful when floating
};

// The one interface WorkspaceManager talks to. AdsBackend (Qt Advanced Docking
// System) is the implementation today; a plain QDockWidget one can replace it
// without touching WorkspaceManager, the panels or the built-in layouts.
class DockBackend {
public:
  virtual ~DockBackend() = default;

  // Short stable tag stored with every saved layout; a layout written by a
  // different backend (or format) is rejected instead of misread.
  virtual QString tag() const = 0;

  // Register a panel. The backend takes ownership of `w`; the widget survives
  // layout rebuilds (it is re-hosted, never recreated).
  virtual void addPanel(const QString &id, const QString &title,
                        QWidget *w) = 0;
  virtual QStringList panelIds() const = 0;

  // Replace the whole layout.
  virtual void applySpec(const LayoutSpec &spec) = 0;
  virtual QByteArray saveState() const = 0;
  // Replace the whole layout from a blob; false (and a usable, empty layout)
  // if the blob is not understood.
  virtual bool restoreState(const QByteArray &state) = 0;

  virtual PanelState panelState(const QString &id) const = 0;
  virtual void showPanel(const QString &id) = 0;
  virtual void hidePanel(const QString &id) = 0;
  virtual void floatPanel(const QString &id) = 0;
  // Put `id` into the tab group of `ontoId` (both must exist).
  virtual void tabPanel(const QString &id, const QString &ontoId) = 0;
  virtual void setFloatGeometry(const QString &id, const QRect &g) = 0;
  // Show/hide action for a Panels menu.
  virtual QAction *viewAction(const QString &id) = 0;

  // Move every floating window that is not reachable on `screens` back onto
  // one. Returns how many were moved.
  virtual int clampFloatingTo(const QList<QRect> &screens) = 0;
};

} // namespace ws
