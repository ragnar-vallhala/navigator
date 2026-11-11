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

#include "DockPanelIds.h"
#include "WorkspaceManager.h"

#include <QHash>
#include <QObject>
#include <QString>
#include <QStringList>
#include <vector>

class QMainWindow;
class QMenu;
class QMenuBar;
class QWidget;

// The dock host of the real Navigator window (todo #187 step 3): every page is
// a registered panel with a stable id, laid out by the workspace system in
// src/ui/workspace/. Only used when the switch is on (see enabled()); the
// legacy QStackedWidget window never constructs one.
//
// The static helpers (ids, titles, built-in workspaces, the switch) need no
// display and are what the unit tests check.
class DockShell : public QObject {
  Q_OBJECT
public:
  struct PanelInfo {
    QString id;
    QString title;
  };
  // Every panel, in registration order (the MRU/recent-views index too).
  static const std::vector<PanelInfo> &panelInfos();
  // Panels pinned to the main window (they hold a QOpenGLWidget).
  static const std::vector<const char *> kFixedPanels;
  // Fly (the default), Tune, Debug link, Sim.
  static std::vector<ws::WorkspaceManager::Builtin> builtins();
  // Every panel id a layout names (tabs and floats, recursively).
  static QStringList referencedIds(const ws::LayoutSpec &spec);

  // The switch: env NAVIGATOR_DOCKING=1 (or true/on/yes), OR the persisted
  // setting (Window > Dockable layout). Always false in a build without the
  // docking library.
  static bool compiledIn();
  static bool envRequested();
  static bool persistedRequested();
  static void setPersisted(bool on);
  static bool enabled();

  explicit DockShell(QMainWindow *host);

  // Hand over a built page. Call for every panel, then start().
  void addPanel(const QString &id, QWidget *w);
  bool hasPanel(const QString &id) const;
  // Create the workspace manager over the panels added so far and apply the
  // default layout. Menus: call addMenus() once (before or after).
  void start();
  // Workspaces and Panels menus, inserted before `before` (null = appended).
  void addMenus(QMenuBar *bar, QAction *before);

  // Show and focus a panel (opens it if closed; never floats or moves it).
  // An unknown id is a no-op that returns false.
  bool showPanel(const QString &id);
  ws::WorkspaceManager *workspaces() { return m_wm; }
  void resetLayout();
  // Restore the last layout (the default if none or unusable); persistLast()
  // on exit.
  void restoreLast();
  void persistLast();

signals:
  void panelShown(const QString &id);
  // A saved layout could not be used (the default was applied): for a toast.
  void layoutProblem(const QString &text);

private:
  void rebuildWorkspaceMenu();

  QMainWindow *m_host;
  QHash<QString, QWidget *> m_widgets;
  ws::WorkspaceManager *m_wm = nullptr;
  QMenu *m_wsMenu = nullptr;
};
