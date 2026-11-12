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

#include "DockBackend.h"
#include "LayoutSpec.h"
#include "PanelRegistry.h"

#include <QList>
#include <QByteArray>
#include <QObject>
#include <QPointer>
#include <QTimer>
#include <QRect>
#include <QStringList>
#include <functional>
#include <memory>
#include <vector>

class QMainWindow;

namespace ws {

// Named workspaces on top of a DockBackend: built-in ones (read-only), user
// ones (save / load / rename / delete), the last layout restored at start, and a
// Reset that always works. Persists through QSettings (the application's
// organisation/name), under the group "workspaces".
//
// A saved layout carries a format number and the backend tag; anything else, an
// empty or a corrupt blob falls back to the default workspace instead of
// failing.
class WorkspaceManager : public QObject {
  Q_OBJECT
public:
  // 2: the real window's panel set (ids differ from the mock's format-1 layouts).
  static constexpr int kFormat = 2;

  struct Builtin {
    QString name;
    LayoutSpec spec;
  };

  // Takes ownership of `backend`. `host` is the main window whose geometry is
  // saved with each workspace.
  WorkspaceManager(QMainWindow *host, std::unique_ptr<DockBackend> backend,
                   QObject *parent = nullptr);
  ~WorkspaceManager() override;

  void registerPanels(const PanelRegistry &reg);
  // The first built-in is the default workspace.
  void setBuiltins(const std::vector<Builtin> &b);

  QStringList builtinNames() const;
  QStringList userNames() const; // sorted
  QString defaultName() const;
  bool isBuiltin(const QString &name) const;
  bool exists(const QString &name) const;
  // Active workspace ("" = the layout was changed and not saved, or restored).
  QString currentName() const { return m_current; }

  // Name rules: non-empty after trimming, not a built-in name (any case).
  bool validUserName(const QString &name) const;
  bool saveCurrentAs(const QString &name); // overwrites a user workspace
  bool load(const QString &name);          // false: unknown or unusable
  bool remove(const QString &name);        // user workspaces only
  bool rename(const QString &from, const QString &to);
  void resetLayout();                      // built-in itself, else the default

  // Last layout (+ window geometry). persistLast() on exit; restoreLast() at
  // start returns false when it fell back to the default.
  void persistLast();
  bool restoreLast();

  // Reachability checks use these screens (tests inject their own).
  void setScreenProvider(std::function<QList<QRect>()> f) {
    m_screens = std::move(f);
  }
  QList<QRect> screens() const;
  // Pull the main window and every floating window back onto a screen.
  int clampToScreens();

  DockBackend *backend() { return m_backend.get(); }

  // The "last layout" slot: whatever is on screen (a built-in, a saved view or
  // an edited, unsaved variant of one) plus the name of the workspace it came
  // from. It is written `kLastDelayMs` after the last layout change, and at
  // once before another workspace loads, in persistLast() (window close) and
  // on app quit. It is its own slot: built-in and saved workspaces are never
  // written by it (only saveCurrentAs() writes those).
  static constexpr int kLastDelayMs = 800;
  void setLastDelay(int ms) { m_lastTimer.setInterval(ms); }
  // Store the last layout now if a change is pending and differs from the slot.
  void flushLast();

signals:
  void workspacesChanged();
  void currentChanged(const QString &name);
  // A saved layout could not be used and the default was applied instead.
  void loadFailed(const QString &name, const QString &reason);
  // The last-layout slot was written (not emitted by persistLast()).
  void lastSaved();

private:
  const Builtin *builtin(const QString &name) const;
  void applyBuiltin(const Builtin &b);
  void setCurrent(const QString &n);
  QString userGroup(const QString &name) const;
  bool hasUnknownPanel(const QByteArray &state) const;
  void onLayoutChanged();
  void markChanged();
  bool writeLast(); // false: nothing usable captured

  QPointer<QMainWindow> m_host;
  QTimer m_lastTimer;
  QByteArray m_lastBlob;   // what the slot holds
  bool m_dirty = false;    // a change was signalled since the slot was written
  bool m_applying = false; // a layout is being built: changes are not edits
  std::unique_ptr<DockBackend> m_backend;
  std::vector<Builtin> m_builtins;
  QString m_current;
  std::function<QList<QRect>()> m_screens;
};

} // namespace ws
