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

#include <QHash>
#include <QPointer>
#include <utility>
#include <vector>

class QMainWindow;

namespace ads {
class CDockManager;
class CDockWidget;
class CDockAreaWidget;
} // namespace ads

namespace ws {

// DockBackend on Qt Advanced Docking System (LGPL-2.1, used as a shared
// library). Every layout change goes through a fresh CDockManager, so a failed
// or half-applied restore can never leave a broken layout behind.
class AdsBackend : public DockBackend {
public:
  explicit AdsBackend(QMainWindow *host);
  ~AdsBackend() override;

  QString tag() const override;
  void addPanel(const QString &id, const QString &title, QWidget *w) override;
  QStringList panelIds() const override;
  void setPanelFixed(const QString &id, bool fixed) override;
  void applySpec(const LayoutSpec &spec) override;
  QByteArray saveState() const override;
  bool restoreState(const QByteArray &state) override;
  PanelState panelState(const QString &id) const override;
  void showPanel(const QString &id) override;
  void hidePanel(const QString &id) override;
  void floatPanel(const QString &id) override;
  void tabPanel(const QString &id, const QString &ontoId) override;
  void setFloatGeometry(const QString &id, const QRect &g) override;
  QAction *viewAction(const QString &id) override;
  int clampFloatingTo(const QList<QRect> &screens) override;

private:
  struct Panel {
    QString id, title;
    bool fixed = false;
    QPointer<QWidget> widget;
    QPointer<ads::CDockWidget> dock; // a child of m_dm: dies with it
  };
  void rebuild();
  void applyFixed(Panel &p);
  Panel *find(const QString &id);
  const Panel *find(const QString &id) const;
  ads::CDockAreaWidget *place(const LayoutNode &n, ads::CDockAreaWidget *anchor,
                              int dir, bool root);

  // The host's widget tree owns the host's children, so either may be deleted
  // before this backend is: hold them weakly and skip what is already gone.
  QPointer<QMainWindow> m_host;
  QPointer<ads::CDockManager> m_dm;
  std::vector<Panel> m_panels;
  // View actions outlive a rebuild (menus hold them); they forward to the
  // current dock.
  QHash<QString, QAction *> m_actions;
  std::vector<std::pair<QPointer<ads::CDockAreaWidget>, QList<int>>> m_pending;
};

} // namespace ws
