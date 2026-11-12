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
#include "DockShell.h"

#include "AdsBackend.h"
#include "Theme.h"

#include <QAction>
#include <QActionGroup>
#include <QInputDialog>
#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QSettings>

#include <algorithm>

using ws::LayoutNode;
using ws::LayoutSpec;
using ws::WorkspaceManager;

namespace {
const char *kDockKey = "ui/dockableLayout";
}
const std::vector<const char *> DockShell::kFixedPanels = {dockpanel::kView3d};
namespace {

void collect(const LayoutNode &n, QStringList &out) {
  out << n.panels;
  for (const LayoutNode &c : n.children)
    collect(c, out);
}
} // namespace

const std::vector<DockShell::PanelInfo> &DockShell::panelInfos() {
  static const std::vector<PanelInfo> kInfos = {
      {dockpanel::kView3d, QObject::tr("Attitude"), "flight"},
      {dockpanel::kImu, QObject::tr("IMU"), "sensors"},
      {dockpanel::kLogs, QObject::tr("Logs"), "debug"},
      {dockpanel::kRc, QObject::tr("RC channels"), "flight"},
      {dockpanel::kPackets, QObject::tr("Packet analyzer"), "debug"},
      {dockpanel::kCalibration, QObject::tr("Calibration"), "tuning"},
      {dockpanel::kMotors, QObject::tr("Motors"), "flight"},
      {dockpanel::kControlLoop, QObject::tr("Control loop"), "tuning"},
      {dockpanel::kGyroNotch, QObject::tr("Gyro notch"), "tuning"},
      {dockpanel::kPerf, QObject::tr("Kernel perf"), "debug"},
      {dockpanel::kSim, QObject::tr("Simulator"), "simulation"},
      {dockpanel::kSettings, QObject::tr("Settings"), "app"},
  };
  return kInfos;
}

const std::vector<DockShell::GroupInfo> &DockShell::panelGroups() {
  static const std::vector<GroupInfo> kGroups = {
      {"flight", QObject::tr("&Flight")},      {"sensors", QObject::tr("&Sensors")},
      {"tuning", QObject::tr("&Tuning")},      {"debug", QObject::tr("&Debug")},
      {"simulation", QObject::tr("S&imulation")}, {"app", QObject::tr("&App")},
  };
  return kGroups;
}

void DockShell::buildPanelsMenu(QMenu *menu, const std::vector<PanelInfo> &infos,
                                const std::function<QAction *(const QString &)> &actionFor) {
  std::vector<GroupInfo> groups = panelGroups();
  const QString other = QStringLiteral("other");
  groups.push_back({other, QObject::tr("&Other")});
  for (const GroupInfo &g : groups) {
    QMenu *sub = nullptr;
    for (const PanelInfo &p : infos) {
      const bool known = std::any_of(groups.begin(), groups.end() - 1,
                                     [&](const GroupInfo &k) { return k.id == p.group; });
      if ((known ? p.group : other) != g.id)
        continue;
      QAction *a = actionFor(p.id);
      if (!a)
        continue;
      if (!sub)
        sub = menu->addMenu(g.title);
      sub->addAction(a);
    }
  }
}

std::vector<WorkspaceManager::Builtin> DockShell::builtins() {
  const auto T = [](QStringList p, QString cur = QString()) {
    return LayoutNode::tabs(p, cur);
  };
  const auto H = Qt::Horizontal, V = Qt::Vertical;
  std::vector<WorkspaceManager::Builtin> b;
  // Fly is the default workspace (first).
  b.push_back({"Fly",
               {LayoutNode::split(
                   H,
                   {T({dockpanel::kView3d}),
                    LayoutNode::split(V, {T({dockpanel::kImu}), T({dockpanel::kLogs})}, {3, 2})},
                   {3, 2})}});
  b.push_back({"Tune",
               {LayoutNode::split(
                   H,
                   {LayoutNode::split(V, {T({dockpanel::kControlLoop}), T({dockpanel::kGyroNotch})}),
                    LayoutNode::split(
                        V, {T({dockpanel::kMotors, dockpanel::kCalibration}, dockpanel::kMotors), T({dockpanel::kLogs})},
                        {3, 1})},
                   {3, 2})}});
  b.push_back({"Debug link",
               {LayoutNode::split(
                   V,
                   {LayoutNode::split(
                        H, {T({dockpanel::kPackets}), T({dockpanel::kPerf, dockpanel::kRc}, dockpanel::kPerf)}, {3, 2}),
                    T({dockpanel::kLogs})},
                   {3, 1})}});
  b.push_back({"Sim",
               {LayoutNode::split(V, {T({dockpanel::kSim}), T({dockpanel::kLogs})}, {4, 1})}});
  return b;
}

QStringList DockShell::referencedIds(const LayoutSpec &spec) {
  QStringList l;
  collect(spec.root, l);
  for (const ws::FloatSpec &f : spec.floats)
    l << f.panels;
  return l;
}

bool DockShell::compiledIn() {
#ifdef NAVIGATOR_HAS_DOCKING
  return true;
#else
  return false;
#endif
}

bool DockShell::envRequested() {
  const QString v = qEnvironmentVariable("NAVIGATOR_DOCKING").trimmed().toLower();
  return v == "1" || v == "true" || v == "on" || v == "yes";
}

bool DockShell::persistedRequested() {
  return QSettings().value(kDockKey, false).toBool();
}

void DockShell::setPersisted(bool on) { QSettings().setValue(kDockKey, on); }

bool DockShell::enabled() {
  return compiledIn() && (envRequested() || persistedRequested());
}

DockShell::DockShell(QMainWindow *host) : QObject(host), m_host(host) {}

void DockShell::addPanel(const QString &id, QWidget *w) {
  if (w && !m_widgets.contains(id))
    m_widgets.insert(id, w);
}

bool DockShell::hasPanel(const QString &id) const {
  return m_widgets.contains(id);
}

void DockShell::start(bool applyLayout) {
  if (m_wm)
    return;
  auto ads = std::make_unique<ws::AdsBackend>(m_host);
  ads->setExtraStyleSheet(Theme::dockStyleSheet());
  m_wm = new WorkspaceManager(m_host, std::move(ads), this);
  ws::PanelRegistry reg;
  for (const PanelInfo &p : panelInfos()) {
    QWidget *w = m_widgets.value(p.id, nullptr);
    if (w) // panels the build lacks (no simulator) are simply not registered
      reg.add(p.id, p.title, [w] { return w; });
  }
  m_wm->registerPanels(reg);
  // A Panels-menu open is a "shown" too (MRU switcher in the window).
  for (const QString &id : m_wm->backend()->panelIds())
    if (QAction *a = m_wm->backend()->viewAction(id))
      connect(a, &QAction::triggered, this, [this, id](bool on) {
        if (on)
          emit panelShown(id);
      });
  // The attitude panel's 2D HUD and 3D airframe draw on a QOpenGLWindow in a
  // window container (GlPaintHost), not a QOpenGLWidget: a QOpenGLWidget
  // re-initialises its GL context on every dock re-parent under ADS 5.1.1 /
  // Qt 6.4 and the window can stop repainting (workspace mock README,
  // tst_workspace_gl). With the container, one context lives for the widget's
  // life across workspace switches (tst_workspace_gl: initializeGL count 1).
  // The panel is still pinned to the main window (can be hidden, shown, split
  // next to; not floated), which is the conservative choice until floating a
  // window container out of the main window is checked on real GPUs. The
  // simulator view uses the same container approach and floats freely.
  for (const char *id : kFixedPanels)
    m_wm->backend()->setPanelFixed(QString::fromLatin1(id), true);
  m_wm->setBuiltins(builtins());
  connect(m_wm, &WorkspaceManager::workspacesChanged, this,
          &DockShell::rebuildWorkspaceMenu);
  connect(m_wm, &WorkspaceManager::currentChanged, this,
          [this](const QString &) { rebuildWorkspaceMenu(); });
  connect(m_wm, &WorkspaceManager::loadFailed, this,
          [this](const QString &n, const QString &why) {
            emit layoutProblem(tr("Could not load \"%1\" (%2): default layout "
                                  "applied")
                                   .arg(n, why));
          });
  if (applyLayout)
    m_wm->load(m_wm->defaultName());
}

void DockShell::addMenus(QMenuBar *bar, QAction *before) {
  if (!m_wm || m_wsMenu)
    return;
  m_wsMenu = new QMenu(tr("&Workspaces"), bar);
  auto *panels = new QMenu(tr("&Panels"), bar);
  buildPanelsMenu(panels, panelInfos(),
                  [this](const QString &id) { return m_wm->backend()->viewAction(id); });
  if (before) {
    bar->insertMenu(before, m_wsMenu);
    bar->insertMenu(before, panels);
  } else {
    bar->addMenu(m_wsMenu);
    bar->addMenu(panels);
  }
  rebuildWorkspaceMenu();
}

void DockShell::rebuildWorkspaceMenu() {
  if (!m_wsMenu || !m_wm)
    return;
  m_wsMenu->clear();
  auto *grp = new QActionGroup(m_wsMenu);
  auto addWs = [&](const QString &n) {
    QAction *a = m_wsMenu->addAction(n);
    a->setCheckable(true);
    a->setChecked(n == m_wm->currentName());
    grp->addAction(a);
    connect(a, &QAction::triggered, this, [this, n] { m_wm->load(n); });
  };
  for (const QString &n : m_wm->builtinNames())
    addWs(n);
  const QStringList users = m_wm->userNames();
  if (!users.isEmpty())
    m_wsMenu->addSeparator();
  for (const QString &n : users)
    addWs(n);
  m_wsMenu->addSeparator();
  m_wsMenu->addAction(tr("&Save layout as…"), this, [this] {
    bool ok = false;
    const QString n =
        QInputDialog::getText(m_host, tr("Save workspace"), tr("Name:"),
                              QLineEdit::Normal, QString(), &ok)
            .trimmed();
    if (!ok || n.isEmpty())
      return;
    if (!m_wm->saveCurrentAs(n))
      QMessageBox::warning(m_host, tr("Save workspace"),
                           tr("\"%1\" is a built-in name or not valid.").arg(n));
  });
  QMenu *ren = m_wsMenu->addMenu(tr("&Rename"));
  QMenu *del = m_wsMenu->addMenu(tr("&Delete"));
  ren->setEnabled(!users.isEmpty());
  del->setEnabled(!users.isEmpty());
  for (const QString &n : users) {
    ren->addAction(n, this, [this, n] {
      bool ok = false;
      const QString to =
          QInputDialog::getText(m_host, tr("Rename workspace"), tr("New name:"),
                                QLineEdit::Normal, n, &ok)
              .trimmed();
      if (ok && !m_wm->rename(n, to))
        QMessageBox::warning(m_host, tr("Rename workspace"),
                             tr("Cannot rename to \"%1\".").arg(to));
    });
    del->addAction(n, this, [this, n] { m_wm->remove(n); });
  }
  m_wsMenu->addSeparator();
  m_wsMenu->addAction(tr("Reset &layout"), this, [this] { resetLayout(); });
}

bool DockShell::showPanel(const QString &id) {
  if (!m_wm || !m_widgets.contains(id))
    return false;
  m_wm->backend()->showPanel(id);
  // Focus it too, so a floated panel comes to the front and takes the keyboard.
  if (QWidget *w = m_widgets.value(id)) {
    if (QWidget *top = w->window())
      top->activateWindow();
    w->setFocus();
  }
  emit panelShown(id);
  return true;
}

void DockShell::resetLayout() {
  if (m_wm)
    m_wm->resetLayout();
}

void DockShell::restoreLast() {
  if (m_wm)
    m_wm->restoreLast();
}

void DockShell::persistLast() {
  if (m_wm)
    m_wm->persistLast();
}
