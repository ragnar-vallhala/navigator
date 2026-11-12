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
#include "AdsBackend.h"

#include "ScreenClamp.h"

#include <DockAreaWidget.h>
#include <DockContainerWidget.h>
#include <DockManager.h>
#include <DockWidget.h>
#include <DockWidgetTab.h>
#include <FloatingDockContainer.h>

#include <QMainWindow>
#include <QTimer>

namespace ws {

namespace {
constexpr int kStateVersion = 2;
}

AdsBackend::AdsBackend(QMainWindow *host) : m_host(host) {
  using M = ads::CDockManager;
  M::setConfigFlags(M::DefaultOpaqueConfig | M::DockAreaHasUndockButton |
                    M::AlwaysShowTabs);
  M::setConfigFlag(M::FocusHighlighting, false);
}

AdsBackend::~AdsBackend() {
  // If the host was deleted first, Qt has already deleted the manager and
  // every dock (and the panel widgets inside them): every pointer below is
  // then null and there is nothing left to tear down.
  // Hand the panels back before the manager (their parent) goes away.
  for (Panel &p : m_panels)
    if (p.dock && p.widget) {
      p.dock->takeWidget();
      p.widget->setParent(nullptr);
    }
  if (m_dm) {
    // When the manager is a child of the host and the host is being deleted,
    // Qt has already deleted the host's own layout (it is the first child), and
    // QMainWindow::takeCentralWidget() would dereference it. Only detach while
    // the host is still whole; otherwise deleting the manager is enough.
    if (m_host && m_host->layout())
      m_host->takeCentralWidget();
    delete m_dm.data();
    m_dm.clear();
  }
  for (Panel &p : m_panels) // the backend owns the panel widgets
    delete p.widget.data();
}

QString AdsBackend::tag() const { return QStringLiteral("ads-v2"); }

AdsBackend::Panel *AdsBackend::find(const QString &id) {
  for (Panel &p : m_panels)
    if (p.id == id)
      return &p;
  return nullptr;
}
const AdsBackend::Panel *AdsBackend::find(const QString &id) const {
  for (const Panel &p : m_panels)
    if (p.id == id)
      return &p;
  return nullptr;
}

void AdsBackend::addPanel(const QString &id, const QString &title,
                          QWidget *w) {
  if (find(id))
    return;
  m_panels.push_back({id, title, false, w, nullptr});
  // A Panels-menu action that stays valid across rebuilds.
  auto *a = new QAction(title, m_host);
  a->setCheckable(true);
  QObject::connect(a, &QAction::triggered, m_host.data(), [this, id](bool on) {
    if (on)
      showPanel(id);
    else
      hidePanel(id);
  });
  m_actions.insert(id, a);
}

void AdsBackend::setPanelFixed(const QString &id, bool fixed) {
  Panel *p = find(id);
  if (!p)
    return;
  p->fixed = fixed;
  applyFixed(*p);
}

// ADS's viewToggled does not fire for every open/close a layout change makes, so
// bring the Panels-menu checks in line with what is really open.
void AdsBackend::syncActions() {
  for (Panel &p : m_panels) {
    QAction *a = m_actions.value(p.id);
    if (a)
      a->setChecked(p.dock && p.dock->dockAreaWidget() && !p.dock->isClosed());
  }
}

void AdsBackend::applyFixed(Panel &p) {
  if (!p.dock)
    return;
  p.dock->setFeature(ads::CDockWidget::DockWidgetFloatable, !p.fixed);
  p.dock->setFeature(ads::CDockWidget::DockWidgetMovable, !p.fixed);
}

QStringList AdsBackend::panelIds() const {
  QStringList l;
  for (const Panel &p : m_panels)
    l << p.id;
  return l;
}

void AdsBackend::rebuild() {
  if (!m_host) // the window is gone: nothing to build into
    return;
  for (Panel &p : m_panels) {
    if (p.dock && p.widget) {
      p.dock->takeWidget();
      p.widget->setParent(nullptr);
    }
    p.dock = nullptr;
  }
  if (m_dm) {
    m_host->takeCentralWidget();
    delete m_dm.data(); // also deletes the docks and any floating windows
    m_dm.clear();
  }
  m_dm = new ads::CDockManager(m_host);
  if (!m_extraStyle.isEmpty())
    m_dm->setStyleSheet(m_dm->styleSheet() + m_extraStyle);
  m_host->setCentralWidget(m_dm);
  for (Panel &p : m_panels) {
    auto *dw = new ads::CDockWidget(m_dm, p.title);
    dw->setObjectName(p.id);
    dw->setFeature(ads::CDockWidget::DockWidgetClosable, true);
    p.dock = dw;
    applyFixed(p);
    if (p.widget)
      dw->setWidget(p.widget, ads::CDockWidget::ForceNoScrollArea);
    // Keep the menu action in step with the panel's visibility.
    QAction *act = m_actions.value(p.id);
    if (act) {
      act->setChecked(false);
      QObject::connect(dw, &ads::CDockWidget::viewToggled, act,
                       [act](bool open) { act->setChecked(open); });
    }
  }
  // ADS only knows a dock widget (for restoreState) once it has been added, so
  // park them all, closed, in one hidden area. Placing a panel later moves it.
  ads::CDockAreaWidget *park = nullptr;
  for (Panel &p : m_panels) {
    if (!park)
      park = m_dm->addDockWidget(ads::CenterDockWidgetArea, p.dock);
    else
      m_dm->addDockWidgetTabToArea(p.dock, park);
  }
  for (Panel &p : m_panels)
    p.dock->closeDockWidget();
}

ads::CDockAreaWidget *AdsBackend::place(const LayoutNode &n,
                                        ads::CDockAreaWidget *anchor, int dir,
                                        bool root) {
  const auto area = static_cast<ads::DockWidgetArea>(dir);
  if (n.kind == LayoutNode::Tabs) {
    ads::CDockAreaWidget *first = nullptr;
    ads::CDockWidget *cur = nullptr;
    for (const QString &id : n.panels) {
      Panel *p = find(id);
      if (!p || !p->dock)
        continue;
      if (!first)
        first = m_dm->addDockWidget(area, p->dock, anchor);
      else
        m_dm->addDockWidgetTabToArea(p->dock, first);
      p->dock->toggleView(true); // it was parked closed; reopen in place
      p->dock->tabWidget()->show();
      if (id == n.current)
        cur = p->dock;
    }
    if (cur)
      cur->setAsCurrentTab();
    return first;
  }
  const int childDir = n.orientation == Qt::Horizontal
                           ? int(ads::RightDockWidgetArea)
                           : int(ads::BottomDockWidgetArea);
  QList<ads::CDockAreaWidget *> firsts;
  ads::CDockAreaWidget *prev = nullptr;
  for (size_t i = 0; i < n.children.size(); ++i) {
    ads::CDockAreaWidget *a;
    if (i == 0)
      a = place(n.children[i], anchor, dir, root);
    else // root level: container edge; nested: next to the previous sibling
      a = place(n.children[i], root ? nullptr : prev, childDir, false);
    if (a) {
      firsts << a;
      prev = a;
    }
  }
  if (firsts.size() > 1 && n.weights.size() == int(firsts.size())) {
    // QSplitter wants pixels: scale the weights to the window along the axis.
    const int total = n.orientation == Qt::Horizontal ? m_host->width()
                                                      : m_host->height();
    int sum = 0;
    for (int w : n.weights)
      sum += w;
    QList<int> px;
    for (int w : n.weights)
      px << (sum > 0 ? total * w / sum : total);
    m_pending.push_back({firsts.first(), px});
    m_dm->setSplitterSizes(firsts.first(), px);
  }
  return firsts.isEmpty() ? nullptr : firsts.first();
}

void AdsBackend::setExtraStyleSheet(const QString &qss) {
  if (m_dm) {
    QString cur = m_dm->styleSheet();
    if (!m_extraStyle.isEmpty() && cur.endsWith(m_extraStyle))
      cur.chop(m_extraStyle.size()); // replace the last extra, never stack it
    m_dm->setStyleSheet(cur + qss);
  }
  m_extraStyle = qss;
}

void AdsBackend::applySpec(const LayoutSpec &spec) {
  rebuild();
  if (!m_dm)
    return;
  m_pending.clear();
  place(spec.root, nullptr, ads::CenterDockWidgetArea, true);
  // The splitters only take proportions once laid out: set them again then.
  QPointer<ads::CDockManager> dm = m_dm;
  auto pending = m_pending;
  QTimer::singleShot(0, m_host.data(), [dm, pending] {
    if (!dm)
      return;
    for (const auto &p : pending)
      if (p.first)
        dm->setSplitterSizes(p.first, p.second);
  });
  for (const FloatSpec &f : spec.floats) {
    ads::CDockWidget *first = nullptr;
    for (const QString &id : f.panels) {
      Panel *p = find(id);
      if (!p || !p->dock || p->fixed) // a fixed panel never floats
        continue;
      if (!first) {
        first = p->dock;
        auto *fc = m_dm->addDockWidgetFloating(p->dock);
        p->dock->toggleView(true);
        p->dock->tabWidget()->show();
        if (fc && f.geometry.isValid())
          fc->setGeometry(f.geometry);
      } else {
        m_dm->addDockWidgetTabToArea(p->dock, first->dockAreaWidget());
        p->dock->toggleView(true);
        p->dock->tabWidget()->show();
      }
    }
  }
  syncActions();
}

QByteArray AdsBackend::saveState() const {
  return m_dm ? m_dm->saveState(kStateVersion) : QByteArray();
}

bool AdsBackend::restoreState(const QByteArray &state) {
  rebuild();
  if (!m_dm || state.isEmpty())
    return false;
  const bool ok = m_dm->restoreState(state, kStateVersion);
  if (!ok)
    rebuild(); // empty, usable layout; the caller applies a default
  syncActions();
  return ok;
}

PanelState AdsBackend::panelState(const QString &id) const {
  PanelState s;
  const Panel *p = find(id);
  if (!p)
    return s;
  s.known = true;
  s.fixed = p->fixed;
  if (!p->dock || p->dock->isClosed() || !p->dock->dockAreaWidget())
    return s;
  s.visible = true;
  auto *c = p->dock->dockContainer();
  s.floating = c && c->isFloating();
  for (ads::CDockWidget *d : p->dock->dockAreaWidget()->openedDockWidgets())
    s.tabGroup << d->objectName();
  s.tabGroup.sort();
  if (s.floating && c->floatingWidget())
    s.floatGeometry = c->floatingWidget()->geometry();
  return s;
}

void AdsBackend::showPanel(const QString &id) {
  Panel *p = find(id);
  if (!p || !p->dock || !m_dm)
    return;
  if (!p->dock->dockAreaWidget())
    m_dm->addDockWidget(ads::RightDockWidgetArea, p->dock);
  else if (p->dock->isClosed())
    p->dock->toggleView(true);
  p->dock->setAsCurrentTab();
  p->dock->raise();
  syncActions();
}

void AdsBackend::hidePanel(const QString &id) {
  Panel *p = find(id);
  if (p && p->dock && p->dock->dockAreaWidget() && !p->dock->isClosed())
    p->dock->closeDockWidget();
  syncActions();
}

void AdsBackend::floatPanel(const QString &id) {
  Panel *p = find(id);
  if (!p || !p->dock || p->fixed)
    return;
  if (!p->dock->dockAreaWidget() || p->dock->isClosed())
    showPanel(id);
  if (!p->dock->isInFloatingContainer())
    p->dock->setFloating();
}

void AdsBackend::tabPanel(const QString &id, const QString &ontoId) {
  Panel *p = find(id);
  Panel *o = find(ontoId);
  if (!p || !o || !p->dock || !o->dock || !m_dm)
    return;
  if (!o->dock->dockAreaWidget())
    showPanel(ontoId);
  m_dm->addDockWidgetTabToArea(p->dock, o->dock->dockAreaWidget());
  p->dock->setAsCurrentTab();
}

void AdsBackend::setFloatGeometry(const QString &id, const QRect &g) {
  const Panel *p = find(id);
  if (!p || !p->dock)
    return;
  auto *c = p->dock->dockContainer();
  if (c && c->isFloating() && c->floatingWidget())
    c->floatingWidget()->setGeometry(g);
}

QAction *AdsBackend::viewAction(const QString &id) {
  return m_actions.value(id, nullptr);
}

int AdsBackend::clampFloatingTo(const QList<QRect> &screens) {
  int moved = 0;
  if (!m_dm)
    return 0;
  for (auto *fw : m_dm->floatingWidgets()) {
    const QRect g = fw->geometry();
    const QRect c = clampRectToScreens(g, screens);
    if (c != g) {
      fw->setGeometry(c);
      ++moved;
    }
  }
  return moved;
}

} // namespace ws
