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
#include "WorkspaceManager.h"

#include "ScreenClamp.h"

#include <QCoreApplication>
#include <QGuiApplication>
#include <QMainWindow>
#include <QScreen>
#include <QSettings>
#include <algorithm>

namespace ws {

namespace {
const QString kRoot = QStringLiteral("workspaces");
const QString kLast = kRoot + QStringLiteral("/last");
const QString kUser = kRoot + QStringLiteral("/user");

struct Stored {
  QString name;
  QByteArray state, geometry;
  int format = 0;
  QString backend;
};

void writeStored(const QString &group, const Stored &s) {
  QSettings q;
  q.beginGroup(group);
  q.setValue("name", s.name);
  q.setValue("format", s.format);
  q.setValue("backend", s.backend);
  q.setValue("state", s.state);
  q.setValue("geometry", s.geometry);
  q.endGroup();
  q.sync();
}

bool readStored(const QString &group, Stored &s) {
  QSettings q;
  q.beginGroup(group);
  if (!q.contains("state"))
    return false;
  s.name = q.value("name").toString();
  s.format = q.value("format", 0).toInt();
  s.backend = q.value("backend").toString();
  s.state = q.value("state").toByteArray();
  s.geometry = q.value("geometry").toByteArray();
  return true;
}
} // namespace

WorkspaceManager::WorkspaceManager(QMainWindow *host,
                                   std::unique_ptr<DockBackend> backend,
                                   QObject *parent)
    : QObject(parent), m_host(host), m_backend(std::move(backend)) {
  m_lastTimer.setSingleShot(true);
  m_lastTimer.setInterval(kLastDelayMs);
  connect(&m_lastTimer, &QTimer::timeout, this, [this] { flushLast(); });
  m_backend->setChangeNotifier([this] { onLayoutChanged(); });
  connect(qApp, &QCoreApplication::aboutToQuit, this, [this] { flushLast(); });
}

WorkspaceManager::~WorkspaceManager() = default;

void WorkspaceManager::registerPanels(const PanelRegistry &reg) {
  for (const PanelDef &d : reg.defs())
    m_backend->addPanel(d.id, d.title, d.create ? d.create() : new QWidget);
}

void WorkspaceManager::setBuiltins(const std::vector<Builtin> &b) {
  m_builtins = b;
}

QStringList WorkspaceManager::builtinNames() const {
  QStringList l;
  for (const Builtin &b : m_builtins)
    l << b.name;
  return l;
}

QString WorkspaceManager::defaultName() const {
  return m_builtins.empty() ? QString() : m_builtins.front().name;
}

const WorkspaceManager::Builtin *
WorkspaceManager::builtin(const QString &name) const {
  for (const Builtin &b : m_builtins)
    if (b.name.compare(name, Qt::CaseInsensitive) == 0)
      return &b;
  return nullptr;
}

bool WorkspaceManager::isBuiltin(const QString &name) const {
  return builtin(name) != nullptr;
}

QString WorkspaceManager::userGroup(const QString &name) const {
  return kUser + '/' + QString::fromLatin1(name.toUtf8().toHex());
}

QStringList WorkspaceManager::userNames() const {
  QSettings q;
  q.beginGroup(kUser);
  QStringList names;
  for (const QString &g : q.childGroups()) {
    const QString n = q.value(g + "/name").toString();
    if (!n.isEmpty() && q.contains(g + "/state"))
      names << n;
  }
  std::sort(names.begin(), names.end(), [](const QString &a, const QString &b) {
    return a.compare(b, Qt::CaseInsensitive) < 0;
  });
  return names;
}

bool WorkspaceManager::exists(const QString &name) const {
  return isBuiltin(name) || userNames().contains(name);
}

bool WorkspaceManager::validUserName(const QString &name) const {
  const QString n = name.trimmed();
  return !n.isEmpty() && n == name && !isBuiltin(n);
}

QList<QRect> WorkspaceManager::screens() const {
  if (m_screens)
    return m_screens();
  QList<QRect> r;
  for (QScreen *s : QGuiApplication::screens())
    r << s->availableGeometry();
  return r;
}

void WorkspaceManager::setCurrent(const QString &n) {
  if (m_current == n)
    return;
  m_current = n;
  emit currentChanged(n);
}

int WorkspaceManager::clampToScreens() {
  const QList<QRect> scr = screens();
  int moved = 0;
  if (m_host && !m_host->isMaximized() && !m_host->isFullScreen()) {
    const QRect g = m_host->frameGeometry();
    const QRect c = clampRectToScreens(g, scr);
    if (c != g) {
      m_host->setGeometry(c);
      ++moved;
    }
  }
  return moved + m_backend->clampFloatingTo(scr);
}

bool WorkspaceManager::hasUnknownPanel(const QByteArray &state) const {
  const QStringList known = m_backend->panelIds();
  for (const QString &id : m_backend->statePanelIds(state))
    if (!known.contains(id))
      return true;
  return false;
}

void WorkspaceManager::onLayoutChanged() {
  if (!m_applying)
    markChanged();
}

void WorkspaceManager::markChanged() {
  m_dirty = true;
  m_lastTimer.start();
}

bool WorkspaceManager::writeLast() {
  const QByteArray st = m_backend->saveState();
  if (st.isEmpty()) // a failed capture never replaces a good slot
    return false;
  Stored s;
  s.name = m_current;
  s.format = kFormat;
  s.backend = m_backend->tag();
  s.state = st;
  s.geometry = m_host ? m_host->saveGeometry() : QByteArray();
  writeStored(kLast, s);
  m_lastBlob = st;
  m_dirty = false;
  return true;
}

void WorkspaceManager::flushLast() {
  m_lastTimer.stop();
  // Only a signalled change counts: a layout that merely settles after a load
  // is not an edit.
  if (!m_dirty || m_applying)
    return;
  if (m_backend->saveState() == m_lastBlob) {
    m_dirty = false;
    return;
  }
  if (writeLast())
    emit lastSaved();
}

void WorkspaceManager::applyBuiltin(const Builtin &b) {
  m_backend->applySpec(b.spec);
  setCurrent(b.name);
}

bool WorkspaceManager::saveCurrentAs(const QString &name) {
  if (!validUserName(name))
    return false;
  Stored s;
  s.name = name;
  s.format = kFormat;
  s.backend = m_backend->tag();
  s.state = m_backend->saveState();
  s.geometry = m_host ? m_host->saveGeometry() : QByteArray();
  writeStored(userGroup(name), s);
  setCurrent(name);
  emit workspacesChanged();
  return true;
}

bool WorkspaceManager::load(const QString &name) {
  flushLast(); // the outgoing layout's pending edit is kept
  const struct Guard {
    bool &f;
    explicit Guard(bool &flag) : f(flag) { f = true; }
    ~Guard() { f = false; }
  } guard(m_applying);
  struct After { // one debounced record of the new layout, after it settled
    WorkspaceManager *w;
    ~After() { w->markChanged(); }
  } after{this};
  if (const Builtin *b = builtin(name)) {
    applyBuiltin(*b);
    clampToScreens();
    return true;
  }
  Stored s;
  if (!userNames().contains(name) || !readStored(userGroup(name), s))
    return false;
  const QString bad = (s.format != kFormat)       ? tr("older or newer format")
                      : (s.backend != m_backend->tag()) ? tr("other backend")
                      : s.state.isEmpty()           ? tr("empty")
                      : hasUnknownPanel(s.state)    ? tr("unknown panel")
                                                    : QString();
  if (bad.isEmpty()) {
    if (m_backend->restoreState(s.state)) {
      // Only move/resize the window once the layout is known good, so a
      // corrupt state does not leave it displaced before the fallback.
      if (m_host && !s.geometry.isEmpty()) {
        // A switch must not change the window's state: a layout saved while
        // the window was maximized (or full-screen) would otherwise maximize
        // it now. Only the normal-state rectangle comes along.
        const bool wasMax = m_host->isMaximized();
        const bool wasFull = m_host->isFullScreen();
        // Nor may a switch move it between monitors: the saved rectangle's
        // size comes along, its position does not (startup restoreLast keeps
        // restoring the position).
        const QPoint keepPos = m_host->pos();
        m_host->restoreGeometry(s.geometry);
        if (!wasFull && m_host->isFullScreen())
          m_host->showNormal();
        if (!wasMax && m_host->isMaximized())
          m_host->showNormal();
        if (!wasMax && !wasFull && !m_host->isMaximized() &&
            !m_host->isFullScreen())
          m_host->move(keepPos); // clampToScreens below fits it to its screen
      }
      setCurrent(name);
      clampToScreens();
      return true;
    }
    emit loadFailed(name, tr("corrupt layout"));
  } else {
    emit loadFailed(name, bad);
  }
  if (!m_builtins.empty()) { // an unusable layout always means the default
    applyBuiltin(m_builtins.front());
    clampToScreens();
  }
  return false;
}

bool WorkspaceManager::remove(const QString &name) {
  if (isBuiltin(name) || !userNames().contains(name))
    return false;
  QSettings q;
  q.remove(userGroup(name));
  q.sync();
  if (m_current == name)
    setCurrent(QString());
  emit workspacesChanged();
  return true;
}

bool WorkspaceManager::rename(const QString &from, const QString &to) {
  if (isBuiltin(from) || !userNames().contains(from) || !validUserName(to) ||
      (to != from && exists(to)))
    return false;
  if (to == from)
    return true;
  Stored s;
  if (!readStored(userGroup(from), s))
    return false;
  s.name = to;
  writeStored(userGroup(to), s);
  QSettings q;
  q.remove(userGroup(from));
  q.sync();
  if (m_current == from)
    setCurrent(to);
  emit workspacesChanged();
  return true;
}

void WorkspaceManager::resetLayout() {
  flushLast();
  const struct Guard {
    bool &f;
    explicit Guard(bool &flag) : f(flag) { f = true; }
    ~Guard() { f = false; }
  } guard(m_applying);
  struct After {
    WorkspaceManager *w;
    ~After() { w->markChanged(); }
  } after{this};
  const Builtin *b = builtin(m_current);
  if (!b && !m_builtins.empty())
    b = &m_builtins.front();
  if (b) {
    applyBuiltin(*b);
    clampToScreens();
  }
}

void WorkspaceManager::persistLast() {
  m_lastTimer.stop();
  writeLast(); // always (window geometry too), unless nothing was captured
}

bool WorkspaceManager::restoreLast() {
  flushLast();
  const struct Guard {
    bool &f;
    explicit Guard(bool &flag) : f(flag) { f = true; }
    ~Guard() { f = false; }
  } guard(m_applying);
  Stored s;
  QString why;
  if (!readStored(kLast, s))
    why = tr("nothing saved");
  else if (s.format != kFormat)
    why = tr("older or newer format");
  else if (s.backend != m_backend->tag())
    why = tr("other backend");
  else if (s.state.isEmpty())
    why = tr("empty");
  else if (hasUnknownPanel(s.state))
    why = tr("unknown panel");
  if (why.isEmpty()) {
    if (m_backend->restoreState(s.state)) {
      // Geometry only after the state loaded (see load()).
      if (m_host && !s.geometry.isEmpty())
        m_host->restoreGeometry(s.geometry);
      // Keep the name only if it still exists, so the menu tick is honest.
      setCurrent(exists(s.name) ? s.name : QString());
      m_lastBlob = s.state;
      m_dirty = false;
      clampToScreens();
      return true;
    }
    why = tr("corrupt layout");
  }
  if (why != tr("nothing saved"))
    emit loadFailed(QStringLiteral("last"), why);
  if (!m_builtins.empty())
    applyBuiltin(m_builtins.front());
  clampToScreens();
  return false;
}

} // namespace ws
