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
    : QObject(parent), m_host(host), m_backend(std::move(backend)) {}

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
                                                    : QString();
  if (bad.isEmpty()) {
    if (m_host && !s.geometry.isEmpty())
      m_host->restoreGeometry(s.geometry);
    if (m_backend->restoreState(s.state)) {
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
  const Builtin *b = builtin(m_current);
  if (!b && !m_builtins.empty())
    b = &m_builtins.front();
  if (b) {
    applyBuiltin(*b);
    clampToScreens();
  }
}

void WorkspaceManager::persistLast() {
  Stored s;
  s.name = m_current;
  s.format = kFormat;
  s.backend = m_backend->tag();
  s.state = m_backend->saveState();
  s.geometry = m_host ? m_host->saveGeometry() : QByteArray();
  writeStored(kLast, s);
}

bool WorkspaceManager::restoreLast() {
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
  if (why.isEmpty()) {
    if (m_host && !s.geometry.isEmpty())
      m_host->restoreGeometry(s.geometry);
    if (m_backend->restoreState(s.state)) {
      // Keep the name only if it still exists, so the menu tick is honest.
      setCurrent(exists(s.name) ? s.name : QString());
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
