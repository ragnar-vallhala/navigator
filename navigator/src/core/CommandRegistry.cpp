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
#include "CommandRegistry.h"

#include <QDebug>

CommandRegistry::CommandRegistry(QObject *parent) : QObject(parent) {}

QAction *CommandRegistry::add(const QString &id, const QString &title,
                              const QString &category,
                              const QKeySequence &defaultSeq, CmdContext when,
                              std::function<void()> onTrigger) {
  if (auto it = m_byId.constFind(id); it != m_byId.constEnd()) {
    qWarning() << "CommandRegistry: duplicate command id" << id
               << "- ignoring re-registration";
    return it->action;
  }

  auto *act = new QAction(title, this);
  act->setObjectName(id); // lets the editor/palette key off the id
  if (!defaultSeq.isEmpty())
    act->setShortcut(defaultSeq);
  if (onTrigger)
    QObject::connect(act, &QAction::triggered, this,
                     [fn = std::move(onTrigger)](bool) { fn(); });

  m_byId.insert(id, Command{id, title, category, defaultSeq, when, act});
  m_order.append(id);
  return act;
}

QAction *CommandRegistry::action(const QString &id) const {
  auto it = m_byId.constFind(id);
  return it == m_byId.constEnd() ? nullptr : it->action;
}

const Command *CommandRegistry::command(const QString &id) const {
  auto it = m_byId.constFind(id);
  return it == m_byId.constEnd() ? nullptr : &it.value();
}

QList<Command> CommandRegistry::all() const {
  QList<Command> out;
  out.reserve(m_order.size());
  for (const QString &id : m_order)
    out.append(m_byId.value(id));
  return out;
}

QStringList CommandRegistry::ids() const { return m_order; }
