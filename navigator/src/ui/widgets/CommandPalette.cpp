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
#include "CommandPalette.h"

#include <algorithm>

#include <QAction>
#include <QEvent>
#include <QKeyEvent>
#include <QLineEdit>
#include <QListWidget>
#include <QVBoxLayout>

#include "CommandRegistry.h"

namespace {
constexpr int kIdRole = Qt::UserRole;
constexpr int kEnabledRole = Qt::UserRole + 1;
} // namespace

bool CommandPalette::fuzzyMatch(const QString &pattern, const QString &text,
                                int &score) {
  score = 0;
  if (pattern.isEmpty())
    return true;
  const QString p = pattern.toLower();
  const QString t = text.toLower();
  int ti = 0, lastMatch = -2;
  for (const QChar pc : p) {
    bool found = false;
    while (ti < t.size()) {
      if (t[ti] == pc) {
        found = true;
        break;
      }
      ++ti;
    }
    if (!found)
      return false;
    score += 1;
    if (ti == 0 || t[ti - 1] == ' ' || t[ti - 1] == '.')
      score += 5; // start-of-word
    if (ti == lastMatch + 1)
      score += 3; // contiguous run
    lastMatch = ti;
    ++ti;
  }
  return true;
}

CommandPalette::CommandPalette(CommandRegistry *registry, QWidget *parent)
    : QDialog(parent), m_registry(registry) {
  setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
  setWindowModality(Qt::ApplicationModal);
  resize(520, 320);

  auto *v = new QVBoxLayout(this);
  v->setContentsMargins(8, 8, 8, 8);
  m_input = new QLineEdit(this);
  m_input->setPlaceholderText(tr("Type a command…"));
  v->addWidget(m_input);
  m_list = new QListWidget(this);
  v->addWidget(m_list);

  connect(m_input, &QLineEdit::textChanged, this, &CommandPalette::rebuild);
  connect(m_list, &QListWidget::itemActivated, this, [this] { runSelected(); });
  // Arrows/Enter/Esc are handled while focus stays in the search box.
  m_input->installEventFilter(this);
  m_input->setFocus();

  rebuild(QString());
}

void CommandPalette::rebuild(const QString &query) {
  m_list->clear();
  struct Row {
    QString id, title, category;
    QKeySequence seq;
    bool enabled;
    int score;
  };
  QList<Row> rows;
  for (const Command &c : m_registry->all()) {
    const QString title = commandDisplayTitle(c.title); // no menu mnemonics
    int sTitle = 0, sCat = 0;
    const bool mTitle = fuzzyMatch(query, title, sTitle);
    const bool mCat = fuzzyMatch(query, c.category, sCat);
    if (!mTitle && !mCat)
      continue;
    const bool enabled = c.action ? c.action->isEnabled() : true;
    rows.append({c.id, title, c.category,
                 c.action ? c.action->shortcut() : QKeySequence(), enabled,
                 std::max(mTitle ? sTitle : -1, mCat ? sCat : -1)});
  }
  // Best score first; stable tiebreak on title so the list doesn't jitter.
  std::stable_sort(rows.begin(), rows.end(), [](const Row &a, const Row &b) {
    if (a.score != b.score)
      return a.score > b.score;
    return a.title < b.title;
  });

  for (const Row &r : rows) {
    QString label = r.title;
    if (!r.category.isEmpty())
      label += "    " + r.category;
    const QString sc = r.seq.toString(QKeySequence::NativeText);
    if (!sc.isEmpty())
      label += "    [" + sc + "]";
    auto *item = new QListWidgetItem(label, m_list);
    item->setData(kIdRole, r.id);
    item->setData(kEnabledRole, r.enabled);
    if (!r.enabled)
      item->setForeground(palette().brush(QPalette::Disabled, QPalette::Text));
  }
  if (m_list->count() > 0)
    m_list->setCurrentRow(0);
}

void CommandPalette::runSelected() {
  QListWidgetItem *item = m_list->currentItem();
  if (!item)
    return;
  if (!item->data(kEnabledRole).toBool())
    return; // disabled command: ignore
  const QString id = item->data(kIdRole).toString();
  QAction *a = m_registry->action(id);
  accept();
  if (a && a->isEnabled())
    a->trigger();
}

bool CommandPalette::eventFilter(QObject *obj, QEvent *event) {
  if (obj == m_input && event->type() == QEvent::KeyPress) {
    auto *ke = static_cast<QKeyEvent *>(event);
    switch (ke->key()) {
    case Qt::Key_Down:
      if (m_list->count())
        m_list->setCurrentRow(
            std::min(m_list->currentRow() + 1, m_list->count() - 1));
      return true;
    case Qt::Key_Up:
      if (m_list->count())
        m_list->setCurrentRow(std::max(m_list->currentRow() - 1, 0));
      return true;
    case Qt::Key_Return:
    case Qt::Key_Enter:
      runSelected();
      return true;
    case Qt::Key_Escape:
      reject();
      return true;
    default:
      break;
    }
  }
  return QDialog::eventFilter(obj, event);
}
