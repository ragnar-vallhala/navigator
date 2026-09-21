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

#include <QDialog>

class CommandRegistry;
class QLineEdit;
class QListWidget;

// Fuzzy command runner (Ctrl+Shift+P / FR-UX-20): a centred popup with a search
// box over CommandRegistry::all(). Typing fuzzy-filters and ranks; Up/Down move
// the selection, Enter triggers the selected command's QAction, Esc dismisses.
// Disabled commands (their `when`-context off) are shown greyed and not run.
class CommandPalette : public QDialog {
  Q_OBJECT

public:
  CommandPalette(CommandRegistry *registry, QWidget *parent = nullptr);

  // Case-insensitive subsequence match of `pattern` in `text`. Returns false
  // if `pattern` is not a subsequence; otherwise true and `score` rewards
  // start-of-word and contiguous matches (higher = better). Static + pure so
  // the ranking is unit-testable without a GUI.
  static bool fuzzyMatch(const QString &pattern, const QString &text,
                         int &score);

protected:
  bool eventFilter(QObject *obj, QEvent *event) override;

private:
  void rebuild(const QString &query);
  void runSelected();

  CommandRegistry *m_registry = nullptr;
  QLineEdit *m_input = nullptr;
  QListWidget *m_list = nullptr;
};
