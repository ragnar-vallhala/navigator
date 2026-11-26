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

#include "ParamApplier.h"
#include "ParamFile.h"

#include <QDialog>

class QDialogButtonBox;
class QLabel;
class QTableWidget;

namespace params {

// What a parameter file would change, shown before anything is sent: one row
// per parameter of the file with the vehicle's value beside the file's, the
// rows that differ ticked, the rows that are the same left alone. Problems of
// the file are listed above the rows and block the apply, as does any reason
// the caller gives (another table, armed, not ready): the diff is still shown.
// The dialog sends nothing itself; Apply emits the ticked rows.
class ParamFileDialog : public QDialog {
  Q_OBJECT
public:
  ParamFileDialog(const Catalog &cat, const ParamFile::Loaded &file,
                  const QVector<ParamFile::DiffRow> &diff, const QString &refuseWhy,
                  QWidget *parent = nullptr);

  // The rows ticked now, with the file's values.
  QVector<ParamApplier::Item> ticked() const;
  bool applyEnabled() const;

signals:
  void applyRequested(const QVector<params::ParamApplier::Item> &items);

private:
  void updateApply();

  QString m_refuseWhy;
  bool m_hasProblems = false;
  QTableWidget *m_table = nullptr;
  QLabel *m_count = nullptr;
  QDialogButtonBox *m_buttons = nullptr;
};

} // namespace params
