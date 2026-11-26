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
#include "ParamClient.h"
#include "ParamFile.h"

#include <QHash>
#include <QPointer>
#include <QSet>
#include <QStyledItemDelegate>
#include <QTimer>
#include <QWidget>

class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;
class QTreeWidget;
class QTreeWidgetItem;

namespace params {
class ParamService;

// What one write outcome says, in words, for the outcome list.
QString outcomeLine(const Client::WriteEvent &e, const Catalog &cat);

// The editor of the value column: a plain line edit whose validator holds the
// catalog range, so a non-numeric, non-finite or out-of-range entry is not
// accepted at all. The value is sent when the entry is accepted with Enter or
// by leaving the field; never while typing, never from a spin-box step.
class ParamValueDelegate : public QStyledItemDelegate {
  Q_OBJECT
public:
  using Submit = std::function<void(int entry, const QString &text)>;
  ParamValueDelegate(const Catalog *cat, Submit submit, QObject *parent = nullptr);
  QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &opt,
                        const QModelIndex &index) const override;
  void setModelData(QWidget *editor, QAbstractItemModel *model,
                    const QModelIndex &index) const override;

private:
  const Catalog *m_cat;
  Submit m_submit;
};

// The parameters of the vehicle: one row per parameter under its group, edited
// in place, with commit / discard per group, "Save all" for the live values, the
// state of the table in a banner, and every write outcome listed in words.
class ParamsPanel : public QWidget {
  Q_OBJECT
public:
  explicit ParamsPanel(ParamService *svc, QWidget *parent = nullptr);

  // The same path the editor takes (tests drive it directly).
  void submitEdit(int entry, const QString &text);
  void setFilter(const QString &text);
  // The file buttons' work, without the file chooser (tests drive them).
  // saveFile: false, with the outcome line added, when nothing was written.
  bool saveFile(const QString &path);
  // Reads the file and opens the diff dialog (nothing is sent until its Apply).
  void loadFile(const QString &path);
  // Why a loaded file may not be applied right now; empty when it may.
  QString whyNotApply(const ParamFile::Loaded &f) const;
  // Room for the rows and the group buttons; below this the dock scrolls.
  QSize minimumSizeHint() const override { return {360, 240}; }

private slots:
  void scheduleRefresh();
  void refreshNow();
  void onWrite(const params::Client::WriteEvent &e);
  void onApplied(const params::ParamApplier::Result &r);

private:
  QHash<QString, ParamFile::VehicleValue> vehicleValues() const;
  void chooseSave();
  void chooseLoad();
  void showResult(const QString &text);
  void build();
  void addOutcome(const QString &line);
  QString badgesFor(const Entry &e, bool *editable) const;

  ParamService *m_svc;
  QLabel *m_banner = nullptr;
  QPushButton *m_refresh = nullptr;
  QPushButton *m_saveFile = nullptr, *m_loadFile = nullptr;
  QLabel *m_result = nullptr;
  QTimer m_resultHide;
  ParamApplier *m_applier = nullptr;
  QPointer<QDialog> m_dialog;
  QLineEdit *m_filter = nullptr;
  QTreeWidget *m_tree = nullptr;
  QListWidget *m_outcomes = nullptr;
  QTimer m_coalesce;
  bool m_built = false;
  int m_builtCount = -1;
  QHash<int, QTreeWidgetItem *> m_groupItems; // group id -> item
  QHash<int, QTreeWidgetItem *> m_items;      // entry index -> item
  QHash<int, QPushButton *> m_commitBtn, m_discardBtn;
  QPushButton *m_saveAll = nullptr;
  QHash<QString, QString> m_typed; // id -> the text sent, while that write is pending
  QSet<QString> m_pending;

  friend class TstParamsPanel;
};

} // namespace params
