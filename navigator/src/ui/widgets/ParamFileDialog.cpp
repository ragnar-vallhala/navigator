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
#include "ParamFileDialog.h"

#include <QDialogButtonBox>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>

namespace params {

namespace {
constexpr int kColId = 0, kColGroup = 1, kColVehicle = 2, kColFile = 3, kColumns = 4;
} // namespace

ParamFileDialog::ParamFileDialog(const Catalog &cat, const ParamFile::Loaded &file,
                                 const QVector<ParamFile::DiffRow> &diff, const QString &refuseWhy,
                                 QWidget *parent)
    : QDialog(parent), m_refuseWhy(refuseWhy), m_hasProblems(!file.ok()) {
  setObjectName(QStringLiteral("paramsFileDialog"));
  setWindowTitle(tr("Parameter file"));
  resize(620, 460);
  auto *top = new QVBoxLayout(this);

  QString meta;
  if (!file.meta.savedAt.isEmpty())
    meta += tr("Saved %1").arg(file.meta.savedAt);
  if (file.meta.sysid >= 0)
    meta += (meta.isEmpty() ? QString() : QStringLiteral(", ")) + tr("vehicle %1").arg(file.meta.sysid);
  if (!meta.isEmpty())
    top->addWidget(new QLabel(meta, this));

  if (!file.ok()) {
    QStringList lines;
    for (const auto &p : file.problems)
      lines << (p.id.isEmpty() ? p.text : QStringLiteral("%1: %2").arg(p.id, p.text));
    auto *l = new QLabel(tr("This file cannot be applied:\n%1").arg(lines.join(QLatin1Char('\n'))), this);
    l->setObjectName(QStringLiteral("paramsFileProblems"));
    l->setWordWrap(true);
    l->setStyleSheet(QStringLiteral("QLabel { font-weight: bold; }"));
    top->addWidget(l);
  }
  if (!refuseWhy.isEmpty()) {
    auto *l = new QLabel(tr("Apply is off: %1").arg(refuseWhy), this);
    l->setObjectName(QStringLiteral("paramsFileRefusal"));
    l->setWordWrap(true);
    top->addWidget(l);
  }

  m_staleLabel = new QLabel(this);
  m_staleLabel->setObjectName(QStringLiteral("paramsFileStale"));
  m_staleLabel->setWordWrap(true);
  m_staleLabel->setStyleSheet(QStringLiteral("QLabel { font-weight: bold; }"));
  m_staleLabel->setVisible(false);
  top->addWidget(m_staleLabel);

  m_table = new QTableWidget(int(diff.size()), kColumns, this);
  m_table->setObjectName(QStringLiteral("paramsFileTable"));
  m_table->setHorizontalHeaderLabels({tr("Parameter"), tr("Group"), tr("Vehicle"), tr("File")});
  m_table->verticalHeader()->setVisible(false);
  m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  m_table->setSelectionMode(QAbstractItemView::NoSelection);
  m_table->horizontalHeader()->setSectionResizeMode(kColId, QHeaderView::Stretch);
  for (int i = 0; i < diff.size(); ++i) {
    const ParamFile::DiffRow &d = diff[i];
    const Entry *e = cat.byId(d.id);
    const Group *g = cat.group(d.group);
    auto *id = new QTableWidgetItem(d.id);
    id->setData(Qt::UserRole, d.file);
    if (d.same || !d.haveVehicle) {
      // Nothing to change (or nothing to compare with: not applied by default).
      id->setFlags(Qt::ItemIsEnabled | (d.same ? Qt::NoItemFlags : Qt::ItemIsUserCheckable));
      if (!d.same)
        id->setCheckState(Qt::Unchecked);
    } else {
      id->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
      id->setCheckState(Qt::Checked);
    }
    m_table->setItem(i, kColId, id);
    m_table->setItem(i, kColGroup, new QTableWidgetItem(g ? g->name : QString::number(d.group)));
    m_table->setItem(i, kColVehicle,
                     new QTableWidgetItem(d.haveVehicle && e ? ParamFile::valueText(*e, d.vehicle)
                                                              : tr("(not read)")));
    m_table->setItem(i, kColFile,
                     new QTableWidgetItem(e ? ParamFile::valueText(*e, d.file) : QString::number(d.file)));
    if (d.same)
      for (int c = 0; c < kColumns; ++c)
        if (auto *it = m_table->item(i, c))
          it->setForeground(palette().color(QPalette::Disabled, QPalette::Text));
  }
  top->addWidget(m_table, 1);

  m_count = new QLabel(this);
  m_count->setObjectName(QStringLiteral("paramsFileCount"));
  top->addWidget(m_count);

  m_buttons = new QDialogButtonBox(QDialogButtonBox::Close, this);
  auto *apply = m_buttons->addButton(tr("Apply"), QDialogButtonBox::AcceptRole);
  apply->setObjectName(QStringLiteral("paramsFileApply"));
  top->addWidget(m_buttons);
  connect(m_buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
  connect(apply, &QPushButton::clicked, this, [this] {
    if (!applyEnabled())
      return;
    emit applyRequested(ticked());
    accept();
  });
  connect(m_table, &QTableWidget::itemChanged, this, [this] { updateApply(); });
  updateApply();
}

QVector<ParamApplier::Item> ParamFileDialog::ticked() const {
  QVector<ParamApplier::Item> out;
  for (int i = 0; i < m_table->rowCount(); ++i) {
    const QTableWidgetItem *it = m_table->item(i, kColId);
    if (it && it->checkState() == Qt::Checked && (it->flags() & Qt::ItemIsUserCheckable))
      out.push_back({it->text(), it->data(Qt::UserRole).toDouble()});
  }
  return out;
}

bool ParamFileDialog::applyEnabled() const {
  return !m_hasProblems && m_refuseWhy.isEmpty() && !ticked().isEmpty();
}

void ParamFileDialog::markStale(const QString &why) {
  if (m_stale)
    return;
  m_stale = true;
  m_refuseWhy = why;
  m_staleLabel->setText(tr("Apply is off: %1. Close this and load the file again.").arg(why));
  m_staleLabel->setVisible(true);
  updateApply();
}

void ParamFileDialog::updateApply() {
  const int n = int(ticked().size());
  m_count->setText(tr("%1 of %2 parameters will change").arg(n).arg(m_table->rowCount()));
  auto *apply = m_buttons->findChild<QPushButton *>(QStringLiteral("paramsFileApply"));
  if (!apply)
    return;
  apply->setEnabled(applyEnabled());
  apply->setToolTip(m_hasProblems          ? tr("The file has problems")
                    : !m_refuseWhy.isEmpty() ? m_refuseWhy
                    : n == 0               ? tr("Nothing is ticked")
                                           : tr("Send the ticked values to the vehicle"));
}

} // namespace params
