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
#include "FaultsPanel.h"

#include "../../core/FaultsModel.h"
#include "../../core/Theme.h"

#include <QDateTime>
#include <QHBoxLayout>
#include <QLabel>
#include <QListView>
#include <QPushButton>
#include <QStyledItemDelegate>
#include <QVBoxLayout>

namespace {

// Severity-coloured row: "[ERROR] NAME: text   x3  12:03:44". Acknowledged
// rows are dimmed.
class FaultDelegate : public QStyledItemDelegate {
public:
  using QStyledItemDelegate::QStyledItemDelegate;

  void initStyleOption(QStyleOptionViewItem *opt,
                       const QModelIndex &ix) const override {
    QStyledItemDelegate::initStyleOption(opt, ix);
    const int sev = ix.data(FaultsModel::SeverityRole).toInt();
    const bool acked = ix.data(FaultsModel::AckedRole).toBool();
    QColor c = FaultsPanel::severityColor(sev);
    if (acked)
      c = Theme::kTextDim;
    opt->palette.setColor(QPalette::Text, c);
    opt->palette.setColor(QPalette::HighlightedText, c);

    QString line = QStringLiteral("[%1] %2")
                       .arg(FaultsPanel::severityLabel(sev), ix.data().toString());
    const int n = ix.data(FaultsModel::CountRole).toInt();
    if (n > 1)
      line += QStringLiteral("   x%1").arg(n);
    const qint64 last = ix.data(FaultsModel::LastSeenRole).toLongLong();
    line += QStringLiteral("   ") +
            QDateTime::fromMSecsSinceEpoch(last).toString(QStringLiteral("HH:mm:ss"));
    if (acked)
      line += QStringLiteral("   (ack)");
    opt->text = line;
  }
};

} // namespace

QColor FaultsPanel::severityColor(int severity) {
  if (severity <= FaultsModel::kErrorSeverity)
    return Theme::kDanger;
  if (severity == 4)
    return Theme::kWarn;
  return Theme::kAccent;
}

QString FaultsPanel::severityLabel(int severity) {
  static const char *const kNames[] = {"EMERGENCY", "ALERT", "CRITICAL",
                                       "ERROR",     "WARNING", "NOTICE",
                                       "INFO",      "DEBUG"};
  if (severity >= 0 && severity < 8)
    return QString::fromLatin1(kNames[severity]);
  return QString::number(severity);
}

FaultsPanel::FaultsPanel(QWidget *parent)
    : QWidget(parent), m_model(new FaultsModel(this)) {
  build();
}

FaultsPanel::FaultsPanel(FaultsModel *model, QWidget *parent)
    : QWidget(parent), m_model(model) {
  build();
}

void FaultsPanel::build() {
  setObjectName(QString::fromLatin1(kPanelId));

  auto *root = new QVBoxLayout(this);
  root->setContentsMargins(6, 6, 6, 6);

  auto *top = new QHBoxLayout;
  m_summary = new QLabel(this);
  m_summary->setObjectName(QStringLiteral("faultsSummary"));
  top->addWidget(m_summary, 1);
  m_ackBtn = new QPushButton(tr("Acknowledge"), this);
  m_ackBtn->setObjectName(QStringLiteral("faultsAck"));
  m_clearBtn = new QPushButton(tr("Clear"), this);
  m_clearBtn->setObjectName(QStringLiteral("faultsClear"));
  top->addWidget(m_ackBtn);
  top->addWidget(m_clearBtn);
  root->addLayout(top);

  m_view = new QListView(this);
  m_view->setObjectName(QStringLiteral("faultsList"));
  m_view->setModel(m_model);
  m_view->setItemDelegate(new FaultDelegate(m_view));
  m_view->setSelectionMode(QAbstractItemView::NoSelection);
  m_view->setEditTriggers(QAbstractItemView::NoEditTriggers);
  m_view->setUniformItemSizes(true);
  root->addWidget(m_view, 1);

  m_empty = new QLabel(tr("No active faults"), this);
  m_empty->setObjectName(QStringLiteral("faultsEmpty"));
  m_empty->setAlignment(Qt::AlignCenter);
  m_empty->setStyleSheet(
      QStringLiteral("color:%1;").arg(Theme::hex(Theme::kTextDim)));
  root->addWidget(m_empty, 1);

  connect(m_ackBtn, &QPushButton::clicked, m_model, &FaultsModel::acknowledgeAll);
  connect(m_clearBtn, &QPushButton::clicked, m_model, &FaultsModel::clear);
  connect(m_model, &FaultsModel::countsChanged, this,
          [this](int unacked, int serious) {
            refresh();
            emit countsChanged(unacked, serious);
          });
  connect(m_model, &QAbstractItemModel::rowsInserted, this, [this] { refresh(); });
  connect(m_model, &QAbstractItemModel::rowsRemoved, this, [this] { refresh(); });
  connect(m_model, &QAbstractItemModel::modelReset, this, [this] { refresh(); });
  refresh();
}

void FaultsPanel::refresh() {
  const int rows = m_model->rowCount();
  const int unacked = m_model->unackedCount();
  m_view->setVisible(rows > 0);
  m_empty->setVisible(rows == 0);
  m_ackBtn->setEnabled(unacked > 0);
  m_clearBtn->setEnabled(rows > 0);
  m_summary->setText(rows == 0 ? tr("No active faults")
                               : tr("%1 active, %2 unacknowledged")
                                     .arg(rows)
                                     .arg(unacked));
  m_summary->setStyleSheet(
      QStringLiteral("font-weight:bold; color:%1;").arg(Theme::hex(badgeColor())));
}

QString FaultsPanel::badgeText() const {
  const int n = m_model->unackedCount();
  return n > 0 ? QString::number(n) : QString();
}

QString FaultsPanel::title() const {
  const QString b = badgeText();
  return b.isEmpty() ? tr("Faults") : tr("Faults (%1)").arg(b);
}

QColor FaultsPanel::badgeColor() const {
  const int w = m_model->worstUnackedSeverity();
  return w < 0 ? Theme::kOk : severityColor(w);
}
