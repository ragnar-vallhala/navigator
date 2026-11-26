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
#include "ParamsPanel.h"
#include "ParamFileDialog.h"
#include "ParamService.h"

#include <QDateTime>
#include <QDoubleValidator>
#include <QFileInfo>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QLocale>
#include <QPushButton>
#include <QTreeWidget>
#include <QVBoxLayout>

namespace params {

namespace {
constexpr int kColName = 0, kColValue = 1, kColVehicle = 2, kColUnit = 3,
              kColDefault = 4, kColState = 5, kColAction = 6, kColumns = 7;
constexpr int kRoleEntry = Qt::UserRole;      // param item: catalog index
constexpr int kRoleGroup = Qt::UserRole + 1;  // group item: group id (0 = Live)
constexpr int kMaxOutcomes = 200;

// Plain notation, never an exponent, so what is shown can be typed back into
// the editor as it stands: 0.00005, not 5e-05. The C locale: a value is not
// shown with a comma in one place and typed with a point in another.
QString numText(const Entry &e, double v) {
  if (e.type != Type::F32)
    return QLocale::c().toString(v, 'f', 0).remove(QLatin1Char(','));
  QString s = QLocale::c().toString(double(float(v)), 'f', 9);
  s.remove(QLatin1Char(','));
  if (s.contains(QLatin1Char('.'))) {
    while (s.endsWith(QLatin1Char('0')))
      s.chop(1);
    if (s.endsWith(QLatin1Char('.')))
      s.chop(1);
  }
  return s == QLatin1String("-0") ? QStringLiteral("0") : s;
}
} // namespace

QString outcomeLine(const Client::WriteEvent &e, const Catalog &cat) {
  QString what;
  if (e.op == Client::Op::Set) {
    what = e.id;
  } else {
    const QString verb = e.op == Client::Op::Commit ? QStringLiteral("commit") : QStringLiteral("discard");
    const Group *g = e.group ? cat.group(e.group) : nullptr;
    what = e.group == 0 ? QStringLiteral("%1 all").arg(verb)
                        : QStringLiteral("%1 %2").arg(verb, g ? g->name : QString::number(e.group));
  }
  QString word;
  switch (e.outcome) {
  case Client::Outcome::Applied: word = QStringLiteral("done"); break;
  case Client::Outcome::Refused: word = QStringLiteral("refused"); break;
  case Client::Outcome::Rejected: word = QStringLiteral("not applied"); break;
  case Client::Outcome::Failed: word = QStringLiteral("failed"); break;
  case Client::Outcome::TimedOut: word = QStringLiteral("no answer"); break;
  case Client::Outcome::Unknown: word = QStringLiteral("outcome unknown"); break;
  }
  return QStringLiteral("%1: %2 (%3)").arg(what, word, e.text);
}

// ------------------------------------------------------------------ delegate
ParamValueDelegate::ParamValueDelegate(const Catalog *cat, Submit submit, QObject *parent)
    : QStyledItemDelegate(parent), m_cat(cat), m_submit(std::move(submit)) {}

QWidget *ParamValueDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &,
                                          const QModelIndex &index) const {
  const int entry = index.data(kRoleEntry).toInt();
  const Entry *e = m_cat && entry >= 0 && entry < m_cat->count() ? &m_cat->entries()[entry] : nullptr;
  if (!e || (e->flags & ReadOnly))
    return nullptr;
  auto *ed = new QLineEdit(parent);
  ed->setLocale(QLocale::c());
  auto *v = new QDoubleValidator(e->min, e->max, e->type == Type::F32 ? 9 : 0, ed);
  v->setNotation(QDoubleValidator::StandardNotation);
  v->setLocale(QLocale::c());
  ed->setValidator(v);
  return ed;
}

void ParamValueDelegate::setModelData(QWidget *editor, QAbstractItemModel *,
                                      const QModelIndex &index) const {
  auto *ed = qobject_cast<QLineEdit *>(editor);
  if (!ed || !ed->hasAcceptableInput())
    return; // out of range or not a number: nothing is sent, the cell reverts
  if (m_submit)
    m_submit(index.data(kRoleEntry).toInt(), ed->text());
}

// --------------------------------------------------------------------- panel
ParamsPanel::ParamsPanel(ParamService *svc, QWidget *parent) : QWidget(parent), m_svc(svc) {
  setObjectName(QStringLiteral("ParamsPanel"));
  auto *top = new QVBoxLayout(this);
  top->setContentsMargins(8, 8, 8, 8);

  auto *bar = new QHBoxLayout;
  m_banner = new QLabel(this);
  m_banner->setWordWrap(true);
  m_banner->setObjectName(QStringLiteral("paramsBanner"));
  m_refresh = new QPushButton(tr("Refresh"), this);
  m_refresh->setObjectName(QStringLiteral("paramsRefresh"));
  m_refresh->setToolTip(tr("Read the whole parameter table from the vehicle again"));
  m_saveFile = new QPushButton(tr("Save to file…"), this);
  m_saveFile->setObjectName(QStringLiteral("paramsSaveFile"));
  m_loadFile = new QPushButton(tr("Load from file…"), this);
  m_loadFile->setObjectName(QStringLiteral("paramsLoadFile"));
  bar->addWidget(m_banner, 1);
  bar->addWidget(m_refresh);
  bar->addWidget(m_saveFile);
  bar->addWidget(m_loadFile);
  top->addLayout(bar);

  m_result = new QLabel(this);
  m_result->setObjectName(QStringLiteral("paramsFileResult"));
  m_result->setWordWrap(true);
  m_result->setVisible(false);
  top->addWidget(m_result);
  m_resultHide.setSingleShot(true);
  m_resultHide.setInterval(10000);
  connect(&m_resultHide, &QTimer::timeout, m_result, &QLabel::hide);
  m_applier = new ParamApplier(m_svc, this);
  connect(m_applier, &ParamApplier::finished, this, &ParamsPanel::onApplied);
  connect(m_saveFile, &QPushButton::clicked, this, &ParamsPanel::chooseSave);
  connect(m_loadFile, &QPushButton::clicked, this, &ParamsPanel::chooseLoad);

  m_filter = new QLineEdit(this);
  m_filter->setPlaceholderText(tr("Filter by name or description"));
  m_filter->setClearButtonEnabled(true);
  top->addWidget(m_filter);

  m_tree = new QTreeWidget(this);
  m_tree->setObjectName(QStringLiteral("paramsTree"));
  m_tree->setColumnCount(kColumns);
  m_tree->setHeaderLabels({tr("Parameter"), tr("Value"), tr("Vehicle"), tr("Unit"), tr("Default"),
                           tr("State"), QString()});
  m_tree->setRootIsDecorated(true);
  m_tree->setUniformRowHeights(false); // the group rows carry buttons and are taller
  m_tree->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::EditKeyPressed |
                          QAbstractItemView::SelectedClicked);
  m_tree->header()->setSectionResizeMode(kColName, QHeaderView::ResizeToContents);
  m_tree->header()->setStretchLastSection(false);
  m_tree->header()->setSectionResizeMode(kColState, QHeaderView::Stretch);
  m_tree->setItemDelegateForColumn(
      kColValue, new ParamValueDelegate(
                     &m_svc->catalog(),
                     [this](int entry, const QString &text) { submitEdit(entry, text); }, this));
  top->addWidget(m_tree, 1);

  m_outcomes = new QListWidget(this);
  m_outcomes->setObjectName(QStringLiteral("paramsOutcomes"));
  m_outcomes->setMaximumHeight(110);
  top->addWidget(m_outcomes);

  m_coalesce.setSingleShot(true);
  m_coalesce.setInterval(0);
  connect(&m_coalesce, &QTimer::timeout, this, &ParamsPanel::refreshNow);
  connect(m_svc, &ParamService::changed, this, &ParamsPanel::scheduleRefresh);
  connect(m_svc, &ParamService::writeFinished, this, &ParamsPanel::onWrite);
  connect(m_refresh, &QPushButton::clicked, this, [this] { m_svc->refresh(); });
  connect(m_filter, &QLineEdit::textChanged, this, &ParamsPanel::setFilter);
  refreshNow();
}

void ParamsPanel::scheduleRefresh() {
  if (!m_coalesce.isActive())
    m_coalesce.start();
}

void ParamsPanel::build() {
  m_tree->clear();
  m_groupItems.clear();
  m_items.clear();
  m_commitBtn.clear();
  m_discardBtn.clear();
  m_saveAll = nullptr;
  const Catalog &cat = m_svc->catalog();
  m_builtCount = cat.count();
  // "Live" (group 0) first, then the catalog's groups in id order.
  int actionW = 0;
  QVector<int> gids{0};
  for (const Group &g : cat.groups())
    gids.push_back(g.id);
  for (int gid : gids) {
    const bool live = gid == 0;
    const Group *g = live ? nullptr : cat.group(gid);
    auto *gi = new QTreeWidgetItem(m_tree);
    gi->setText(kColName, live ? tr("Live") : g->name);
    gi->setData(kColName, kRoleGroup, gid);
    gi->setFirstColumnSpanned(false);
    gi->setToolTip(kColName, live ? tr("Values that apply at once; \"Save all\" stores them")
                                  : (g ? g->doc : QString()));
    QFont f = gi->font(kColName);
    f.setBold(true);
    gi->setFont(kColName, f);
    m_groupItems.insert(gid, gi);
    auto *box = new QWidget;
    auto *bl = new QHBoxLayout(box);
    bl->setContentsMargins(2, 0, 2, 0);
    if (live) {
      m_saveAll = new QPushButton(tr("Save all"), box);
      m_saveAll->setObjectName(QStringLiteral("paramsSaveAll"));
      connect(m_saveAll, &QPushButton::clicked, this, [this] {
        QString why;
        if (!m_svc->commit(0, &why))
          addOutcome(tr("save all: not sent (%1)").arg(why));
      });
      bl->addWidget(m_saveAll);
    } else {
      auto *c = new QPushButton(tr("Commit"), box);
      auto *d = new QPushButton(tr("Discard"), box);
      c->setObjectName(QStringLiteral("paramsCommit%1").arg(gid));
      d->setObjectName(QStringLiteral("paramsDiscard%1").arg(gid));
      connect(c, &QPushButton::clicked, this, [this, gid] {
        QString why;
        if (!m_svc->commit(gid, &why))
          addOutcome(tr("commit: not sent (%1)").arg(why));
      });
      connect(d, &QPushButton::clicked, this, [this, gid] {
        QString why;
        if (!m_svc->discard(gid, &why))
          addOutcome(tr("discard: not sent (%1)").arg(why));
      });
      bl->addWidget(c);
      bl->addWidget(d);
      m_commitBtn.insert(gid, c);
      m_discardBtn.insert(gid, d);
    }
    m_tree->setItemWidget(gi, kColAction, box);
    // The buttons keep their own size: the row is as tall as they are and the
    // action column as wide as the widest set of them.
    const QSize bs = box->sizeHint();
    gi->setSizeHint(kColAction, bs);
    actionW = qMax(actionW, bs.width());
  }
  for (const Entry &e : cat.entries()) {
    QTreeWidgetItem *parent = m_groupItems.value(e.group);
    if (!parent)
      continue;
    auto *it = new QTreeWidgetItem(parent);
    it->setData(kColValue, kRoleEntry, e.index);
    it->setText(kColName, e.id);
    it->setText(kColUnit, e.unit);
    it->setText(kColDefault, numText(e, e.def));
    QString tip = e.doc;
    tip += tr("\nRange %1 to %2").arg(numText(e, e.min), numText(e, e.max));
    if (!e.rangeWhy.isEmpty())
      tip += QStringLiteral(" (%1)").arg(e.rangeWhy);
    it->setToolTip(kColName, tip);
    it->setToolTip(kColValue, tip);
    m_items.insert(e.index, it);
  }
  m_tree->header()->setSectionResizeMode(kColAction, QHeaderView::Fixed);
  m_tree->setColumnWidth(kColAction, actionW + 8);
  m_tree->expandAll();
  m_built = true;
}

QString ParamsPanel::badgesFor(const Entry &e, bool *editable) const {
  const Client &c = m_svc->client();
  const Client::Value *v = c.value(e.id);
  QStringList b;
  bool ed = c.state() == Client::State::Ready && v && v->have && !(e.flags & ReadOnly);
  if (e.flags & ReadOnly)
    b << tr("read-only");
  if ((e.flags & LockedWhenArmed)) {
    if (m_svc->armed()) {
      b << tr("locked: armed");
      ed = false;
    } else {
      b << tr("locked when armed");
    }
  }
  if (e.flags & RebootRequired)
    b << tr("needs reboot");
  if (v && v->have) {
    if (v->flags & Staged)
      b << tr("staged");
    if (v->flags & Unsaved)
      b << tr("unsaved");
    if (v->flags & NonDefault)
      b << tr("changed");
    if (v->flags & RebootPending)
      b << tr("reboot pending");
  }
  if (editable)
    *editable = ed;
  return b.join(QStringLiteral(", "));
}

void ParamsPanel::refreshNow() {
  if (!m_built || m_builtCount != m_svc->catalog().count())
    build();
  if (m_dialog) {
    const QString stale = whyStale();
    if (!stale.isEmpty())
      static_cast<ParamFileDialog *>(m_dialog.data())->markStale(stale);
  }
  const Client &c = m_svc->client();
  m_banner->setText(c.stateText());
  const Client::State st = c.state();
  const bool ok = st == Client::State::Ready;
  m_banner->setProperty("ok", ok);
  m_banner->setStyleSheet(ok ? QString() : QStringLiteral("QLabel#paramsBanner { font-weight: bold; }"));
  m_refresh->setVisible(st == Client::State::Mismatch || st == Client::State::Incomplete ||
                        st == Client::State::NoAnswer);
  m_refresh->setEnabled(true);
  {
    const QString why = !ok ? tr("The parameter table is not ready")
                            : (m_svc->locked() || m_applier->busy()) ? tr("A change is being applied")
                                                                      : QString();
    m_saveFile->setEnabled(why.isEmpty());
    m_saveFile->setToolTip(why.isEmpty() ? tr("Save the tuning parameters to a file") : why);
    m_loadFile->setEnabled(why.isEmpty());
    m_loadFile->setToolTip(why.isEmpty() ? tr("Compare a saved file with the vehicle, then apply it") : why);
  }

  const Catalog &cat = m_svc->catalog();
  QHash<int, int> staged, unsaved;
  for (const Entry &e : cat.entries()) {
    QTreeWidgetItem *it = m_items.value(e.index);
    if (!it)
      continue;
    const Client::Value *v = c.value(e.id);
    bool editable = false;
    const QString badges = badgesFor(e, &editable);
    // Mismatch / Incomplete show what the vehicle sent, raw, read-only.
    QString vehicle;
    QString shown;
    if (v && v->have) {
      shown = numText(e, v->v);
    } else if (c.raw().contains(e.index)) {
      double d = 0;
      if (ParamCodec::valueOf(c.raw().value(e.index), d))
        shown = numText(e, d);
    }
    if (m_pending.contains(e.id)) {
      vehicle = shown; // the vehicle's value, next to the edit that is on its way
      shown = m_typed.value(e.id, shown);
    }
    // Do not overwrite a cell the user is typing into.
    const bool editing = m_tree->indexWidget(m_tree->indexFromItem(it, kColValue)) != nullptr;
    if (!editing)
      it->setText(kColValue, shown);
    it->setText(kColVehicle, vehicle);
    it->setText(kColState, badges);
    Qt::ItemFlags f = it->flags();
    f.setFlag(Qt::ItemIsEditable, false);
    it->setFlags(f);
    if (editable && !m_pending.contains(e.id)) {
      f.setFlag(Qt::ItemIsEditable, true);
      it->setFlags(f);
    }
    if (v && v->have) {
      if (v->flags & Staged)
        ++staged[e.group];
      if (v->flags & Unsaved)
        ++unsaved[e.group];
    }
  }
  for (auto gi = m_groupItems.begin(); gi != m_groupItems.end(); ++gi) {
    const int gid = gi.key();
    QStringList s;
    if (staged.value(gid))
      s << tr("%1 staged").arg(staged.value(gid));
    if (unsaved.value(gid))
      s << tr("%1 unsaved").arg(unsaved.value(gid));
    gi.value()->setText(kColState, s.join(QStringLiteral(", ")));
  }
  for (auto it = m_commitBtn.begin(); it != m_commitBtn.end(); ++it) {
    const QString why = m_svc->whyNotCommit(it.key());
    it.value()->setEnabled(why.isEmpty());
    it.value()->setToolTip(why.isEmpty() ? tr("Apply and save the staged values of this group") : why);
  }
  for (auto it = m_discardBtn.begin(); it != m_discardBtn.end(); ++it) {
    const QString why = m_svc->whyNotDiscard(it.key());
    it.value()->setEnabled(why.isEmpty());
    it.value()->setToolTip(why.isEmpty() ? tr("Drop the staged values of this group") : why);
  }
  if (m_saveAll) {
    int any = 0;
    for (int n : unsaved)
      any += n;
    for (int n : staged)
      any += n;
    const QString why = m_svc->whyNotCommit(0);
    m_saveAll->setVisible(any > 0);
    m_saveAll->setEnabled(why.isEmpty());
    m_saveAll->setToolTip(why.isEmpty() ? tr("Apply every staged group and save every unsaved value") : why);
  }
  setFilter(m_filter->text());
}

void ParamsPanel::setFilter(const QString &text) {
  const QString f = text.trimmed();
  const Catalog &cat = m_svc->catalog();
  for (const Entry &e : cat.entries()) {
    QTreeWidgetItem *it = m_items.value(e.index);
    if (it)
      it->setHidden(!f.isEmpty() && !e.id.contains(f, Qt::CaseInsensitive) &&
                    !e.doc.contains(f, Qt::CaseInsensitive));
  }
  for (QTreeWidgetItem *gi : std::as_const(m_groupItems)) {
    bool any = false;
    for (int i = 0; i < gi->childCount(); ++i)
      any = any || !gi->child(i)->isHidden();
    gi->setHidden(!f.isEmpty() && !any);
  }
}

void ParamsPanel::submitEdit(int entry, const QString &text) {
  const Catalog &cat = m_svc->catalog();
  if (entry < 0 || entry >= cat.count())
    return;
  const Entry &e = cat.entries()[entry];
  bool ok = false;
  const double v = QLocale::c().toDouble(text.trimmed(), &ok);
  if (!ok) {
    addOutcome(tr("%1: \"%2\" is not a number").arg(e.id, text));
    return;
  }
  QString why;
  if (!m_svc->set(e.id, v, &why)) { // the range, type, lock and state are checked again here
    addOutcome(tr("%1: not sent (%2)").arg(e.id, why));
    return;
  }
  m_pending.insert(e.id);
  m_typed.insert(e.id, numText(e, v));
  scheduleRefresh();
}

QHash<QString, ParamFile::VehicleValue> ParamsPanel::vehicleValues() const {
  QHash<QString, ParamFile::VehicleValue> h;
  for (const Entry &e : m_svc->catalog().entries()) {
    const Client::Value *v = m_svc->client().value(e.id);
    if (v && v->have)
      h.insert(e.id, {v->v, v->flags, true});
  }
  return h;
}

void ParamsPanel::showResult(const QString &text) {
  m_result->setText(text);
  m_result->setVisible(true);
  m_resultHide.start();
  addOutcome(text);
}

void ParamsPanel::chooseSave() {
  const QString name = QStringLiteral("params_%1.json")
                           .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss")));
  const QString path = QFileDialog::getSaveFileName(this, tr("Save parameters"), name,
                                                    tr("Parameter files (*.json)"));
  if (!path.isEmpty())
    saveFile(path);
}

void ParamsPanel::chooseLoad() {
  const QString path = QFileDialog::getOpenFileName(this, tr("Load parameters"), QString(),
                                                    tr("Parameter files (*.json)"));
  if (!path.isEmpty())
    loadFile(path);
}

bool ParamsPanel::saveFile(const QString &path) {
  QByteArray data;
  QString why;
  if (m_svc->client().state() != Client::State::Ready) {
    showResult(tr("save to file: not saved (the parameter table is not ready)"));
    return false;
  }
  if (!ParamFile::save(m_svc->catalog(), vehicleValues(), -1, QDateTime::currentDateTimeUtc(), &data,
                       &why) ||
      !ParamFile::writeFile(path, data, &why)) {
    showResult(tr("save to file: not saved (%1)").arg(why));
    return false;
  }
  showResult(tr("saved the parameters to %1").arg(QFileInfo(path).fileName()));
  return true;
}

QString ParamsPanel::whyStale() const {
  const auto now = vehicleValues();
  for (const QString &id : m_snapshotIds) {
    const auto a = m_snapshot.constFind(id);
    const auto b = now.constFind(id);
    const bool ha = a != m_snapshot.constEnd(), hb = b != now.constEnd();
    if (ha != hb || (ha && (a->v != b->v || a->flags != b->flags)))
      return tr("%1 changed on the vehicle after the file was compared").arg(id);
  }
  return QString();
}

QString ParamsPanel::whyNotApply(const ParamFile::Loaded &f) const {
  if (!f.ok())
    return tr("the file has problems");
  QString why;
  if (!ParamFile::sameTable(f, m_svc->catalog(), &why))
    return why;
  if (m_svc->client().state() != Client::State::Ready)
    return tr("the parameter table is not ready");
  if (m_svc->armed())
    return tr("the vehicle is armed (or its state is not known)");
  if (m_svc->locked() || m_applier->busy())
    return tr("a change is being applied");
  return QString();
}

void ParamsPanel::loadFile(const QString &path) {
  ParamFile::Loaded f;
  QString why;
  if (!ParamFile::readFile(path, m_svc->catalog(), &f, &why)) {
    showResult(tr("load from file: %1").arg(why));
    return;
  }
  if (m_dialog)
    m_dialog->deleteLater();
  m_snapshot = vehicleValues();
  m_snapshotIds.clear();
  for (const ParamFile::Row &r : f.rows)
    m_snapshotIds << r.id;
  auto *dlg = new ParamFileDialog(m_svc->catalog(), f,
                                  ParamFile::diff(f, m_svc->catalog(), m_snapshot),
                                  whyNotApply(f), this);
  dlg->setAttribute(Qt::WA_DeleteOnClose);
  connect(dlg, &ParamFileDialog::applyRequested, this,
          [this, f](const QVector<ParamApplier::Item> &items) {
            // The state may have moved while the dialog was open: ask again.
            QString why = whyNotApply(f);
            if (why.isEmpty())
              why = whyStale();
            if (why.isEmpty())
              m_applier->apply(items, &why);
            if (!why.isEmpty())
              showResult(tr("load from file: not applied (%1)").arg(why));
          });
  m_dialog = dlg;
  dlg->open();
}

void ParamsPanel::onApplied(const ParamApplier::Result &r) {
  showResult(r.ok ? tr("loaded from file: %1").arg(r.text)
                  : tr("load from file: not applied (%1)").arg(r.text));
  scheduleRefresh();
}

void ParamsPanel::onWrite(const Client::WriteEvent &e) {
  if (e.op == Client::Op::Set) {
    m_pending.remove(e.id);
    m_typed.remove(e.id);
  }
  addOutcome(outcomeLine(e, m_svc->catalog()));
  scheduleRefresh();
}

void ParamsPanel::addOutcome(const QString &line) {
  m_outcomes->insertItem(0, QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss  ")) + line);
  while (m_outcomes->count() > kMaxOutcomes)
    delete m_outcomes->takeItem(m_outcomes->count() - 1);
}

} // namespace params
