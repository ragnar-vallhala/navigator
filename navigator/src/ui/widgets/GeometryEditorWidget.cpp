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
#include "GeometryEditorWidget.h"

#include "../../core/Theme.h"
#include "../../core/ui/Buttons.h"
#include "../../vsim/AirframePresets.h"
#include "../../vsim/MassProperties.h"
#include "../../vsim/MeshLoader.h"
#include "../../vsim/VehicleJson.h"
#include "CollapsibleSection.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QMatrix4x4>
#include <QVBoxLayout>

#include <cmath>

namespace {

QDoubleSpinBox *spin(double lo, double hi, int decimals, double step,
                     double val, const QString &suffix = QString()) {
  auto *s = new QDoubleSpinBox();
  s->setRange(lo, hi);
  s->setDecimals(decimals);
  s->setSingleStep(step);
  s->setValue(val);
  if (!suffix.isEmpty())
    s->setSuffix(suffix);
  s->setButtonSymbols(QAbstractSpinBox::UpDownArrows);
  return s;
}

} // namespace

GeometryEditorWidget::GeometryEditorWidget(QWidget *parent) : QWidget(parent) {
  buildUi();
  syncConfigToUi();
  showMassProps();
}

void GeometryEditorWidget::buildUi() {
  auto *root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);

  // -- Save / Load vehicle (portable JSON, cross-project) --
  {
    auto *row = new QHBoxLayout();
    auto *save = new ui::GhostButton(tr("Save Vehicle…"), this);
    save->setToolTip(tr("Export this vehicle (mesh path, transform, mass, "
                        "inertia, motors) to a portable .vveh file."));
    auto *load = new ui::GhostButton(tr("Load Vehicle…"), this);
    load->setToolTip(
        tr("Import a vehicle .vveh file, e.g. from another project."));
    row->addWidget(save);
    row->addWidget(load);
    row->addSpacing(12);
    row->addWidget(new QLabel(tr("Preset:"), this));
    presetCombo_ = new QComboBox(this);
    presetCombo_->setToolTip(
        tr("Load a built-in airframe (geometry; its tuned PID gains are sent "
           "to the firmware when the sim starts)."));
    presetCombo_->addItem(tr("Custom"), QString());
    for (const auto &p : vsim::airframePresets())
      presetCombo_->addItem(p.name, p.id);
    row->addWidget(presetCombo_);
    row->addStretch();
    root->addLayout(row);
    connect(save, &QPushButton::clicked, this,
            &GeometryEditorWidget::onSaveFile);
    connect(load, &QPushButton::clicked, this,
            &GeometryEditorWidget::onLoadFile);
    connect(presetCombo_, &QComboBox::activated, this,
            &GeometryEditorWidget::onPresetActivated);
  }

  // -- Mesh, Mass & Inertia --
  {
    auto *sec = new CollapsibleSection(tr("Mesh, Mass & Inertia"), this);
    auto *g = new QWidget();
    auto *v = new QVBoxLayout(g);
    v->setContentsMargins(0, 0, 0, 0);

    auto *meshRow = new QHBoxLayout();
    meshRow->addWidget(new QLabel(tr("Mesh:"), g));
    meshEdit_ = new QLineEdit(g);
    meshEdit_->setPlaceholderText(tr("STL / OBJ / PLY / glTF…"));
    meshRow->addWidget(meshEdit_, 1);
    auto *browse = new ui::GhostButton(tr("Browse"), g);
    meshRow->addWidget(browse);
    v->addLayout(meshRow);

    auto *numRow = new QHBoxLayout();
    numRow->addWidget(new QLabel(tr("Scale (m/unit):"), g));
    scaleSpin_ = spin(1e-4, 1e4, 4, 0.1, 1.0);
    numRow->addWidget(scaleSpin_);
    numRow->addSpacing(12);
    numRow->addWidget(new QLabel(tr("Mass:"), g));
    massSpin_ = spin(0.01, 100.0, 3, 0.05, 1.0, QStringLiteral(" kg"));
    numRow->addWidget(massSpin_);
    numRow->addStretch();
    auto *compute = new ui::PrimaryButton(tr("Load && Compute"), g);
    numRow->addWidget(compute);
    v->addLayout(numRow);

    // Body-frame placement of the mesh: applied (after scale) before mass
    // properties, so the inertia tensor follows the chosen orientation.
    auto triplet = [this, g, v](QDoubleSpinBox **x, QDoubleSpinBox **y,
                                QDoubleSpinBox **z, const QString &label,
                                double lo, double hi, int dec, double step,
                                const QString &sfx) {
      auto *row = new QHBoxLayout();
      row->addWidget(new QLabel(label, g));
      *x = spin(lo, hi, dec, step, 0, sfx);
      *y = spin(lo, hi, dec, step, 0, sfx);
      *z = spin(lo, hi, dec, step, 0, sfx);
      for (auto *s : {*x, *y, *z})
        s->setMaximumWidth(70);
      row->addWidget(*x);
      row->addWidget(*y);
      row->addWidget(*z);
      row->addStretch();
      v->addLayout(row);
    };
    triplet(&transX_, &transY_, &transZ_, tr("Translate:"), -100, 100, 3, 0.01,
            QStringLiteral(" m"));
    triplet(&rotX_, &rotY_, &rotZ_, tr("Rotate (°):"), -360, 360, 1, 1.0,
            QStringLiteral("°"));

    statusLbl_ = new QLabel(tr("No mesh loaded — using default box."), g);
    statusLbl_->setStyleSheet(
        QString("color:%1;").arg(Theme::hex(Theme::kTextMuted)));
    statusLbl_->setWordWrap(true);
    v->addWidget(statusLbl_);

    comLbl_ = new QLabel(g);
    comLbl_->setStyleSheet("font-family:monospace;");
    v->addWidget(comLbl_);

    inertiaLbl_ = new QLabel(g);
    inertiaLbl_->setStyleSheet(QString("font-family:monospace; color:%1;")
                                   .arg(Theme::hex(Theme::kAccent)));
    v->addWidget(inertiaLbl_);

    sec->setContentWidget(g);
    root->addWidget(sec);

    connect(browse, &QPushButton::clicked, this,
            &GeometryEditorWidget::onBrowse);
    connect(compute, &QPushButton::clicked, this,
            &GeometryEditorWidget::onCompute);
  }

  // -- Motor grid --
  {
    // Gizmo hint, then one collapsible section per rotor with its fields
    // stacked vertically (a form) so the panel never scrolls horizontally.
    auto *hint = new QLabel(
        tr("Tip: in the 3D view (sim stopped) click a motor, then "
           "G = move · R = rotate axis · X/Y/Z lock · click confirm · Esc."),
        this);
    hint->setWordWrap(true);
    hint->setStyleSheet(QString("color:%1; font-size:11px;")
                            .arg(Theme::hex(Theme::kTextMuted)));
    root->addWidget(hint);

    const char *names[4] = {"Motor 1 (FR)", "Motor 2 (RR)", "Motor 3 (RL)",
                            "Motor 4 (FL)"};
    for (int i = 0; i < 4; ++i) {
      auto &r = rows_[i];
      auto *sec =
          new CollapsibleSection(tr(names[i]), this, /*expanded=*/i == 0);
      auto *body = new QWidget();
      auto *form = new QFormLayout(body);
      form->setContentsMargins(0, 0, 0, 0);
      form->setLabelAlignment(Qt::AlignRight);

      // 4 decimals: the small presets (a 65 mm whoop) sit at a few cm, and the
      // spinbox rounds to its decimals when the form is read back.
      r.px = spin(-2, 2, 4, 0.005, 0, QStringLiteral(" m"));
      r.py = spin(-2, 2, 4, 0.005, 0, QStringLiteral(" m"));
      r.pz = spin(-2, 2, 4, 0.005, 0, QStringLiteral(" m"));
      r.ax = spin(-1, 1, 3, 0.05, 0);
      r.ay = spin(-1, 1, 3, 0.05, 0);
      r.az = spin(-1, 1, 3, 0.05, -1);
      r.spin = new QComboBox();
      r.spin->addItem(tr("CCW (+)"), +1);
      r.spin->addItem(tr("CW (−)"), -1);
      // Wide enough for every preset (whoop k_moment 2.7e-11, 5668 rad/s) with
      // four significant figures surviving the spinbox round trip.
      r.kt = spin(0, 1, 12, 1e-7, 1.234e-6);
      r.km = spin(0, 1, 14, 1e-9, 1.59e-8);
      r.wmax = spin(0, 20000, 0, 50, 3365, QStringLiteral(" rad/s"));
      r.tau = spin(0.001, 0.5, 4, 0.001, 0.0125, QStringLiteral(" s"));

      form->addRow(tr("Pos X"), r.px);
      form->addRow(tr("Pos Y"), r.py);
      form->addRow(tr("Pos Z"), r.pz);
      form->addRow(tr("Axis X"), r.ax);
      form->addRow(tr("Axis Y"), r.ay);
      form->addRow(tr("Axis Z"), r.az);
      form->addRow(tr("Spin"), r.spin);
      form->addRow(tr("k_thrust"), r.kt);
      form->addRow(tr("k_moment"), r.km);
      form->addRow(tr("max ω"), r.wmax);
      form->addRow(tr("τ (spin-up)"), r.tau);

      sec->setContentWidget(body);
      root->addWidget(sec);
    }
  }

  // -- Apply --
  {
    auto *row = new QHBoxLayout();
    row->addStretch();
    auto *apply = new ui::SuccessButton(tr("Apply to Sim"), this);
    apply->setToolTip(
        tr("Push mass, inertia, and motor layout to the running simulator."));
    row->addWidget(apply);
    root->addLayout(row);
    connect(apply, &QPushButton::clicked, this, &GeometryEditorWidget::onApply);
  }

  // Any manual edit of the airframe fields makes the vehicle a custom one: the
  // Preset combo goes back to "Custom" so Start does not send a preset's gains
  // for a modified airframe. Loading a config into the form (a preset, a file,
  // restore) is guarded by syncing_ and does not count as an edit.
  {
    auto edited = [this] { onFormEdited(); };
    const auto dbl = QOverload<double>::of(&QDoubleSpinBox::valueChanged);
    const auto idx = QOverload<int>::of(&QComboBox::currentIndexChanged);
    for (QDoubleSpinBox *sp : {scaleSpin_, massSpin_, transX_, transY_, transZ_,
                               rotX_, rotY_, rotZ_})
      connect(sp, dbl, this, edited);
    connect(meshEdit_, &QLineEdit::textEdited, this, edited);
    for (auto &r : rows_) {
      for (QDoubleSpinBox *sp : {r.px, r.py, r.pz, r.ax, r.ay, r.az, r.kt,
                                 r.km, r.wmax, r.tau})
        connect(sp, dbl, this, edited);
      connect(r.spin, idx, this, edited);
    }
  }

  // Pack sections to the top; without this the QVBoxLayout spreads the
  // slack between collapsed sections so they sit evenly apart.
  root->addStretch(1);
}

void GeometryEditorWidget::onFormEdited() {
  if (syncing_ || presetCombo_->currentIndex() == 0)
    return;
  setPresetId(QString());
  emit presetChosen(QString());
}

QString GeometryEditorWidget::presetId() const {
  return presetCombo_->currentData().toString();
}

void GeometryEditorWidget::syncConfigToUi() {
  const bool wasSyncing = syncing_;
  syncing_ = true;
  meshEdit_->setText(cfg_.meshPath);
  scaleSpin_->setValue(cfg_.scale);
  massSpin_->setValue(cfg_.mass);
  transX_->setValue(cfg_.translate.x());
  transY_->setValue(cfg_.translate.y());
  transZ_->setValue(cfg_.translate.z());
  rotX_->setValue(cfg_.rotate.x());
  rotY_->setValue(cfg_.rotate.y());
  rotZ_->setValue(cfg_.rotate.z());
  for (int i = 0; i < 4; ++i) {
    const auto &m = cfg_.motors[i];
    auto &r = rows_[i];
    r.px->setValue(m.pos.x());
    r.py->setValue(m.pos.y());
    r.pz->setValue(m.pos.z());
    r.ax->setValue(m.axis.x());
    r.ay->setValue(m.axis.y());
    r.az->setValue(m.axis.z());
    r.spin->setCurrentIndex(m.spin >= 0 ? 0 : 1);
    r.kt->setValue(m.k_thrust);
    r.km->setValue(m.k_moment);
    r.wmax->setValue(m.max_omega);
    r.tau->setValue(m.tau);
  }
  syncing_ = wasSyncing;
}

void GeometryEditorWidget::syncUiToConfig() {
  cfg_.meshPath = meshEdit_->text();
  cfg_.scale = static_cast<float>(scaleSpin_->value());
  cfg_.mass = static_cast<float>(massSpin_->value());
  cfg_.translate =
      QVector3D(transX_->value(), transY_->value(), transZ_->value());
  cfg_.rotate = QVector3D(rotX_->value(), rotY_->value(), rotZ_->value());
  for (int i = 0; i < 4; ++i) {
    auto &m = cfg_.motors[i];
    const auto &r = rows_[i];
    m.pos = QVector3D(r.px->value(), r.py->value(), r.pz->value());
    m.axis = QVector3D(r.ax->value(), r.ay->value(), r.az->value());
    m.spin = r.spin->currentData().toInt();
    m.k_thrust = static_cast<float>(r.kt->value());
    m.k_moment = static_cast<float>(r.km->value());
    m.max_omega = static_cast<float>(r.wmax->value());
    m.tau = static_cast<float>(r.tau->value());
  }
}

void GeometryEditorWidget::setMotorFromGizmo(int i, QVector3D posComFrame,
                                             QVector3D axis) {
  if (i < 0 || i >= 4)
    return;
  auto &m = cfg_.motors[i];
  // The gizmo works in the rendered CoM/body frame; invert the body placement
  // transform so the stored value is in the raw model-origin frame (the frame
  // physicsConfig() re-applies bodyXform()+CoM to). render = x*raw - com.
  bool ok = false;
  const QMatrix4x4 xi = bodyXform().inverted(&ok);
  const QVector3D placed = posComFrame + cfg_.com; // CoM frame -> placed frame
  m.pos = ok ? xi.map(placed) : placed;
  m.axis = ok ? xi.mapVector(axis).normalized() : axis;
  // Reflect in the row's spinboxes (block in case anything is wired).
  auto &r = rows_[i];
  for (auto *s : {r.px, r.py, r.pz, r.ax, r.ay, r.az})
    s->blockSignals(true);
  r.px->setValue(m.pos.x());
  r.py->setValue(m.pos.y());
  r.pz->setValue(m.pos.z());
  r.ax->setValue(m.axis.x());
  r.ay->setValue(m.axis.y());
  r.az->setValue(m.axis.z());
  for (auto *s : {r.px, r.py, r.pz, r.ax, r.ay, r.az})
    s->blockSignals(false);
  emit previewUpdated();
}

void GeometryEditorWidget::showMassProps() {
  comLbl_->setText(tr("CoM: (%1, %2, %3) mm")
                       .arg(cfg_.com.x() * 1000.0, 0, 'f', 1)
                       .arg(cfg_.com.y() * 1000.0, 0, 'f', 1)
                       .arg(cfg_.com.z() * 1000.0, 0, 'f', 1));
  const auto &I = cfg_.inertia;
  inertiaLbl_->setText(QString("I [kg·m²] =\n"
                               " %1  %2  %3\n %4  %5  %6\n %7  %8  %9")
                           .arg(I[0], 9, 'g', 3)
                           .arg(I[1], 9, 'g', 3)
                           .arg(I[2], 9, 'g', 3)
                           .arg(I[3], 9, 'g', 3)
                           .arg(I[4], 9, 'g', 3)
                           .arg(I[5], 9, 'g', 3)
                           .arg(I[6], 9, 'g', 3)
                           .arg(I[7], 9, 'g', 3)
                           .arg(I[8], 9, 'g', 3));
}

bool GeometryEditorWidget::loadAndCompute(QString *err) {
  syncUiToConfig();
  vsim::LoadedMesh mesh = vsim::loadMesh(cfg_.meshPath, cfg_.scale, err);
  if (!mesh.valid)
    return false;

  // Body-frame placement: rotate (XYZ Euler, deg) then translate, on top
  // of the scale already applied by the loader. The SAME transform is applied
  // to the motor layout in physicsConfig() so motors stay attached to the body.
  const QMatrix4x4 xform = bodyXform();
  meshPos_.resize(mesh.positions.size());
  meshNrm_.resize(mesh.normals.size());
  for (size_t i = 0; i < mesh.positions.size(); ++i)
    meshPos_[i] = xform.map(mesh.positions[i]);
  for (size_t i = 0; i < mesh.normals.size(); ++i)
    meshNrm_[i] = xform.mapVector(mesh.normals[i]).normalized();

  vsim::MassProperties mp = vsim::computeMassProperties(meshPos_, cfg_.mass);
  if (!mp.valid) {
    if (err)
      *err = tr("mesh is not a closed solid (zero volume)");
    return false;
  }

  cfg_.com = mp.com;
  // Recenter the mesh on the CoM so the renderer draws it about the same
  // point the physics integrates. Motor arms get the matching shift via
  // physicsConfig(); the inertia tensor is already about the CoM.
  for (auto &v : meshPos_)
    v -= mp.com;
  // Row-major symmetric tensor into the 9-float config.
  cfg_.inertia = {mp.ixx, mp.ixy, mp.ixz, mp.ixy, mp.iyy,
                  mp.iyz, mp.ixz, mp.iyz, mp.izz};
  return true;
}

void GeometryEditorWidget::onBrowse() {
  const QString path = QFileDialog::getOpenFileName(
      this, tr("Select airframe mesh"),
      QFileInfo(meshEdit_->text()).absolutePath(),
      tr("Meshes (*.stl *.obj *.ply *.glb *.gltf *.dae *.fbx);;All files (*)"));
  if (!path.isEmpty())
    meshEdit_->setText(path);
}

void GeometryEditorWidget::onCompute() {
  QString err;
  if (!loadAndCompute(&err)) {
    statusLbl_->setText(tr("Load failed: %1").arg(err));
    statusLbl_->setStyleSheet(
        QString("color:%1;").arg(Theme::hex(Theme::kDanger)));
    return;
  }
  // The mesh + motor arms are recentered on the CoM, so the offset is
  // handled, not a problem — report it as information.
  const double comMag = cfg_.com.length();
  statusLbl_->setText(
      tr("Loaded %1 triangles. CoM offset %2 mm from model origin "
         "(handled — dynamics about CoM).")
          .arg(meshPos_.size() / 3)
          .arg(comMag * 1000.0, 0, 'f', 1));
  statusLbl_->setStyleSheet(QString("color:%1;").arg(Theme::hex(Theme::kOk)));
  showMassProps();
  emit previewUpdated();
}

void GeometryEditorWidget::setPresetId(const QString &id) {
  const int i = presetCombo_->findData(id);
  QSignalBlocker b(presetCombo_);
  presetCombo_->setCurrentIndex(i < 0 ? 0 : i);
}

void GeometryEditorWidget::onPresetActivated(int index) {
  const QString id = presetCombo_->itemData(index).toString();
  const vsim::AirframePreset *p = vsim::findAirframePreset(id);
  if (!p) { // "Custom": keep what is in the form
    emit presetChosen(QString());
    return;
  }
  setConfig(p->geometry);
  loadedName_ = p->name;
  statusLbl_->setText(tr("Loaded preset \"%1\" (real pack %2; its PID gains "
                         "are sent when the sim starts).")
                          .arg(p->name, p->pack));
  statusLbl_->setStyleSheet(QString("color:%1;").arg(Theme::hex(Theme::kOk)));
  emit previewUpdated(); // refresh the preview and persist
  emit presetChosen(p->id);
}

void GeometryEditorWidget::onApply() {
  syncUiToConfig();
  emit geometryApplied();
}

void GeometryEditorWidget::onSaveFile() {
  syncUiToConfig();
  QString path = QFileDialog::getSaveFileName(
      this, tr("Save vehicle"), QString(), tr("Vayu vehicle (*.vveh)"));
  if (path.isEmpty())
    return;
  if (!path.endsWith(".vveh", Qt::CaseInsensitive))
    path += ".vveh";
  QFile f(path);
  if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
    statusLbl_->setText(tr("Save failed: %1").arg(f.errorString()));
    statusLbl_->setStyleSheet(
        QString("color:%1;").arg(Theme::hex(Theme::kDanger)));
    return;
  }
  f.write(QJsonDocument(vsim::geometryToJson(cfg_)).toJson(QJsonDocument::Indented));
  loadedName_ = QFileInfo(path).fileName();
  statusLbl_->setText(tr("Saved vehicle to %1").arg(loadedName_));
  statusLbl_->setStyleSheet(QString("color:%1;").arg(Theme::hex(Theme::kOk)));
}

void GeometryEditorWidget::onLoadFile() {
  const QString path =
      QFileDialog::getOpenFileName(this, tr("Load vehicle"), QString(),
                                   tr("Vayu vehicle (*.vveh);;All files (*)"));
  if (path.isEmpty())
    return;
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly)) {
    statusLbl_->setText(tr("Load failed: %1").arg(f.errorString()));
    statusLbl_->setStyleSheet(
        QString("color:%1;").arg(Theme::hex(Theme::kDanger)));
    return;
  }
  QJsonParseError pe;
  const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &pe);
  if (pe.error != QJsonParseError::NoError || !doc.isObject()) {
    statusLbl_->setText(tr("Load failed: invalid vehicle JSON"));
    statusLbl_->setStyleSheet(
        QString("color:%1;").arg(Theme::hex(Theme::kDanger)));
    return;
  }
  // setConfig syncs the form, re-loads the mesh + recomputes if its path
  // resolves (else keeps the stored inertia/CoM), and fires previewUpdated.
  setConfig(vsim::geometryFromJson(doc.object()));
  setPresetId(QString());
  emit presetChosen(QString());
  loadedName_ = QFileInfo(path).fileName();
  statusLbl_->setText(tr("Loaded vehicle from %1").arg(loadedName_));
  statusLbl_->setStyleSheet(QString("color:%1;").arg(Theme::hex(Theme::kOk)));
}

void GeometryEditorWidget::setConfig(const vsim::GeometryConfig &c) {
  cfg_ = c;
  syncConfigToUi();
  // If the persisted mesh still resolves, load it so the preview + inertia
  // are live without making the user click Compute again.
  if (!cfg_.meshPath.isEmpty() && QFileInfo::exists(cfg_.meshPath)) {
    QString err;
    if (loadAndCompute(&err)) {
      statusLbl_->setText(tr("Loaded %1 triangles.").arg(meshPos_.size() / 3));
      statusLbl_->setStyleSheet(
          QString("color:%1;").arg(Theme::hex(Theme::kOk)));
      emit previewUpdated();
    }
  }
  showMassProps();
}
