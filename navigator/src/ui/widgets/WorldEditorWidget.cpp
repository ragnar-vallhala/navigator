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
#include "WorldEditorWidget.h"

#include "../../core/ui/Buttons.h"
#include "CollapsibleSection.h"

#include "../../core/Theme.h"
#include "../../vsim/ProceduralWorld.h"

#include <QAbstractSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLineEdit>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QVBoxLayout>

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

const QString kUserPrefix = QStringLiteral("user:");

vsim::UserPresetStore userPresetStore() {
  vsim::UserPresetStore st;
  QStringList reserved;
  for (const auto &p : vsim::worldPresetList())
    reserved << p.first << p.second;
  st.setReserved(vsim::UserPresetStore::Kind::World, reserved);
  return st;
}

QJsonObject vec3ToJson(const QVector3D &v) {
  return QJsonObject{{"x", v.x()}, {"y", v.y()}, {"z", v.z()}};
}
QVector3D vec3FromJson(const QJsonValue &v, const QVector3D &def) {
  if (!v.isObject())
    return def;
  const QJsonObject o = v.toObject();
  return QVector3D(o.value("x").toDouble(def.x()),
                   o.value("y").toDouble(def.y()),
                   o.value("z").toDouble(def.z()));
}

QJsonObject worldToJson(const vsim::WorldConfig &w) {
  QJsonObject root;
  root["format"] = "vayu-world";
  root["version"] = 2;
  root["gravity"] = w.gravity;
  root["ground_z"] = w.ground_z;
  root["restitution"] = w.restitution;
  root["linear_drag"] = w.linear_drag;
  root["angular_drag"] = w.angular_drag;
  root["ground_right_gain"] = w.ground_right_gain;
  root["ground_right_damp"] = w.ground_right_damp;
  root["world_mesh_path"] = w.worldMeshPath;
  root["world_scale"] = w.worldScale;
  root["world_up_axis"] = w.worldUpAxis;
  root["world_mesh_restitution"] = w.worldMeshRestitution;
  root["world_mesh_double_sided"] = w.worldMeshDoubleSided;
  root["world_mesh_offset"] = vec3ToJson(w.worldMeshOffset);
  QJsonArray obs;
  for (const vsim::Obstacle &o : w.obstacles) {
    QJsonObject jo{{"type", o.type},
                   {"pos", vec3ToJson(o.pos)},
                   {"size", vec3ToJson(o.size)},
                   {"rotate", vec3ToJson(o.rotate)},
                   {"restitution", o.restitution}};
    if (o.style != 0) // look only; omitted for plain shapes (old files unchanged)
      jo["style"] = o.style;
    obs.append(jo);
  }
  root["obstacles"] = obs;
  if (!w.proceduralBiome.isEmpty()) {
    root["procedural"] =
        QJsonObject{{"biome", w.proceduralBiome},
                    {"seed", static_cast<double>(w.proceduralSeed)},
                    {"size_m", w.proceduralSizeM},
                    {"resolution", w.proceduralResolution},
                    {"height_m", w.field.heightM},
                    {"feature_m", w.field.featureM},
                    {"mountain_mix", w.field.mountainMix},
                    {"max_slope", w.field.maxSlope},
                    {"ridge_soft", w.field.ridgeSoft},
                    {"preset", w.worldPreset},
                    {"shape", w.field.shape},
                    {"palette", w.field.palette},
                    {"mountain_bias", w.field.mountainBias},
                    {"flat_radius_m", w.field.flatRadiusM},
                    {"relief_m", w.field.reliefM},
                    {"col_brown", w.field.colBrownT},
                    {"col_rock", w.field.colRockT},
                    {"col_snow", w.field.colSnowT},
                    {"col_slope", w.field.colSlopeT},
                    {"grass_spacing", w.flora.spacing},
                    {"grass_max_frac", w.flora.grassMaxFrac},
                    {"grass_slope_lo", w.flora.slopeLo},
                    {"grass_slope_hi", w.flora.slopeHi},
                    {"flower_frac", w.flora.flowerFrac},
                    {"blades_per_cell", w.flora.bladesPerCell},
                    {"blade_height_mean", w.flora.heightMean},
                    {"blade_height_dev", w.flora.heightStdDev},
                    {"look_sun_intensity", w.look.sunIntensity},
                    {"look_ambient_strength", w.look.ambientStrength},
                    {"look_brightness", w.look.brightness},
                    {"look_tip_warmth", w.look.tipWarmth},
                    {"look_backlight", w.look.backlight},
                    {"look_sheen", w.look.sheen},
                    {"look_vein_strength", w.look.veinStrength},
                    {"look_root_darkness", w.look.rootDarkness},
                    {"look_face_camera_frac", w.look.faceCameraFrac}};
  }
  return root;
}

vsim::WorldConfig worldFromJson(const QJsonObject &root) {
  vsim::WorldConfig w; // defaults fill anything missing
  w.gravity = root.value("gravity").toDouble(w.gravity);
  w.ground_z = root.value("ground_z").toDouble(w.ground_z);
  w.restitution = root.value("restitution").toDouble(w.restitution);
  w.linear_drag = root.value("linear_drag").toDouble(w.linear_drag);
  w.angular_drag = root.value("angular_drag").toDouble(w.angular_drag);
  w.ground_right_gain =
      root.value("ground_right_gain").toDouble(w.ground_right_gain);
  w.ground_right_damp =
      root.value("ground_right_damp").toDouble(w.ground_right_damp);
  w.worldMeshPath = root.value("world_mesh_path").toString(w.worldMeshPath);
  w.worldScale = root.value("world_scale").toDouble(w.worldScale);
  w.worldUpAxis = root.value("world_up_axis").toInt(w.worldUpAxis);
  w.worldMeshRestitution =
      root.value("world_mesh_restitution").toDouble(w.worldMeshRestitution);
  w.worldMeshDoubleSided =
      root.value("world_mesh_double_sided").toBool(w.worldMeshDoubleSided);
  w.worldMeshOffset =
      vec3FromJson(root.value("world_mesh_offset"), w.worldMeshOffset);
  for (const QJsonValue &v : root.value("obstacles").toArray()) {
    const QJsonObject o = v.toObject();
    vsim::Obstacle ob;
    ob.type = o.value("type").toInt(ob.type);
    ob.pos = vec3FromJson(o.value("pos"), ob.pos);
    ob.size = vec3FromJson(o.value("size"), ob.size);
    ob.rotate = vec3FromJson(o.value("rotate"), ob.rotate);
    ob.restitution = o.value("restitution").toDouble(ob.restitution);
    ob.style = o.value("style").toInt(ob.style);
    w.obstacles.push_back(ob);
  }
  const QJsonObject pg = root.value("procedural").toObject();
  w.proceduralBiome = pg.value("biome").toString(w.proceduralBiome);
  // The finite "meadow" arena is gone: old saves stream as the endless biome
  // and keep the meadow ground cover (palette set below, 0 = not chosen).
  const bool legacyMeadow = w.proceduralBiome.compare(
                                QStringLiteral("meadow"),
                                Qt::CaseInsensitive) == 0;
  if (legacyMeadow)
    w.proceduralBiome = QStringLiteral("endless");
  w.proceduralSeed = static_cast<quint32>(
      pg.value("seed").toDouble(static_cast<double>(w.proceduralSeed)));
  w.proceduralSizeM = pg.value("size_m").toDouble(w.proceduralSizeM);
  w.proceduralResolution = pg.value("resolution").toInt(w.proceduralResolution);
  w.field.heightM = pg.value("height_m").toDouble(w.field.heightM);
  w.field.featureM = pg.value("feature_m").toDouble(w.field.featureM);
  w.field.mountainMix = pg.value("mountain_mix").toDouble(w.field.mountainMix);
  w.field.maxSlope = pg.value("max_slope").toDouble(w.field.maxSlope);
  w.field.ridgeSoft = pg.value("ridge_soft").toDouble(w.field.ridgeSoft);
  w.worldPreset = pg.value("preset").toString(w.worldPreset);
  w.field.shape = pg.value("shape").toInt(w.field.shape);
  w.field.palette = pg.value("palette").toInt(w.field.palette);
  if (legacyMeadow && w.field.palette == 0)
    w.field.palette = vsim::procgen::kPalMeadow;
  w.field.mountainBias =
      pg.value("mountain_bias").toDouble(w.field.mountainBias);
  w.field.flatRadiusM = pg.value("flat_radius_m").toDouble(w.field.flatRadiusM);
  w.field.reliefM = pg.value("relief_m").toDouble(0.0); // absent in old files
  w.field.colBrownT = pg.value("col_brown").toDouble(w.field.colBrownT);
  w.field.colRockT = pg.value("col_rock").toDouble(w.field.colRockT);
  w.field.colSnowT = pg.value("col_snow").toDouble(w.field.colSnowT);
  w.field.colSlopeT = pg.value("col_slope").toDouble(w.field.colSlopeT);
  w.flora.spacing = pg.value("grass_spacing").toDouble(w.flora.spacing);
  w.flora.grassMaxFrac =
      pg.value("grass_max_frac").toDouble(w.flora.grassMaxFrac);
  w.flora.slopeLo = pg.value("grass_slope_lo").toDouble(w.flora.slopeLo);
  w.flora.slopeHi = pg.value("grass_slope_hi").toDouble(w.flora.slopeHi);
  w.flora.flowerFrac = pg.value("flower_frac").toDouble(w.flora.flowerFrac);
  w.flora.bladesPerCell =
      pg.value("blades_per_cell").toDouble(w.flora.bladesPerCell);
  w.flora.heightMean =
      pg.value("blade_height_mean").toDouble(w.flora.heightMean);
  w.flora.heightStdDev =
      pg.value("blade_height_dev").toDouble(w.flora.heightStdDev);
  w.look.sunIntensity =
      pg.value("look_sun_intensity").toDouble(w.look.sunIntensity);
  w.look.ambientStrength =
      pg.value("look_ambient_strength").toDouble(w.look.ambientStrength);
  w.look.brightness = pg.value("look_brightness").toDouble(w.look.brightness);
  w.look.tipWarmth = pg.value("look_tip_warmth").toDouble(w.look.tipWarmth);
  w.look.backlight = pg.value("look_backlight").toDouble(w.look.backlight);
  w.look.sheen = pg.value("look_sheen").toDouble(w.look.sheen);
  w.look.veinStrength =
      pg.value("look_vein_strength").toDouble(w.look.veinStrength);
  w.look.rootDarkness =
      pg.value("look_root_darkness").toDouble(w.look.rootDarkness);
  w.look.faceCameraFrac =
      pg.value("look_face_camera_frac").toDouble(w.look.faceCameraFrac);
  return w;
}
} // namespace

WorldEditorWidget::WorldEditorWidget(QWidget *parent) : QWidget(parent) {
  buildUi();
  syncConfigToUi();
  syncWindToUi();
}

void WorldEditorWidget::buildUi() {
  auto *root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(6);

  // -- Environment --
  {
    auto *sec =
        new CollapsibleSection(tr("Environment"), this, /*expanded=*/false);
    auto *body = new QWidget();
    auto *form = new QFormLayout(body);
    gravity_ = spin(0.0, 30.0, 2, 0.1, cfg_.gravity, QStringLiteral(" m/s²"));
    groundZ_ = spin(-50.0, 50.0, 2, 0.1, cfg_.ground_z, QStringLiteral(" m"));
    rest_ = spin(0.0, 1.0, 2, 0.05, cfg_.restitution);
    form->addRow(tr("Gravity:"), gravity_);
    form->addRow(tr("Ground Z (NED):"), groundZ_);
    form->addRow(tr("Restitution:"), rest_);
    sec->setContentWidget(body);
    root->addWidget(sec);
  }

  // -- Aerodynamics --
  {
    auto *sec =
        new CollapsibleSection(tr("Aerodynamics"), this, /*expanded=*/false);
    auto *body = new QWidget();
    auto *form = new QFormLayout(body);
    linDrag_ =
        spin(0.0, 5.0, 3, 0.01, cfg_.linear_drag, QStringLiteral(" N·s/m"));
    angDrag_ =
        spin(0.0, 1.0, 4, 0.001, cfg_.angular_drag, QStringLiteral(" N·m·s"));
    rightGain_ = spin(0.0, 200.0, 1, 1.0, cfg_.ground_right_gain);
    rightGain_->setToolTip(tr("How hard a tipped airframe topples to level on "
                              "the ground (0 = stays as it lands)."));
    rightDamp_ = spin(0.0, 50.0, 1, 0.5, cfg_.ground_right_damp);
    rightDamp_->setToolTip(tr("Damping of the righting rotation (higher = "
                              "settles flatter, less overshoot)."));
    form->addRow(tr("Linear drag:"), linDrag_);
    form->addRow(tr("Angular drag:"), angDrag_);
    form->addRow(tr("Ground righting gain:"), rightGain_);
    form->addRow(tr("Ground righting damp:"), rightDamp_);
    sec->setContentWidget(body);
    root->addWidget(sec);
  }

  // -- Wind & turbulence (its own staged Apply, mockup World tab) --
  buildWindSection(root);
  buildBaroSection(root);
  buildGpsSection(root);

  // -- Apply / Save / Load --
  {
    auto *row = new QHBoxLayout();
    auto *save = new ui::GhostButton(tr("Save…"), this);
    save->setToolTip(tr("Export the world (environment + obstacles) to a "
                        "portable .vworld file."));
    auto *load = new ui::GhostButton(tr("Load…"), this);
    load->setToolTip(tr("Import a .vworld file."));
    row->addWidget(save);
    row->addWidget(load);
    row->addStretch();
    auto *apply = new ui::SuccessButton(tr("Apply to Sim"), this);
    apply->setToolTip(tr("Push environment + aerodynamics to the simulator."));
    row->addWidget(apply);
    root->addLayout(row);
    connect(apply, &QPushButton::clicked, this, &WorldEditorWidget::onApply);
    connect(save, &QPushButton::clicked, this, &WorldEditorWidget::onSaveWorld);
    connect(load, &QPushButton::clicked, this, &WorldEditorWidget::onLoadWorld);

    fileStatus_ = new QLabel(this);
    fileStatus_->setStyleSheet(QString("color:%1; font-size:11px;")
                                   .arg(Theme::hex(Theme::kTextMuted)));
    root->addWidget(fileStatus_);
  }

  // -- Procedural world --
  buildProceduralSection(root);
  buildProceduralTuningSection(root);

  // -- World mesh --
  buildWorldMeshSection(root);

  // -- Obstacles --
  buildObstacleSection(root);

  root->addStretch();
}

void WorldEditorWidget::buildProceduralSection(QVBoxLayout *root) {
  auto *sec =
      new CollapsibleSection(tr("Procedural world"), this, /*expanded=*/false);
  auto *body = new QWidget();
  auto *col = new QVBoxLayout(body);

  auto *hint = new QLabel(
      tr("Generate the world from a biome instead of importing a mesh. "
         "Takes precedence over the world mesh below; same seed → same world."),
      body);
  hint->setWordWrap(true);
  hint->setStyleSheet(
      QString("color:%1; font-size:11px;").arg(Theme::hex(Theme::kTextMuted)));
  col->addWidget(hint);

  auto *form = new QFormLayout();
  procBiome_ = new QComboBox(body);
  procBiome_->addItem(tr("None (use world mesh)"), QString());
  procBiome_->addItem(tr("Procedural (endless)"), QStringLiteral("endless"));
  // Preset-only biomes (hills/alpine/...) are not listed: picking one here
  // would stream with the default terrain. They come from the World preset
  // combo. Every procedural world streams endlessly; only an imported mesh is
  // finite.

  procPreset_ = new QComboBox(body);
  procPreset_->addItem(tr("Custom"), QString());
  for (const auto &p : vsim::worldPresetList())
    procPreset_->addItem(p.second, p.first);
  builtinPresetCount_ = procPreset_->count();
  procPreset_->setToolTip(
      tr("One-click procedural worlds, all generated from the Seed below "
         "(same seed -> same world). Replaces the terrain settings and the "
         "obstacle list; racing/yard/forest add gates, boxes or trees that "
         "the simulator collides with."));

  procSeed_ = new QSpinBox(body);
  procSeed_->setRange(0, 2147483647);
  procSeed_->setValue(static_cast<int>(cfg_.proceduralSeed));
  procSeed_->setToolTip(tr("Random seed — change it for a different world."));

  form->addRow(tr("World preset:"), procPreset_);
  {
    // Under the combo, in a wrapping-free row of two short buttons.
    auto *prow = new QHBoxLayout();
    savePresetBtn_ = new ui::GhostButton(tr("Save as preset…"), body);
    savePresetBtn_->setToolTip(
        tr("Save the current world under a name; it is then listed in the "
           "World preset combo after the built-ins."));
    delPresetBtn_ = new ui::GhostButton(tr("Delete preset"), body);
    delPresetBtn_->setToolTip(
        tr("Delete the selected user preset (built-in presets cannot be "
           "deleted)."));
    prow->addWidget(savePresetBtn_);
    prow->addWidget(delPresetBtn_);
    prow->addStretch();
    form->addRow(QString(), prow);
    connect(savePresetBtn_, &QPushButton::clicked, this,
            &WorldEditorWidget::onSavePreset);
    connect(delPresetBtn_, &QPushButton::clicked, this,
            &WorldEditorWidget::onDeletePreset);
    connect(procPreset_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this] { updatePresetButtons(); });
    refreshUserPresets(QString());
  }
  form->addRow(tr("Biome:"), procBiome_);
  form->addRow(tr("Seed:"), procSeed_);
  col->addLayout(form);

  // Pull the four widgets into cfg_ (no regen — that's the button's job).
  auto pull = [this] {
    if (procSyncing_)
      return;
    cfg_.proceduralBiome = procBiome_->currentData().toString();
    cfg_.proceduralSeed = static_cast<quint32>(procSeed_->value());
  };

  // Changing the biome regenerates immediately (it switches the whole world
  // on/off); the numeric knobs apply on the explicit Generate button so
  // dragging a spinner doesn't rebuild the mesh + BVH on every step.
  connect(procBiome_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          [this, pull] {
            if (procSyncing_)
              return;
            pull();
            cfg_.worldPreset.clear(); // a manual biome pick is a custom world
            if (procPreset_) {
              procSyncing_ = true;
              procPreset_->setCurrentIndex(0);
              procSyncing_ = false;
            }
            emit worldMeshChanged();
          });
  connect(procPreset_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          [this, pull](int) {
            if (procSyncing_)
              return;
            const QString id = procPreset_->currentData().toString();
            if (id.startsWith(kUserPrefix)) {
              const QString name = id.mid(kUserPrefix.size());
              QJsonObject obj;
              if (!userPresetStore().load(vsim::UserPresetStore::Kind::World,
                                          name, &obj)) {
                if (fileStatus_)
                  fileStatus_->setText(
                      tr("Could not load preset \"%1\".").arg(name));
                refreshUserPresets(QString()); // vanished or corrupt
                return;
              }
              loadWorldObject(obj); // same path as Load...
              procSyncing_ = true;  // setConfig showed Custom; show the preset
              const int i = procPreset_->findData(id);
              procPreset_->setCurrentIndex(i < 0 ? 0 : i);
              procSyncing_ = false;
              updatePresetButtons();
              if (fileStatus_)
                fileStatus_->setText(tr("Loaded preset \"%1\" (%2 obstacles)")
                                         .arg(name)
                                         .arg(cfg_.obstacles.size()));
              return;
            }
            if (id.isEmpty()) {
              cfg_.worldPreset.clear();
              return;
            }
            pull(); // pick up the seed field
            if (!vsim::applyWorldPreset(cfg_, id, cfg_.proceduralSeed))
              return;
            cfg_.worldPreset = id;
            syncConfigToUi();
            emit obstaclesChanged();
            emit worldMeshChanged();
          });
  connect(procSeed_, QOverload<int>::of(&QSpinBox::valueChanged), this,
          [pull] { pull(); });

  {
    auto *row = new QHBoxLayout();
    auto *gen = new ui::GhostButton(tr("Generate"), body);
    gen->setToolTip(
        tr("(Re)generate the world from the current biome params. With a "
           "world preset selected, a changed seed or preset re-rolls its "
           "terrain and obstacles; otherwise your terrain knobs and obstacle "
           "edits are kept."));
    connect(gen, &QPushButton::clicked, this, [this, pull] {
      pull();
      if (vsim::procgen::presetNeedsReapply(
              lastPreset_.toStdString(), lastPresetSeed_,
              cfg_.worldPreset.toStdString(), cfg_.proceduralSeed) &&
          vsim::applyWorldPreset(cfg_, cfg_.worldPreset, cfg_.proceduralSeed)) {
        syncConfigToUi();
        emit obstaclesChanged();
      }
      if (cfg_.proceduralBiome.isEmpty())
        return;
      emit worldMeshChanged();
    });
    row->addWidget(gen);
    row->addStretch();
    col->addLayout(row);
  }

  sec->setContentWidget(body);
  root->addWidget(sec);
}

void WorldEditorWidget::buildWindSection(QVBoxLayout *root) {
  auto *sec =
      new CollapsibleSection(tr("Wind & Turbulence"), this, /*expanded=*/false);
  auto *body = new QWidget();
  auto *col = new QVBoxLayout(body);

  auto *form = new QFormLayout();
  windEnable_ = new QCheckBox(tr("Enable wind"), body);
  windN_ = spin(-50.0, 50.0, 2, 0.5, wind_.steady.x(), QStringLiteral(" m/s"));
  windE_ = spin(-50.0, 50.0, 2, 0.5, wind_.steady.y(), QStringLiteral(" m/s"));
  windD_ = spin(-50.0, 50.0, 2, 0.5, wind_.steady.z(), QStringLiteral(" m/s"));
  gustAmp_ = spin(0.0, 50.0, 2, 0.5, wind_.gustAmp, QStringLiteral(" m/s"));
  gustPeriod_ =
      spin(0.0, 120.0, 1, 0.5, wind_.gustPeriod, QStringLiteral(" s"));
  turbSigma_ = spin(0.0, 20.0, 2, 0.1, wind_.turbSigma, QStringLiteral(" m/s"));
  gustPeriod_->setToolTip(tr("Gust period; 0 disables the gust."));
  turbSigma_->setToolTip(
      tr("Dryden turbulence RMS intensity; 0 = smooth wind."));

  windPreset_ = new QComboBox(body);
  windPreset_->addItem(tr("Custom"), QString());
  for (const auto &p : vsim::windPresetList())
    windPreset_->addItem(p.second, p.first);
  windPreset_->setToolTip(
      tr("Fill the fields below from a preset (calm ~1 m/s, breezy ~4 m/s "
         "with light gusts, gusty ~6 m/s with strong gusts); the heading comes "
         "from the world seed. Press Apply to Sim to push it."));
  connect(windPreset_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, [this](int) {
            if (windSyncing_)
              return;
            const QString id = windPreset_->currentData().toString();
            if (id.isEmpty())
              return;
            bool ok = false;
            const vsim::WindConfig w =
                vsim::windPresetConfig(id, cfg_.proceduralSeed, &ok);
            if (!ok)
              return;
            wind_ = w;
            syncWindToUi();
            if (windApplyBtn_)
              windApplyBtn_->setEnabled(true); // staged: user presses Apply
          });
  form->addRow(tr("Preset:"), windPreset_);
  form->addRow(windEnable_);
  form->addRow(tr("Steady N:"), windN_);
  form->addRow(tr("Steady E:"), windE_);
  form->addRow(tr("Steady D:"), windD_);
  form->addRow(tr("Gust amplitude:"), gustAmp_);
  form->addRow(tr("Gust period:"), gustPeriod_);
  form->addRow(tr("Turbulence σ:"), turbSigma_);
  col->addLayout(form);

  windReadout_ = new QLabel(tr("wind: —"), body);
  windReadout_->setStyleSheet(
      QString("color:%1; font-size:11px;").arg(Theme::hex(Theme::kTextMuted)));
  col->addWidget(windReadout_);

  {
    auto *row = new QHBoxLayout();
    row->addStretch();
    windApplyBtn_ = new ui::SuccessButton(tr("Apply to Sim"), body);
    windApplyBtn_->setToolTip(tr("Push the wind field to the simulator."));
    windApplyBtn_->setEnabled(false); // enabled when a value changes (dirty)
    row->addWidget(windApplyBtn_);
    col->addLayout(row);
    connect(windApplyBtn_, &QPushButton::clicked, this,
            &WorldEditorWidget::onApplyWind);
  }

  // Dirty tracking: any edit re-enables Apply (staged-apply UX, D2).
  auto markDirty = [this] {
    if (windSyncing_)
      return;
    if (windApplyBtn_)
      windApplyBtn_->setEnabled(true);
    if (windPreset_ && windPreset_->currentIndex() != 0) {
      const QSignalBlocker b(windPreset_); // edited by hand -> Custom
      windPreset_->setCurrentIndex(0);
    }
  };
  for (QDoubleSpinBox *s :
       {windN_, windE_, windD_, gustAmp_, gustPeriod_, turbSigma_})
    connect(s, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            [markDirty](double) { markDirty(); });
  connect(windEnable_, &QCheckBox::toggled, this,
          [markDirty](bool) { markDirty(); });

  sec->setContentWidget(body);
  root->addWidget(sec);
}

void WorldEditorWidget::buildBaroSection(QVBoxLayout *root) {
  auto *sec = new CollapsibleSection(tr("Barometer error"), this,
                                     /*expanded=*/false);
  auto *body = new QWidget();
  auto *col = new QVBoxLayout(body);
  auto *form = new QFormLayout();
  // The ranges are the engine's own limits (it drops a message outside them), so
  // the controls cannot produce a value the sim would refuse.
  baroNoise_ = spin(0.0, 500.0, 1, 0.5, baro_.noisePa, QStringLiteral(" Pa"));
  baroWalk_ =
      spin(0.0, 50.0, 3, 0.05, baro_.walkPaPerSqrtS, QStringLiteral(" Pa/√s"));
  baroLag_ = spin(0.0, 2000.0, 0, 10.0, baro_.lagMs, QStringLiteral(" ms"));
  baroRate_ = spin(0.0, 1000.0, 0, 1.0, baro_.rateHz, QStringLiteral(" Hz"));
  baroRate_->setSpecialValueText(tr("every step"));
  baroNoise_->setToolTip(
      tr("White noise per reading, 1 sigma. About 12 Pa is one metre of height."));
  baroWalk_->setToolTip(
      tr("A random-walk offset that wanders rather than ramps."));
  baroLag_->setToolTip(tr("First-order response time constant."));
  baroRate_->setToolTip(
      tr("The part's output rate (sample-and-hold). 'every step' = a new "
         "reading each physics step."));
  form->addRow(tr("Noise:"), baroNoise_);
  form->addRow(tr("Drift:"), baroWalk_);
  form->addRow(tr("Lag:"), baroLag_);
  form->addRow(tr("Output rate:"), baroRate_);
  col->addLayout(form);
  {
    auto *row = new QHBoxLayout();
    row->addStretch();
    baroApplyBtn_ = new ui::SuccessButton(tr("Apply to Sim"), body);
    baroApplyBtn_->setToolTip(tr("Push the barometer error to the simulator."));
    baroApplyBtn_->setEnabled(false); // enabled when a value changes (dirty)
    row->addWidget(baroApplyBtn_);
    col->addLayout(row);
    connect(baroApplyBtn_, &QPushButton::clicked, this,
            &WorldEditorWidget::onApplyBaro);
  }
  for (QDoubleSpinBox *s : {baroNoise_, baroWalk_, baroLag_, baroRate_})
    connect(s, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            [this](double) {
              if (!baroSyncing_ && baroApplyBtn_)
                baroApplyBtn_->setEnabled(true);
            });
  sec->setContentWidget(body);
  root->addWidget(sec);
}

void WorldEditorWidget::syncBaroToUi() {
  baroSyncing_ = true;
  if (baroNoise_)
    baroNoise_->setValue(baro_.noisePa);
  if (baroWalk_)
    baroWalk_->setValue(baro_.walkPaPerSqrtS);
  if (baroLag_)
    baroLag_->setValue(baro_.lagMs);
  if (baroRate_)
    baroRate_->setValue(baro_.rateHz);
  baroSyncing_ = false;
  if (baroApplyBtn_)
    baroApplyBtn_->setEnabled(false); // freshly synced = clean
}

void WorldEditorWidget::syncUiToBaro() {
  baro_.noisePa = static_cast<float>(baroNoise_->value());
  baro_.walkPaPerSqrtS = static_cast<float>(baroWalk_->value());
  baro_.lagMs = static_cast<float>(baroLag_->value());
  baro_.rateHz = static_cast<float>(baroRate_->value());
}

void WorldEditorWidget::onApplyBaro() {
  syncUiToBaro();
  if (baroApplyBtn_)
    baroApplyBtn_->setEnabled(false); // clean until next edit
  emit baroApplied();
}

void WorldEditorWidget::setBaroConfig(const vsim::BaroErrConfig &b) {
  baro_ = b;
  syncBaroToUi();
}

void WorldEditorWidget::buildGpsSection(QVBoxLayout *root) {
  auto *sec = new CollapsibleSection(tr("GPS receiver"), this,
                                     /*expanded=*/false);
  auto *body = new QWidget();
  auto *col = new QVBoxLayout(body);
  auto *form = new QFormLayout();
  // The ranges are the engine's own limits (it drops a message outside them), so
  // the controls cannot produce a value the sim would refuse.
  gpsEnable_ = new QCheckBox(tr("Model a GPS receiver"), body);
  gpsEnable_->setChecked(gps_.enabled);
  gpsEnable_->setToolTip(
      tr("Off: the engine reports no GPS and the firmware sees no fix."));
  gpsRate_ = spin(1.0, 50.0, 0, 1.0, gps_.rateHz, QStringLiteral(" Hz"));
  gpsPosNoise_ =
      spin(0.0, 50.0, 2, 0.1, gps_.posNoiseM, QStringLiteral(" m"));
  gpsVelNoise_ =
      spin(0.0, 5.0, 2, 0.05, gps_.velNoiseMps, QStringLiteral(" m/s"));
  gpsLag_ = spin(0.0, 1000.0, 0, 10.0, gps_.lagMs, QStringLiteral(" ms"));
  gpsSats_ = spin(4.0, 32.0, 0, 1.0, gps_.sats);
  gpsRate_->setToolTip(tr("Fix rate: a new report this many times a second."));
  gpsPosNoise_->setToolTip(tr("Position noise per report, 1 sigma."));
  gpsVelNoise_->setToolTip(tr("Velocity noise per report, 1 sigma."));
  gpsLag_->setToolTip(
      tr("How old a report is when it arrives (the receiver's latency)."));
  gpsSats_->setToolTip(tr("Satellites in use that the receiver reports."));
  form->addRow(gpsEnable_);
  form->addRow(tr("Fix rate:"), gpsRate_);
  form->addRow(tr("Position noise:"), gpsPosNoise_);
  form->addRow(tr("Velocity noise:"), gpsVelNoise_);
  form->addRow(tr("Latency:"), gpsLag_);
  form->addRow(tr("Satellites:"), gpsSats_);
  col->addLayout(form);
  {
    auto *row = new QHBoxLayout();
    row->addStretch();
    gpsApplyBtn_ = new ui::SuccessButton(tr("Apply to Sim"), body);
    gpsApplyBtn_->setToolTip(
        tr("Push the GPS receiver model to the simulator. A new setting also "
           "ends any receiver fault."));
    gpsApplyBtn_->setEnabled(false); // enabled when a value changes (dirty)
    row->addWidget(gpsApplyBtn_);
    col->addLayout(row);
    connect(gpsApplyBtn_, &QPushButton::clicked, this,
            &WorldEditorWidget::onApplyGps);
  }
  for (QDoubleSpinBox *s :
       {gpsRate_, gpsPosNoise_, gpsVelNoise_, gpsLag_, gpsSats_})
    connect(s, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            [this](double) {
              if (!gpsSyncing_ && gpsApplyBtn_)
                gpsApplyBtn_->setEnabled(true);
            });
  connect(gpsEnable_, &QCheckBox::toggled, this, [this](bool) {
    if (!gpsSyncing_ && gpsApplyBtn_)
      gpsApplyBtn_->setEnabled(true);
  });

  // The receiver fault: an event, sent at once. It only has an effect while the
  // model above is on and applied.
  auto *fault = new QFormLayout();
  glitchMode_ = new QComboBox(body);
  glitchMode_->addItem(tr("Position jump"), static_cast<int>(vsim::GpsGlitchMode::Offset));
  glitchMode_->addItem(tr("Loss of fix"), static_cast<int>(vsim::GpsGlitchMode::Dropout));
  glitchMode_->addItem(tr("Frozen position"), static_cast<int>(vsim::GpsGlitchMode::Freeze));
  glitchN_ = spin(-100000.0, 100000.0, 1, 1.0, 0.0, QStringLiteral(" m"));
  glitchE_ = spin(-100000.0, 100000.0, 1, 1.0, 0.0, QStringLiteral(" m"));
  glitchD_ = spin(-100000.0, 100000.0, 1, 1.0, 0.0, QStringLiteral(" m"));
  glitchDur_ = spin(0.0, 3600.0, 1, 1.0, 10.0, QStringLiteral(" s"));
  glitchDur_->setSpecialValueText(tr("until cleared"));
  glitchMode_->setToolTip(
      tr("Position jump: the reported position moves by the offset and stays. "
         "Loss of fix: no reports. Frozen position: reports repeat the last "
         "good one."));
  glitchDur_->setToolTip(tr("Simulated seconds the fault lasts."));
  fault->addRow(tr("Fault:"), glitchMode_);
  fault->addRow(tr("Offset north:"), glitchN_);
  fault->addRow(tr("Offset east:"), glitchE_);
  fault->addRow(tr("Offset down:"), glitchD_);
  fault->addRow(tr("Duration:"), glitchDur_);
  col->addLayout(fault);
  {
    auto *row = new QHBoxLayout();
    row->addStretch();
    glitchClearBtn_ = new QPushButton(tr("End fault"), body);
    glitchSendBtn_ = new ui::SuccessButton(tr("Send fault"), body);
    glitchSendBtn_->setToolTip(tr("Start the fault now."));
    glitchClearBtn_->setToolTip(tr("End the receiver fault now."));
    row->addWidget(glitchClearBtn_);
    row->addWidget(glitchSendBtn_);
    col->addLayout(row);
    connect(glitchSendBtn_, &QPushButton::clicked, this,
            [this] { onSendGpsGlitch(false); });
    connect(glitchClearBtn_, &QPushButton::clicked, this,
            [this] { onSendGpsGlitch(true); });
  }
  // The offset only means something for a position jump.
  auto offsetsOnlyForJump = [this] {
    const bool jump = glitchMode_->currentData().toInt() ==
                      static_cast<int>(vsim::GpsGlitchMode::Offset);
    for (QDoubleSpinBox *s : {glitchN_, glitchE_, glitchD_})
      s->setEnabled(jump);
  };
  connect(glitchMode_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          [offsetsOnlyForJump](int) { offsetsOnlyForJump(); });
  offsetsOnlyForJump();

  engineIgnored_ = new QLabel(body);
  engineIgnored_->setWordWrap(true);
  engineIgnored_->setStyleSheet(
      QString("color:%1; font-size:11px;").arg(Theme::hex(Theme::kTextMuted)));
  engineIgnored_->hide();
  col->addWidget(engineIgnored_);

  sec->setContentWidget(body);
  root->addWidget(sec);
}

void WorldEditorWidget::syncGpsToUi() {
  gpsSyncing_ = true;
  if (gpsEnable_)
    gpsEnable_->setChecked(gps_.enabled);
  if (gpsRate_)
    gpsRate_->setValue(gps_.rateHz);
  if (gpsPosNoise_)
    gpsPosNoise_->setValue(gps_.posNoiseM);
  if (gpsVelNoise_)
    gpsVelNoise_->setValue(gps_.velNoiseMps);
  if (gpsLag_)
    gpsLag_->setValue(gps_.lagMs);
  if (gpsSats_)
    gpsSats_->setValue(gps_.sats);
  gpsSyncing_ = false;
  if (gpsApplyBtn_)
    gpsApplyBtn_->setEnabled(false); // freshly synced = clean
}

void WorldEditorWidget::syncUiToGps() {
  gps_.enabled = gpsEnable_->isChecked();
  gps_.rateHz = static_cast<float>(gpsRate_->value());
  gps_.posNoiseM = static_cast<float>(gpsPosNoise_->value());
  gps_.velNoiseMps = static_cast<float>(gpsVelNoise_->value());
  gps_.lagMs = static_cast<float>(gpsLag_->value());
  gps_.sats = static_cast<int>(gpsSats_->value());
}

void WorldEditorWidget::syncUiToGlitch() {
  glitch_.mode = static_cast<vsim::GpsGlitchMode>(glitchMode_->currentData().toInt());
  glitch_.offsetN = static_cast<float>(glitchN_->value());
  glitch_.offsetE = static_cast<float>(glitchE_->value());
  glitch_.offsetD = static_cast<float>(glitchD_->value());
  glitch_.durationS = static_cast<float>(glitchDur_->value());
}

void WorldEditorWidget::onApplyGps() {
  syncUiToGps();
  if (gpsApplyBtn_)
    gpsApplyBtn_->setEnabled(false); // clean until next edit
  emit gpsApplied();
}

void WorldEditorWidget::onSendGpsGlitch(bool clear) {
  if (clear) {
    glitch_ = vsim::GpsGlitchConfig{}; // mode None ends the fault
  } else {
    syncUiToGlitch();
  }
  emit gpsGlitchRequested();
}

void WorldEditorWidget::setGpsConfig(const vsim::GpsConfig &g) {
  gps_ = g;
  syncGpsToUi();
}

void WorldEditorWidget::setGpsGlitchConfig(const vsim::GpsGlitchConfig &f) {
  glitch_ = f;
  if (!glitchMode_)
    return;
  // None is not a form choice (End fault is its button): show a jump.
  const int idx = glitchMode_->findData(static_cast<int>(f.mode));
  glitchMode_->setCurrentIndex(idx >= 0 ? idx : 0);
  glitchN_->setValue(f.offsetN);
  glitchE_->setValue(f.offsetE);
  glitchD_->setValue(f.offsetD);
  glitchDur_->setValue(f.durationS);
}

void WorldEditorWidget::setEngineIgnoredCount(uint32_t n) {
  if (!engineIgnored_ || n == ignoredShown_)
    return;
  ignoredShown_ = n;
  if (n == 0) {
    engineIgnored_->hide();
    return;
  }
  engineIgnored_->setText(
      tr("The engine ignored %n control message(s) it does not recognise. "
         "Its module may be older than the SDK this build expects.",
         nullptr, static_cast<int>(n)));
  engineIgnored_->show();
}

QString WorldEditorWidget::serializeProcedural(const vsim::WorldConfig &w) {
  const QJsonObject p = worldToJson(w).value("procedural").toObject();
  if (p.isEmpty())
    return QString();
  return QString::fromUtf8(QJsonDocument(p).toJson(QJsonDocument::Compact));
}

bool WorldEditorWidget::deserializeProcedural(vsim::WorldConfig &w,
                                              const QString &json) {
  QJsonParseError err;
  const QJsonDocument d = QJsonDocument::fromJson(json.toUtf8(), &err);
  if (err.error != QJsonParseError::NoError || !d.isObject())
    return false;
  const vsim::WorldConfig t =
      worldFromJson(QJsonObject{{"procedural", d.object()}});
  w.proceduralBiome = t.proceduralBiome;
  w.proceduralSeed = t.proceduralSeed;
  w.proceduralSizeM = t.proceduralSizeM;
  w.proceduralResolution = t.proceduralResolution;
  w.worldPreset = t.worldPreset;
  w.field = t.field;
  w.flora = t.flora;
  w.look = t.look;
  return true;
}

void WorldEditorWidget::syncWindToUi() {
  windSyncing_ = true;
  if (windN_)
    windN_->setValue(wind_.steady.x());
  if (windE_)
    windE_->setValue(wind_.steady.y());
  if (windD_)
    windD_->setValue(wind_.steady.z());
  if (gustAmp_)
    gustAmp_->setValue(wind_.gustAmp);
  if (gustPeriod_)
    gustPeriod_->setValue(wind_.gustPeriod);
  if (turbSigma_)
    turbSigma_->setValue(wind_.turbSigma);
  if (windEnable_)
    windEnable_->setChecked(wind_.enabled);
  windSyncing_ = false;
  if (windApplyBtn_)
    windApplyBtn_->setEnabled(false); // freshly synced = clean
}

void WorldEditorWidget::syncUiToWind() {
  wind_.steady = QVector3D(static_cast<float>(windN_->value()),
                           static_cast<float>(windE_->value()),
                           static_cast<float>(windD_->value()));
  wind_.gustAmp = static_cast<float>(gustAmp_->value());
  wind_.gustPeriod = static_cast<float>(gustPeriod_->value());
  wind_.turbSigma = static_cast<float>(turbSigma_->value());
  wind_.enabled = windEnable_->isChecked();
}

void WorldEditorWidget::onApplyWind() {
  syncUiToWind();
  if (windApplyBtn_)
    windApplyBtn_->setEnabled(false); // clean until next edit
  emit windApplied();
}

void WorldEditorWidget::setWindConfig(const vsim::WindConfig &w) {
  wind_ = w;
  syncWindToUi();
}

void WorldEditorWidget::setWindReadout(float speedMs, float dirDeg) {
  if (!windReadout_)
    return;
  windReadout_->setText(
      tr("wind: %1 m/s @ %2°").arg(speedMs, 0, 'f', 1).arg(dirDeg, 0, 'f', 0));
}

void WorldEditorWidget::buildProceduralTuningSection(QVBoxLayout *root) {
  auto *sec =
      new CollapsibleSection(tr("Procedural tuning"), this, /*expanded=*/false);
  auto *body = new QWidget();
  auto *col = new QVBoxLayout(body);

  auto *hint =
      new QLabel(tr("Live knobs for the Endless biome — edit, then press "
                    "Generate above to "
                    "apply. (Seed is in the section above.)"),
                 body);
  hint->setWordWrap(true);
  hint->setStyleSheet(
      QString("color:%1; font-size:11px;").arg(Theme::hex(Theme::kTextMuted)));
  col->addWidget(hint);

  // A spinbox bound to a float in cfg_. Writes on edit (Generate applies the
  // result) and registers a syncer so setConfig can refresh it from cfg_.
  auto knob = [this](QFormLayout *form, const QString &label, double lo,
                     double hi, int dec, double step, float *ref,
                     const QString &suffix) {
    QDoubleSpinBox *s = spin(lo, hi, dec, step, *ref, suffix);
    form->addRow(label, s);
    connect(s, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            [this, ref](double v) {
              if (procSyncing_)
                return;
              *ref = static_cast<float>(v);
              cfg_.worldPreset.clear(); // hand-tuned: no longer the preset
              if (procPreset_) {
                procSyncing_ = true;
                procPreset_->setCurrentIndex(0);
                procSyncing_ = false;
              }
            });
    procKnobSyncers_.push_back([s, ref] { s->setValue(*ref); });
    procKnobSpins_.push_back(s);
  };
  auto group = [&](const QString &title) {
    auto *l = new QLabel(title, body);
    l->setStyleSheet(QString("color:%1; font-weight:bold; font-size:11px;")
                         .arg(Theme::hex(Theme::kTextMuted)));
    col->addWidget(l);
  };

  group(tr("Terrain"));
  {
    auto *f = new QFormLayout();
    knob(f, tr("Peak height:"), 0.5, 250, 0, 5, &cfg_.field.heightM, tr(" m"));
    knob(f, tr("Feature size:"), 40, 800, 0, 10, &cfg_.field.featureM,
         tr(" m"));
    knob(f, tr("Mountain amount:"), 0, 1, 2, 0.05, &cfg_.field.mountainMix, {});
    knob(f, tr("Max slope (rise/run):"), 0.2, 8.0, 2, 0.1, &cfg_.field.maxSlope,
         {});
    col->addLayout(f);
  }
  group(tr("Surface colour — elevation fraction (0=valley, 1=peak)"));
  {
    auto *f = new QFormLayout();
    knob(f, tr("Brown above:"), 0, 2, 2, 0.02, &cfg_.field.colBrownT, {});
    knob(f, tr("Rock above:"), 0, 2, 2, 0.02, &cfg_.field.colRockT, {});
    knob(f, tr("Snow above:"), 0, 2, 2, 0.02, &cfg_.field.colSnowT, {});
    knob(f, tr("Slope→rock:"), 0, 1, 2, 0.02, &cfg_.field.colSlopeT, {});
    col->addLayout(f);
  }
#ifdef VAYU_SIM_GRASS
  group(tr("Grass — flatness 1=flat, 0=vertical"));
  {
    auto *f = new QFormLayout();
    knob(f, tr("Spacing (density):"), 0.15, 4.0, 2, 0.05, &cfg_.flora.spacing,
         tr(" m"));
    knob(f, tr("Blades per cell:"), 1, 12, 0, 1, &cfg_.flora.bladesPerCell, {});
    knob(f, tr("Height limit:"), 0, 1, 2, 0.02, &cfg_.flora.grassMaxFrac, {});
    knob(f, tr("Slope start:"), 0, 1, 2, 0.02, &cfg_.flora.slopeLo, {});
    knob(f, tr("Slope full:"), 0, 1, 2, 0.02, &cfg_.flora.slopeHi, {});
    knob(f, tr("Flower fraction:"), 0, 0.4, 2, 0.01, &cfg_.flora.flowerFrac,
         {});
    knob(f, tr("Blade height mean:"), 0.05, 1.5, 2, 0.02,
         &cfg_.flora.heightMean, tr(" m"));
    knob(f, tr("Blade height σ:"), 0.0, 0.5, 2, 0.01, &cfg_.flora.heightStdDev,
         tr(" m"));
    col->addLayout(f);
  }
  group(tr("Grass shading & lighting"));
  {
    auto *f = new QFormLayout();
    knob(f, tr("Sun intensity:"), 0, 3, 2, 0.05, &cfg_.look.sunIntensity, {});
    knob(f, tr("Ambient strength:"), 0, 3, 2, 0.05, &cfg_.look.ambientStrength,
         {});
    knob(f, tr("Grass brightness:"), 0, 3, 2, 0.05, &cfg_.look.brightness, {});
    knob(f, tr("Tip warmth:"), 0, 2, 2, 0.05, &cfg_.look.tipWarmth, {});
    knob(f, tr("Backlight (SSS):"), 0, 3, 2, 0.05, &cfg_.look.backlight, {});
    knob(f, tr("Sheen:"), 0, 3, 2, 0.05, &cfg_.look.sheen, {});
    knob(f, tr("Vein strength:"), 0, 3, 2, 0.05, &cfg_.look.veinStrength, {});
    knob(f, tr("Root darkness:"), 0, 0.6, 2, 0.02, &cfg_.look.rootDarkness, {});
    knob(f, tr("Face-camera fraction:"), 0, 1, 2, 0.05,
         &cfg_.look.faceCameraFrac, {});
    col->addLayout(f);
  }
#endif // VAYU_SIM_GRASS

  sec->setContentWidget(body);
  root->addWidget(sec);
}

void WorldEditorWidget::buildWorldMeshSection(QVBoxLayout *root) {
  auto *sec =
      new CollapsibleSection(tr("World mesh"), this, /*expanded=*/false);
  auto *body = new QWidget();
  auto *col = new QVBoxLayout(body);

  worldMeshLabel_ = new QLabel(tr("(none)"), body);
  worldMeshLabel_->setWordWrap(true);
  worldMeshLabel_->setStyleSheet(
      QString("color:%1; font-size:11px;").arg(Theme::hex(Theme::kTextMuted)));
  col->addWidget(worldMeshLabel_);

  {
    auto *row = new QHBoxLayout();
    auto *imp = new ui::GhostButton(tr("Import…"), body);
    imp->setToolTip(tr("Import a world mesh (.glb/.gltf/.obj/.stl/.blend). "
                       "Rendered now; collision is a later phase."));
    auto *clr = new ui::DangerButton(tr("Clear"), body);
    connect(imp, &QPushButton::clicked, this,
            &WorldEditorWidget::onImportWorldMesh);
    connect(clr, &QPushButton::clicked, this,
            &WorldEditorWidget::onClearWorldMesh);
    row->addWidget(imp);
    row->addWidget(clr);
    row->addStretch();
    col->addLayout(row);
  }
  {
    auto *form = new QFormLayout();
    worldScale_ = spin(0.0001, 10000.0, 4, 0.1, cfg_.worldScale);
    worldScale_->setToolTip(tr("Mesh units → metres."));
    worldUpAxis_ = new QComboBox(body);
    worldUpAxis_->addItem(tr("Z-up (Blender)"), 0);
    worldUpAxis_->addItem(tr("Y-up (glTF)"), 1);
    worldUpAxis_->setCurrentIndex(cfg_.worldUpAxis == 1 ? 1 : 0);
    auto onEdit = [this] {
      if (worldMeshSyncing_)
        return;
      cfg_.worldScale = static_cast<float>(worldScale_->value());
      cfg_.worldUpAxis = worldUpAxis_->currentData().toInt();
      cfg_.worldMeshOffset =
          QVector3D(worldOffset_[0]->value(), worldOffset_[1]->value(),
                    worldOffset_[2]->value());
      if (!cfg_.worldMeshPath.isEmpty())
        emit worldMeshChanged();
    };
    connect(worldScale_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [onEdit] { onEdit(); });
    connect(worldUpAxis_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [onEdit] { onEdit(); });
    form->addRow(tr("Scale:"), worldScale_);
    form->addRow(tr("Up axis:"), worldUpAxis_);

    // World placement (NED metres): move the imported world so the drone's
    // spawn (the origin) isn't trapped inside/under it. X=north, Y=east,
    // Z=down — so a negative Z lifts the world above the spawn.
    auto *offRow = new QHBoxLayout();
    const char *lbl[3] = {"N", "E", "D"};
    for (int i = 0; i < 3; ++i) {
      worldOffset_[i] = spin(-10000.0, 10000.0, 2, 0.5, cfg_.worldMeshOffset[i],
                             QStringLiteral(" m"));
      worldOffset_[i]->setToolTip(tr("World placement offset (NED): N(orth)/"
                                     "E(ast)/D(own). Negative D lifts it up."));
      offRow->addWidget(new QLabel(tr(lbl[i]), body));
      offRow->addWidget(worldOffset_[i]);
      connect(worldOffset_[i],
              QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
              [onEdit] { onEdit(); });
    }
    form->addRow(tr("Offset:"), offRow);
    col->addLayout(form);
  }

  sec->setContentWidget(body);
  root->addWidget(sec);
}

void WorldEditorWidget::onImportWorldMesh() {
  const QString path = QFileDialog::getOpenFileName(
      this, tr("Import world mesh"), QString(),
      tr("Meshes (*.glb *.gltf *.obj *.stl *.ply *.blend);;All files (*)"));
  if (path.isEmpty())
    return;
  cfg_.worldMeshPath = path;
  if (worldMeshLabel_)
    worldMeshLabel_->setText(QFileInfo(path).fileName());
  emit worldMeshChanged();
}

void WorldEditorWidget::onClearWorldMesh() {
  cfg_.worldMeshPath.clear();
  if (worldMeshLabel_)
    worldMeshLabel_->setText(tr("(none)"));
  emit worldMeshChanged();
}

void WorldEditorWidget::onSaveWorld() {
  syncUiToConfig();
  QString path = QFileDialog::getSaveFileName(this, tr("Save world"), QString(),
                                              tr("Vayu world (*.vworld)"));
  if (path.isEmpty())
    return;
  if (!path.endsWith(".vworld", Qt::CaseInsensitive))
    path += ".vworld";
  QFile f(path);
  if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
    if (fileStatus_)
      fileStatus_->setText(tr("Save failed: %1").arg(f.errorString()));
    return;
  }
  f.write(QJsonDocument(worldToJson(cfg_)).toJson(QJsonDocument::Indented));
  if (fileStatus_)
    fileStatus_->setText(tr("Saved %1 (%2 obstacles)")
                             .arg(QFileInfo(path).fileName())
                             .arg(cfg_.obstacles.size()));
}

void WorldEditorWidget::onLoadWorld() {
  const QString path =
      QFileDialog::getOpenFileName(this, tr("Load world"), QString(),
                                   tr("Vayu world (*.vworld);;All files (*)"));
  if (path.isEmpty())
    return;
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly)) {
    if (fileStatus_)
      fileStatus_->setText(tr("Load failed: %1").arg(f.errorString()));
    return;
  }
  QJsonParseError pe;
  const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &pe);
  if (pe.error != QJsonParseError::NoError || !doc.isObject()) {
    if (fileStatus_)
      fileStatus_->setText(tr("Load failed: invalid world JSON"));
    return;
  }
  loadWorldObject(doc.object());
  if (fileStatus_)
    fileStatus_->setText(tr("Loaded %1 (%2 obstacles)")
                             .arg(QFileInfo(path).fileName())
                             .arg(cfg_.obstacles.size()));
}

void WorldEditorWidget::loadWorldObject(const QJsonObject &o) {
  setConfig(worldFromJson(o));
  // Push to the daemon (env) + renderer/persistence (obstacles) + reload the
  // imported world mesh (a .vworld can carry world_mesh_path/scale/up-axis, so
  // without this the mesh fields update but the geometry never (re)loads).
  emit worldApplied();
  emit obstaclesChanged();
  emit worldMeshChanged();
}

QString WorldEditorWidget::userPresetName() const {
  const QString d = procPreset_ ? procPreset_->currentData().toString() : QString();
  return d.startsWith(kUserPrefix) ? d.mid(kUserPrefix.size()) : QString();
}

void WorldEditorWidget::updatePresetButtons() {
  if (delPresetBtn_)
    delPresetBtn_->setEnabled(!userPresetName().isEmpty());
}

void WorldEditorWidget::refreshUserPresets(const QString &select) {
  QSignalBlocker b(procPreset_);
  while (procPreset_->count() > builtinPresetCount_)
    procPreset_->removeItem(procPreset_->count() - 1);
  const QStringList names =
      userPresetStore().list(vsim::UserPresetStore::Kind::World);
  if (!names.isEmpty())
    procPreset_->insertSeparator(procPreset_->count());
  for (const QString &n : names)
    procPreset_->addItem(n, kUserPrefix + n);
  const int i =
      select.isEmpty() ? -1 : procPreset_->findData(kUserPrefix + select);
  procPreset_->setCurrentIndex(i < 0 ? 0 : i);
  updatePresetButtons();
}

vsim::UserPresetStore::SaveResult
WorldEditorWidget::saveUserPreset(const QString &name, bool overwrite) {
  syncUiToConfig();
  vsim::UserPresetStore st = userPresetStore();
  QString saved;
  const auto r = st.save(vsim::UserPresetStore::Kind::World, name,
                         worldToJson(cfg_), overwrite, &saved);
  if (r != vsim::UserPresetStore::SaveResult::Ok)
    return r;
  refreshUserPresets(saved);
  if (fileStatus_)
    fileStatus_->setText(tr("Saved preset \"%1\" (%2 obstacles)")
                             .arg(saved)
                             .arg(cfg_.obstacles.size()));
  return r;
}

bool WorldEditorWidget::deleteUserPreset(const QString &name) {
  vsim::UserPresetStore st = userPresetStore();
  if (!st.remove(vsim::UserPresetStore::Kind::World, name))
    return false;
  const bool wasShown = userPresetName().compare(name, Qt::CaseInsensitive) == 0;
  refreshUserPresets(wasShown ? QString() : userPresetName());
  if (fileStatus_)
    fileStatus_->setText(tr("Deleted preset \"%1\".").arg(name));
  return true;
}

void WorldEditorWidget::onSavePreset() {
  bool ok = false;
  const QString name = QInputDialog::getText(
      this, tr("Save as preset"),
      tr("Preset name (max %1 characters):")
          .arg(vsim::UserPresetStore::kMaxNameLength),
      QLineEdit::Normal, userPresetName(), &ok);
  if (!ok)
    return;
  auto r = saveUserPreset(name, false);
  if (r == vsim::UserPresetStore::SaveResult::Exists &&
      QMessageBox::question(this, tr("Replace preset"),
                            tr("A preset named \"%1\" exists. Replace it?")
                                .arg(name.trimmed())) == QMessageBox::Yes)
    r = saveUserPreset(name, true);
  if (r != vsim::UserPresetStore::SaveResult::Ok && fileStatus_)
    fileStatus_->setText(tr("Preset not saved: %1")
                             .arg(vsim::UserPresetStore::saveResultText(r)));
}

void WorldEditorWidget::onDeletePreset() {
  const QString name = userPresetName();
  if (name.isEmpty())
    return;
  if (QMessageBox::question(this, tr("Delete preset"),
                            tr("Delete the preset \"%1\"?").arg(name)) !=
      QMessageBox::Yes)
    return;
  if (!deleteUserPreset(name) && fileStatus_)
    fileStatus_->setText(tr("Could not delete preset \"%1\".").arg(name));
}

void WorldEditorWidget::buildObstacleSection(QVBoxLayout *root) {
  auto *sec = new CollapsibleSection(tr("Obstacles"), this, /*expanded=*/false);
  auto *body = new QWidget();
  auto *col = new QVBoxLayout(body);

  auto *hint = new QLabel(
      tr("Click a shape in the 3D view to select; G move · R rotate · S scale, "
         "X/Y/Z to constrain, click to confirm / Esc to cancel."));
  hint->setWordWrap(true);
  hint->setStyleSheet(
      QString("color:%1; font-size:11px;").arg(Theme::hex(Theme::kTextDim)));
  col->addWidget(hint);

  obsList_ = new QListWidget();
  obsList_->setMaximumHeight(120);
  obsList_->setToolTip(tr("Static world shapes. Drone flies through them for "
                          "now (collision is a later phase)."));
  connect(obsList_, &QListWidget::currentRowChanged, this,
          &WorldEditorWidget::onObstacleSelected);
  col->addWidget(obsList_);

  {
    auto *btns = new QHBoxLayout();
    auto *addBox = new ui::GhostButton(tr("+ Box"));
    auto *addSph = new ui::GhostButton(tr("+ Sphere"));
    auto *addCyl = new ui::GhostButton(tr("+ Cyl"));
    auto *del = new ui::DangerButton(tr("Remove"));
    connect(addBox, &QPushButton::clicked, this,
            [this] { onAddObstacle(vsim::Obstacle::Box); });
    connect(addSph, &QPushButton::clicked, this,
            [this] { onAddObstacle(vsim::Obstacle::Sphere); });
    connect(addCyl, &QPushButton::clicked, this,
            [this] { onAddObstacle(vsim::Obstacle::Cylinder); });
    connect(del, &QPushButton::clicked, this,
            &WorldEditorWidget::onRemoveObstacle);
    btns->addWidget(addBox);
    btns->addWidget(addSph);
    btns->addWidget(addCyl);
    btns->addStretch();
    btns->addWidget(del);
    col->addLayout(btns);
  }

  // Per-obstacle edit grid. size: box=full extents (X,Y,Z); sphere=radius is
  // X; cylinder=radius X, height Z. pos/rot in the NED world frame.
  auto *grid = new QGridLayout();
  grid->setHorizontalSpacing(4);
  grid->setVerticalSpacing(2);
  auto field = [&](int row, const QString &label, QDoubleSpinBox **out, int n) {
    grid->addWidget(new QLabel(label), row, 0);
    for (int i = 0; i < n; ++i) {
      out[i] = spin(-100.0, 100.0, 3, 0.1, 0.0);
      connect(out[i], QOverload<double>::of(&QDoubleSpinBox::valueChanged),
              this, [this] { onObstacleFieldChanged(); });
      grid->addWidget(out[i], row, 1 + i);
    }
  };
  field(0, tr("Pos (m):"), obsPos_, 3);
  field(1, tr("Size (m):"), obsSize_, 3);
  for (auto *s : obsSize_)
    s->setRange(0.05, 100.0);
  field(2, tr("Rot (°):"), obsRot_, 3);
  for (auto *s : obsRot_)
    s->setRange(-180.0, 180.0);
  grid->addWidget(new QLabel(tr("Restit.:")), 3, 0);
  obsRest_ = spin(0.0, 1.0, 2, 0.05, 0.3);
  connect(obsRest_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          [this] { onObstacleFieldChanged(); });
  grid->addWidget(obsRest_, 3, 1);
  col->addLayout(grid);

  sec->setContentWidget(body);
  root->addWidget(sec);

  refreshObstacleList();
  syncFormFromSelection();
}

void WorldEditorWidget::onAddObstacle(int type) {
  vsim::Obstacle o;
  o.type = type;
  if (type == vsim::Obstacle::Sphere)
    o.size = {0.5f, 0.5f, 0.5f};
  if (type == vsim::Obstacle::Cylinder)
    o.size = {0.4f, 0.4f, 1.0f};
  cfg_.obstacles.push_back(o);
  refreshObstacleList();
  if (obsList_)
    obsList_->setCurrentRow(cfg_.obstacles.size() - 1);
  emit obstaclesChanged();
}

void WorldEditorWidget::onRemoveObstacle() {
  const int r = obsList_ ? obsList_->currentRow() : -1;
  if (r < 0 || r >= cfg_.obstacles.size())
    return;
  cfg_.obstacles.remove(r);
  refreshObstacleList();
  emit obstaclesChanged();
}

void WorldEditorWidget::onObstacleSelected(int row) {
  syncFormFromSelection();
  emit obstacleSelectionChanged(row);
}

void WorldEditorWidget::setObstacleFromGizmo(int index, QVector3D pos,
                                             QVector3D size, QVector3D rotate) {
  if (index < 0 || index >= cfg_.obstacles.size())
    return;
  vsim::Obstacle &o = cfg_.obstacles[index];
  o.pos = pos;
  o.size = size;
  o.rotate = rotate;
  if (obsList_ && obsList_->currentRow() != index)
    obsList_->setCurrentRow(index);
  syncFormFromSelection();
  refreshObstacleList();
  emit obstaclesChanged();
}

void WorldEditorWidget::selectObstacleRow(int index) {
  if (!obsList_)
    return;
  if (index >= 0 && index < cfg_.obstacles.size()) {
    if (obsList_->currentRow() != index)
      obsList_->setCurrentRow(index);
  } else {
    obsList_->setCurrentRow(-1);
  }
}

void WorldEditorWidget::onObstacleFieldChanged() {
  if (obsSyncing_)
    return;
  const int r = obsList_ ? obsList_->currentRow() : -1;
  if (r < 0 || r >= cfg_.obstacles.size())
    return;
  vsim::Obstacle &o = cfg_.obstacles[r];
  o.pos =
      QVector3D(obsPos_[0]->value(), obsPos_[1]->value(), obsPos_[2]->value());
  o.size = QVector3D(obsSize_[0]->value(), obsSize_[1]->value(),
                     obsSize_[2]->value());
  o.rotate =
      QVector3D(obsRot_[0]->value(), obsRot_[1]->value(), obsRot_[2]->value());
  o.restitution = static_cast<float>(obsRest_->value());
  refreshObstacleList();
  if (obsList_)
    obsList_->setCurrentRow(r); // refresh keeps selection
  emit obstaclesChanged();
}

void WorldEditorWidget::refreshObstacleList() {
  if (!obsList_)
    return;
  const int keep = obsList_->currentRow();
  QSignalBlocker b(obsList_);
  obsList_->clear();
  static const char *kName[] = {"Box", "Sphere", "Cylinder"};
  for (const vsim::Obstacle &o : cfg_.obstacles) {
    const int t = (o.type >= 0 && o.type <= 2) ? o.type : 0;
    obsList_->addItem(QString("%1  (%2, %3, %4)")
                          .arg(kName[t])
                          .arg(o.pos.x(), 0, 'f', 1)
                          .arg(o.pos.y(), 0, 'f', 1)
                          .arg(o.pos.z(), 0, 'f', 1));
  }
  if (keep >= 0 && keep < cfg_.obstacles.size())
    obsList_->setCurrentRow(keep);
}

void WorldEditorWidget::syncFormFromSelection() {
  const int r = obsList_ ? obsList_->currentRow() : -1;
  const bool ok = (r >= 0 && r < cfg_.obstacles.size());
  obsSyncing_ = true;
  if (ok) {
    const vsim::Obstacle &o = cfg_.obstacles[r];
    const QVector3D pv[3] = {o.pos, o.size, o.rotate};
    QDoubleSpinBox **grp[3] = {obsPos_, obsSize_, obsRot_};
    for (int g = 0; g < 3; ++g)
      for (int i = 0; i < 3; ++i)
        grp[g][i]->setValue(pv[g][i]);
    obsRest_->setValue(o.restitution);
  }
  for (int i = 0; i < 3; ++i) {
    obsPos_[i]->setEnabled(ok);
    obsSize_[i]->setEnabled(ok);
    obsRot_[i]->setEnabled(ok);
  }
  obsRest_->setEnabled(ok);
  obsSyncing_ = false;
}

void WorldEditorWidget::syncConfigToUi() {
  lastPreset_ = cfg_.worldPreset;
  lastPresetSeed_ = cfg_.proceduralSeed;
  gravity_->setValue(cfg_.gravity);
  groundZ_->setValue(cfg_.ground_z);
  rest_->setValue(cfg_.restitution);
  linDrag_->setValue(cfg_.linear_drag);
  angDrag_->setValue(cfg_.angular_drag);
  rightGain_->setValue(cfg_.ground_right_gain);
  rightDamp_->setValue(cfg_.ground_right_damp);
  syncProcToUi();
}

// Refresh every widget the procedural section's pull() reads (biome, preset,
// seed, size, resolution, knobs) from cfg_, silently (procSyncing_), so a
// later pull() cannot write stale values back over an applied preset.
void WorldEditorWidget::syncProcToUi() {
  procSyncing_ = true;
  if (procBiome_) {
    // Drop the read-only "<biome> (preset)" entry of a previous preset.
    for (int i = procBiome_->count() - 1; i >= 0; --i)
      if (vsim::isFieldArenaBiome(procBiome_->itemData(i).toString()) &&
          procBiome_->itemData(i).toString() != cfg_.proceduralBiome)
        procBiome_->removeItem(i);
    int idx = procBiome_->findData(cfg_.proceduralBiome);
    if (idx < 0 && vsim::isFieldArenaBiome(cfg_.proceduralBiome)) {
      // Preset-only biome: show it (read-only entry) rather than "None".
      procBiome_->addItem(tr("%1 (preset)").arg(cfg_.proceduralBiome),
                          cfg_.proceduralBiome);
      idx = procBiome_->count() - 1;
    }
    procBiome_->setCurrentIndex(idx >= 0 ? idx : 0);
  }
  if (procPreset_) {
    const int pi = procPreset_->findData(cfg_.worldPreset);
    procPreset_->setCurrentIndex(pi >= 0 ? pi : 0);
  }
  if (procSeed_)
    procSeed_->setValue(static_cast<int>(cfg_.proceduralSeed));
  for (auto &f : procKnobSyncers_)
    f(); // refresh the tuning knobs from cfg_
  procSyncing_ = false;
}

void WorldEditorWidget::syncUiToConfig() {
  cfg_.gravity = static_cast<float>(gravity_->value());
  cfg_.ground_z = static_cast<float>(groundZ_->value());
  cfg_.restitution = static_cast<float>(rest_->value());
  cfg_.linear_drag = static_cast<float>(linDrag_->value());
  cfg_.angular_drag = static_cast<float>(angDrag_->value());
  cfg_.ground_right_gain = static_cast<float>(rightGain_->value());
  cfg_.ground_right_damp = static_cast<float>(rightDamp_->value());
}

void WorldEditorWidget::onApply() {
  syncUiToConfig();
  emit worldApplied();
}

void WorldEditorWidget::setConfig(const vsim::WorldConfig &c) {
  cfg_ = c;
  syncConfigToUi();
  refreshObstacleList();
  syncFormFromSelection();
  worldMeshSyncing_ = true;
  if (worldMeshLabel_)
    worldMeshLabel_->setText(cfg_.worldMeshPath.isEmpty()
                                 ? tr("(none)")
                                 : QFileInfo(cfg_.worldMeshPath).fileName());
  if (worldScale_)
    worldScale_->setValue(cfg_.worldScale);
  if (worldUpAxis_)
    worldUpAxis_->setCurrentIndex(cfg_.worldUpAxis == 1 ? 1 : 0);
  for (int i = 0; i < 3; ++i)
    if (worldOffset_[i])
      worldOffset_[i]->setValue(cfg_.worldMeshOffset[i]);
  worldMeshSyncing_ = false;
}
