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

#include "../../vsim/UserPresets.h"
#include "../../vsim/VsimTypes.h"

#include <QWidget>

#include <functional>
#include <vector>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QListWidget;
class QPushButton;
class QSpinBox;
class QVBoxLayout;

// WorldEditorWidget — the "World" tab's environment + aerodynamics, plus a
// list of static 3D obstacles (boxes/spheres/cylinders). Apply pushes a
// WorldConfig to the daemon; obstacle edits emit obstaclesChanged() live so
// the renderer + persistence stay in sync without an explicit Apply.
class WorldEditorWidget : public QWidget {
  Q_OBJECT
  friend class TstWorldEditor; // headless test drives the real handlers
public:
  explicit WorldEditorWidget(QWidget *parent = nullptr);

  const vsim::WorldConfig &config() const { return cfg_; }
  void setConfig(const vsim::WorldConfig &c);

  // Wind & turbulence (its own staged Apply button, mockup World tab).
  const vsim::WindConfig &windConfig() const { return wind_; }
  void setWindConfig(const vsim::WindConfig &w);
  // Barometer error (its own staged Apply): noise, drift, lag, output rate.
  const vsim::BaroErrConfig &baroConfig() const { return baro_; }
  void setBaroConfig(const vsim::BaroErrConfig &b);

  // Procedural-world settings (biome, seed, size, field, flora, look, preset id)
  // as a compact JSON string, using the same keys as the .vworld export. Used
  // by SimulatorWidget to persist knob tweaks across restarts. Empty string if
  // no procedural biome is set; deserialize leaves `w` untouched on bad input.
  static QString serializeProcedural(const vsim::WorldConfig &w);
  static bool deserializeProcedural(vsim::WorldConfig &w, const QString &json);
  // User presets: saved worlds listed after the built-in world presets.
  vsim::UserPresetStore::SaveResult saveUserPreset(const QString &name,
                                                   bool overwrite);
  bool deleteUserPreset(const QString &name);
  QString userPresetName() const; // "" if the combo is on Custom / a built-in

  // Live wind readout fed from the pose stream (instantaneous speed + heading).
  void setWindReadout(float speedMs, float dirDeg);

  // Apply a 3D-gizmo edit of obstacle `index` (world pos/size/rotate).
  void setObstacleFromGizmo(int index, QVector3D pos, QVector3D size,
                            QVector3D rotate);
  // Select an obstacle row (driven by a click in the 3D view).
  void selectObstacleRow(int index);

signals:
  void worldApplied();     // user committed env → push to daemon + persist
  void windApplied();      // user committed wind → push to daemon + persist
  void baroApplied();      // user committed the barometer error → push + persist
  void obstaclesChanged(); // obstacle list/edit changed → preview + persist
  void obstacleSelectionChanged(int index); // list selection → 3D highlight
  void worldMeshChanged(); // imported mesh path/scale/up-axis changed

private slots:
  void onApply();
  void onApplyWind();
  void onApplyBaro();
  void onImportWorldMesh();
  void onClearWorldMesh();
  void onAddObstacle(int type);
  void onRemoveObstacle();
  void onObstacleSelected(int row);
  void onObstacleFieldChanged();
  void onSaveWorld(); // export env + obstacles to a portable .vworld (JSON)
  void onLoadWorld(); // import a .vworld
  void onSavePreset();
  void onDeletePreset();

private:
  void buildUi();
  void refreshUserPresets(const QString &select);
  void updatePresetButtons();
  // Load a world from its portable JSON and push it like Load... does.
  void loadWorldObject(const QJsonObject &o);
  void buildWindSection(QVBoxLayout *root);
  void buildBaroSection(QVBoxLayout *root);
  void buildProceduralSection(QVBoxLayout *root);
  void buildProceduralTuningSection(QVBoxLayout *root);
  void buildWorldMeshSection(QVBoxLayout *root);
  void buildObstacleSection(QVBoxLayout *root);
  void syncConfigToUi();
  void syncUiToConfig();
  void syncProcToUi();          // cfg_ -> procedural section widgets
  void syncWindToUi();          // wind_ -> spinboxes/checkbox
  void syncUiToWind();          // spinboxes/checkbox -> wind_
  void syncBaroToUi();          // baro_ -> spinboxes
  void syncUiToBaro();          // spinboxes -> baro_
  void refreshObstacleList();   // list labels from cfg_.obstacles
  void syncFormFromSelection(); // selected obstacle → form fields

  vsim::WorldConfig cfg_;

  QDoubleSpinBox *gravity_ = nullptr;
  QDoubleSpinBox *groundZ_ = nullptr;
  QDoubleSpinBox *rest_ = nullptr;
  QDoubleSpinBox *linDrag_ = nullptr;
  QDoubleSpinBox *angDrag_ = nullptr;
  QDoubleSpinBox *rightGain_ = nullptr; // ground-contact righting gain
  QDoubleSpinBox *rightDamp_ = nullptr; // righting damping

  // Wind & turbulence.
  vsim::WindConfig wind_;
  QDoubleSpinBox *windN_ = nullptr;
  QDoubleSpinBox *windE_ = nullptr;
  QDoubleSpinBox *windD_ = nullptr;
  QDoubleSpinBox *gustAmp_ = nullptr;
  QDoubleSpinBox *gustPeriod_ = nullptr;
  QDoubleSpinBox *turbSigma_ = nullptr;
  QCheckBox *windEnable_ = nullptr;
  QPushButton *windApplyBtn_ = nullptr; // enabled only when dirty
  class QLabel *windReadout_ = nullptr; // live speed/dir from the pose stream
  bool windSyncing_ = false;            // guard sync from marking dirty
  // Barometer error.
  vsim::BaroErrConfig baro_;
  QDoubleSpinBox *baroNoise_ = nullptr;
  QDoubleSpinBox *baroWalk_ = nullptr;
  QDoubleSpinBox *baroLag_ = nullptr;
  QDoubleSpinBox *baroRate_ = nullptr;
  QPushButton *baroApplyBtn_ = nullptr; // enabled only when dirty
  bool baroSyncing_ = false;

  // Obstacle editor.
  QListWidget *obsList_ = nullptr;
  QDoubleSpinBox *obsPos_[3] = {nullptr, nullptr, nullptr};
  QDoubleSpinBox *obsSize_[3] = {nullptr, nullptr, nullptr};
  QDoubleSpinBox *obsRot_[3] = {nullptr, nullptr, nullptr};
  QDoubleSpinBox *obsRest_ = nullptr;
  bool obsSyncing_ = false;            // guard form-sync from re-emitting
  class QLabel *fileStatus_ = nullptr; // save/load feedback

  // World-mesh import controls.
  class QLabel *worldMeshLabel_ = nullptr;
  QDoubleSpinBox *worldScale_ = nullptr;
  QComboBox *worldUpAxis_ = nullptr;
  QDoubleSpinBox *worldOffset_[3] = {nullptr, nullptr,
                                     nullptr}; // NED placement
  bool worldMeshSyncing_ = false;

  // Procedural world controls (Phase 0). Biome "None" = off (use imported
  // mesh); selecting a biome generates and takes precedence.
  QComboBox *procBiome_ = nullptr;
  QString lastPreset_; // preset id/seed at the last apply or load
  quint32 lastPresetSeed_ = 0;
  QComboBox *procPreset_ = nullptr; // world preset (biome + terrain + obstacles)
  QPushButton *savePresetBtn_ = nullptr;
  QPushButton *delPresetBtn_ = nullptr;
  int builtinPresetCount_ = 0; // procPreset_ entries before the user presets
  QComboBox *windPreset_ = nullptr; // wind preset (calm / breezy / gusty)
  QSpinBox *procSeed_ = nullptr;
  bool procSyncing_ = false;
  // Generator-tuning knobs: each pushes a sync lambda (cfg_ -> spinbox) here so
  // setConfig can refresh them all without a member per knob.
  std::vector<std::function<void()>> procKnobSyncers_;
  std::vector<QDoubleSpinBox *> procKnobSpins_; // same order as the syncers
};
