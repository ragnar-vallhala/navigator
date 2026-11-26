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
// tst_worldeditor.cpp — World tab preset wiring, driven through the real
// widget handlers (offscreen). Guards the bug where applying a preset left the
// biome widgets stale, so the next pull() (knob edit + Generate, or a seed edit)
// wrote the previous biome back over the preset. Also: procedural worlds are
// endless, so there is no finite meadow/size/resolution to pick.
#include <QtTest/QtTest>

#include "ui/widgets/WorldEditorWidget.h"
#include "vsim/ProceduralWorld.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QSignalSpy>
#include <QSpinBox>
#include <QTemporaryDir>

class TstWorldEditor : public QObject {
  Q_OBJECT
private:
  static void pick(WorldEditorWidget &we, const char *id) {
    const int i = we.procPreset_->findData(QString::fromLatin1(id));
    QVERIFY(i > 0);
    we.procPreset_->setCurrentIndex(i); // the user's combo path
  }
  static void clickGenerate(WorldEditorWidget &we) {
    for (QPushButton *b : we.findChildren<QPushButton *>())
      if (b->text() == QLatin1String("Generate")) {
        b->click();
        return;
      }
    QFAIL("Generate button not found");
  }
  static int presetBiomeEntries(const WorldEditorWidget &we) {
    int n = 0;
    for (int i = 0; i < we.procBiome_->count(); ++i)
      if (we.procBiome_->itemData(i).toString() != QLatin1String("endless") &&
          !we.procBiome_->itemData(i).toString().isEmpty())
        ++n;
    return n;
  }

  QTemporaryDir dataDir_; // private user-preset store (XDG_DATA_HOME)

  static QPushButton *button(WorldEditorWidget &we, const QString &text) {
    for (QPushButton *b : we.findChildren<QPushButton *>())
      if (b->text() == text)
        return b;
    return nullptr;
  }

private slots:
  void initTestCase() {
    QVERIFY(dataDir_.isValid());
    qputenv("XDG_DATA_HOME", dataDir_.path().toLocal8Bit());
  }

  void userPresetSaveListSelectDelete() {
    WorldEditorWidget we;
    const int builtins = we.procPreset_->count();
    QPushButton *del = button(we, "Delete preset");
    QVERIFY(button(we, "Save as preset\u2026"));
    QVERIFY(del);
    QVERIFY(!del->isEnabled());

    vsim::WorldConfig w;
    QVERIFY(vsim::applyWorldPreset(w, "forest", 5));
    we.setConfig(w);
    const auto nObs = we.config().obstacles.size();
    QVERIFY(nObs > 0);
    QCOMPARE(we.saveUserPreset(" My Forest ", false),
             vsim::UserPresetStore::SaveResult::Ok);
    QCOMPARE(we.procPreset_->count(), builtins + 2); // separator + entry
    QCOMPARE(we.userPresetName(), QString("My Forest"));
    QVERIFY(del->isEnabled());

    // Another world, then pick the user preset: the saved world comes back.
    pick(we, "desert");
    QVERIFY(!del->isEnabled());
    QCOMPARE(we.config().proceduralBiome, QString("desert"));
    QSignalSpy applied(&we, &WorldEditorWidget::worldApplied);
    QSignalSpy obs(&we, &WorldEditorWidget::obstaclesChanged);
    const int i = we.procPreset_->findData(QStringLiteral("user:My Forest"));
    QVERIFY(i > 0);
    we.procPreset_->setCurrentIndex(i);
    QCOMPARE(we.config().proceduralBiome, QString("forest"));
    QCOMPARE(we.config().obstacles.size(), nObs);
    QCOMPARE(we.userPresetName(), QString("My Forest")); // combo stays on it
    QVERIFY(del->isEnabled());
    QVERIFY(!applied.isEmpty() && !obs.isEmpty()); // pushed like Load...

    QVERIFY(we.deleteUserPreset("My Forest"));
    QCOMPARE(we.procPreset_->count(), builtins);
    QCOMPARE(we.procPreset_->currentData().toString(), QString());
    QVERIFY(!del->isEnabled());
    WorldEditorWidget we2; // a later launch sees it gone
    QCOMPARE(we2.procPreset_->count(), builtins);
  }

  void userPresetRejectsBadAndBuiltInNames() {
    WorldEditorWidget we;
    using R = vsim::UserPresetStore::SaveResult;
    QCOMPARE(we.saveUserPreset("../x", false), R::InvalidName);
    QCOMPARE(we.saveUserPreset("", false), R::InvalidName);
    QCOMPARE(we.saveUserPreset("forest", false), R::Reserved);
    QCOMPARE(we.saveUserPreset("Custom", false), R::Reserved);
    QCOMPARE(we.saveUserPreset("W1", false), R::Ok);
    QCOMPARE(we.saveUserPreset("w1", false), R::Exists);
    QVERIFY(we.deleteUserPreset("W1"));
  }

  void presetSyncsWidgets() {
    WorldEditorWidget we;
    pick(we, "desert");
    const auto &c = we.config();
    QCOMPARE(c.proceduralBiome, QString("desert"));
    QCOMPARE(c.worldPreset, QString("desert"));
    // Every widget pull() reads shows the preset, not the previous biome.
    QCOMPARE(we.procBiome_->currentData().toString(), QString("desert"));
    QCOMPARE(float(we.procKnobSpins_[0]->value()), c.field.heightM);
    QCOMPARE(we.procPreset_->currentData().toString(), QString("desert"));
    // Switching presets drops the previous "(preset)" biome entry.
    QCOMPARE(presetBiomeEntries(we), 1);
    pick(we, "alpine");
    QCOMPARE(presetBiomeEntries(we), 1);
    QCOMPARE(we.procBiome_->currentData().toString(), QString("alpine"));
  }

  void knobEditThenGenerateKeepsPreset() {
    WorldEditorWidget we;
    pick(we, "desert");
    const auto preset = we.config(); // copy of the applied preset
    we.procKnobSpins_[0]->setValue(20.0); // user edits Peak height
    QCOMPARE(we.config().worldPreset, QString()); // now Custom
    QCOMPARE(we.procPreset_->currentIndex(), 0);
    clickGenerate(we);
    const auto &c = we.config();
    QCOMPARE(c.proceduralBiome, QString("desert"));
    QCOMPARE(c.proceduralSizeM, preset.proceduralSizeM);
    QCOMPARE(c.field.shape, preset.field.shape);
    QCOMPARE(c.field.palette, preset.field.palette);
    QCOMPARE(c.field.featureM, preset.field.featureM);
    QCOMPARE(c.field.heightM, 20.0f); // the edit survived Generate
    QCOMPARE(c.obstacles.size(), preset.obstacles.size());
  }

  void seedEditThenGenerateRerollsSamePreset() {
    WorldEditorWidget we;
    pick(we, "alpine");
    const auto preset = we.config();
    we.procSeed_->setValue(77); // user edits the seed field
    // Merely editing the seed must not clobber the biome.
    QCOMPARE(we.config().proceduralBiome, QString("alpine"));
    QCOMPARE(we.config().proceduralSizeM, preset.proceduralSizeM);
    clickGenerate(we);
    const auto &c = we.config();
    QCOMPARE(c.proceduralSeed, quint32(77));
    QCOMPARE(c.worldPreset, QString("alpine"));
    QCOMPARE(c.proceduralBiome, QString("alpine"));
    QCOMPARE(c.proceduralSizeM, preset.proceduralSizeM);
    QCOMPARE(c.field.heightM, preset.field.heightM);
    QCOMPARE(c.field.seed, quint32(77));
  }

  // The relief height survives a save and a restart (.vworld and the restart path
  // both go through the procedural JSON); an old file without it loads as none.
  void reliefSurvivesSerialization() {
    vsim::WorldConfig w;
    QVERIFY(vsim::applyWorldPreset(w, "alpine", 5));
    QVERIFY(w.field.reliefM > 0.0f);
    const QString json = WorldEditorWidget::serializeProcedural(w);
    vsim::WorldConfig back;
    QVERIFY(WorldEditorWidget::deserializeProcedural(back, json));
    QCOMPARE(back.field.reliefM, w.field.reliefM);
    vsim::WorldConfig old;
    old.field.reliefM = 0.25f; // stale value must not leak into an old file
    QVERIFY(WorldEditorWidget::deserializeProcedural(
        old, QStringLiteral("{\"biome\":\"alpine\",\"seed\":5}")));
    QCOMPARE(old.field.reliefM, 0.0f);
  }

  void legacyFiniteMeadowLoadsAsEndless() {
    WorldEditorWidget we;
    // No finite arena entry and no size/resolution controls any more.
    for (int i = 0; i < we.procBiome_->count(); ++i)
      QVERIFY(we.procBiome_->itemData(i).toString() != QLatin1String("meadow"));
    // An old save of the finite meadow (with its size/resolution) streams now.
    vsim::WorldConfig w;
    QVERIFY(WorldEditorWidget::deserializeProcedural(
        w, QStringLiteral("{\"biome\":\"meadow\",\"seed\":21,"
                          "\"size_m\":300,\"resolution\":192}")));
    QCOMPARE(w.proceduralBiome, QString("endless"));
    QCOMPARE(w.field.palette, int(vsim::procgen::kPalMeadow)); // keeps its look
    QCOMPARE(w.proceduralSeed, quint32(21));
    QVERIFY(vsim::isStreamedBiome(w.proceduralBiome));
    we.setConfig(w);
    QCOMPARE(we.procBiome_->currentData().toString(), QString("endless"));
  }

  void meadowPresetKeepsMeadowLook() {
    vsim::WorldConfig w;
    QVERIFY(vsim::applyWorldPreset(w, "meadow", 9));
    QVERIFY(vsim::isStreamedBiome(w.proceduralBiome));
    QCOMPARE(w.field.palette, int(vsim::procgen::kPalMeadow));
    QCOMPARE(w.field.seed, quint32(9));
    QCOMPARE(w.flora.seed, quint32(9));
    // Not the Endless look: a different palette from the bare endless default.
    QVERIFY(w.field.palette != vsim::procgen::FieldParams{}.palette);
  }

  void everyPresetStreams() {
    for (const auto &p : vsim::worldPresetList()) {
      vsim::WorldConfig w;
      QVERIFY(vsim::applyWorldPreset(w, p.first, 5));
      QVERIFY2(vsim::isStreamedBiome(w.proceduralBiome), qPrintable(p.first));
    }
    QVERIFY(!vsim::isStreamedBiome(QString()));
  }

  // The barometer section: its spinboxes stop at the engine's limits (the same
  // numbers tst_sitl_engine proves SimWorker accepts at the top and refuses just
  // beyond), Apply is staged, and applying emits baroApplied with the values.
  void baroSectionStagesAndAppliesWithinTheEnginesLimits() {
    WorldEditorWidget we;
    QCOMPARE(we.baroNoise_->maximum(), 500.0);
    QCOMPARE(we.baroWalk_->maximum(), 50.0);
    QCOMPARE(we.baroLag_->maximum(), 2000.0);
    QCOMPARE(we.baroRate_->maximum(), 1000.0);
    QCOMPARE(we.baroNoise_->minimum(), 0.0);
    QCOMPARE(we.baroRate_->minimum(), 0.0);
    QVERIFY(!we.baroApplyBtn_->isEnabled()); // clean at start
    QSignalSpy spy(&we, &WorldEditorWidget::baroApplied);
    we.baroNoise_->setValue(3.0);
    we.baroWalk_->setValue(0.25);
    we.baroLag_->setValue(120.0);
    we.baroRate_->setValue(25.0);
    QVERIFY(we.baroApplyBtn_->isEnabled()); // an edit makes it dirty
    QCOMPARE(spy.count(), 0);               // staged: nothing sent yet
    we.baroApplyBtn_->click();
    QCOMPARE(spy.count(), 1);
    QVERIFY(!we.baroApplyBtn_->isEnabled()); // clean until the next edit
    const vsim::BaroErrConfig c = we.baroConfig();
    QCOMPARE(c.noisePa, 3.0f);
    QCOMPARE(c.walkPaPerSqrtS, 0.25f);
    QCOMPARE(c.lagMs, 120.0f);
    QCOMPARE(c.rateHz, 25.0f);
    // An out-of-range value is clamped by the control, never carried through.
    we.baroNoise_->setValue(9999.0);
    we.baroApplyBtn_->click();
    QCOMPARE(we.baroConfig().noisePa, 500.0f);
  }

  void setBaroConfigSyncsWidgetsAndStaysClean() {
    WorldEditorWidget we;
    vsim::BaroErrConfig c;
    c.noisePa = 7.0f;
    c.lagMs = 200.0f;
    QSignalSpy spy(&we, &WorldEditorWidget::baroApplied);
    we.setBaroConfig(c);
    QCOMPARE(we.baroNoise_->value(), 7.0);
    QCOMPARE(we.baroLag_->value(), 200.0);
    QVERIFY(!we.baroApplyBtn_->isEnabled()); // a restore is not an edit
    QCOMPARE(spy.count(), 0);                // and sends nothing by itself
  }

  // The GPS section: its controls stop at the engine's limits (the numbers
  // tst_sitl_engine proves SimWorker accepts at the top and refuses beyond),
  // Apply is staged, and a restore is not an edit.
  void gpsSectionStagesAndAppliesWithinTheEnginesLimits() {
    WorldEditorWidget we;
    QCOMPARE(we.gpsRate_->minimum(), 1.0);
    QCOMPARE(we.gpsRate_->maximum(), 50.0);
    QCOMPARE(we.gpsPosNoise_->maximum(), 50.0);
    QCOMPARE(we.gpsVelNoise_->maximum(), 5.0);
    QCOMPARE(we.gpsLag_->maximum(), 1000.0);
    QCOMPARE(we.gpsSats_->minimum(), 4.0);
    QCOMPARE(we.gpsSats_->maximum(), 32.0);
    QVERIFY(!we.gpsApplyBtn_->isEnabled()); // clean at start
    QSignalSpy spy(&we, &WorldEditorWidget::gpsApplied);
    we.gpsEnable_->setChecked(true);
    we.gpsRate_->setValue(5.0);
    we.gpsPosNoise_->setValue(1.5);
    we.gpsVelNoise_->setValue(0.2);
    we.gpsLag_->setValue(120.0);
    we.gpsSats_->setValue(9.0);
    QVERIFY(we.gpsApplyBtn_->isEnabled());
    QCOMPARE(spy.count(), 0); // staged: nothing sent yet
    we.gpsApplyBtn_->click();
    QCOMPARE(spy.count(), 1);
    QVERIFY(!we.gpsApplyBtn_->isEnabled());
    const vsim::GpsConfig c = we.gpsConfig();
    QVERIFY(c.enabled);
    QCOMPARE(c.rateHz, 5.0f);
    QCOMPARE(c.posNoiseM, 1.5f);
    QCOMPARE(c.velNoiseMps, 0.2f);
    QCOMPARE(c.lagMs, 120.0f);
    QCOMPARE(c.sats, 9);
    // Out of range is clamped by the control, never carried through.
    we.gpsRate_->setValue(9999.0);
    we.gpsApplyBtn_->click();
    QCOMPARE(we.gpsConfig().rateHz, 50.0f);
    // Even the checkbox alone is an edit.
    we.gpsEnable_->setChecked(false);
    QVERIFY(we.gpsApplyBtn_->isEnabled());
  }

  void setGpsConfigSyncsWidgetsAndStaysClean() {
    WorldEditorWidget we;
    vsim::GpsConfig c;
    c.enabled = true;
    c.rateHz = 8.0f;
    c.sats = 14;
    QSignalSpy spy(&we, &WorldEditorWidget::gpsApplied);
    we.setGpsConfig(c);
    QVERIFY(we.gpsEnable_->isChecked());
    QCOMPARE(we.gpsRate_->value(), 8.0);
    QCOMPARE(we.gpsSats_->value(), 14.0);
    QVERIFY(!we.gpsApplyBtn_->isEnabled()); // a restore is not an edit
    QCOMPARE(spy.count(), 0);               // and sends nothing by itself
  }

  // The receiver fault is an event: each button asks once, nothing is staged,
  // and ending it asks for mode None.
  void gpsFaultIsRequestedAtOnceAndEndsWithModeNone() {
    WorldEditorWidget we;
    QSignalSpy spy(&we, &WorldEditorWidget::gpsGlitchRequested);
    QCOMPARE(we.glitchN_->maximum(), 100000.0);
    QCOMPARE(we.glitchN_->minimum(), -100000.0);
    QCOMPARE(we.glitchDur_->maximum(), 3600.0);
    we.glitchN_->setValue(30.0);
    we.glitchE_->setValue(-40.0);
    we.glitchDur_->setValue(20.0);
    QCOMPARE(spy.count(), 0); // editing does not send
    we.glitchSendBtn_->click();
    QCOMPARE(spy.count(), 1);
    QCOMPARE(we.gpsGlitchConfig().mode, vsim::GpsGlitchMode::Offset);
    QCOMPARE(we.gpsGlitchConfig().offsetN, 30.0f);
    QCOMPARE(we.gpsGlitchConfig().offsetE, -40.0f);
    QCOMPARE(we.gpsGlitchConfig().durationS, 20.0f);
    we.glitchMode_->setCurrentIndex(we.glitchMode_->findData(
        static_cast<int>(vsim::GpsGlitchMode::Dropout)));
    QVERIFY(!we.glitchN_->isEnabled()); // an offset means nothing for a dropout
    we.glitchSendBtn_->click();
    QCOMPARE(we.gpsGlitchConfig().mode, vsim::GpsGlitchMode::Dropout);
    we.glitchClearBtn_->click();
    QCOMPARE(spy.count(), 3);
    QCOMPARE(we.gpsGlitchConfig().mode, vsim::GpsGlitchMode::None);
  }

  void theIgnoredMessageCountShowsOnlyWhenNonZero() {
    WorldEditorWidget we;
    // The label's own state: the section holding it starts collapsed, so what
    // is on screen depends on that too.
    QVERIFY(we.engineIgnored_->isHidden());
    we.setEngineIgnoredCount(3);
    QVERIFY(!we.engineIgnored_->isHidden());
    QVERIFY(we.engineIgnored_->text().contains("3"));
    we.setEngineIgnoredCount(0);
    QVERIFY(we.engineIgnored_->isHidden());
  }

  void setConfigRestoresWidgets() {
    WorldEditorWidget we;
    vsim::WorldConfig w;
    QVERIFY(vsim::applyWorldPreset(w, "forest", 5));
    we.setConfig(w);
    QCOMPARE(we.procBiome_->currentData().toString(), QString("forest"));
    clickGenerate(we); // unchanged id/seed: keeps everything
    QCOMPARE(we.config().proceduralBiome, QString("forest"));
    QCOMPARE(we.config().obstacles.size(), w.obstacles.size());
  }
};

QTEST_MAIN(TstWorldEditor)
#include "tst_worldeditor.moc"
