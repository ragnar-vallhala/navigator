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
// biome/size/resolution widgets stale, so the next pull() (knob edit + Generate,
// or a seed edit) wrote meadow 256/192 back over the preset.
#include <QtTest/QtTest>

#include "ui/widgets/WorldEditorWidget.h"
#include "vsim/ProceduralWorld.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QSpinBox>

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
      if (we.procBiome_->itemData(i).toString() != QLatin1String("meadow") &&
          we.procBiome_->itemData(i).toString() != QLatin1String("endless") &&
          !we.procBiome_->itemData(i).toString().isEmpty())
        ++n;
    return n;
  }

private slots:
  void presetSyncsWidgets() {
    WorldEditorWidget we;
    pick(we, "desert");
    const auto &c = we.config();
    QCOMPARE(c.proceduralBiome, QString("desert"));
    QCOMPARE(c.worldPreset, QString("desert"));
    // Every widget pull() reads shows the preset, not the old meadow values.
    QCOMPARE(we.procBiome_->currentData().toString(), QString("desert"));
    QCOMPARE(float(we.procSizeM_->value()), c.proceduralSizeM);
    QCOMPARE(we.procResolution_->value(), c.proceduralResolution);
    QCOMPARE(float(we.procSizeM_->value()), 400.0f);
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
    QCOMPARE(c.proceduralResolution, preset.proceduralResolution);
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
    // Merely editing the seed must not clobber biome/size/resolution.
    QCOMPARE(we.config().proceduralBiome, QString("alpine"));
    QCOMPARE(we.config().proceduralSizeM, preset.proceduralSizeM);
    QCOMPARE(we.config().proceduralResolution, preset.proceduralResolution);
    clickGenerate(we);
    const auto &c = we.config();
    QCOMPARE(c.proceduralSeed, quint32(77));
    QCOMPARE(c.worldPreset, QString("alpine"));
    QCOMPARE(c.proceduralBiome, QString("alpine"));
    QCOMPARE(c.proceduralSizeM, preset.proceduralSizeM);
    QCOMPARE(c.proceduralResolution, preset.proceduralResolution);
    QCOMPARE(c.field.heightM, preset.field.heightM);
    QCOMPARE(c.field.seed, quint32(77));
  }

  void setConfigRestoresWidgets() {
    WorldEditorWidget we;
    vsim::WorldConfig w;
    QVERIFY(vsim::applyWorldPreset(w, "forest", 5));
    we.setConfig(w);
    QCOMPARE(we.procBiome_->currentData().toString(), QString("forest"));
    QCOMPARE(float(we.procSizeM_->value()), 300.0f);
    clickGenerate(we); // unchanged id/seed: keeps everything
    QCOMPARE(we.config().proceduralBiome, QString("forest"));
    QCOMPARE(we.config().obstacles.size(), w.obstacles.size());
  }
};

QTEST_MAIN(TstWorldEditor)
#include "tst_worldeditor.moc"
