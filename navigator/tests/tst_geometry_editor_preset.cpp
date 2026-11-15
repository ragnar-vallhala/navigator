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
// The Vehicle page's Preset combo against the form: picking a preset keeps the
// combo on that preset (loading it is not an edit), and any manual edit of an
// airframe field flips it to Custom and says so, so Start sends no preset gains
// for a modified airframe. Offscreen, no engine.

#include "AirframePresets.h"
#include "GeometryEditorWidget.h"
#include "VehicleJson.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPushButton>
#include <QLineEdit>
#include <QSignalSpy>
#include <QTemporaryDir>
#include <QtTest>

class TstGeometryEditorPreset : public QObject {
  Q_OBJECT

  static QComboBox *combo(GeometryEditorWidget &w) {
    const auto cs = w.findChildren<QComboBox *>();
    for (QComboBox *c : cs)
      if (c->findData(QStringLiteral("whoop65")) >= 0)
        return c;
    return nullptr;
  }
  // Choose a preset the way the user does (the activated() path).
  static void pick(GeometryEditorWidget &w, const QString &id) {
    QComboBox *c = combo(w);
    QVERIFY(c);
    const int i = c->findData(id);
    QVERIFY(i >= 0);
    c->setCurrentIndex(i);
    emit c->activated(i);
  }

  QTemporaryDir dataDir_; // private user-preset store (XDG_DATA_HOME)

  static QPushButton *button(GeometryEditorWidget &w, const QString &text) {
    for (QPushButton *b : w.findChildren<QPushButton *>())
      if (b->text() == text)
        return b;
    return nullptr;
  }
  static QStringList comboTexts(QComboBox *c) {
    QStringList t;
    for (int i = 0; i < c->count(); ++i)
      t << (c->itemText(i).isEmpty() ? QStringLiteral("|") : c->itemText(i));
    return t;
  }
  // Choose by item text (what the user clicks).
  static void pickText(GeometryEditorWidget &w, const QString &text) {
    QComboBox *c = combo(w);
    const int i = c->findText(text);
    QVERIFY2(i >= 0, qPrintable(text));
    c->setCurrentIndex(i);
    emit c->activated(i);
  }

private slots:
  void initTestCase() {
    QVERIFY(dataDir_.isValid());
    qputenv("XDG_DATA_HOME", dataDir_.path().toLocal8Bit());
  }

  // ---- user presets ----
  void userPresetSaveListSelectDelete() {
    GeometryEditorWidget w;
    QComboBox *c = combo(w);
    const int builtins = c->count();
    QPushButton *del = button(w, "Delete preset");
    QVERIFY(button(w, "Save as preset\u2026"));
    QVERIFY(del);
    QVERIFY(!del->isEnabled()); // Custom

    massSpin(w)->setValue(0.77);
    QSignalSpy chosen(&w, &GeometryEditorWidget::presetChosen);
    QCOMPARE(w.saveUserPreset("  Heavy One ", false),
             vsim::UserPresetStore::SaveResult::Ok);
    // After the built-ins: a separator, then the user preset, which is shown.
    QCOMPARE(c->count(), builtins + 2);
    QCOMPARE(comboTexts(c).mid(builtins), QStringList({"|", "Heavy One"}));
    QCOMPARE(w.userPresetName(), QString("Heavy One"));
    QCOMPARE(w.presetId(), QString()); // no firmware gain set
    QVERIFY(del->isEnabled());
    QVERIFY(!chosen.isEmpty());
    QCOMPARE(chosen.last().at(0).toString(), QString());

    // Picking a built-in disables Delete; the user entry stays listed.
    pick(w, "whoop65");
    QVERIFY(!del->isEnabled());
    QCOMPARE(w.userPresetName(), QString());

    // A fresh widget (a later launch) lists it and loads its geometry.
    GeometryEditorWidget w2;
    pickText(w2, "Heavy One");
    QVERIFY(std::fabs(w2.config().mass - 0.77f) < 1e-4f);
    QCOMPARE(w2.userPresetName(), QString("Heavy One"));
    QVERIFY(button(w2, "Delete preset")->isEnabled());
    // Editing the form turns it back into Custom and disables Delete.
    QDoubleSpinBox *m2 = nullptr; // the mass field now shows the loaded 0.77
    for (QDoubleSpinBox *sp : w2.findChildren<QDoubleSpinBox *>())
      if (std::fabs(sp->value() - 0.77) < 1e-9)
        m2 = sp;
    QVERIFY(m2);
    m2->setValue(0.5);
    QCOMPARE(w2.userPresetName(), QString());
    QVERIFY(!button(w2, "Delete preset")->isEnabled());

    QVERIFY(w2.deleteUserPreset("Heavy One"));
    QVERIFY(!w2.deleteUserPreset("Heavy One"));
    QCOMPARE(combo(w2)->count(), builtins); // separator gone with the last one
    GeometryEditorWidget w3;
    QCOMPARE(combo(w3)->count(), builtins);
  }

  void userPresetRejectsBadAndBuiltInNames() {
    GeometryEditorWidget w;
    using R = vsim::UserPresetStore::SaveResult;
    QCOMPARE(w.saveUserPreset("../x", false), R::InvalidName);
    QCOMPARE(w.saveUserPreset("", false), R::InvalidName);
    QCOMPARE(w.saveUserPreset("whoop65", false), R::Reserved); // a built-in id
    QCOMPARE(w.saveUserPreset(vsim::airframePresets().first().name, false),
             R::Reserved); // a built-in label
    QCOMPARE(w.saveUserPreset("Dup", false), R::Ok);
    QCOMPARE(w.saveUserPreset("dup", false), R::Exists);
    QCOMPARE(w.saveUserPreset("dup", true), R::Ok);
    QVERIFY(w.deleteUserPreset("dup"));
  }

  void presetPickKeepsPresetAndLoadsGeometry() {
    GeometryEditorWidget w;
    QSignalSpy chosen(&w, &GeometryEditorWidget::presetChosen);
    pick(w, "10in");
    QCOMPARE(w.presetId(), QString("10in"));
    // Exactly one presetChosen, for the preset: loading the form did not read
    // as an edit and bounce it to Custom.
    QCOMPARE(chosen.size(), 1);
    QCOMPARE(chosen.at(0).at(0).toString(), QString("10in"));
    QCOMPARE(w.config().mass, 1.52f);
    pick(w, "whoop65");
    QCOMPARE(w.presetId(), QString("whoop65"));
    QCOMPARE(w.config().mass, 0.029f);
  }

  void editingAFieldFlipsToCustom_data() {
    QTest::addColumn<QString>("field"); // object-less: index into the spinboxes
    QTest::newRow("mass") << "mass";
    QTest::newRow("motor field") << "motor";
  }
  void editingAFieldFlipsToCustom() {
    QFETCH(QString, field);
    GeometryEditorWidget w;
    pick(w, "5in");
    QCOMPARE(w.presetId(), QString("5in"));
    QSignalSpy chosen(&w, &GeometryEditorWidget::presetChosen);

    // Mass is the spinbox whose value is the preset's mass; a motor field is
    // the one holding its k_thrust.
    const double target = field == "mass" ? 0.682 : 1.234e-6;
    QDoubleSpinBox *hit = nullptr;
    for (QDoubleSpinBox *sp : w.findChildren<QDoubleSpinBox *>())
      if (std::fabs(sp->value() - target) < 1e-9 * std::max(1.0, target) ||
          std::fabs(sp->value() - target) < 1e-6 * target) {
        hit = sp;
        break;
      }
    QVERIFY(hit);
    hit->setValue(hit->value() * 1.1); // a user edit (valueChanged)

    QCOMPARE(w.presetId(), QString());
    QCOMPARE(chosen.size(), 1);
    QVERIFY(chosen.at(0).at(0).toString().isEmpty());
    // A second edit stays Custom and does not announce again.
    hit->setValue(hit->value() * 1.1);
    QCOMPARE(chosen.size(), 1);
  }

  void switchingSpinDirectionFlipsToCustom() {
    GeometryEditorWidget w;
    pick(w, "5in");
    // Motor 1's spin combo: a QComboBox that is not the preset picker.
    QComboBox *spinCombo = nullptr;
    for (QComboBox *c : w.findChildren<QComboBox *>())
      if (c != combo(w)) {
        spinCombo = c;
        break;
      }
    QVERIFY(spinCombo);
    spinCombo->setCurrentIndex(spinCombo->currentIndex() == 0 ? 1 : 0);
    QCOMPARE(w.presetId(), QString());
  }

  void loadingAConfigIsNotAnEdit() {
    GeometryEditorWidget w;
    pick(w, "7in");
    QSignalSpy chosen(&w, &GeometryEditorWidget::presetChosen);
    // setConfig is what restore and Load Vehicle use; it must not flip.
    const auto *p = vsim::findAirframePreset("3in");
    QVERIFY(p);
    w.setConfig(p->geometry);
    QCOMPARE(w.presetId(), QString("7in")); // the caller sets the id
    QCOMPARE(chosen.size(), 0);
  }

  // ---- prop_radius: load -> edit -> save -> reload ----
  static QDoubleSpinBox *radiusSpin(GeometryEditorWidget &w) {
    return w.findChild<QDoubleSpinBox *>("propRadiusSpin");
  }
  // Commit the form into config() the way the Apply button does.
  static void apply(GeometryEditorWidget &w) {
    for (QPushButton *b : w.findChildren<QPushButton *>())
      if (b->text() == QStringLiteral("Apply to Sim")) {
        b->click();
        return;
      }
    QFAIL("Apply button not found");
  }
  static vsim::GeometryConfig viaFile(const vsim::GeometryConfig &c) {
    const QByteArray bytes =
        QJsonDocument(vsim::geometryToJson(c)).toJson(QJsonDocument::Indented);
    return vsim::geometryFromJson(QJsonDocument::fromJson(bytes).object());
  }

  void propRadiusLoadEditSaveReload() {
    QJsonObject o = vsim::geometryToJson(vsim::GeometryConfig{});
    o["prop_radius"] = 0.0889; // a file that carries it
    GeometryEditorWidget w;
    QDoubleSpinBox *sp = radiusSpin(w);
    QVERIFY(sp);
    w.setConfig(vsim::geometryFromJson(o));
    QCOMPARE(sp->value(), 0.0889);
    // Round-trips unchanged when untouched.
    apply(w);
    QCOMPARE(w.config().propRadius, 0.0889f);
    QCOMPARE(viaFile(w.config()).propRadius, 0.0889f);
    // Edit -> save -> reload into a fresh editor.
    sp->setValue(0.1234);
    apply(w);
    QCOMPARE(w.config().propRadius, 0.1234f);
    const vsim::GeometryConfig reloaded = viaFile(w.config());
    QCOMPARE(reloaded.propRadius, 0.1234f);
    GeometryEditorWidget w2;
    w2.setConfig(reloaded);
    QCOMPARE(radiusSpin(w2)->value(), 0.1234);
    // Out-of-range edits are clamped to the model's 0.004-0.5 m range.
    sp->setValue(9.0);
    QCOMPARE(sp->value(), 0.5);
    sp->setValue(0.0);
    QCOMPARE(sp->value(), 0.004);
  }

  void propRadiusMissingFromFileKeepsDefault() {
    QJsonObject o = vsim::geometryToJson(vsim::GeometryConfig{});
    o.remove("prop_radius");
    const vsim::GeometryConfig c = vsim::geometryFromJson(o);
    QCOMPARE(c.propRadius, vsim::GeometryConfig{}.propRadius);
    GeometryEditorWidget w;
    w.setConfig(c);
    QCOMPARE(radiusSpin(w)->value(), 0.0622);
    apply(w);
    QCOMPARE(w.config().propRadius, 0.0622f);
  }

  void editingPropRadiusFlipsToCustom() {
    GeometryEditorWidget w;
    pick(w, "5in");
    QSignalSpy chosen(&w, &GeometryEditorWidget::presetChosen);
    radiusSpin(w)->setValue(0.07);
    QCOMPARE(w.presetId(), QString());
    QCOMPARE(chosen.size(), 1);
  }

  // ---- box model: inertia follows mass + arm length ----
  static QDoubleSpinBox *massSpin(GeometryEditorWidget &w) {
    for (QDoubleSpinBox *sp : w.findChildren<QDoubleSpinBox *>())
      if (std::fabs(sp->value() - 0.682) < 1e-9)
        return sp;
    return nullptr;
  }
  static QDoubleSpinBox *posXSpin(GeometryEditorWidget &w) {
    for (QDoubleSpinBox *sp : w.findChildren<QDoubleSpinBox *>())
      if (std::fabs(sp->value() - 0.0866) < 1e-9) // Motor 1 pos X
        return sp;
    return nullptr;
  }

  void massEditRecomputesInertia() {
    GeometryEditorWidget w;
    const vsim::GeometryConfig d;
    apply(w);
    QCOMPARE(w.config().inertia[0], d.inertia[0]); // untouched: unchanged
    QDoubleSpinBox *m = massSpin(w);
    QVERIFY(m);
    m->setValue(1.364); // twice the mass
    apply(w);
    for (int i : {0, 4, 8})
      QVERIFY(std::fabs(w.config().inertia[i] - 2.0f * d.inertia[i]) <
              2e-3f * d.inertia[i]);
    // What the engine gets (physicsConfig) carries the new value too.
    QCOMPARE(w.physicsConfig().inertia, w.config().inertia);
  }

  void armEditRecomputesInertia() {
    GeometryEditorWidget w;
    QDoubleSpinBox *x = posXSpin(w);
    QVERIFY(x);
    const float iyy0 = w.config().inertia[4];
    x->setValue(0.1732); // motor 1 only: mean(x^2) 0.0075 -> 0.013125 (x1.75)
    apply(w);
    QVERIFY(std::fabs(w.config().inertia[4] / iyy0 - 1.75f) < 0.01f);
  }

  // An old saved config with a hand-set inertia loads as it is and is only
  // replaced once mass or arms are edited.
  void handSetInertiaKeptUntilEdit() {
    vsim::GeometryConfig c;
    c.inertia = {9e-3f, 0, 0, 0, 9e-3f, 0, 0, 0, 9e-3f};
    GeometryEditorWidget w;
    w.setConfig(c);
    apply(w);
    QCOMPARE(w.config().inertia[0], 9e-3f);
    massSpin(w)->setValue(0.8);
    apply(w);
    QVERIFY(w.config().inertia[0] < 3e-3f);
  }
};

QTEST_MAIN(TstGeometryEditorPreset)
#include "tst_geometry_editor_preset.moc"
