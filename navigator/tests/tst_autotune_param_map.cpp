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
#include <QtTest>

#include "AutotuneParamMap.h"

using namespace autotune_params;
using namespace params;

namespace {
QString catalogPath() {
  return qEnvironmentVariable("PARAMS_CATALOG_JSON",
                              QStringLiteral(NAVLINK_GEN_DIR "/params_catalog.json"));
}
double valueOf(const QVector<ParamApplier::Item> &v, const QString &id, bool *found = nullptr) {
  for (const auto &i : v)
    if (i.id == id) {
      if (found)
        *found = true;
      return i.value;
    }
  if (found)
    *found = false;
  return 0;
}
} // namespace

// The autotuner's Apply as parameters, against the real catalog of the build.
class TstAutotuneParamMap : public QObject {
  Q_OBJECT
  Catalog m_cat;

private slots:
  void initTestCase() {
    QString err;
    QVERIFY2(Catalog::loadFile(catalogPath(), m_cat, &err), qPrintable(err));
  }
  void everyGainSlotHasItsParametersInTheCatalog();
  void anAutotuneResultMapsToTheRightValues();
  void anAngleSlotWithMoreThanPIsRefused();
  void aSlotThatDoesNotExistIsRefused();
  void aCatalogWithoutTheParametersIsRefused();
  void everyMappedValueIsCheckedByTheCatalogRange();
  void theRouteFollowsWhatTheVehicleOffers();
};

void TstAutotuneParamMap::everyGainSlotHasItsParametersInTheCatalog() {
  QVector<PidSetCmd> all;
  for (int axis = 0; axis < 3; ++axis) {
    all.push_back({1, axis, 0.01f, 0.001f, 0.0001f, 0.0f});
    all.push_back({0, axis, 3.0f, 0.0f, 0.0f, 0.0f});
  }
  QVector<ParamApplier::Item> items;
  QString why;
  QVERIFY2(pidCmdsToItems(all, m_cat, items, &why), qPrintable(why));
  QCOMPARE(items.size(), 3 * 4 + 3);
  for (const auto &i : items)
    QVERIFY2(m_cat.byId(i.id), qPrintable(i.id));
}

void TstAutotuneParamMap::anAutotuneResultMapsToTheRightValues() {
  const QStringList names = {"rate_kp", "rate_ki", "rate_kd", "angle_kp",
                             "yaw_rate_kp", "yaw_rate_ki", "yaw_rate_kd"};
  const QVector<double> x = {0.01, 0.002, 0.0003, 4.0, 0.02, 0.004, 0.0002};
  QVector<ParamApplier::Item> items;
  QVERIFY(pidCmdsToItems(autotuneGainsToCommands(names, x), m_cat, items));
  bool f = false;
  QCOMPARE(valueOf(items, QStringLiteral("RATE_ROLL_KP"), &f), double(0.01f));
  QVERIFY(f);
  QCOMPARE(valueOf(items, QStringLiteral("RATE_PITCH_KI")), double(0.002f));
  QCOMPARE(valueOf(items, QStringLiteral("RATE_PITCH_KD")), double(0.0003f));
  QCOMPARE(valueOf(items, QStringLiteral("ANG_PITCH_KP")), double(4.0f));
  QCOMPARE(valueOf(items, QStringLiteral("RATE_YAW_KP")), double(0.02f));
  QCOMPARE(valueOf(items, QStringLiteral("RATE_YAW_KFF"), &f), 0.0); // the old command wrote kff 0 too
  QVERIFY(f);
  valueOf(items, QStringLiteral("ANG_YAW_KP"), &f);
  QVERIFY(!f); // yaw is rate-controlled only here
}

void TstAutotuneParamMap::anAngleSlotWithMoreThanPIsRefused() {
  QVector<ParamApplier::Item> items;
  QString why;
  QVERIFY(!pidCmdsToItems({{0, 0, 3.0f, 0.1f, 0.0f, 0.0f}}, m_cat, items, &why));
  QVERIFY2(why.contains(QLatin1String("angle")), qPrintable(why));
  QVERIFY(items.isEmpty());
}

void TstAutotuneParamMap::aSlotThatDoesNotExistIsRefused() {
  QVector<ParamApplier::Item> items;
  QVERIFY(!pidCmdsToItems({{1, 3, 1, 0, 0, 0}}, m_cat, items));
  QVERIFY(!pidCmdsToItems({{2, 0, 1, 0, 0, 0}}, m_cat, items));
  QVERIFY(items.isEmpty());
}

void TstAutotuneParamMap::aCatalogWithoutTheParametersIsRefused() {
  Catalog none; // a vehicle table with none of them
  QVector<ParamApplier::Item> items;
  QString why;
  QVERIFY(!pidCmdsToItems({{1, 0, 0.01f, 0, 0, 0}}, none, items, &why));
  QVERIFY2(why.contains(QLatin1String("no parameters")), qPrintable(why));
  QVERIFY(items.isEmpty());
}

void TstAutotuneParamMap::everyMappedValueIsCheckedByTheCatalogRange() {
  // The optimiser may propose a gain the vehicle's range does not allow; the
  // applier (not this map) refuses it, so check the range really is there.
  QVector<ParamApplier::Item> items;
  QVERIFY(pidCmdsToItems({{1, 0, 5.0f, 0, 0, 0}}, m_cat, items)); // far above RATE_ROLL_KP max
  const Entry *e = m_cat.byId(QStringLiteral("RATE_ROLL_KP"));
  QVERIFY(e);
  QString r;
  QVERIFY(!Catalog::accepts(*e, valueOf(items, QStringLiteral("RATE_ROLL_KP")), &r));
  QVERIFY(!r.isEmpty());
}

void TstAutotuneParamMap::theRouteFollowsWhatTheVehicleOffers() {
  using S = Client::State;
  QCOMPARE(routeFor(S::Ready), Route::Params);
  QCOMPARE(routeFor(S::NoCaps), Route::Legacy);
  for (S s : {S::Unsynced, S::ReadingCrc, S::NoAnswer, S::Listing, S::Mismatch, S::Incomplete})
    QCOMPARE(routeFor(s), Route::NotReady);
}

QTEST_APPLESS_MAIN(TstAutotuneParamMap)
#include "tst_autotune_param_map.moc"
