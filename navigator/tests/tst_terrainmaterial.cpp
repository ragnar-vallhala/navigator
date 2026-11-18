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
// tst_terrainmaterial.cpp -- the terrain material's splat rule and quality
// default (TerrainMaterial.h, Qt-free). The shader is generated from the same
// constants, so these properties are the shader's too.
#include <QtTest/QtTest>

#include "TerrainMaterial.h"

using namespace vsim::terrain;

class TstTerrainMaterial : public QObject {
  Q_OBJECT
private slots:
  void weightsSumToOneAndStayInRange_data() {
    QTest::addColumn<float>("slope");
    QTest::addColumn<float>("r");
    QTest::addColumn<float>("g");
    QTest::addColumn<float>("noise");
    for (float slope : {0.0f, 0.1f, 0.2f, 0.3f, 0.45f, 0.8f, 1.0f})
      for (float noise : {0.0f, 0.5f, 1.0f})
        for (float r : {0.1f, 0.3f, 0.5f})
          QTest::newRow(qPrintable(QStringLiteral("s%1 n%2 r%3").arg(slope).arg(noise).arg(r)))
              << slope << r << 0.35f << noise;
  }
  void weightsSumToOneAndStayInRange() {
    QFETCH(float, slope);
    QFETCH(float, r);
    QFETCH(float, g);
    QFETCH(float, noise);
    const Splat s = splatWeights(slope, r, g, noise);
    QVERIFY(s.grass >= -1e-6f && s.grass <= 1.0f + 1e-6f);
    QVERIFY(s.rock >= 0.0f && s.rock <= 1.0f);
    QVERIFY(s.dirt >= 0.0f && s.dirt <= 1.0f);
    QCOMPARE(s.grass + s.rock + s.dirt, 1.0f);
  }
  void flatGreenFieldIsGrass() {
    const Splat s = splatWeights(0.0f, 0.20f, 0.45f, 0.5f);
    QVERIFY(s.grass > 0.999f);
  }
  void steepGroundIsRockWhateverTheCover() {
    for (float r : {0.1f, 0.5f}) {
      const Splat s = splatWeights(0.7f, r, 0.35f, 0.5f);
      QVERIFY(s.rock > 0.999f);
      QVERIFY(s.dirt < 1e-4f); // rock takes precedence over dirt
    }
  }
  void brownCoverOnFlatGroundIsDirt() {
    const Splat s = splatWeights(0.02f, 0.55f, 0.40f, 0.5f);
    QVERIFY2(s.dirt > 0.9f, "a brown track must show the dirt photo");
    QVERIFY(s.rock < 0.01f);
  }
  void rockGrowsWithSlopeAndNoiseNudgesTheEdge() {
    float prev = 0.0f;
    for (int i = 0; i <= 20; ++i) {
      const float r = splatWeights(i / 20.0f, 0.2f, 0.45f, 0.5f).rock;
      QVERIFY2(r >= prev - 1e-6f, "rock must not decrease as the slope grows");
      prev = r;
    }
    // The same slope shows more rock where the noise is high.
    const float lo = splatWeights(0.3f, 0.2f, 0.45f, 0.1f).rock;
    const float hi = splatWeights(0.3f, 0.2f, 0.45f, 0.9f).rock;
    QVERIFY(hi > lo);
  }
  void snowIsBrightAndUnsaturated() {
    QVERIFY(snowWeight(0.85f, 0.87f, 0.90f) > 0.99f);     // fresh snow
    QVERIFY(snowWeight(0.70f, 0.70f, 0.72f) > 0.99f);     // shaded snow
    QVERIFY(snowWeight(0.25f, 0.45f, 0.20f) < 0.001f);    // grass
    QVERIFY(snowWeight(0.55f, 0.45f, 0.30f) < 0.001f);    // dirt / sand
    QVERIFY(snowWeight(0.30f, 0.30f, 0.30f) < 0.001f);    // dark grey rock
    QVERIFY(snowWeight(0.85f, 0.78f, 0.55f) < 0.5f);      // pale sand, saturated
    float prev = 0.0f;                                    // grows with brightness
    for (int i = 0; i <= 20; ++i) {
      const float v = 0.4f + 0.5f * i / 20.0f;
      const float w = snowWeight(v, v, v);
      QVERIFY(w >= prev - 1e-6f);
      prev = w;
    }
  }
  void softwareRasterisersDefaultToLow_data() {
    QTest::addColumn<QString>("renderer");
    QTest::addColumn<bool>("low");
    QTest::newRow("llvmpipe") << QStringLiteral("llvmpipe (LLVM 15.0.7, 256 bits)") << true;
    QTest::newRow("Mesa llvmpipe upper") << QStringLiteral("Mesa LLVMPIPE") << true;
    QTest::newRow("softpipe") << QStringLiteral("softpipe") << true;
    QTest::newRow("swrast") << QStringLiteral("Software Rasterizer swrast") << true;
    QTest::newRow("RTX") << QStringLiteral("NVIDIA GeForce RTX 3050/PCIe/SSE2") << false;
    QTest::newRow("Intel") << QStringLiteral("Mesa Intel(R) UHD Graphics 620") << false;
    QTest::newRow("empty") << QString() << false;
  }
  void softwareRasterisersDefaultToLow() {
    QFETCH(QString, renderer);
    QFETCH(bool, low);
    const QByteArray b = renderer.toLatin1();
    QCOMPARE(defaultQuality(b.constData()), low ? Quality::Low : Quality::High);
  }
  void nullRendererStringIsHigh() {
    QCOMPARE(defaultQuality(nullptr), Quality::High);
  }
};

QTEST_MAIN(TstTerrainMaterial)
#include "tst_terrainmaterial.moc"
