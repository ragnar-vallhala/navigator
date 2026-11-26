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
// tst_waterfield.cpp -- ponds as real basins (procgen/WaterField, Qt-free).
// The carve lives in TerrainField::height(), the one height function the mesh,
// collision, minimap and helipads read, so these checks are on that function.
#include <QtTest/QtTest>

#include "procgen/GroundCover.h"
#include "procgen/Helipads.h"
#include "procgen/TerrainField.h"
#include "procgen/WaterField.h"

#include <atomic>
#include <cmath>
#include <thread>

using namespace vsim::procgen;

namespace {

FieldParams hills(uint32_t seed = 7u) {
  FieldParams p;
  p.seed = seed;
  p.palette = kPalHills;
  p.heightM = 70.0f;
  p.flatRadiusM = 12.0f;
  return p;
}

std::vector<WaterBody> bodies(const TerrainField &f, float half = 1500.0f) {
  return f.water().bodiesIn(f.heightSource(), -half, -half, half, half);
}

} // namespace

class TstWaterField : public QObject {
  Q_OBJECT
private slots:
  // The shared body cache must not change a single height, however many threads
  // fill it and in whatever order (chunks are meshed on a thread pool).
  void bodyCacheGivesTheSameHeightsOnAnyThread() {
    const TerrainField ref(hills());
    constexpr int kN = 40000;
    std::vector<float> want(kN);
    const auto at = [](int i) {
      return std::pair{float((i * 37) % 3000) - 1500.0f, float((i * 91) % 3000) - 1500.0f};
    };
    {
      // Reference: a fresh field per sample, so no cache is ever reused.
      for (int i = 0; i < kN; i += 50) {
        const TerrainField fresh(hills());
        want[size_t(i)] = fresh.height(at(i).first, at(i).second);
      }
    }
    const TerrainField shared(hills());
    std::vector<std::thread> pool;
    std::vector<int> bad(4, 0);
    for (int t = 0; t < 4; ++t)
      pool.emplace_back([&, t] {
        for (int n = 0; n < kN; ++n) {
          const int i = (t % 2 == 0) ? n : kN - 1 - n; // opposite sweep orders
          const float h = shared.height(at(i).first, at(i).second);
          if (i % 50 == 0 && h != want[size_t(i)])
            ++bad[size_t(t)];
        }
      });
    for (auto &th : pool)
      th.join();
    for (int t = 0; t < 4; ++t)
      QCOMPARE(bad[size_t(t)], 0);
    QCOMPARE(shared.height(at(123).first, at(123).second),
             ref.height(at(123).first, at(123).second));
  }

  // Eight threads on one field while two more build, sample and drop fields of
  // their own. The cache is per field and each thread keeps a front of its last
  // cells, keyed by the field's id: a thread that samples two fields in the same
  // cell must get each its own ground, and nothing may hang or differ.
  void bodyCacheSurvivesEightThreadsAndFieldChurn() {
    constexpr int kN = 20000;
    const auto at = [](int i) {
      return std::pair{float((i * 37) % 3000) - 1500.0f, float((i * 91) % 3000) - 1500.0f};
    };
    std::vector<float> want(kN);
    for (int i = 0; i < kN; i += 50) {
      const TerrainField fresh(hills());
      want[size_t(i)] = fresh.height(at(i).first, at(i).second);
    }
    const TerrainField shared(hills());
    std::atomic<int> bad{0};
    std::vector<std::thread> pool;
    for (int t = 0; t < 8; ++t)
      pool.emplace_back([&, t] {
        for (int n = 0; n < kN; ++n) {
          const int i = (t % 2 == 0) ? n : kN - 1 - n;
          const float h = shared.height(at(i).first, at(i).second);
          if (i % 50 == 0 && h != want[size_t(i)])
            ++bad;
        }
      });
    for (int t = 0; t < 2; ++t)
      pool.emplace_back([&, t] {
        for (int k = 0; k < 60; ++k) {
          const uint32_t sa = uint32_t(100 + 1000 * t + k), sb = sa + 7;
          const TerrainField a(hills(sa));
          const auto bs = bodies(a, 600.0f);
          if (bs.empty())
            continue;
          // The middle of A's first pond: B (another seed) has other ground there.
          const float px = bs[0].cx, py = bs[0].cy;
          // References from a thread that has never seen either field.
          float refA = 0, refB = 0;
          std::thread([&] {
            refA = TerrainField(hills(sa)).height(px, py);
            refB = TerrainField(hills(sb)).height(px, py);
          }).join();
          const TerrainField b(hills(sb));
          // Same thread, same cell, two fields: the thread-local front must not
          // answer for the wrong field.
          if (a.height(px, py) != refA || b.height(px, py) != refB || a.height(px, py) != refA)
            ++bad;
        }
      });
    for (auto &th : pool)
      th.join();
    QCOMPARE(bad.load(), 0);
  }

  void hillsWorldHasPonds() {
    int n = 0;
    for (uint32_t seed : {7u, 11u, 23u})
      n += int(bodies(TerrainField(hills(seed))).size());
    QVERIFY2(n >= 6, qPrintable(QString("bodies=%1").arg(n)));
  }

  void deterministicPerSeed() {
    const TerrainField a(hills()), b(hills());
    const auto ba = bodies(a), bb = bodies(b);
    QCOMPARE(ba.size(), bb.size());
    for (size_t i = 0; i < ba.size(); ++i) {
      QCOMPARE(ba[i].cx, bb[i].cx);
      QCOMPARE(ba[i].level, bb[i].level);
    }
    QVERIFY(bodies(TerrainField(hills(8u))).size() != ba.size() ||
            bodies(TerrainField(hills(8u)))[0].cx != ba[0].cx);
  }

  void groundUnderWaterIsLevelMinusBowl() {
    const TerrainField f(hills());
    const auto bs = bodies(f);
    QVERIFY(!bs.empty());
    for (const WaterBody &b : bs) {
      // Centre: level - depth (parabola bottom).
      QVERIFY(std::fabs(f.height(b.cx, b.cy) - (b.level - b.depthM)) < 1e-3f);
      // Anywhere inside the shore the ground is below the water surface.
      for (int i = 0; i < 24; ++i) {
        const float a = 6.2831853f * float(i) / 24.0f;
        const float r = 0.9f * b.shoreRadius(a);
        const float h = f.height(b.cx + r * std::cos(a), b.cy + r * std::sin(a));
        QVERIFY(h < b.level);
        QVERIFY(h >= b.level - b.depthM - 1e-3f);
      }
    }
  }

  void bankRisesToTheTerrainAndNeverDipsBelowTheLevel() {
    const TerrainField f(hills());
    for (const WaterBody &b : bodies(f)) {
      for (int i = 0; i < 24; ++i) {
        const float a = 6.2831853f * float(i) / 24.0f;
        const float R = b.shoreRadius(a);
        for (int k = 0; float(k) < b.bankM - 0.5f; ++k) {
          const float d = R + float(k);
          QVERIFY2(f.height(b.cx + d * std::cos(a), b.cy + d * std::sin(a)) >=
                       b.level - 0.01f,
                   "ground outside the shore is below the water level");
        }
      }
    }
  }

  void basinsAreGentleAndNonNegative() {
    const TerrainField f(hills());
    float maxSlope = 0.0f, minH = 1e9f;
    for (const WaterBody &b : bodies(f))
      for (int xi = 0; xi <= 120; ++xi)
        for (int yi = 0; yi <= 30; ++yi) {
          const float x = b.cx - 60.0f + float(xi), y = b.cy - 60.0f + 4.0f * float(yi);
          const float h = f.height(x, y);
          minH = std::min(minH, h);
          maxSlope = std::max(
              maxSlope, std::fabs(f.height(x + 1.0f, y) - h));
        }
    QVERIFY(minH >= 0.0f);
    QVERIFY2(maxSlope < 0.8f, qPrintable(QString("slope=%1").arg(maxSlope)));
  }

  void spawnStaysDry() {
    for (uint32_t seed : {7u, 11u, 23u, 99u}) {
      const TerrainField f(hills(seed));
      QVERIFY(!f.water().nearWater(f.heightSource(), 0.0f, 0.0f, 25.0f));
      for (const WaterBody &b : bodies(f))
        QVERIFY(std::hypot(b.cx, b.cy) > f.water().params().spawnClearM);
    }
  }

  void duneWorldAndNoPondPalettesHaveNoWater() {
    FieldParams d = hills();
    d.shape = 1;
    QVERIFY(bodies(TerrainField(d)).empty());
    FieldParams off = hills();
    off.waterDepthM = 0.0f;
    QVERIFY(bodies(TerrainField(off)).empty());
    FieldParams desert = hills();
    desert.palette = kPalDesert;
    QVERIFY(bodies(TerrainField(desert)).empty());
  }

  void waterOffLeavesTheTerrainUntouched() {
    FieldParams off = hills();
    off.waterDepthM = 0.0f;
    const TerrainField with(hills()), without(off);
    const auto bs = bodies(with);
    QVERIFY(!bs.empty());
    // Far from every body the two terrains are identical.
    const WaterBody &b = bs[0];
    QCOMPARE(with.height(b.cx + 120.0f, b.cy), without.height(b.cx + 120.0f, b.cy));
    QVERIFY(with.height(b.cx, b.cy) < without.height(b.cx, b.cy));
  }

  void coverPaintsThePondAndKeepsGrassOut() {
    const TerrainField f(hills());
    const WaterBody b = bodies(f)[0];
    const float h = f.height(b.cx, b.cy);
    const auto in = f.colorAt(b.cx, b.cy, h, 1.0f);
    const auto ps = GroundCover::pondStyle(kPalHills);
    QVERIFY(std::fabs(in.color.z - ps.water.z) < 0.02f);
    QCOMPARE(in.grass, 0.0f);
    const auto out = f.colorAt(b.cx + 80.0f, b.cy, f.height(b.cx + 80.0f, b.cy), 1.0f);
    QVERIFY(out.grass > 0.0f);
  }

  // The wet bank follows the height above the water, not a fixed-width band.
  void wetBankIsOneAtTheWaterlineAndFadesWithHeight() {
    const TerrainField f(hills());
    for (const WaterBody &b : bodies(f)) {
      for (int i = 0; i < 24; ++i) {
        const float a = 6.2831853f * float(i) / 24.0f;
        const float ca = std::cos(a), sa = std::sin(a), R = b.shoreRadius(a);
        const auto wet = [&](float d) {
          const float x = b.cx + d * ca, y = b.cy + d * sa;
          return f.water().wetBank(f.heightSource(), x, y, f.height(x, y));
        };
        QVERIFY(wet(R + 0.1f) > 0.85f);   // right at the waterline
        QVERIFY(wet(R - 3.0f) > 0.99f);   // under the water
        QCOMPARE(wet(R + b.bankM + 1.0f), 0.0f); // past the carved bank
        // It follows the ground's height above the level: dry wherever the bank
        // stands more than kWetHeightM over the water, however far from the shore.
        for (int k = 0; float(k) < b.bankM; ++k) {
          const float x = b.cx + (R + float(k)) * ca, y = b.cy + (R + float(k)) * sa;
          const float h = f.height(x, y);
          if (h - b.level >= WaterField::kWetHeightM)
            QCOMPARE(f.water().wetBank(f.heightSource(), x, y, h), 0.0f);
        }
        // Higher than kWetHeightM above the level: dry.
        const float x = b.cx + (R + 6.0f) * ca, y = b.cy + (R + 6.0f) * sa;
        QCOMPARE(f.water().wetBank(f.heightSource(), x, y,
                                   b.level + WaterField::kWetHeightM + 0.5f), 0.0f);
      }
    }
  }

  void shoreGroundIsDarkerThanTheShoreTintAndThePondKeepsItsColour() {
    const TerrainField f(hills());
    const auto ps = GroundCover::pondStyle(kPalHills);
    const auto luma = [](const PgVec3 &c) { return 0.3f * c.x + 0.59f * c.y + 0.11f * c.z; };
    float wetL = 0.0f;
    int n = 0;
    for (const WaterBody &b : bodies(f))
      for (int i = 0; i < 24; ++i) {
        const float a = 6.2831853f * float(i) / 24.0f;
        const float d = b.shoreRadius(a) + 0.6f;
        const float x = b.cx + d * std::cos(a), y = b.cy + d * std::sin(a);
        wetL += luma(f.colorAt(x, y, f.height(x, y), 1.0f).color);
        ++n;
      }
    wetL /= float(n);
    QVERIFY2(wetL < 0.9f * luma(ps.shore),
             qPrintable(QString("wet=%1 shore=%2").arg(wetL).arg(luma(ps.shore))));
    // The pond itself is still the palette's water colour (Low shows only this).
    const WaterBody b = bodies(f)[0];
    const auto in = f.colorAt(b.cx, b.cy, f.height(b.cx, b.cy), 1.0f);
    QVERIFY(std::fabs(in.color.z - ps.water.z) < 0.02f);
  }

  void helipadsAvoidPonds() {
    const TerrainField f(hills());
    HelipadParams hp;
    hp.blocked = [&](float x, float y, float r) {
      return f.water().nearWater(f.heightSource(), x, y, r);
    };
    const auto h = [&](float x, float y) { return f.height(x, y); };
    const auto pads = helipadsAround(h, h, 0.0f, 0.0f, 900.0f, hp);
    QVERIFY(pads.size() > 3);
    for (const PgVec3 &p : pads)
      QVERIFY(!f.water().nearWater(f.heightSource(), p.x, p.y, hp.radiusM));
  }
};

QTEST_MAIN(TstWaterField)
#include "tst_waterfield.moc"
