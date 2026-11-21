// GL-free: the colliders are the size of what is drawn. Every tree species is
// scaled to its own VegTreeKind height by the renderer's species table, so the
// collider height must equal the rendered height of every tree model (within
// 1%), the trunk radius the one measured on the asset, and a rock's collider
// radius half its rendered span.
#include "VegetationAssets.h"
#include "VegetationRenderer.h"
#include "procgen/TerrainField.h"
#include "procgen/Vegetation.h"

#include <QtTest>

#include <cmath>

using namespace vsim;
using namespace vsim::procgen;

class TstVegetationScale : public QObject {
  Q_OBJECT
private slots:
  // The drawn tree is the solid tree: for every tree variant and every scale the
  // collider is as tall as the mesh the renderer draws, and its radius matches
  // the trunk measured on the asset (manifest "trunkRadius") at that size.
  void colliderMatchesRenderedTree() {
    VegAssets assets;
    QString err;
    QVERIFY2(loadVegetationAssets(assets, &err, ":/vegetation", false), qPrintable(err));
    int n = 0;
    const VegSpeciesInfo *tab = vegSpeciesTable(&n);
    int trees = 0;
    QVector<bool> kindSeen(kVegTreeKindCount, false);
    for (int i = 0; i < n; ++i) {
      if (tab[i].cat != kVegTree)
        continue;
      const int kind = vegTreeKindIndex(tab[i].id);
      QVERIFY2(kind >= 0, tab[i].id);
      kindSeen[kind] = true;
      QCOMPARE(tab[i].targetSize, vegTreeKind(kind).heightM);
      const VegSpeciesAssets *sp = assets.find(tab[i].id);
      QVERIFY2(sp, tab[i].id);
      float radiusSum = 0.0f;
      for (const VegVariant &v : sp->variants) {
        for (float scale : {0.7f, 1.0f, 1.45f}) {
          VegInstance inst;
          inst.species = kVegTree;
          inst.kind = uint8_t(kind);
          inst.scale = scale;
          const std::vector<VegCollider> c = vegetationColliders({inst});
          QCOMPARE(int(c.size()), 1);
          const float rendered = vegUnitScale(tab[i], v) * v.bboxMax.y() * scale;
          QVERIFY2(std::fabs(c[0].height - rendered) <= 0.01f * rendered, qPrintable(v.name));
        }
        QVERIFY2(v.trunkRadius > 0.05f, qPrintable(v.name));
        radiusSum += v.trunkRadius * vegUnitScale(tab[i], v);
        ++trees;
      }
      // Species trunk radius = mean drawn trunk radius of its variants (+-25 %).
      const float drawn = radiusSum / float(sp->variants.size());
      const float coll = vegTreeKind(kind).trunkRadiusM;
      QVERIFY2(std::fabs(coll - drawn) <= 0.25f * drawn,
               qPrintable(QString("%1: collider %2 m, drawn trunk %3 m")
                              .arg(tab[i].id).arg(coll).arg(drawn)));
    }
    for (bool b : kindSeen)
      QVERIFY(b);
    QVERIFY(trees >= 7);
  }

  // Every broadleaf species has at least two variants and the placement picks
  // each of them for a real share of its instances, from the position alone.
  void broadleafVariantsAreBothPlaced() {
    VegAssets assets;
    QString err;
    QVERIFY2(loadVegetationAssets(assets, &err, ":/vegetation", false), qPrintable(err));
    FieldParams fp;
    fp.palette = kPalForest;
    const TerrainField f(fp);
    const VegParams p = vegParamsForPalette(kPalForest);
    std::vector<VegInstance> trees;
    for (int j = -2; j <= 1; ++j)
      for (int i = -2; i <= 1; ++i)
        for (const VegInstance &v : scatterVegetation(f, i, j, 256.0f, p))
          if (v.species == kVegTree)
            trees.push_back(v);
    int checked = 0;
    for (int kind = 0; kind < kVegTreeKindCount; ++kind) {
      if (vegTreeKind(kind).conifer)
        continue;
      const VegSpeciesAssets *sp = assets.find(QString::fromLatin1(vegTreeKind(kind).id));
      QVERIFY(sp);
      const int pool = int(sp->variants.size());
      QVERIFY2(pool >= 2, vegTreeKind(kind).id);
      std::vector<int> hit(size_t(pool), 0);
      int total = 0;
      for (const VegInstance &v : trees)
        if (v.kind == kind) {
          const int pick = vegPickVariant(v, pool);
          QVERIFY(pick >= 0 && pick < pool);
          QCOMPARE(pick, vegPickVariant(v, pool)); // deterministic
          ++hit[size_t(pick)];
          ++total;
        }
      QVERIFY2(total > 40, vegTreeKind(kind).id);
      for (int h : hit)
        QVERIFY2(h > total / (pool * 3), qPrintable(QString("%1: %2 of %3").arg(vegTreeKind(kind).id).arg(h).arg(total)));
      ++checked;
    }
    QVERIFY(checked >= 2);
  }

  // The packed tilt (yaw + 8 x code) is decoded by the mesh vertex shader only;
  // the billboard shader reads that channel as a plain yaw. That is correct as
  // long as no species that carries a tilt (rocks) is ever drawn as an
  // impostor, and only trees are: a later change that gives a rock an impostor
  // must teach the billboard vertex shader the tilt first.
  void onlyTreesTakeImpostors() {
    int n = 0;
    const VegSpeciesInfo *tab = vegSpeciesTable(&n);
    int rocks = 0, impostors = 0;
    for (int i = 0; i < n; ++i) {
      if (tab[i].cat == kVegRock) {
        QVERIFY2(!tab[i].impostor, tab[i].id);
        QCOMPARE(tab[i].fallbackMax, tab[i].maxDist);
        // Past its last mesh LOD a rock is culled, never billboarded.
        for (float d : {5.0f, 60.0f, 140.0f, 255.0f}) {
          const VegLodChoice c = vegChooseLod(tab[i], d, true);
          for (int k = 0; k < c.n; ++k)
            QVERIFY2(c.lod[k] <= 2, tab[i].id);
        }
        ++rocks;
      }
      if (tab[i].impostor) {
        QCOMPARE(int(tab[i].cat), int(kVegTree));
        ++impostors;
      }
    }
    QVERIFY(rocks >= 3);
    QVERIFY(impostors >= 4);
  }

  void rockColliderRadiusIsHalfTheRenderedSpan() {
    VegAssets assets;
    QString err;
    QVERIFY2(loadVegetationAssets(assets, &err, ":/vegetation", false), qPrintable(err));
    int n = 0;
    const VegSpeciesInfo *tab = vegSpeciesTable(&n);
    int rocks = 0;
    for (int i = 0; i < n; ++i) {
      if (tab[i].cat != kVegRock)
        continue;
      const VegSpeciesAssets *sp = assets.find(tab[i].id);
      QVERIFY2(sp, tab[i].id);
      for (const VegVariant &v : sp->variants) {
        VegInstance inst;
        inst.species = kVegRock;
        inst.scale = 2.0f; // above the collider threshold
        const std::vector<VegCollider> c = vegetationColliders({inst});
        QCOMPARE(int(c.size()), 1);
        const float span = vegUnitScale(tab[i], v) * std::max(v.footprintX, v.footprintZ) * 2.0f;
        QVERIFY2(std::fabs(c[0].radius - 0.5f * span) <= 0.01f * 0.5f * span, qPrintable(v.name));
        ++rocks;
      }
    }
    QVERIFY(rocks >= 3);
  }
};

QTEST_GUILESS_MAIN(TstVegetationScale)
#include "tst_vegetation_scale.moc"
