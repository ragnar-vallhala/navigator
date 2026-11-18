// GL-free: the colliders are the size of what is drawn. Every tree species is
// scaled to kVegTreeHeightM by the renderer's species table, so the collider
// height must equal the rendered height of every tree model (within 1%), and a
// rock's collider radius half its rendered span.
#include "VegetationAssets.h"
#include "VegetationRenderer.h"
#include "procgen/Vegetation.h"

#include <QtTest>

#include <cmath>

using namespace vsim;
using namespace vsim::procgen;

class TstVegetationScale : public QObject {
  Q_OBJECT
private slots:
  void colliderHeightEqualsRenderedHeight() {
    VegAssets assets;
    QString err;
    QVERIFY2(loadVegetationAssets(assets, &err, ":/vegetation", false), qPrintable(err));
    int n = 0;
    const VegSpeciesInfo *tab = vegSpeciesTable(&n);
    int trees = 0;
    for (int i = 0; i < n; ++i) {
      if (tab[i].cat != kVegTree)
        continue;
      const VegSpeciesAssets *sp = assets.find(tab[i].id);
      QVERIFY2(sp, tab[i].id);
      for (const VegVariant &v : sp->variants) {
        for (float scale : {0.8f, 1.0f, 1.5f}) {
          VegInstance inst;
          inst.species = kVegTree;
          inst.scale = scale;
          const std::vector<VegCollider> c = vegetationColliders({inst});
          QCOMPARE(int(c.size()), 1);
          const float rendered = vegUnitScale(tab[i], v) * v.bboxMax.y() * scale;
          QVERIFY2(std::fabs(c[0].height - rendered) <= 0.01f * rendered, qPrintable(v.name));
        }
        ++trees;
      }
    }
    QVERIFY(trees >= 5);
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
