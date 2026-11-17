// PROTOTYPE harness (never merged): render real procgen terrain, with and
// without the textured/tone-mapped look (NAV_GFX_PROTO), into SHOT_DIR.
#include <QOpenGLWindow>
#include <QtTest>
#define private public
#define protected public
#include "SimRendererWidget.h"
#undef private
#undef protected
#include "ChunkStreamer.h"
#include "procgen/TerrainField.h"
#include "procgen/WorldPresets.h"
#include <QGuiApplication>
#include <QSurfaceFormat>
#include <QWidget>
using vsim::SimRendererWidget;
using namespace vsim::procgen;
class TstShotWorld : public QObject {
  Q_OBJECT
private slots:
  void shots() {
    QSurfaceFormat fmt; fmt.setVersion(4, 6); fmt.setProfile(QSurfaceFormat::CoreProfile); fmt.setSamples(4);
    QSurfaceFormat::setDefaultFormat(fmt);
    const QString out = qEnvironmentVariable("SHOT_DIR");
    const QStringList ids = qEnvironmentVariable("WORLDS", "hills,alpine,forest,flat").split(',');
    for (const QString &id : ids) {
      PresetData d;
      QVERIFY(buildWorldPreset(id.toStdString(), 21u, d));
      const TerrainField f(d.field);
      for (int proto = 0; proto < 2; ++proto) {
        qputenv("NAV_GFX_PROTO", proto ? "1" : "0");
        QWidget host; auto *r = new SimRendererWidget();
        auto *box = QWidget::createWindowContainer(r, &host);
        const int W = qEnvironmentVariableIntValue("SHOT_W") ? qEnvironmentVariableIntValue("SHOT_W") : 960, H = W * 9 / 16; box->setFixedSize(W, H); host.resize(W, H); host.show();
        QVERIFY(QTest::qWaitForWindowExposed(r));
        for (int cy = -2; cy <= 1; ++cy)
          for (int cx = -2; cx <= 1; ++cx) {
            const ProcMesh m = meshFieldChunk(f, cx, cy, kStreamChunkM, kStreamChunkRes);
            const auto c = vsim::toChunkMeshData(qint64(cx + 100) * 1000 + (cy + 100), m);
            r->setWorldChunk(c.key, c.positions, c.normals, c.colors);
          }
        const float gz = surfaceHeight(f, 0, 0, kStreamChunkM, kStreamChunkRes);
        r->cam_radius_ = 7.0f; r->cam_pitch_ = 0.18f; r->cam_yaw_ = 0.3f; r->userZoomed_ = true;
        vsim::SimSnapshot s; s.pos_w = {0, 0, -(gz + 4.0f)};
        for (int i = 0; i < 8; ++i) { r->setSnapshot(s); QTest::qWait(60); }
        QImage im = r->grabFrame();
        const QString fn = QString("%1/world_%2_%3.png").arg(out, id, proto ? "after" : "before");
        QVERIFY(!im.isNull()); QVERIFY(im.save(fn)); qInfo("saved %s", qPrintable(fn));
        QElapsedTimer tm; tm.start();
        for (int i = 0; i < 60; ++i) { r->setSnapshot(s); (void)r->grabFrame(); }
        qInfo("FRAMETIME %s proto=%d %.1f ms/frame", qPrintable(id), proto, tm.elapsed() / 60.0);
        // high view: 100 m up, looking across the field
        s.pos_w = {0, 0, -(gz + 100.0f)}; r->cam_radius_ = 6.0f; r->cam_pitch_ = 0.55f;
        for (int i = 0; i < 6; ++i) { r->setSnapshot(s); QTest::qWait(60); }
        QImage hi = r->grabFrame();
        QVERIFY(hi.save(QString("%1/world_%2_%3_high.png").arg(out, id, proto ? "after" : "before")));
      }
    }
  }
};
QTEST_MAIN(TstShotWorld)
#include "tst_shot_world.moc"
