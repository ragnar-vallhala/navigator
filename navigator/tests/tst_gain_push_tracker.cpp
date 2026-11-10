// The airframe-gain push state machine: pending, 1 s retries, ACK counting,
// give-up after 30 s, re-arm on preset change, custom vehicle never pending.
// Clock-free: every call passes the time explicitly.
#include <QtTest>

#include "GainPushTracker.h"

using A = GainPushTracker::Action;

class TstGainPushTracker : public QObject {
  Q_OBJECT
private slots:
  void pushIsPending() {
    GainPushTracker t;
    QVERIFY(!t.pending());
    t.begin("5in", 6, 0);
    QVERIFY(t.pending());
    QCOMPARE(t.expected(), 6);
    QCOMPARE(t.id(), QString("5in"));
  }
  void retriesEverySecond() {
    GainPushTracker t;
    t.begin("5in", 6, 0);
    QCOMPARE(t.tick(500), A::None);
    QCOMPARE(t.tick(1000), A::Resend);
    QCOMPARE(t.tick(1500), A::None);
    QCOMPARE(t.tick(2000), A::Resend);
    QCOMPARE(t.retries(), 2);
    QVERIFY(t.pending());
  }
  void ackedOnceAfterAllSlots() {
    GainPushTracker t;
    t.begin("5in", 6, 0);
    int fired = 0;
    for (int i = 0; i < 6; ++i)
      fired += t.onAck(GainPushTracker::kResultOk, 100 + i) ? 1 : 0;
    QCOMPARE(fired, 1); // reported exactly once
    QVERIFY(!t.pending());
    QVERIFY(t.accepted());
    QVERIFY(!t.onAck(GainPushTracker::kResultOk, 200)); // late extra: ignored
    QCOMPARE(t.tick(5000), A::None);                    // no more retries
  }
  void rejectedAcksDoNotComplete() {
    GainPushTracker t;
    t.begin("5in", 6, 0);
    for (int i = 0; i < 6; ++i)
      QVERIFY(!t.onAck(1 /*TEMPORARILY_REJECTED*/, 10));
    QVERIFY(t.pending());
    QCOMPARE(t.rejectedCount(), 6);
    QCOMPARE(t.tick(1000), A::Resend);
    for (int i = 0; i < 5; ++i)
      QVERIFY(!t.onAck(GainPushTracker::kResultOk, 1100));
    QVERIFY(t.onAck(GainPushTracker::kResultOk, 1100));
    QVERIFY(!t.pending());
  }
  void acksCountSinceLastSend() {
    GainPushTracker t;
    t.begin("5in", 6, 0);
    for (int i = 0; i < 4; ++i)
      t.onAck(GainPushTracker::kResultOk, 10);
    QCOMPARE(t.tick(1000), A::Resend); // the partial count restarts
    QCOMPARE(t.okCount(), 0);
    t.noteSent(1200);
    QCOMPARE(t.okCount(), 0);
  }
  void givesUpOnceAtCap() {
    GainPushTracker t;
    t.begin("5in", 6, 0);
    int gaveUp = 0, resends = 0;
    for (qint64 now = 1000; now <= 40000; now += 1000) {
      const A a = t.tick(now);
      gaveUp += a == A::GaveUp;
      resends += a == A::Resend;
    }
    QCOMPARE(gaveUp, 1);
    QCOMPARE(resends, 29); // 1 s .. 29 s; the 30 s tick gives up
    QVERIFY(t.failed());
    QVERIFY(!t.pending());
  }
  void presetChangeRearms() {
    GainPushTracker t;
    t.begin("5in", 6, 0);
    t.onAck(GainPushTracker::kResultOk, 10);
    t.cancel(); // preset changed
    QVERIFY(!t.pending());
    QCOMPARE(t.tick(5000), A::None);
    t.begin("3in", 6, 6000);
    QVERIFY(t.pending());
    QCOMPARE(t.id(), QString("3in"));
    QCOMPARE(t.okCount(), 0);
    QCOMPARE(t.retries(), 0);
    QCOMPARE(t.tick(6500), A::None);
    QCOMPARE(t.tick(7000), A::Resend); // timers restart from the new begin
  }
  void rearmAfterFailureAndAccept() {
    GainPushTracker t;
    t.begin("5in", 6, 0);
    t.tick(30000);
    QVERIFY(t.failed());
    t.begin("5in", 6, 31000);
    QVERIFY(!t.failed());
    QVERIFY(t.pending());
  }
  void customVehicleNeverPending() {
    GainPushTracker t;
    t.begin("", 0, 0);
    QVERIFY(!t.pending());
    QCOMPARE(t.tick(100000), A::None);
    QVERIFY(!t.onAck(GainPushTracker::kResultOk, 0));
    t.begin("", 6, 0); // empty id wins over a stray count
    QVERIFY(!t.pending());
  }
};

QTEST_APPLESS_MAIN(TstGainPushTracker)
#include "tst_gain_push_tracker.moc"
