// CommandRetrier / PendingCommand: sync-gated, ack-retried Start commands.
// Clock-free: every call passes the time explicitly. The "firmware" below is
// a model of router_command_gate: TEMPORARILY_REJECTED to everything until the
// clock is synced, never queued. What this cannot show is the real engine
// answering; that needs the SITL module (tst_airframe_flight / a live run).
#include <QtTest>

#include "CommandRetrier.h"

using K = PendingCommand::Ack;
using A = PendingCommand::Action;

namespace {
constexpr quint32 kMix = 8197, kMode = 8198, kPid = 8195;
struct Fw { // gate model
  bool synced = false;
  int received = 0;
};
} // namespace

class TstCommandRetrier : public QObject {
  Q_OBJECT
private slots:
  // ---- PendingCommand ----
  void pendingBasics() {
    PendingCommand p;
    QVERIFY(!p.pending());
    p.begin(6, 0);
    QVERIFY(p.pending());
    QCOMPARE(p.tick(999), A::None);
    QCOMPARE(p.tick(1000), A::Resend);
    QCOMPARE(p.tick(2000), A::Resend);
    QCOMPARE(p.retries(), 2);
    int fired = 0;
    for (int i = 0; i < 6; ++i)
      fired += p.onAck(0) == K::Accepted;
    QCOMPARE(fired, 1);
    QVERIFY(!p.pending());
    QCOMPARE(p.tick(9000), A::None);
    QCOMPARE(p.onAck(0), K::None);
  }
  void tempRejectedStaysPending() {
    PendingCommand p;
    p.begin(2, 0);
    QCOMPARE(p.onAck(1), K::None);
    QCOMPARE(p.onAck(1), K::None);
    QVERIFY(p.pending());
    QCOMPARE(p.rejectedCount(), 2);
    QCOMPARE(p.tick(1000), A::Resend);
    QCOMPARE(p.onAck(0), K::None);
    QCOMPARE(p.onAck(0), K::Accepted);
  }
  void hardErrorFailsOnce() {
    PendingCommand p;
    p.begin(1, 0);
    QCOMPARE(p.onAck(4), K::Failed);
    QVERIFY(p.failed() && !p.pending());
    QCOMPARE(p.lastError(), 4);
    QCOMPARE(p.onAck(4), K::None);
  }
  void givesUpOnceAtCap() {
    PendingCommand p;
    p.begin(1, 0);
    int gave = 0, resend = 0;
    for (qint64 t = 1000; t <= 40000; t += 1000) {
      const A a = p.tick(t);
      gave += a == A::GaveUp;
      resend += a == A::Resend;
    }
    QCOMPARE(gave, 1);
    QCOMPARE(resend, 29);
    QVERIFY(p.failed());
  }
  void zeroExpectedNeverPending() {
    PendingCommand p;
    p.begin(0, 0);
    QVERIFY(!p.pending());
    QCOMPARE(p.tick(99999), A::None);
  }

  // ---- CommandRetrier ----
  void armWhileUnsyncedDefersThenSyncSends() {
    CommandRetrier r;
    int sent = 0;
    QVERIFY(r.arm("mix", kMix, 1, "mix", "bad", [&] { ++sent; }, 0));
    QCOMPARE(sent, 0); // waits for sync
    r.setSynced(true, 100);
    QCOMPARE(sent, 1);
    r.setSynced(true, 200); // not an edge
    QCOMPARE(sent, 1);
  }
  void armWhileSyncedSendsNow() {
    CommandRetrier r;
    r.setSynced(true, 0);
    int sent = 0;
    r.arm("mix", kMix, 1, "mix", "bad", [&] { ++sent; }, 0);
    QCOMPARE(sent, 1);
  }
  void retriesWithoutSyncFlagAsSafetyNet() {
    CommandRetrier r;
    int sent = 0;
    r.arm("mix", kMix, 1, "mix", "bad", [&] { ++sent; }, 0);
    r.tick(500);
    QCOMPARE(sent, 0);
    r.tick(1000);
    QCOMPARE(sent, 1);
    r.tick(2000);
    QCOMPARE(sent, 2);
  }
  void acceptedLoggedOnce() {
    CommandRetrier r;
    QStringList log;
    r.setLog([&](const QString &s) { log << s; });
    r.arm("gains", kPid, 6, "5in gains", "stock gains", [] {}, 0);
    r.onAck(kPid, 1);
    for (int i = 0; i < 8; ++i)
      r.onAck(kPid, 0);
    QCOMPARE(log.size(), 1);
    QVERIFY(log[0].contains("accepted 5in gains (6/6 ACK OK"));
    QVERIFY(!r.anyPending());
  }
  void giveUpWarnsOnce() {
    CommandRetrier r;
    QStringList log;
    r.setLog([&](const QString &s) { log << s; });
    r.arm("mix", kMix, 1, "mix", "may topple", [] {}, 0);
    for (qint64 t = 1000; t <= 45000; t += 1000)
      r.tick(t);
    QCOMPARE(log.size(), 1);
    QVERIFY(log[0].contains("did not acknowledge mix; may topple"));
  }
  void refusedLoggedOnce() {
    CommandRetrier r;
    QStringList log;
    r.setLog([&](const QString &s) { log << s; });
    r.arm("mix", kMix, 1, "mix", "bad", [] {}, 0);
    r.onAck(kMix, 3);
    r.onAck(kMix, 3);
    QCOMPARE(log.size(), 1);
    QVERIFY(log[0].contains("refused mix (result 3)"));
  }
  void acksRouteByMsgid() {
    CommandRetrier r;
    r.arm("mix", kMix, 1, "mix", "x", [] {}, 0);
    r.arm("mode", kMode, 1, "mode", "x", [] {}, 0);
    r.onAck(kMode, 0);
    QVERIFY(r.pending("mix"));
    QVERIFY(!r.pending("mode"));
    r.onAck(99, 0); // unrelated command
    QVERIFY(r.pending("mix"));
  }
  void rearmAndCancel() {
    CommandRetrier r;
    r.arm("gains", kPid, 6, "5in", "x", [] {}, 0);
    r.cancel("gains");
    QVERIFY(!r.anyPending());
    int sent = 0;
    r.setSynced(true, 10);
    r.arm("gains", kPid, 6, "3in", "x", [&] { ++sent; }, 20);
    QCOMPARE(sent, 1);
    QVERIFY(r.pending("gains"));
    r.cancelAll();
    QVERIFY(!r.anyPending());
    QVERIFY(!r.arm("custom", kPid, 0, "none", "x", [] {}, 0));
  }

  // Every Start command ends ACK OK against the gate model, whatever order
  // sync and Start happen in.
  void startCommandsAllEndAckOkBeforeSyncStart() { scenario(false); }
  void startCommandsAllEndAckOkAfterSyncStart() { scenario(true); }

private:
  void scenario(bool syncFirst) {
    CommandRetrier r;
    Fw fw;
    QStringList log;
    r.setLog([&](const QString &s) { log << s; });
    auto sender = [&](quint32 msgid, int frames) {
      return [&r, &fw, msgid, frames] {
        for (int i = 0; i < frames; ++i)
          r.onAck(msgid, fw.synced ? 0 : 1);
      };
    };
    qint64 t = 0;
    if (syncFirst) {
      fw.synced = true;
      r.setSynced(true, t);
    }
    r.arm("mix", kMix, 1, "mix", "x", sender(kMix, 1), t);
    r.arm("mode", kMode, 1, "mode", "x", sender(kMode, 1), t);
    r.arm("gains", kPid, 6, "gains", "x", sender(kPid, 6), t);
    if (!syncFirst) {
      for (t = 1000; t <= 3000; t += 1000)
        r.tick(t); // sends are rejected, still pending
      QVERIFY(r.anyPending());
      QVERIFY(log.isEmpty());
      fw.synced = true;
      r.setSynced(true, t + 100);
    }
    for (int i = 0; i < 3 && r.anyPending(); ++i)
      r.tick(t += 1000);
    QVERIFY(!r.anyPending());
    QCOMPARE(log.size(), 3); // one accept line each, no warnings
    for (const QString &l : log)
      QVERIFY(l.startsWith("firmware accepted"));
  }
};

QTEST_APPLESS_MAIN(TstCommandRetrier)
#include "tst_command_retrier.moc"
