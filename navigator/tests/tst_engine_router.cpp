// EngineRouter: host -> firmware actions run on the engine-stepping thread
// while a dispatcher is installed, and at once on the caller otherwise.
#include "EngineRouter.h"
#include <QtTest>
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

using vsim::EngineRouter;

class TstEngineRouter : public QObject {
  Q_OBJECT
private slots:
  void runsAtOnceWithoutADispatcher() {
    EngineRouter r;
    int n = 0;
    QVERIFY(!r.run([&] { ++n; }));
    QCOMPARE(n, 1);
  }

  void queuesWhileADispatcherIsInstalled() {
    EngineRouter r;
    std::vector<EngineRouter::Op> q;
    r.set([&](EngineRouter::Op op) { q.push_back(std::move(op)); });
    int n = 0;
    QVERIFY(r.run([&] { ++n; }));
    QCOMPARE(n, 0); // not run by the caller
    QCOMPARE(int(q.size()), 1);
    q[0]();
    QCOMPARE(n, 1);
    r.set({}); // back to direct
    QVERIFY(!r.run([&] { ++n; }));
    QCOMPARE(n, 2);
  }

  // Four GUI-like threads hand actions in while a stepper thread drains them
  // between steps. Every action must run on the stepper thread, none lost, and
  // after set({}) the stepper's last drain finds nothing more can arrive.
  void everyActionRunsOnTheSteppingThread() {
    EngineRouter r;
    std::mutex qm;
    std::vector<EngineRouter::Op> q;
    r.set([&](EngineRouter::Op op) {
      std::lock_guard<std::mutex> lk(qm);
      q.push_back(std::move(op));
    });

    std::atomic<bool> stop{false};
    std::thread::id stepperId;
    long ran = 0;      // touched only by the stepper thread
    long wrongThread = 0;
    auto drain = [&] {
      std::vector<EngineRouter::Op> batch;
      {
        std::lock_guard<std::mutex> lk(qm);
        batch.swap(q);
      }
      for (auto &op : batch)
        op();
    };
    std::thread stepper([&] {
      stepperId = std::this_thread::get_id();
      while (!stop.load())
        drain();
      drain(); // the loop's last drain, on the stepping thread, as SimWorker does
    });

    constexpr int kThreads = 4, kEach = 2000;
    std::vector<std::thread> senders;
    for (int t = 0; t < kThreads; ++t)
      senders.emplace_back([&] {
        for (int i = 0; i < kEach; ++i)
          r.run([&] {
            if (std::this_thread::get_id() != stepperId)
              ++wrongThread;
            ++ran;
          });
      });
    for (auto &s : senders)
      s.join();
    r.set({}); // waits out any dispatch in flight
    stop = true;
    stepper.join();
    QCOMPARE(ran, long(kThreads) * kEach);
    QCOMPARE(wrongThread, 0L);
  }
};

QTEST_APPLESS_MAIN(TstEngineRouter)
#include "tst_engine_router.moc"
