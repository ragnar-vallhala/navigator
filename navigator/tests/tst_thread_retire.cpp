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
// vsim::joinOrAbandon: a worker that stops is joined; one that is stuck is
// left running and orphaned instead of being terminated or destroyed.

#include "ThreadRetire.h"

#include <QElapsedTimer>
#include <QSemaphore>
#include <QtTest>

namespace {
// Blocks until released, like a thread stuck in a semaphore wait.
class StuckThread : public QThread {
public:
  QSemaphore release;
  QSemaphore started;

protected:
  void run() override {
    started.release();
    release.acquire();
  }
};

class QuickThread : public QThread {
protected:
  void run() override {}
};
} // namespace

class TstThreadRetire : public QObject {
  Q_OBJECT
private slots:
  void aFinishedThreadIsJoined() {
    QObject parent;
    QuickThread t;
    t.setParent(&parent);
    t.start();
    QVERIFY(vsim::joinOrAbandon(&t, 5000));
    QVERIFY(t.isFinished());
    QCOMPARE(t.parent(), &parent); // untouched when it stopped
  }
  void aStuckThreadIsAbandonedNotKilled() {
    auto *parent = new QObject;
    auto *t = new StuckThread;
    t->setParent(parent);
    t->start();
    QVERIFY(t->started.tryAcquire(1, 5000));

    QElapsedTimer clock;
    clock.start();
    QVERIFY(!vsim::joinOrAbandon(t, 200));
    QVERIFY2(clock.elapsed() >= 150 && clock.elapsed() < 3000,
             "the wait is bounded by the given time");

    QVERIFY(t->isRunning());         // not terminated
    QVERIFY(t->parent() == nullptr); // out of its parent

    // Deleting the old parent used to delete the still-running thread, which
    // aborts the process; it must now leave the thread alone.
    delete parent;
    QVERIFY(t->isRunning());

    t->release.release(); // unstick it and clean up the way a real run would
    QVERIFY(t->wait(5000));
    delete t;
  }
  void aNullThreadCountsAsJoined() { QVERIFY(vsim::joinOrAbandon(nullptr, 10)); }
};

QTEST_MAIN(TstThreadRetire)
#include "tst_thread_retire.moc"
