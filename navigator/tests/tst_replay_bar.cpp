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
#include <QPushButton>
#include <QSignalSpy>
#include <QTemporaryDir>
#include <QtTest>

#include "RecordSink.h"
#include "ReplayBar.h"
#include "ReplaySource.h"

// Phase-2E GUI smoke: the bar drives a bound ReplaySource (play toggles it) and
// emits exitRequested. Buttons are reached via objectName-independent text
// lookup. Transport correctness itself is covered by tst_replay_source.
class TstReplayBar : public QObject {
  Q_OBJECT

private:
  QTemporaryDir dir;
  QString logPath;

  QPushButton *button(QWidget *w, const QString &text) {
    for (auto *b : w->findChildren<QPushButton *>())
      if (b->text() == text)
        return b;
    return nullptr;
  }

private slots:
  void initTestCase() {
    logPath = dir.filePath("rec.bin");
    RecordSink s;
    QVERIFY(s.open(logPath, 1, 0));
    s.writeFrame(0, QByteArray("a"));
    s.writeFrame(100000, QByteArray("b"));
    s.close();
  }

  void playButtonTogglesSource();
  void exitButtonEmitsSignal();
  void unbindIsSafe();
};

void TstReplayBar::playButtonTogglesSource() {
  ReplaySource src;
  QVERIFY(src.open(logPath));
  ReplayBar bar;
  bar.bind(&src);

  // Transport buttons are icon-only now; the play button carries a stable
  // objectName for lookup. Clicking it toggles the source play state.
  auto *play = bar.findChild<QPushButton *>("rpPlay");
  QVERIFY(play);
  play->click();
  QVERIFY(src.isPlaying());
  play->click();
  QVERIFY(!src.isPlaying());
}

void TstReplayBar::exitButtonEmitsSignal() {
  ReplayBar bar;
  QSignalSpy spy(&bar, &ReplayBar::exitRequested);
  auto *exit = button(&bar, "Exit Replay");
  QVERIFY(exit);
  exit->click();
  QCOMPARE(spy.count(), 1);
}

void TstReplayBar::unbindIsSafe() {
  ReplaySource src;
  QVERIFY(src.open(logPath));
  ReplayBar bar;
  bar.bind(&src);
  bar.bind(nullptr); // must not crash; controls become inert
  auto *play = bar.findChild<QPushButton *>("rpPlay");
  QVERIFY(play);
  play->click(); // no bound source -> no effect, no crash
  QVERIFY(!src.isPlaying());
}

QTEST_MAIN(TstReplayBar)
#include "tst_replay_bar.moc"
