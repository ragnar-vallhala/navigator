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
// A native window container is not clipped by the widgets that hold it, so a
// dock area smaller than its content let the simulator's 3D view stick out over
// the Logs dock. NativeClip masks the embedded window to its ancestors. Needs
// real native windows: skips on "offscreen" (run with QT_QPA_PLATFORM=xcb).
#include "NativeClip.h"

#include <QGuiApplication>
#include <QOpenGLWindow>
#include <QtTest>

class TstNativeClip : public QObject {
  Q_OBJECT
private slots:
  void initTestCase() {
    if (QGuiApplication::platformName() != "xcb")
      QSKIP("needs native child windows (QT_QPA_PLATFORM=xcb)");
  }

  void clipsToTheHoldingWidget() {
    QWidget holder;
    holder.resize(300, 100);
    auto *win = new QOpenGLWindow;
    QWidget *box = QWidget::createWindowContainer(win, &holder);
    box->setGeometry(0, 20, 300, 300); // taller than the holder: overflows
    auto *clip = new NativeClip(box, win);
    holder.show();
    QVERIFY(QTest::qWaitForWindowExposed(&holder));
    QTRY_VERIFY(clip->masked());
    QCOMPARE(win->mask().boundingRect(), QRect(0, 0, 300, 80));

    // The holder shrinks: the clip follows.
    holder.resize(300, 60);
    QTRY_COMPARE(win->mask().boundingRect(), QRect(0, 0, 300, 40));

    // Big enough again: no mask at all.
    holder.resize(300, 400);
    QTRY_VERIFY(win->mask().isEmpty());
    QVERIFY(!clip->masked());
  }

  // Floating a dock re-parents an ancestor of the container, not the
  // container: the mask must follow the new chain.
  void followsAReparentedAncestor() {
    QWidget winA;
    winA.resize(300, 300);
    auto *mid = new QWidget(&winA); // re-parented below: owned by its parent
    mid->setGeometry(0, 0, 300, 100);
    auto *win = new QOpenGLWindow;
    QWidget *box = QWidget::createWindowContainer(win, mid);
    box->setGeometry(0, 0, 300, 250);
    auto *clip = new NativeClip(box, win);
    winA.show();
    QVERIFY(QTest::qWaitForWindowExposed(&winA));
    QTRY_COMPARE(win->mask().boundingRect(), QRect(0, 0, 300, 100));

    QWidget winB; // a "floating window" of another size
    winB.resize(300, 60);
    mid->setParent(&winB);
    mid->setGeometry(0, 0, 300, 200);
    winB.show();
    QVERIFY(QTest::qWaitForWindowExposed(&winB));
    QTRY_COMPARE(win->mask().boundingRect(), QRect(0, 0, 300, 60));
    winB.resize(300, 40); // only the new top-level changes
    QTRY_COMPARE(win->mask().boundingRect(), QRect(0, 0, 300, 40));
    Q_UNUSED(clip);
  }

  void hidesWhenFullyClippedAndComesBack() {
    QWidget holder;
    holder.resize(300, 100);
    auto *win = new QOpenGLWindow;
    QWidget *box = QWidget::createWindowContainer(win, &holder);
    box->setGeometry(0, 150, 300, 100); // entirely below the holder
    auto *clip = new NativeClip(box, win);
    holder.show();
    QVERIFY(QTest::qWaitForWindowExposed(&holder));
    QTRY_VERIFY(clip->hiddenByClip());
    QVERIFY(!win->isVisible());

    holder.resize(300, 200); // part of it shows again
    QTRY_VERIFY(win->isVisible());
    QVERIFY(!clip->hiddenByClip());
    QCOMPARE(win->mask().boundingRect(), QRect(0, 0, 300, 50));
  }

  void doesNotShowAWindowItDidNotHide() {
    QWidget holder;
    holder.resize(300, 100);
    auto *win = new QOpenGLWindow;
    QWidget *box = QWidget::createWindowContainer(win, &holder);
    box->setGeometry(0, 0, 300, 100);
    auto *clip = new NativeClip(box, win);
    holder.show();
    QVERIFY(QTest::qWaitForWindowExposed(&holder));
    box->hide(); // the owner hides it
    QTest::qWait(50);
    holder.resize(300, 120);
    QTest::qWait(50);
    QVERIFY(!win->isVisible());
    QVERIFY(!clip->hiddenByClip());
  }

  void visibleRectIsTheAncestorIntersection() {
    QWidget outer;
    outer.resize(200, 200);
    QWidget inner(&outer);
    inner.setGeometry(50, 50, 100, 100);
    QWidget box(&inner);
    box.setGeometry(-10, 30, 150, 150);
    outer.show();
    QCOMPARE(NativeClip::visibleRect(&box), QRect(10, 0, 100, 70));
  }
};

QTEST_MAIN(TstNativeClip)
#include "tst_native_clip.moc"
