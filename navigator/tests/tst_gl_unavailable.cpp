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
// tst_gl_unavailable.cpp - the 3D views must not crash when no OpenGL context
// can be created; they show "OpenGL is not available" in their place.
//
// The test turns GL off itself (QT_XCB_GL_INTEGRATION=none, set before the
// QApplication exists), so it needs the xcb platform (an Xvfb) to be a real
// check; on another platform it skips when GL happens to work there.
// (A QOpenGLWindow shown with no GL crashes inside Qt itself, so the views
// must not create one; that is what is asserted.)
#include "AttitudeWidget.h"
#include "Drone3DWidget.h"
#include "GlAvailability.h"

#include <QApplication>
#include <QLabel>
#include <QtTest>

class TstGlUnavailable : public QObject {
  Q_OBJECT
private slots:
  void initTestCase() {
    if (glavail::available())
      QSKIP("this platform can make a GL context; nothing to test");
    QVERIFY(!glavail::unavailableReason().isEmpty());
  }

  void dockedViewsShowANoticeInsteadOfCrashing() {
    QWidget host;
    auto *lay = new QVBoxLayout(&host);
    auto *att = new AttitudeDockedWidget(&host);
    auto *d3 = new Drone3DDockedWidget(&host);
    lay->addWidget(att);
    lay->addWidget(d3);
    host.resize(600, 600);
    host.show();
    QTest::qWait(400); // the expose that used to crash in initializeGL
    for (QWidget *w : {static_cast<QWidget *>(att), static_cast<QWidget *>(d3)}) {
      auto *notice = w->findChild<QLabel *>("glUnavailableNotice");
      QVERIFY(notice);
      QVERIFY(notice->text().startsWith("OpenGL is not available:"));
    }
    QCOMPARE(att->glInitCount(), 0);
    att->setAttitude(0.1f, 0.2f, 0.3f); // updating a view with no window is fine
    QTest::qWait(100);
  }
};

int main(int argc, char **argv) {
  qputenv("QT_XCB_GL_INTEGRATION", "none");
  QApplication app(argc, argv);
  TstGlUnavailable t;
  return QTest::qExec(&t, argc, argv);
}

#include "tst_gl_unavailable.moc"
