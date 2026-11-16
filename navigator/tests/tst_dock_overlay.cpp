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
// The dock drop overlay and the floating windows must be able to paint after
// the dock host has been built. DockShell freezes the host's painting while it
// builds a layout (SwitchFreeze); Qt's recursive re-enable skips child
// *windows*, and the ADS drop overlays (and any floating container) are child
// windows of the dock manager, so they used to stay frozen for good: no area
// highlight, no cross icons while dragging a panel.
#include "DockShell.h"

#include <QApplication>
#include <QLabel>
#include <QMainWindow>
#include <QSettings>
#include <QTemporaryDir>
#include <QtTest>

class TstDockOverlay : public QObject {
  Q_OBJECT
  QTemporaryDir m_dir;

private slots:
  void initTestCase() {
    QVERIFY(m_dir.isValid());
    QCoreApplication::setOrganizationName("NAVRobotecTest");
    QCoreApplication::setApplicationName("tst_dock_overlay");
    QSettings::setDefaultFormat(QSettings::IniFormat);
    QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, m_dir.path());
  }

  void childWindowsOfTheDockHostCanPaint() {
    QMainWindow win;
    win.resize(1200, 800);
    auto *shell = new DockShell(&win);
    for (const auto &pi : DockShell::panelInfos())
      shell->addPanel(pi.id, new QLabel(pi.id));
    shell->start();
    win.show();
    QVERIFY(QTest::qWaitForWindowExposed(&win));
    QTest::qWait(300);
    QVERIFY(win.updatesEnabled());
    int overlays = 0;
    const auto widgets = win.findChildren<QWidget *>();
    for (QWidget *w : widgets) {
      if (!w->isWindow())
        continue;
      const QByteArray cls = w->metaObject()->className();
      if (cls.startsWith("ads::CDockOverlay"))
        ++overlays;
      QVERIFY2(w->updatesEnabled(),
               qPrintable(QString::fromLatin1(cls) +
                          " is a child window with painting still frozen"));
    }
    // The manager owns a container overlay and a dock-area overlay, each with
    // a cross: all four must have been looked at.
    QVERIFY2(overlays >= 4, qPrintable(QString::number(overlays)));
  }
};

QTEST_MAIN(TstDockOverlay)
#include "tst_dock_overlay.moc"
