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
// A floated panel is a tool window of the main window: Qt::Tool makes Qt set
// WM_TRANSIENT_FOR the main window and _NET_WM_WINDOW_TYPE_UTILITY on X11, so
// a desktop that cycles "windows of the same app" (GNOME Ctrl+`) skips it and
// it is not an app window of its own. ADS creates it as a plain Qt::Window.
#include "AdsBackend.h"
#include "WorkspaceManager.h"

#include <DockWidget.h>
#include <FloatingDockContainer.h>

#include <QLabel>
#include <QMainWindow>
#include <QSettings>
#include <QTemporaryDir>
#include <QtTest>

using namespace ws;

class TstFloatingTool : public QObject {
  Q_OBJECT
  QTemporaryDir m_dir;

private slots:
  void initTestCase() {
    QVERIFY(m_dir.isValid());
    QCoreApplication::setOrganizationName("NAVRobotecTest");
    QCoreApplication::setApplicationName("tst_floating_tool");
    QSettings::setDefaultFormat(QSettings::IniFormat);
    QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, m_dir.path());
  }

  void floatedPanelIsAToolWindow() {
    QMainWindow win;
    win.resize(1100, 700);
    auto b = std::make_unique<AdsBackend>(&win);
    AdsBackend *ads = b.get();
    WorkspaceManager wm(&win, std::move(b), &win);
    PanelRegistry r;
    for (const char *id : {"a", "b"}) {
      const QString s = QString::fromLatin1(id);
      r.add(s, s, [s] { return new QLabel(s); });
    }
    wm.registerPanels(r);
    using N = LayoutNode;
    wm.setBuiltins({{"S", {N::split(Qt::Horizontal, {N::tabs({"a"}), N::tabs({"b"})})}}});
    wm.setScreenProvider([] { return QList<QRect>{QRect(0, 0, 1600, 900)}; });
    win.show();
    wm.load("S");
    ads->floatPanel("b");
    auto *d = win.findChild<ads::CDockWidget *>("b");
    QVERIFY(d && d->isInFloatingContainer());
    auto *fw = d->floatingDockContainer();
    QVERIFY(fw->isWindow());
    QVERIFY2((fw->windowFlags() & Qt::WindowType_Mask) == Qt::Tool,
             "floating container is not a Qt::Tool window");
  }
};

QTEST_MAIN(TstFloatingTool)
#include "tst_floating_tool.moc"
