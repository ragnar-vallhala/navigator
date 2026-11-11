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
// Workspace mock: a standalone app to prove the docking + workspace system
// before any real Navigator page is migrated. Every panel is a blank widget
// showing only its name. Not linked into Navigator.
//
// Test hooks (used for the screenshots): --script "step;step;..." with steps
//   workspace:NAME  float:ID  tab:ID+ONTO  hide:ID  show:ID  save:NAME
//   rename:A>B  delete:NAME  reset  geom:ID,x,y,w,h  shot:PATH  quit
#include "AdsBackend.h"
#include "WorkspaceManager.h"

#include <QActionGroup>
#include <QApplication>
#include <QCloseEvent>
#include <QCommandLineParser>
#include <QGuiApplication>
#include <QInputDialog>
#include <QLabel>
#include <QMainWindow>
#include <QMenuBar>
#include <QMessageBox>
#include <QScreen>
#include <QStatusBar>
#include <QTimer>

#include "MockPanels.h"

using namespace ws;

class MockWindow : public QMainWindow {
  Q_OBJECT
public:
  MockWindow() {
    setWindowTitle(tr("Workspace mock"));
    resize(1400, 800);
    m_wm = new WorkspaceManager(this, std::make_unique<AdsBackend>(this), this);
    m_wm->registerPanels(mockPanels());
    m_wm->setBuiltins(mockBuiltins());
    connect(m_wm, &WorkspaceManager::workspacesChanged, this,
            &MockWindow::rebuildMenu);
    connect(m_wm, &WorkspaceManager::currentChanged, this, [this](auto &n) {
      statusBar()->showMessage(
          n.isEmpty() ? tr("Workspace: (modified)") : tr("Workspace: %1").arg(n));
      rebuildMenu();
    });
    connect(m_wm, &WorkspaceManager::loadFailed, this,
            [this](const QString &n, const QString &why) {
              statusBar()->showMessage(
                  tr("Could not load \"%1\" (%2): default layout applied")
                      .arg(n, why),
                  8000);
            });
    buildMenus();
  }
  WorkspaceManager *wm() { return m_wm; }

protected:
  void closeEvent(QCloseEvent *e) override {
    m_wm->persistLast();
    QMainWindow::closeEvent(e);
  }

private:
  void buildMenus() {
    m_wsMenu = menuBar()->addMenu(tr("&Workspaces"));
    m_panelsMenu = menuBar()->addMenu(tr("&Panels"));
    for (const QString &id : m_wm->backend()->panelIds())
      m_panelsMenu->addAction(m_wm->backend()->viewAction(id));
    rebuildMenu();
  }

  void rebuildMenu() {
    m_wsMenu->clear();
    auto *grp = new QActionGroup(m_wsMenu);
    auto addWs = [&](const QString &n) {
      QAction *a = m_wsMenu->addAction(n);
      a->setCheckable(true);
      a->setChecked(n == m_wm->currentName());
      grp->addAction(a);
      connect(a, &QAction::triggered, this, [this, n] { m_wm->load(n); });
    };
    for (const QString &n : m_wm->builtinNames())
      addWs(n);
    const QStringList users = m_wm->userNames();
    if (!users.isEmpty())
      m_wsMenu->addSeparator();
    for (const QString &n : users)
      addWs(n);
    m_wsMenu->addSeparator();
    m_wsMenu->addAction(tr("&Save layout as…"), this, [this] {
      bool ok = false;
      const QString n = QInputDialog::getText(this, tr("Save workspace"),
                                              tr("Name:"), QLineEdit::Normal,
                                              QString(), &ok)
                            .trimmed();
      if (!ok || n.isEmpty())
        return;
      if (!m_wm->saveCurrentAs(n))
        QMessageBox::warning(this, tr("Save workspace"),
                             tr("\"%1\" is a built-in name or not valid.").arg(n));
    });
    QMenu *ren = m_wsMenu->addMenu(tr("&Rename"));
    QMenu *del = m_wsMenu->addMenu(tr("&Delete"));
    ren->setEnabled(!users.isEmpty());
    del->setEnabled(!users.isEmpty());
    for (const QString &n : users) {
      ren->addAction(n, this, [this, n] {
        bool ok = false;
        const QString to = QInputDialog::getText(this, tr("Rename workspace"),
                                                 tr("New name:"),
                                                 QLineEdit::Normal, n, &ok)
                               .trimmed();
        if (ok && !m_wm->rename(n, to))
          QMessageBox::warning(this, tr("Rename workspace"),
                               tr("Cannot rename to \"%1\".").arg(to));
      });
      del->addAction(n, this, [this, n] { m_wm->remove(n); });
    }
    m_wsMenu->addSeparator();
    m_wsMenu->addAction(tr("Reset &layout"), this, [this] { m_wm->resetLayout(); });
  }

  WorkspaceManager *m_wm = nullptr;
  QMenu *m_wsMenu = nullptr, *m_panelsMenu = nullptr;
};

// Drive the window from --script, for repeatable screenshots.
static void runScript(MockWindow &w, const QStringList &steps) {
  auto *t = new QTimer(&w);
  t->setInterval(700);
  auto idx = std::make_shared<int>(0);
  QObject::connect(t, &QTimer::timeout, &w, [&w, steps, idx, t] {
    if (*idx >= steps.size()) {
      t->stop();
      return;
    }
    const QString s = steps[(*idx)++];
    const QString cmd = s.section(':', 0, 0), arg = s.section(':', 1);
    auto *wm = w.wm();
    auto *b = wm->backend();
    if (cmd == "workspace")
      wm->load(arg);
    else if (cmd == "float")
      b->floatPanel(arg);
    else if (cmd == "tab")
      b->tabPanel(arg.section('+', 0, 0), arg.section('+', 1));
    else if (cmd == "hide")
      b->hidePanel(arg);
    else if (cmd == "show")
      b->showPanel(arg);
    else if (cmd == "save")
      wm->saveCurrentAs(arg);
    else if (cmd == "rename")
      wm->rename(arg.section('>', 0, 0), arg.section('>', 1));
    else if (cmd == "delete")
      wm->remove(arg);
    else if (cmd == "reset")
      wm->resetLayout();
    else if (cmd == "geom") {
      const QStringList p = arg.split(',');
      if (p.size() == 5)
        b->setFloatGeometry(p[0], QRect(p[1].toInt(), p[2].toInt(),
                                        p[3].toInt(), p[4].toInt()));
    } else if (cmd == "shot")
      QGuiApplication::primaryScreen()->grabWindow(0).save(arg);
    else if (cmd == "quit")
      w.close();
  });
  t->start();
}

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  app.setOrganizationName("NAVRobotec");
  app.setApplicationName("navigator-workspace-mock");
  QCommandLineParser p;
  QCommandLineOption script("script", "test script", "steps");
  p.addOption(script);
  p.process(app);

  MockWindow w;
  w.show();
  w.wm()->restoreLast();
  if (p.isSet(script))
    runScript(w, p.value(script).split(';', Qt::SkipEmptyParts));
  return app.exec();
}

#include "main.moc"
