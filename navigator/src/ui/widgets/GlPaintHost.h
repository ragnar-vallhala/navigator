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
#pragma once

#include <QOpenGLFunctions>
#include <QOpenGLWindow>
#include <QPainter>
#include <QVBoxLayout>
#include <QWidget>

// A QPainter-drawn view on a QOpenGLWindow inside a window container, with the
// widget API of a QOpenGLWidget (update(), show/hide events).
//
// Why not QOpenGLWidget: under Qt Advanced Docking a QOpenGLWidget re-runs
// initializeGL() (drops its GL context) every time its dock is re-parented, and
// the main window can then stop presenting. A window container keeps one
// context for the widget's life (tests/tst_workspace_gl.cpp,
// tools/workspace_mock/README.md; same approach as the simulator's 3D view).
// Subclasses draw in paintScene(); call update() as before.
class GlPaintHost : public QWidget {
public:
  explicit GlPaintHost(QWidget *parent = nullptr) : QWidget(parent) {
    m_win = new Win(this);
    m_box = QWidget::createWindowContainer(m_win, this);
    auto *l = new QVBoxLayout(this);
    l->setContentsMargins(0, 0, 0, 0);
    l->addWidget(m_box);
  }
  ~GlPaintHost() override {
    // Delete the container (and with it the window) first, so a paint during
    // teardown cannot call the pure-virtual paintScene() of a half-destroyed
    // object.
    delete m_box;
    m_box = nullptr;
  }
  // Repaint the window that holds the pixels. (Named apart from
  // QWidget::update(), which this used to shadow; subclasses call it
  // unqualified.)
  void updateScene() { m_win->update(); }
  // initializeGL() calls of the hosted window (1 for the widget's life).
  int glInitCount() const { return m_win->inits; }

protected:
  virtual void paintScene(QPainter &p) = 0;

private:
  class Win : public QOpenGLWindow, protected QOpenGLFunctions {
  public:
    explicit Win(GlPaintHost *h) : QOpenGLWindow(NoPartialUpdate), m_h(h) {}
    int inits = 0;

  protected:
    void initializeGL() override {
      initializeOpenGLFunctions();
      ++inits;
      glClearColor(0.08f, 0.09f, 0.12f, 1.0f);
    }
    void resizeGL(int w, int h) override { glViewport(0, 0, w, h); }
    void paintGL() override {
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      QPainter p(this);
      m_h->paintScene(p);
    }

  private:
    GlPaintHost *m_h;
  };
  Win *m_win = nullptr;
  QWidget *m_box = nullptr;
};
