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

#include <QElapsedTimer>
#include <QFontDatabase>
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <QOpenGLWindow>
#include <QVBoxLayout>
#include <QPainter>
#include <QTimer>
#include <cmath>

// A stand-in for SimRendererWidget: a QOpenGLWidget that animates continuously
// and counts how often its GL context was (re)created or destroyed, so docking
// can be checked against what a real GL panel needs to survive.
class GlPanel : public QOpenGLWidget, protected QOpenGLFunctions {
  Q_OBJECT
public:
  explicit GlPanel(QWidget *parent = nullptr) : QOpenGLWidget(parent) {
    setMinimumSize(160, 120);
    auto *t = new QTimer(this);
    connect(t, &QTimer::timeout, this, qOverload<>(&QWidget::update));
    t->start(16);
  }
  int frames() const { return m_frames; }
  int initCount() const { return m_inits; }    // initializeGL() calls
  int ctxDestroyed() const { return m_destroyed; } // aboutToBeDestroyed

protected:
  void initializeGL() override {
    initializeOpenGLFunctions();
    ++m_inits;
    if (context() != m_ctx) { // a new context: count when it goes away
      m_ctx = context();
      connect(m_ctx, &QOpenGLContext::aboutToBeDestroyed, this,
              [this] { ++m_destroyed; });
    }
  }
  void paintGL() override {
    ++m_frames;
    const float p = float(m_frames % 120) / 120.f;
    glClearColor(0.1f + 0.5f * p, 0.2f, 0.6f - 0.4f * p, 1.f); // GL, not QPainter
    glClear(GL_COLOR_BUFFER_BIT);
    QPainter pt(this);
    pt.setRenderHint(QPainter::Antialiasing);
    pt.translate(width() / 2.0, height() / 2.0);
    pt.rotate(m_frames * 3.0);
    pt.setBrush(Qt::white);
    pt.drawRect(QRectF(-40, -10, 80, 20));
    pt.resetTransform();
    pt.setPen(Qt::white);
    pt.drawText(8, 18, QStringLiteral("GL panel  frame %1  init %2  ctx-lost %3")
                           .arg(m_frames).arg(m_inits).arg(m_destroyed));
  }

private:
  int m_frames = 0, m_inits = 0, m_destroyed = 0;
  QOpenGLContext *m_ctx = nullptr;
};

// The same animation as a QOpenGLWindow embedded with createWindowContainer():
// the 3D view gets its own native surface and context, so re-parenting the
// container never recreates the context and the main window's backing store
// is not involved. Same counters as GlPanel.
class GlWindowPanel : public QWidget {
  Q_OBJECT
  class Win : public QOpenGLWindow, protected QOpenGLFunctions {
  public:
    Win() {
      auto *t = new QTimer(this);
      connect(t, &QTimer::timeout, this, qOverload<>(&QPaintDeviceWindow::update));
      t->start(16);
    }
    int frames = 0, inits = 0, destroyed = 0;

  protected:
    void initializeGL() override {
      initializeOpenGLFunctions();
      ++inits;
      if (context() != m_ctx) {
        m_ctx = context();
        connect(m_ctx, &QOpenGLContext::aboutToBeDestroyed, this, [this] { ++destroyed; });
      }
    }
    void paintGL() override {
      ++frames;
      const float p = float(frames % 120) / 120.f;
      glClearColor(0.1f + 0.5f * p, 0.2f, 0.6f - 0.4f * p, 1.f);
      glClear(GL_COLOR_BUFFER_BIT);
      QPainter pt(this);
      pt.setRenderHint(QPainter::Antialiasing);
      pt.translate(width() / 2.0, height() / 2.0);
      pt.rotate(frames * 3.0);
      pt.setBrush(Qt::white);
      pt.drawRect(QRectF(-40, -10, 80, 20));
      pt.resetTransform();
      pt.setPen(Qt::white);
      pt.drawText(8, 18, QStringLiteral("GL window  frame %1  init %2  ctx-lost %3")
                             .arg(frames).arg(inits).arg(destroyed));
    }

  private:
    QOpenGLContext *m_ctx = nullptr;
  };

public:
  explicit GlWindowPanel(QWidget *parent = nullptr) : QWidget(parent), m_win(new Win) {
    setMinimumSize(160, 120);
    auto *l = new QVBoxLayout(this);
    l->setContentsMargins(0, 0, 0, 0);
    l->addWidget(QWidget::createWindowContainer(m_win, this));
  }
  int frames() const { return m_win->frames; }
  int initCount() const { return m_win->inits; }
  int ctxDestroyed() const { return m_win->destroyed; }
  QImage grabFramebuffer() { return m_win->grabFramebuffer(); }

private:
  Win *m_win;
};
