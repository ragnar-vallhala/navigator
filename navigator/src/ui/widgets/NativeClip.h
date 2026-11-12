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

#include <QEvent>
#include <QObject>
#include <QPointer>
#include <QRect>
#include <QTimer>
#include <QVector>
#include <QWidget>
#include <QWindow>

// Keeps a native window container (QWidget::createWindowContainer) inside the
// widgets that hold it.
//
// A native child window is not clipped by its parent *widgets*: when a layout
// cannot give the container its minimum size (a dock area can be dragged down
// to 60x40 whatever its content asks for) the window simply sticks out over
// the neighbouring docks. This clips the container with a window mask to the
// intersection of every ancestor's rectangle, and re-does that whenever the
// container or one of its ancestors moves, resizes, shows, hides or is
// re-parented (a dock being floated or tabbed).
class NativeClip : public QObject {
public:
  NativeClip(QWidget *box, QWindow *win) : QObject(box), m_box(box), m_win(win) {
    m_timer.setSingleShot(true);
    m_timer.setInterval(0);
    QObject::connect(&m_timer, &QTimer::timeout, this, [this] { apply(); });
    m_box->installEventFilter(this);
    rewatch();
    schedule();
  }

  // The clip rectangle in the container's own coordinates.
  static QRect visibleRect(const QWidget *box) {
    QRect r = box->rect();
    for (const QWidget *p = box->parentWidget(); p; p = p->parentWidget()) {
      r &= QRect(box->mapFromGlobal(p->mapToGlobal(QPoint(0, 0))), p->size());
      if (p->isWindow())
        break;
    }
    return r;
  }

  // Apply now (also what the deferred update calls).
  void apply() {
    if (!m_box)
      return;
    const QRect full = m_box->rect();
    const QRect vis = visibleRect(m_box);
    // The mask goes on the embedded QWindow: a widget mask on the container
    // would not touch the foreign native window it hosts.
    if (!m_win)
      return;
    if (vis == full || full.isEmpty()) {
      if (m_masked) {
        m_win->setMask(QRegion());
        m_masked = false;
      }
    } else if (vis.isEmpty()) {
      m_win->setMask(QRegion(0, 0, 1, 1)); // an empty region means "no mask"
      m_masked = true;
    } else {
      m_win->setMask(QRegion(vis));
      m_masked = true;
    }
  }

  bool masked() const { return m_masked; }

  bool eventFilter(QObject *o, QEvent *e) override {
    switch (e->type()) {
    case QEvent::Move:
    case QEvent::Resize:
    case QEvent::Show:
    case QEvent::Hide:
    case QEvent::LayoutRequest:
      schedule();
      break;
    case QEvent::ParentChange:
      if (o == m_box)
        rewatch();
      schedule();
      break;
    default:
      break;
    }
    if (e->type() == QEvent::Show && o == m_box)
      rewatch(); // re-parented ancestors announce themselves by re-showing
    return false;
  }

private:
  void schedule() { m_timer.start(); }
  void rewatch() {
    for (const QPointer<QWidget> &w : m_watched)
      if (w)
        w->removeEventFilter(this);
    m_watched.clear();
    for (QWidget *p = m_box->parentWidget(); p; p = p->parentWidget()) {
      p->installEventFilter(this);
      m_watched.push_back(p);
      if (p->isWindow())
        break;
    }
  }

  QPointer<QWidget> m_box;
  QPointer<QWindow> m_win;
  QVector<QPointer<QWidget>> m_watched;
  QTimer m_timer;
  bool m_masked = false;
};
