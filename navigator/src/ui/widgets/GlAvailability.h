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

#include <QGuiApplication>
#include <QLabel>
#include <QOpenGLContext>
#include <QString>
#include <QSurfaceFormat>
#include <QWidget>

// Whether this machine can make an OpenGL context at all. The 3D views (the
// simulator, the attitude horizon, the 3D drone) are GL windows, and a GL window
// on a platform that cannot create a context crashes inside Qt. They ask here
// first and show glMissingNotice() in their place when it says no.
//
// The probe makes a context (never made current, no window) with the format the
// views ask for, else a plain one, once per process; QOpenGLContext::create()
// is the same call a window makes.
namespace glavail {

// Empty when a context can be created, else a short human-readable reason.
inline QString unavailableReason() {
  static const QString reason = [] {
    if (!QGuiApplication::instance())
      return QString::fromLatin1("no GUI application");
    // The format the views ask for (app default plus depth and stencil), then
    // a plain one: Qt's windows fall back the same way, so a driver that
    // refuses 4.6 core but gives some context is still "available".
    QSurfaceFormat want = QSurfaceFormat::defaultFormat();
    want.setDepthBufferSize(24);
    want.setStencilBufferSize(8);
    for (const QSurfaceFormat &f : {want, QSurfaceFormat()}) {
      QOpenGLContext probe;
      probe.setFormat(f);
      if (probe.create() && probe.isValid())
        return QString();
    }
    const QString text =
        QObject::tr("no OpenGL context could be created on the '%1' display "
                    "platform (check the graphics driver, and that "
                    "QT_XCB_GL_INTEGRATION is not set to none)")
            .arg(QGuiApplication::platformName());
    qWarning("OpenGL is not available: %s", qPrintable(text));
    return text;
  }();
  return reason;
}

inline bool available() { return unavailableReason().isEmpty(); }

// A label that stands in for a GL view.
inline QLabel *makeNotice(QWidget *parent) {
  auto *l = new QLabel(
      QObject::tr("OpenGL is not available: %1").arg(unavailableReason()),
      parent);
  l->setAlignment(Qt::AlignCenter);
  l->setWordWrap(true);
  l->setObjectName(QStringLiteral("glUnavailableNotice"));
  return l;
}

} // namespace glavail
