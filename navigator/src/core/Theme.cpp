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
#include "Theme.h"

#include <QApplication>
#include <QFile>
#include <QPalette>
#include <QStyleFactory>
#include <QTextStream>

namespace Theme {

QString loadStyleSheet() {
  QFile f(":/styles/dark.qss");
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
    return {};
  return QString::fromUtf8(f.readAll());
}

void apply() {
  qApp->setStyle(QStyleFactory::create("Fusion"));

  QPalette p;
  p.setColor(QPalette::Window, kBg);
  p.setColor(QPalette::WindowText, kTextMuted);
  p.setColor(QPalette::Base, kBase);
  p.setColor(QPalette::AlternateBase, kSurface);
  p.setColor(QPalette::ToolTipBase, kSurfaceAlt);
  p.setColor(QPalette::ToolTipText, kTextMuted);
  p.setColor(QPalette::Text, kTextMuted);
  p.setColor(QPalette::Button, kSurfaceAlt);
  p.setColor(QPalette::ButtonText, kTextMuted);
  p.setColor(QPalette::BrightText, kDanger);
  p.setColor(QPalette::Link, kAccent);
  p.setColor(QPalette::Highlight, kAccent);
  p.setColor(QPalette::HighlightedText, Qt::black);
  p.setColor(QPalette::Disabled, QPalette::Text, kTextDim);
  p.setColor(QPalette::Disabled, QPalette::ButtonText, kTextDim);
  qApp->setPalette(p);

  const QString qss = loadStyleSheet();
  if (!qss.isEmpty())
    qApp->setStyleSheet(qss);
}

} // namespace Theme
