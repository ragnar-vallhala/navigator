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
#include "AboutDialog.h"

#include <QApplication>
#include <QDialogButtonBox>
#include <QDir>
#include <QLabel>
#include <QVBoxLayout>

#ifndef NAVIGATOR_VERSION
#define NAVIGATOR_VERSION "dev"
#endif

AboutDialog::AboutDialog(QWidget *parent) : QDialog(parent) {
  setWindowTitle(tr("About Navigator"));

  auto *v = new QVBoxLayout(this);

  auto *title = new QLabel(tr("Navigator"), this);
  QFont tf = title->font();
  tf.setPointSizeF(tf.pointSizeF() * 1.6);
  tf.setBold(true);
  title->setFont(tf);
  v->addWidget(title);

  // Rich-text body: version/build/runtime + flight-stack components.
  const QString body =
      tr("<b>Version</b> %1<br>"
         "<b>Built</b> %2 %3<br>"
         "<b>Qt</b> %4<br>"
         "<b>Telemetry protocol</b> v1<br><br>"
         "<b>Flight stack</b><br>"
         "• Navigator — ground control station<br>"
         "• Vayu — flight controller firmware<br>"
         "• vaios — real-time OS<br>"
         "• NavHAL — hardware abstraction layer")
          .arg(QStringLiteral(NAVIGATOR_VERSION), QStringLiteral(__DATE__),
               QStringLiteral(__TIME__), QStringLiteral(QT_VERSION_STR));
  auto *label = new QLabel(body, this);
  label->setTextFormat(Qt::RichText);
  label->setTextInteractionFlags(Qt::TextBrowserInteraction);
  v->addWidget(label);

  auto *box = new QDialogButtonBox(QDialogButtonBox::Close, this);
  connect(box, &QDialogButtonBox::rejected, this, &QDialog::reject);
  connect(box, &QDialogButtonBox::accepted, this, &QDialog::accept);
  v->addWidget(box);
}

QString AboutDialog::docsPath() {
  const QString appDir = QCoreApplication::applicationDirPath();
  const QStringList candidates = {
      appDir + "/docs",
      appDir + "/../docs",
      appDir + "/../navigator/docs",
      // Source-tree fallback for a dev run from the build directory.
      appDir + "/../../docs",
  };
  for (const QString &c : candidates) {
    QDir d(c);
    if (d.exists())
      return d.absolutePath();
  }
  return QString();
}
