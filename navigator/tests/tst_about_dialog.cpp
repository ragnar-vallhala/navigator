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
#include <QLabel>
#include <QtTest>

#include "AboutDialog.h"

// Phase-3A/3B smoke: the About dialog renders the app identity, and docsPath()
// returns either a real directory or an empty string (never garbage).
class TstAboutDialog : public QObject {
  Q_OBJECT

private slots:
  void showsAppName();
  void docsPathIsEmptyOrExists();
};

void TstAboutDialog::showsAppName() {
  AboutDialog dlg;
  bool found = false;
  for (auto *l : dlg.findChildren<QLabel *>())
    if (l->text().contains("Navigator")) {
      found = true;
      break;
    }
  QVERIFY(found);
}

void TstAboutDialog::docsPathIsEmptyOrExists() {
  const QString p = AboutDialog::docsPath();
  if (!p.isEmpty())
    QVERIFY(QDir(p).exists());
}

QTEST_MAIN(TstAboutDialog)
#include "tst_about_dialog.moc"
