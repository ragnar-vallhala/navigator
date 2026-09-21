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

#include <QDialog>
#include <QString>

// Static About dialog (FR-UX-22): app name, version, build stamp, Qt + protocol
// versions, and the flight-stack components. Opened from Help ▸ About.
class AboutDialog : public QDialog {
  Q_OBJECT

public:
  explicit AboutDialog(QWidget *parent = nullptr);

  // First existing candidate for the bundled documentation directory, or an
  // empty string if none is found. Static + filesystem-only so Help ▸
  // Documentation (FR-UX-23) and tests can both resolve it.
  static QString docsPath();
};
