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

#include <QStringList>
#include <QWidget>

// AUX segmented switch indicator (mockup .sw2/.sw3): a labelled row of segments
// with the active one highlighted, driven by an RC aux channel value. 2-way
// splits at 1500µs; 3-way splits at 1300/1700µs.
class AuxSwitch : public QWidget {
  Q_OBJECT

public:
  AuxSwitch(const QString &label, const QStringList &segments,
            QWidget *parent = nullptr);

  // Set the active segment directly, or from an RC channel value (µs).
  void setActive(int index);
  void setFromChannel(int us);

protected:
  void paintEvent(QPaintEvent *event) override;
  QSize sizeHint() const override { return {220, 22}; }

private:
  QString m_label;
  QStringList m_segments;
  int m_active = -1; // -1 = N/A
};
