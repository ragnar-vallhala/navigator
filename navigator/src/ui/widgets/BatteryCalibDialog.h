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

#include "../../core/BatteryCalib.h"

#include <QByteArray>
#include <QDialog>

class QDoubleSpinBox;
class QLabel;
class QPushButton;

// Two-point battery divider calibration. The operator reads the live raw count
// (BATTERY.counts), types what a multimeter says at the pack, and captures;
// twice, with the pack in two states far apart. The dialog fits, shows the
// residuals, refuses an out-of-band result, and produces the 16-byte
// 0:batcal.bin blob.
//
// There is NO upload from here: Navigator has no file-transfer (XFER) client
// yet, so the blob is saved to a file the operator copies to the card. The
// vehicle must be rebooted after the file is on the card.
class BatteryCalibDialog : public QDialog {
  Q_OBJECT
public:
  explicit BatteryCalibDialog(QWidget *parent = nullptr);

  // Latest BATTERY sample. `usable` = CONVERTED and not RAILED: the count is
  // evidence of the divider even when no pack is PRESENT by the old scale.
  void setLive(quint16 counts, bool usable);

  // Capture the live count as point `i` (0/1) with the volts typed for it.
  // False when there is no usable live count.
  bool capture(int i);
  void setVolts(int i, double volts);

  const batcal::Fit &fit() const { return m_fit; }
  QByteArray blob() const; // empty unless the fit is ok
  bool saveTo(const QString &path, QString *err = nullptr) const;

private:
  void refit();
  void saveClicked();

  QDoubleSpinBox *m_volts[2] = {nullptr, nullptr};
  QLabel *m_pointLbl[2] = {nullptr, nullptr};
  QPushButton *m_captureBtn[2] = {nullptr, nullptr};
  QLabel *m_liveLbl = nullptr;
  QLabel *m_resultLbl = nullptr;
  QPushButton *m_saveBtn = nullptr;
  quint16 m_live = 0;
  bool m_liveOk = false;
  bool m_have[2] = {false, false};
  batcal::Point m_pt[2];
  batcal::Fit m_fit;
};
