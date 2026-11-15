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
#include "BatteryCalibDialog.h"

#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QLabel>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QSaveFile>
#include <QVBoxLayout>
#include <QGridLayout>

BatteryCalibDialog::BatteryCalibDialog(QWidget *parent) : QDialog(parent) {
  setWindowTitle(tr("Battery calibration"));
  auto *v = new QVBoxLayout(this);

  auto *intro = new QLabel(
      tr("Measure the pack with a multimeter at two states far apart (charged "
         "and nearly flat). For each: type the multimeter volts, then Capture "
         "to take the live ADC count. A short baseline puts the measurement "
         "error straight into the offset."),
      this);
  intro->setWordWrap(true);
  v->addWidget(intro);

  m_liveLbl = new QLabel(this);
  v->addWidget(m_liveLbl);

  auto *g = new QGridLayout;
  for (int i = 0; i < 2; ++i) {
    m_volts[i] = new QDoubleSpinBox(this);
    m_volts[i]->setRange(0.0, 60.0);
    m_volts[i]->setDecimals(3);
    m_volts[i]->setSuffix(tr(" V"));
    m_captureBtn[i] = new QPushButton(tr("Capture"), this);
    m_pointLbl[i] = new QLabel(tr("not captured"), this);
    g->addWidget(new QLabel(tr("Point %1").arg(i + 1), this), i, 0);
    g->addWidget(m_volts[i], i, 1);
    g->addWidget(m_captureBtn[i], i, 2);
    g->addWidget(m_pointLbl[i], i, 3);
    connect(m_captureBtn[i], &QPushButton::clicked, this,
            [this, i] { capture(i); });
    connect(m_volts[i], &QDoubleSpinBox::valueChanged, this, [this, i] {
      if (m_have[i]) {
        m_pt[i].volts = m_volts[i]->value();
        refit();
      }
    });
  }
  v->addLayout(g);

  m_resultLbl = new QLabel(this);
  m_resultLbl->setWordWrap(true);
  m_resultLbl->setTextInteractionFlags(Qt::TextSelectableByMouse);
  v->addWidget(m_resultLbl);

  auto *note = new QLabel(
      tr("Upload sends the file to the vehicle's SD card as %1 over the "
         "link. Or save it and copy it to the card yourself. Either way, "
         "reboot the vehicle afterwards: the calibration is read once at "
         "start-up.")
          .arg(QString::fromLatin1(batcal::kCardPath)),
      this);
  note->setWordWrap(true);
  v->addWidget(note);

  m_uploadBar = new QProgressBar(this);
  m_uploadBar->setRange(0, 100);
  m_uploadBar->setVisible(false);
  v->addWidget(m_uploadBar);
  m_uploadLbl = new QLabel(this);
  m_uploadLbl->setWordWrap(true);
  m_uploadLbl->setTextInteractionFlags(Qt::TextSelectableByMouse);
  v->addWidget(m_uploadLbl);

  auto *bb = new QDialogButtonBox(this);
  m_uploadBtn =
      bb->addButton(tr("Upload to vehicle"), QDialogButtonBox::ActionRole);
  m_saveBtn = bb->addButton(tr("Save batcal.bin…"), QDialogButtonBox::ActionRole);
  connect(m_uploadBtn, &QPushButton::clicked, this, [this] {
    if (m_uploading && m_uploader)
      m_uploader->cancel();
    else
      startUpload();
  });
  bb->addButton(QDialogButtonBox::Close);
  connect(m_saveBtn, &QPushButton::clicked, this,
          &BatteryCalibDialog::saveClicked);
  connect(bb, &QDialogButtonBox::rejected, this, &QDialog::reject);
  v->addWidget(bb);

  setLive(0, false);
  refit();
  updateUploadUi();
}

void BatteryCalibDialog::setLive(quint16 counts, bool usable) {
  m_live = counts;
  m_liveOk = usable;
  m_liveLbl->setText(usable ? tr("Live ADC count: %1").arg(counts)
                            : tr("Live ADC count: no usable reading"));
  for (auto *b : m_captureBtn)
    b->setEnabled(usable);
}

void BatteryCalibDialog::setVolts(int i, double volts) {
  if (i < 0 || i > 1)
    return;
  m_volts[i]->setValue(volts);
}

bool BatteryCalibDialog::capture(int i) {
  if (i < 0 || i > 1 || !m_liveOk)
    return false;
  m_pt[i].counts = m_live;
  m_pt[i].volts = m_volts[i]->value();
  m_have[i] = true;
  m_pointLbl[i]->setText(tr("%1 counts").arg(m_live));
  refit();
  return true;
}

void BatteryCalibDialog::refit() {
  m_fit = batcal::Fit();
  if (!m_have[0] || !m_have[1]) {
    m_resultLbl->setText(tr("Capture both points to fit."));
    m_saveBtn->setEnabled(false);
    updateUploadUi();
    return;
  }
  QVector<batcal::Point> pts{m_pt[0], m_pt[1]};
  m_fit = batcal::fit(pts);
  QString t;
  if (m_fit.residualsMv.size() == 2) {
    for (int i = 0; i < 2; ++i)
      t += tr("Point %1: %2 counts, %3 V, residual %4 mV\n")
               .arg(i + 1)
               .arg(m_pt[i].counts, 0, 'f', 0)
               .arg(m_pt[i].volts, 0, 'f', 3)
               .arg(m_fit.residualsMv[i], 0, 'f', 1);
  }
  if (m_fit.ok) {
    t += tr("volts per count %1, offset %2 counts")
             .arg(m_fit.vpc, 0, 'f', 6)
             .arg(m_fit.offset, 0, 'f', 2);
  } else {
    t += tr("REFUSED: %1").arg(m_fit.error);
  }
  m_resultLbl->setText(t);
  m_saveBtn->setEnabled(m_fit.ok);
  updateUploadUi();
}

QByteArray BatteryCalibDialog::blob() const {
  return m_fit.ok ? batcal::pack(m_fit.vpc, m_fit.offset) : QByteArray();
}

bool BatteryCalibDialog::saveTo(const QString &path, QString *err) const {
  const QByteArray b = blob();
  if (b.isEmpty()) {
    if (err)
      *err = tr("no valid calibration to save");
    return false;
  }
  QSaveFile f(path);
  if (!f.open(QIODevice::WriteOnly) || f.write(b) != b.size() || !f.commit()) {
    if (err)
      *err = f.errorString();
    return false;
  }
  return true;
}

void BatteryCalibDialog::saveClicked() {
  const QString path = QFileDialog::getSaveFileName(
      this, tr("Save battery calibration"), QStringLiteral("batcal.bin"),
      tr("Calibration file (*.bin)"));
  if (path.isEmpty())
    return;
  QString err;
  if (!saveTo(path, &err)) {
    QMessageBox::warning(this, windowTitle(), err);
    return;
  }
  QMessageBox::information(
      this, windowTitle(),
      tr("Saved. Copy it to the SD card as %1 and reboot the vehicle for it "
         "to take effect.")
          .arg(QString::fromLatin1(batcal::kCardPath)));
}

void BatteryCalibDialog::setUploader(XferUploader *u) {
  if (m_uploader == u)
    return;
  if (m_uploader)
    m_uploader->disconnect(this);
  m_uploader = u;
  if (u) {
    connect(u, &XferUploader::progress, this,
            &BatteryCalibDialog::onUploadProgress);
    connect(u, &XferUploader::finished, this,
            &BatteryCalibDialog::onUploadFinished);
  }
  updateUploadUi();
}

void BatteryCalibDialog::setUploadAvailable(bool available, const QString &why) {
  m_uploadAvail = available;
  m_uploadWhy = why;
  if (!available && m_uploading && m_uploader)
    m_uploader->cancel(); // link no longer allows transmit
  updateUploadUi();
}

bool BatteryCalibDialog::uploadEnabled() const {
  return m_uploadBtn->isEnabled();
}

QString BatteryCalibDialog::uploadStatus() const { return m_uploadLbl->text(); }

void BatteryCalibDialog::updateUploadUi() {
  if (m_uploading) {
    m_uploadBtn->setText(tr("Cancel upload"));
    m_uploadBtn->setEnabled(true);
    m_uploadBtn->setToolTip(QString());
    return;
  }
  m_uploadBtn->setText(tr("Upload to vehicle"));
  const bool ok = m_uploader && m_uploadAvail && m_fit.ok;
  m_uploadBtn->setEnabled(ok);
  QString tip;
  if (!m_uploader)
    tip = tr("File upload is not available here.");
  else if (!m_uploadAvail)
    tip = m_uploadWhy.isEmpty() ? tr("No live link to the vehicle.") : m_uploadWhy;
  else if (!m_fit.ok)
    tip = tr("Capture both points first.");
  m_uploadBtn->setToolTip(tip);
}

bool BatteryCalibDialog::startUpload() {
  if (m_uploading || !m_uploader || !m_uploadAvail || !m_fit.ok)
    return false;
  const QByteArray b = blob();
  if (b.isEmpty())
    return false;
  m_uploading = true;
  m_uploadBar->setValue(0);
  m_uploadBar->setVisible(true);
  m_uploadLbl->setText(tr("Uploading %1 ...").arg(QString::fromLatin1(batcal::kCardPath)));
  if (!m_uploader->start(QString::fromLatin1(batcal::kCardPath), b)) {
    m_uploading = false;
    m_uploadBar->setVisible(false);
    m_uploadLbl->setText(tr("Upload not started: %1").arg(m_uploader->lastError()));
    updateUploadUi();
    return false;
  }
  updateUploadUi();
  return true;
}

void BatteryCalibDialog::onUploadProgress(quint32 acked, quint32 total) {
  if (!m_uploading)
    return;
  m_uploadBar->setValue(total ? int(100LL * acked / total) : 0);
}

void BatteryCalibDialog::onUploadFinished(XferUploader::Result r,
                                          const QString &msg) {
  if (!m_uploading)
    return; // someone else's upload
  m_uploading = false;
  m_uploadBar->setVisible(r == XferUploader::Result::Ok);
  if (r == XferUploader::Result::Ok) {
    m_uploadBar->setValue(100);
    m_uploadLbl->setText(
        tr("Uploaded %1 to the vehicle. Reboot the vehicle for the "
           "calibration to take effect.")
            .arg(QString::fromLatin1(batcal::kCardPath)));
  } else if (r == XferUploader::Result::Unconfirmed) {
    m_uploadLbl->setText(
        tr("Sent, but the vehicle did not confirm it wrote %1 (%2). The file "
           "may be on the card. Reboot the vehicle and check that the "
           "calibration applied, or upload again.")
            .arg(QString::fromLatin1(batcal::kCardPath), msg));
  } else if (r == XferUploader::Result::Cancelled) {
    m_uploadLbl->setText(
        tr("Upload cancelled. The vehicle's %1 may be missing or partial; "
           "upload again or copy the saved file to the card.")
            .arg(QString::fromLatin1(batcal::kCardPath)));
  } else {
    m_uploadLbl->setText(
        tr("Upload failed: %1. The vehicle's %2 may be incomplete (it is "
           "replaced when an upload opens); retry, or use Save and copy the "
           "file to the card.")
            .arg(msg, QString::fromLatin1(batcal::kCardPath)));
  }
  updateUploadUi();
}

void BatteryCalibDialog::done(int r) {
  if (m_uploading && m_uploader)
    m_uploader->cancel();
  QDialog::done(r);
}
