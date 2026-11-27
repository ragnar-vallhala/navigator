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
#include "GyroNotchWidget.h"

#include "../../protocol/CommandCodec.h"
#include "../core/Notify.h"
#include "ParamApplier.h"
#include "ParamClient.h"
#include "ParamRoute.h"
#include "ParamService.h"

#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>

extern "C" {
#include "navlink_msgs.h"
}

namespace {
// Compile-time firmware defaults (gyro_notch.c) so the panel opens on the values
// the FC actually boots with.
constexpr double kDefQ = 8.0;
constexpr double kDefFmin = 60.0;
constexpr double kDefFmax = 450.0;
constexpr double kDefMinRatio = 4.0;

QDoubleSpinBox *mkSpin(double lo, double hi, double step, double val,
                       const QString &suffix) {
  auto *s = new QDoubleSpinBox;
  s->setRange(lo, hi);
  s->setSingleStep(step);
  s->setValue(val);
  s->setDecimals(2);
  if (!suffix.isEmpty())
    s->setSuffix(suffix);
  return s;
}
} // namespace

GyroNotchWidget::GyroNotchWidget(QWidget *parent) : QWidget(parent) {
  auto *root = new QVBoxLayout(this);

  auto *title = new QLabel(tr("Dynamic Gyro Notch"));
  title->setStyleSheet("font-size:18px; font-weight:600;");
  root->addWidget(title);

  // ---- Tuning (CMD_SET_GYRO_NOTCH) ----------------------------------------
  auto *tune = new QGroupBox(tr("Detection"));
  auto *form = new QFormLayout(tune);

  m_enable = new QCheckBox(tr("Enabled"));
  form->addRow(m_enable);

  m_autoband = new QCheckBox(tr("Auto-band (learn on next hover)"));
  m_autoband->setToolTip(
      tr("One-shot: the FC characterises the hover spectrum and tightens the "
         "band around it. Cleared after it is sent."));
  form->addRow(m_autoband);

  m_q = mkSpin(0.5, 30.0, 0.5, kDefQ, QString());
  m_fmin = mkSpin(10.0, 490.0, 5.0, kDefFmin, tr(" Hz"));
  m_fmax = mkSpin(10.0, 490.0, 5.0, kDefFmax, tr(" Hz"));
  m_minRatio = mkSpin(1.0, 50.0, 0.5, kDefMinRatio, QString());
  form->addRow(tr("Q (width)"), m_q);
  form->addRow(tr("Band min"), m_fmin);
  form->addRow(tr("Band max"), m_fmax);
  form->addRow(tr("Min peak ratio"), m_minRatio);

  m_applyBtn = new QPushButton(tr("Apply to FC"));
  form->addRow(m_applyBtn);
  root->addWidget(tune);

  // ---- Live readout (NOTCH_STATUS) ----------------------------------------
  auto *live = new QGroupBox(tr("Tracked centers"));
  auto *grid = new QGridLayout(live);
  grid->addWidget(new QLabel(tr("Axis")), 0, 0);
  for (int slot = 0; slot < 3; ++slot)
    grid->addWidget(new QLabel(tr("Notch %1").arg(slot + 1)), 0, slot + 1);

  const char *axisNames[3] = {"Roll", "Pitch", "Yaw"};
  for (int axis = 0; axis < 3; ++axis) {
    grid->addWidget(new QLabel(tr(axisNames[axis])), axis + 1, 0);
    for (int slot = 0; slot < 3; ++slot) {
      m_center[axis][slot] = new QLabel(QStringLiteral("—"));
      m_center[axis][slot]->setStyleSheet("font-family:monospace;");
      grid->addWidget(m_center[axis][slot], axis + 1, slot + 1);
    }
  }
  root->addWidget(live);

  m_statusLabel = new QLabel(tr("NOT CONNECTED"));
  root->addWidget(m_statusLabel);

  root->addStretch(1);
  auto *back = new QPushButton(tr("Back"));
  root->addWidget(back);

  connect(back, &QPushButton::clicked, this,
          &GyroNotchWidget::backToHomeRequested);
  connect(m_applyBtn, &QPushButton::clicked, this,
          &GyroNotchWidget::onApplyClicked);
  // Any edit by the user keeps the vehicle's values from overwriting the boxes.
  const auto edited = [this] {
    if (!m_filling)
      m_dirty = true;
  };
  connect(m_enable, &QCheckBox::toggled, this, edited);
  for (QDoubleSpinBox *s : {m_q, m_fmin, m_fmax, m_minRatio})
    connect(s, qOverload<double>(&QDoubleSpinBox::valueChanged), this, edited);

  m_autobandWait.setSingleShot(true);
  m_autobandWait.setInterval(kAutobandAckMs);
  connect(&m_autobandWait, &QTimer::timeout, this, [this] {
    if (m_autobandSeq < 0)
      return;
    m_autobandSeq = -1;
    m_statusLabel->setText(tr("Auto-band: no answer"));
    Notify::warn(this, tr("The vehicle did not answer the auto-band command; it may or may not be armed"));
  });

  setConnected(false);
}

void GyroNotchWidget::setProtocol(DroneProtocol *protocol) {
  m_protocol = protocol;
  if (m_protocol) {
    connect(m_protocol, &DroneProtocol::notchStatusReceived, this,
            &GyroNotchWidget::onNotchStatus);
    connect(m_protocol, &DroneProtocol::commandAckReceived, this,
            &GyroNotchWidget::onCommandAck);
  }
}

void GyroNotchWidget::setParams(params::ParamService *svc) {
  m_params = svc;
  m_applier = new params::ParamApplier(svc, this);
  connect(svc, &params::ParamService::changed, this,
          &GyroNotchWidget::syncFromVehicle);
  connect(m_applier, &params::ParamApplier::finished, this,
          [this](const params::ParamApplier::Result &r) {
            setBusy(false);
            if (r.ok) {
              Notify::ok(this, tr("Notch settings applied"));
              m_dirty = false;
              syncFromVehicle();
              if (m_wantAutoband) { // the settings are in: now learn the band
                quint8 seq = 0;
                emit commandRequested(CommandCodec::encodeNotchAutoband(CommandCodec::targetSys(), &seq));
                m_autoband->setChecked(false);
                // Armed only when the vehicle says so.
                m_autobandSeq = seq;
                m_autobandWait.start();
                m_statusLabel->setText(tr("Arming auto-band…"));
              }
            } else if (r.unknown) {
              Notify::warn(this, tr("Lost the link while applying: the vehicle "
                                    "may or may not have taken the notch settings"));
            } else {
              Notify::error(this, tr("Notch settings not applied: %1").arg(r.text));
            }
            m_wantAutoband = false;
          });
  syncFromVehicle();
}

void GyroNotchWidget::onCommandAck(quint32 command, quint8 reqSeq, quint8 result) {
  if (m_autobandSeq < 0 || command != NAVLINK_MSGID_CMD_NOTCH_AUTOBAND || reqSeq != m_autobandSeq)
    return;
  if (result == params::Client::kInProgress) { // alive: keep waiting
    m_autobandWait.start();
    return;
  }
  m_autobandSeq = -1;
  m_autobandWait.stop();
  if (result == params::Client::kAccepted) {
    m_statusLabel->setText(tr("Auto-band armed"));
    Notify::ok(this, tr("Auto-band armed: it learns the band on the next hover"));
  } else {
    m_statusLabel->setText(tr("Auto-band not armed"));
    Notify::warn(this, tr("The vehicle did not arm auto-band (result %1)").arg(result));
  }
}

void GyroNotchWidget::setBusy(bool busy) {
  m_busy = busy;
  // An edit while the apply runs is refused: the boxes are locked with it.
  for (QWidget *w : {static_cast<QWidget *>(m_enable), static_cast<QWidget *>(m_autoband),
                     static_cast<QWidget *>(m_q), static_cast<QWidget *>(m_fmin),
                     static_cast<QWidget *>(m_fmax), static_cast<QWidget *>(m_minRatio)})
    w->setEnabled(!busy);
  updateApplyEnabled();
}

void GyroNotchWidget::updateApplyEnabled() {
  m_applyBtn->setEnabled(m_connected && !m_busy);
}

// With the table ready the boxes show the vehicle's values and take their
// ranges from the catalog; an edit of the user's is not overwritten.
void GyroNotchWidget::syncFromVehicle() {
  if (!m_params || m_busy ||
      params::routeFor(m_params->client().state()) != params::Route::Params)
    return;
  const auto &cat = m_params->catalog();
  const auto &cl = m_params->client();
  const auto load = [&](const QString &id, QDoubleSpinBox *box) {
    const params::Entry *e = cat.byId(id);
    if (!e)
      return;
    m_filling = true;
    box->setRange(e->min, e->max);
    if (!m_dirty)
      if (const auto *v = cl.value(id); v && v->have)
        box->setValue(v->v);
    m_filling = false;
  };
  load(QStringLiteral("NOTCH_Q"), m_q);
  load(QStringLiteral("NOTCH_FMIN"), m_fmin);
  load(QStringLiteral("NOTCH_FMAX"), m_fmax);
  load(QStringLiteral("NOTCH_MINRATIO"), m_minRatio);
  if (!m_dirty)
    if (const auto *v = cl.value(QStringLiteral("NOTCH_EN")); v && v->have) {
      m_filling = true;
      m_enable->setChecked(v->v != 0.0);
      m_filling = false;
    }
}

void GyroNotchWidget::applyAsParams() {
  const double fmin = m_fmin->value(), fmax = m_fmax->value();
  if (fmin >= fmax) {
    Notify::warn(this, tr("Notch not applied: the band minimum must be below the band maximum"));
    return;
  }
  const QVector<params::ParamApplier::Item> items = {
      {QStringLiteral("NOTCH_EN"), m_enable->isChecked() ? 1.0 : 0.0},
      {QStringLiteral("NOTCH_Q"), m_q->value()},
      {QStringLiteral("NOTCH_FMIN"), fmin},
      {QStringLiteral("NOTCH_FMAX"), fmax},
      {QStringLiteral("NOTCH_MINRATIO"), m_minRatio->value()},
  };
  QString why;
  if (!m_applier->apply(items, &why)) {
    Notify::warn(this, tr("Notch not applied: %1").arg(why));
    return;
  }
  m_wantAutoband = m_autoband->isChecked();
  setBusy(true);
}

void GyroNotchWidget::setConnected(bool connected) {
  m_connected = connected;
  updateApplyEnabled();
  if (!connected)
    m_statusLabel->setText(tr("NOT CONNECTED"));
}

void GyroNotchWidget::onApplyClicked() {
  if (m_params) {
    switch (params::routeFor(m_params->client().state())) {
    case params::Route::Params:
      applyAsParams();
      return;
    case params::Route::NotReady:
      Notify::warn(this, tr("The vehicle's parameters are not loaded yet — try again in a moment"));
      return;
    case params::Route::Legacy:
      break; // older firmware: the old command below
    }
  }
  // The FC treats a non-positive detection field as "leave unchanged"; the panel
  // always sends real positive values, so a full tune is applied every time.
  emit commandRequested(CommandCodec::encodeSetGyroNotch(
      m_enable->isChecked(), static_cast<float>(m_q->value()),
      static_cast<float>(m_fmin->value()), static_cast<float>(m_fmax->value()),
      static_cast<float>(m_minRatio->value()), m_autoband->isChecked()));
  // Auto-band is a one-shot trigger; untick so it isn't re-armed on the next Apply.
  m_autoband->setChecked(false);
}

void GyroNotchWidget::onNotchStatus(const NotchStatusData &d) {
  m_statusLabel->setText(d.enabled ? tr("Notch ENABLED")
                                   : tr("Notch disabled"));
  for (int axis = 0; axis < 3; ++axis) {
    for (int slot = 0; slot < 3; ++slot) {
      float hz = d.centerHz[axis][slot];
      m_center[axis][slot]->setText(
          hz > 0.0f
              ? QStringLiteral("%1 Hz").arg(static_cast<double>(hz), 0, 'f', 1)
              : QStringLiteral("—"));
    }
  }
}
