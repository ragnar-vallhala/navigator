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

// Shared by SimulatorWidget and the simulator panels (SimViewPanel,
// SimVehiclePanel, SimWorldPanel, SimControlsPanel, SimAutotunePanel): the
// settings keys, the firmware motor-geometry push and two small widgets. Not a
// public header; include it only from those .cpp files.

#include "SimulatorWidget.h"

#include "core/Units.h"

#include "../../vsim/AirframePresets.h"
#include "../../vsim/MeshLoader.h"
#include "../../vsim/SimPaths.h"
#include "../../vsim/SitlModule.h"
#include "protocol/CommandCodec.h"
#include "navlink_msgs.h" // NAVLINK_MSGID_*: the ack's `command`
#include "../../vsim/ProceduralWorld.h"
#include "../../vsim/WorldMeshBuilder.h"
#include "CollapsibleSection.h"
#include "comm/PortArbiter.h"
#include "core/Theme.h"
#include "core/ui/Buttons.h"

#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QDesktopServices>
#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QFutureWatcher>
#include <QTimer>
#include <QtConcurrent>
#include <QEvent>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QSlider>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QScrollArea>
#include <QScrollBar>
#include <QSerialPortInfo>
#include <QLineEdit>
#include <QSettings>
#include <QSplitter>
#include <QStackedWidget>
#include <QUrl>
#include <QByteArray>
#include <QDataStream>
#include <QFile>
#include <QVBoxLayout>
#include <QProcess>
#include <QRandomGenerator>
#include <QSpinBox>
#include <QPlainTextEdit>
#include <QLabel>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QThread>
#include <QTableWidget>
#include <QHeaderView>
#include <QPainter>
#include <QPainterPath>
#include <QFrame>
#include <QMouseEvent>

#include "AutotuneWorker.h"
#include "Space.h"

#include <cmath>
#include <cstdio> // ::rename (atomic blob publish)

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h> // MoveFileExW (atomic blob publish)
#endif
#include <unistd.h>

// Live step/chirp response plot: the roll-axis setpoint (dashed) vs measured
// (solid) angle from the latest autotune excitation window. No Q_OBJECT — it's
// driven by setData() from a lambda, not signals/slots.
class ResponsePlot : public QWidget {
public:
  explicit ResponsePlot(QWidget *parent = nullptr) : QWidget(parent) {
    setMinimumHeight(110);
  }
  void setData(const QVector<double> &sp, const QVector<double> &meas) {
    sp_ = sp;
    meas_ = meas;
    update();
  }

protected:
  void paintEvent(QPaintEvent *) override {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    const QRectF r = rect().adjusted(2, 2, -2, -2);
    p.fillRect(r, QColor(0x1A, 0x1D, 0x27));
    p.setPen(QPen(QColor(0x3E, 0x44, 0x52), 1));
    p.drawRect(r);
    if (sp_.isEmpty() && meas_.isEmpty()) {
      QFont f = p.font();
      f.setBold(true);
      f.setPixelSize(std::max(18, int(r.height() / 2)));
      p.setFont(f);
      p.setPen(QColor(0xE8, 0xF0, 0xFE, 40)); // faded watermark
      p.drawText(r, Qt::AlignCenter, QStringLiteral("NA"));
      return;
    }
    // Symmetric vertical scale around 0, fit to the data (deg).
    double mx = 1.0;
    for (double v : sp_)
      mx = std::max(mx, std::abs(v));
    for (double v : meas_)
      mx = std::max(mx, std::abs(v));
    mx *= 1.1;
    const int n = std::max(sp_.size(), meas_.size());
    auto toPt = [&](int i, double v) {
      const double x =
          r.left() + r.width() * (n > 1 ? double(i) / (n - 1) : 0.5);
      const double y = r.center().y() - (v / mx) * (r.height() / 2 - 4);
      return QPointF(x, y);
    };
    // Zero line.
    p.setPen(QPen(QColor(255, 255, 255, 24), 1));
    p.drawLine(QPointF(r.left(), r.center().y()),
               QPointF(r.right(), r.center().y()));
    auto poly = [&](const QVector<double> &d, QColor c, Qt::PenStyle st) {
      if (d.size() < 2)
        return;
      QPainterPath path;
      path.moveTo(toPt(0, d[0]));
      for (int i = 1; i < d.size(); ++i)
        path.lineTo(toPt(i, d[i]));
      p.setPen(QPen(c, 1.5, st));
      p.drawPath(path);
    };
    poly(sp_, QColor(0x8A, 0x92, 0xA6), Qt::DashLine);    // setpoint
    poly(meas_, QColor(0xE0, 0x6C, 0x75), Qt::SolidLine); // measured (roll)
  }

private:
  QVector<double> sp_, meas_;
};

// Titled pane in the instrument rail beside the 3D view (mockup .pip). These
// used to float, draggable and resizable, over the 3D view; the view is now a
// native QOpenGLWindow in a window container, which is always above sibling
// widgets, so nothing can be stacked on it. Hosts any content widget.
class PipPane : public QFrame {
public:
  PipPane(const QString &title, QWidget *content, int height, QWidget *parent)
      : QFrame(parent) {
    setObjectName("PipPane");
    setStyleSheet("#PipPane{background:#11141b;border:1px solid #3E4452;}");
    auto *v = new QVBoxLayout(this);
    v->setContentsMargins(1, 1, 1, 1);
    v->setSpacing(0);
    auto *t = new QLabel(title, this);
    t->setStyleSheet(
        "background:#21252B; color:#8A92A6; font-size:9px; "
        "font-weight:bold; letter-spacing:.5px; padding:2px 5px;");
    t->setFixedHeight(16);
    v->addWidget(t);
    content->setParent(this);
    v->addWidget(content, 1);
    setFixedHeight(height);
  }
};

namespace simui {
inline constexpr int kFmAngle = 0, kFmAcro = 1, kFmSrcGcs = 1;

// Push the vehicle's motor layout into the firmware mixer so the control mix
// matches the physics (stable for any quad layout, not just the firmware's
// default numbering).
//
// Sent as CMD_SET_MOTOR_GEOMETRY, the same frame a real GCS sends. That means
// the firmware's command gate applies: until the time-sync handshake completes
// this is TEMPORARILY_REJECTED and the mix is NOT changed. Poking the mixer
// function directly used to hide exactly that, leaving the firmware on its
// default mix while the UI reported success.
inline void pushFirmwareMotorGeometry(const vsim::GeometryConfig &g) {
  float x[4], y[4], spin[4];
  for (int i = 0; i < 4; ++i) {
    x[i] = g.motors[i].pos.x();
    y[i] = g.motors[i].pos.y();
    spin[i] = static_cast<float>(g.motors[i].spin);
  }
  SitlModule::instance().send(CommandCodec::encodeSetMotorGeometry(x, y, spin));
}

inline constexpr const char *kRepoRootSettingKey = "simulator/repoRoot";
inline constexpr const char *kLogDirSettingKey = "simulator/logDir";
inline constexpr const char *kGeomGroup = "simulator/geometry";
inline constexpr const char *kWorldGroup = "simulator/world";
inline constexpr const char *kAudioKey = "simulator/propAudio";
inline constexpr const char *kSeedKey = "simulator/resetSeed";         // fixed seed
inline constexpr const char *kSeedRandomKey = "simulator/resetSeedRandom"; // per Start
inline constexpr int kDefaultResetSeed = 1337;
inline constexpr const char *kImuHzKey = "simulator/imuHz";
inline constexpr const char *kPhysHzKey = "simulator/physicsHz";
inline constexpr const char *kPoseHzKey = "simulator/poseHz";
// Defaults: IMU/firmware loop at 1 kHz (matches real hardware so PID tuning
// transfers); physics at 8 kHz (8 RK4 substeps/sample — finer integration, same
// sample rate); 60 Hz render.
inline constexpr int kDefImuHz = 1000, kDefPhysHz = 8000, kDefPoseHz = 60;
inline constexpr const char *kRcEnableKey = "simulator/rcEnable";
inline constexpr const char *kRcSourceKey = "simulator/rcSource";     // 0=js 1=uart
inline constexpr const char *kRcPathKey = "simulator/rcPath";         // joystick dev
inline constexpr const char *kRcUartPathKey = "simulator/rcUartPath"; // serial dev
// The device each RC source starts on before one is saved: a WinMM joystick
// number and a COM port on Windows (see RcBridge).
#ifdef _WIN32
inline constexpr const char *kRcPathDefault = "0";
inline constexpr const char *kRcUartPathDefault = "COM3";
#else
inline constexpr const char *kRcPathDefault = "/dev/input/js0";
inline constexpr const char *kRcUartPathDefault = "/dev/ttyUSB0";
#endif
inline constexpr const char *kRcCompactKey = "simulator/compactRcRow";
inline constexpr const char *kRcBaudKey = "simulator/rcBaud";
inline constexpr const char *kRcMapAxisKey = "simulator/rcMapAxis"; // + func
inline constexpr const char *kRcMapInvKey = "simulator/rcMapInv";   // + func

// Default joystick axis for each function (roll,pitch,throttle,yaw,arm).
inline constexpr int kRcDefaultAxis[5] = {0, 1, 2, 3, 4};
inline constexpr const char *kRcFuncName[5] = {"Roll", "Pitch", "Throttle", "Yaw",
                                        "Arm"};

} // namespace simui

using namespace simui;
