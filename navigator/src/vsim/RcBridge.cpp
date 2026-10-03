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
#include "RcBridge.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <mmsystem.h> // joyGetPosEx
#else
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <algorithm>
#include <string>

namespace {

// ---- the input device: the only I/O that differs by platform -------------
// io_t is an open UART (or, on Linux, a js device); readSome never blocks and
// returns bytes read, 0 for none right now, -1 for an error.
#ifdef _WIN32
using io_t = HANDLE;
const io_t kNoIo = INVALID_HANDLE_VALUE;

// A COM port in raw 8N1 at `baud`, reads that return at once (MAXDWORD read
// interval: the O_NONBLOCK + VMIN=VTIME=0 of termios). COM10 and up only open
// through the \\.\ device namespace, which COM1-9 accept too.
io_t openUart(const char *path, int baud) {
  const std::string dev = std::strncmp(path, "\\\\.\\", 4) == 0
                              ? std::string(path)
                              : std::string("\\\\.\\") + path;
  HANDLE h = CreateFileA(dev.c_str(), GENERIC_READ, 0, nullptr, OPEN_EXISTING,
                         0, nullptr);
  if (h == INVALID_HANDLE_VALUE)
    return kNoIo;
  DCB dcb{};
  dcb.DCBlength = sizeof dcb;
  dcb.BaudRate = (DWORD)baud;
  dcb.fBinary = TRUE;
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;
  dcb.fDtrControl = DTR_CONTROL_ENABLE;
  dcb.fRtsControl = RTS_CONTROL_ENABLE;
  COMMTIMEOUTS to = {MAXDWORD, 0, 0, 0, 0};
  if (!SetCommState(h, &dcb) || !SetCommTimeouts(h, &to)) {
    CloseHandle(h);
    return kNoIo;
  }
  return h;
}

long readSome(io_t h, char *buf, size_t n) {
  DWORD got = 0;
  return ReadFile(h, buf, (DWORD)n, &got, nullptr) ? (long)got : -1;
}

void closeIo(io_t h) { CloseHandle(h); }

QString ioError() { return QString("Win32 error %1").arg(GetLastError()); }
#else
using io_t = int;
const io_t kNoIo = -1;

long readSome(io_t fd, char *buf, size_t n) {
  const ssize_t r = ::read(fd, buf, n);
  if (r < 0)
    return errno == EAGAIN || errno == EWOULDBLOCK ? 0 : -1;
  return (long)r;
}

void closeIo(io_t fd) { ::close(fd); }

QString ioError() { return QString::fromLocal8Bit(::strerror(errno)); }
#endif

#ifndef _WIN32
// Linux joystick API event (linux/joystick.h), redeclared to avoid a
// kernel-header dependency.
struct js_event {
  uint32_t time;  // ms
  int16_t value;  // -32767..32767 for axes
  uint8_t type;   // 0x01 button, 0x02 axis, |0x80 = synthetic init
  uint8_t number; // axis/button index
};
constexpr uint8_t kJsButton = 0x01;
constexpr uint8_t kJsAxis = 0x02;

#endif

int axisToUs(int v, bool invert) {
  if (invert)
    v = -v;
  const int us = 1500 + (v * 500) / 32767;
  return std::clamp(us, 1000, 2000);
}

#ifndef _WIN32
speed_t baudConst(int b) {
  switch (b) {
  case 9600:
    return B9600;
  case 19200:
    return B19200;
  case 38400:
    return B38400;
  case 57600:
    return B57600;
  case 115200:
    return B115200;
  case 230400:
    return B230400;
  case 460800:
    return B460800;
  case 921600:
    return B921600;
  default:
    return B115200;
  }
}

// Open a serial port in raw mode at `baud` for reading CSV RC frames.
int openUart(const char *path, int baud) {
  int fd = ::open(path, O_RDONLY | O_NOCTTY | O_NONBLOCK);
  if (fd < 0)
    return -1;
  struct termios tio;
  if (::tcgetattr(fd, &tio) == 0) {
    ::cfmakeraw(&tio);
    const speed_t sp = baudConst(baud);
    ::cfsetispeed(&tio, sp);
    ::cfsetospeed(&tio, sp);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    ::tcsetattr(fd, TCSANOW, &tio);
  }
  return fd;
}
#endif

// Parse a CSV line of microsecond channel values. Assigns the first up-to-6
// fields to roll/pitch/throttle/yaw/arm/ch6 (clamped 1000..2000); leaves any
// missing trailing channels untouched so a short frame holds the last value.
// ch6 is the acro/angle flight-mode switch (firmware: > 1500 = acro).
void parseCsvFrame(const std::string &line, int *roll, int *pitch, int *thr,
                   int *yaw, int *arm, int *ch6) {
  int *out[6] = {roll, pitch, thr, yaw, arm, ch6};
  const char *p = line.c_str();
  for (int i = 0; i < 6 && *p; ++i) {
    char *end = nullptr;
    const long v = std::strtol(p, &end, 10);
    if (end == p)
      break; // no number here
    *out[i] = std::clamp((int)v, 1000, 2000);
    p = end;
    while (*p == ',' || *p == ' ' || *p == '\t')
      ++p;
  }
}

} // namespace

RcBridge::RcBridge(QObject *parent) : QThread(parent) {
  // Default 1:1 mapping: func f ← axis f, no invert.
  for (int f = 0; f < 5; ++f) {
    mapAxis_[f] = f;
    mapInv_[f] = 0;
  }
}

void RcBridge::setMapping(int func, int axis, bool invert) {
  if (func < 0 || func >= 5)
    return;
  mapAxis_[func].store(axis, std::memory_order_relaxed);
  mapInv_[func].store(invert ? 1 : 0, std::memory_order_relaxed);
}

RcBridge::~RcBridge() {
  requestStop();
  // Must be fully stopped before ~QThread, else Qt qFatal()s with "QThread:
  // Destroyed while thread is still running" (the SIGABRT seen on app exit if
  // the run loop is mid-blocking-read). Force it down if it won't stop, same as
  // the sim/autotune threads do.
  if (isRunning() && !wait(500)) {
    terminate();
    wait(1000);
  }
#ifdef _WIN32
  if (pipe_)
    CloseHandle(pipe_);
#else
  if (master_fd_ >= 0)
    ::close(master_fd_);
#endif
}

#ifdef _WIN32
// A named pipe stands in for the pty. Outbound byte pipe, one instance, in
// PIPE_NOWAIT mode so that neither waiting for the firmware to connect nor a
// full buffer can stall the 50 Hz loop -- the pty's O_NONBLOCK. The firmware's
// feeder opens it by path, exactly as it opens a tty.
bool RcBridge::openPty(QString *err) {
  slave_path_ = QStringLiteral("\\\\.\\pipe\\vayu_rc_%1").arg(GetCurrentProcessId());
  HANDLE h = CreateNamedPipeW(
      reinterpret_cast<LPCWSTR>(slave_path_.utf16()), PIPE_ACCESS_OUTBOUND,
      PIPE_TYPE_BYTE | PIPE_NOWAIT, 1, 4096, 4096, 0, nullptr);
  if (h == INVALID_HANDLE_VALUE) {
    if (err)
      *err = QString("RC pipe setup failed: %1").arg(ioError());
    slave_path_.clear();
    return false;
  }
  pipe_ = h;
  return true;
}

// Accept the firmware's connection if it has (re)opened the pipe, then write.
// When it closes its end (the feeder reopens after an error), the pipe is
// disconnected so the next open can connect -- otherwise the one instance
// would stay taken by a reader that is gone.
void RcBridge::writeOut(const char *buf, int n) {
  if (!pipe_)
    return;
  if (!ConnectNamedPipe(pipe_, nullptr)) {
    const DWORD e = GetLastError();
    if (e == ERROR_NO_DATA) { // the reader went away
      DisconnectNamedPipe(pipe_);
      return;
    }
    if (e != ERROR_PIPE_CONNECTED)
      return; // ERROR_PIPE_LISTENING: no reader yet, drop the frame
  }
  DWORD w = 0;
  if (!WriteFile(pipe_, buf, (DWORD)n, &w, nullptr) &&
      GetLastError() == ERROR_NO_DATA)
    DisconnectNamedPipe(pipe_);
  // A full pipe writes nothing: the frame is dropped, the next is 20 ms away.
}
#else
bool RcBridge::openPty(QString *err) {
  master_fd_ = ::posix_openpt(O_RDWR | O_NOCTTY);
  if (master_fd_ < 0 || ::grantpt(master_fd_) != 0 ||
      ::unlockpt(master_fd_) != 0) {
    if (err)
      *err = QString("pty setup failed: %1").arg(::strerror(errno));
    if (master_fd_ >= 0) {
      ::close(master_fd_);
      master_fd_ = -1;
    }
    return false;
  }
  const char *sn = ::ptsname(master_fd_);
  if (!sn) {
    if (err)
      *err = QStringLiteral("ptsname failed");
    ::close(master_fd_);
    master_fd_ = -1;
    return false;
  }
  slave_path_ = QString::fromLocal8Bit(sn);
  return true;
}
#endif

void RcBridge::requestStop() { stop_.store(true, std::memory_order_release); }

void RcBridge::run() {
  stop_.store(false, std::memory_order_release);

  // Throttle idles low; everything else centered until the device reports.
  axis_.fill(0);
  axis_[2] = -32767;
  button_.fill(0);

  // Effective source actually open: Joystick, Uart, or kSrcNone (no device →
  // neutral frames). The Uart source resolves to kSrcNone until the UI calls
  // setUartConnected(true), so the sim never opens the board's tty on its own.
  constexpr int kSrcNone = -2;
  int cur_src = -99; // last effective source opened (force first (re)open)
  io_t fd = kNoIo;   // uart (or Linux js) device; the pty/pipe is separate
#ifdef _WIN32
  int js_index = -1; // WinMM joystick number for the Joystick source
#endif
  std::string uline; // UART CSV line accumulator
  // Last UART-parsed channels, held between frames.
  int u_roll = 1500, u_pitch = 1500, u_thr = 1000, u_yaw = 1500, u_arm = 1500;
  int u_ch6 = 1000; // transmitter's ch6 (acro switch); 1000 = stabilise default

  auto close_src = [&]() {
    if (fd != kNoIo) {
      closeIo(fd);
      fd = kNoIo;
    }
#ifdef _WIN32
    js_index = -1;
#endif
  };

  while (!stop_.load(std::memory_order_acquire)) {
    const int sel = source_.load(std::memory_order_acquire);
    // Gate the Uart source behind the explicit connect flag.
    const int want =
        (sel == Uart && !uartConnected_.load(std::memory_order_acquire))
            ? kSrcNone
            : sel;

    // (Re)open the input device when the effective source changes. master_fd_
    // (the pty the firmware reads) stays put, so switching source is seamless.
    if (want != cur_src) {
      close_src();
      uline.clear();
      QString path;
      int baud;
      {
        QMutexLocker lk(&cfg_mtx_);
        path = (want == Uart) ? uart_path_ : js_path_;
        baud = uart_baud_;
      }
      if (want == kSrcNone) {
        // No device claimed — stream neutral frames, hold the port free.
        emit logLine(QStringLiteral("RC: UART disconnected — neutral frames"));
        axis_.fill(0);
        axis_[2] = -32767;
        button_.fill(0);
      } else if (want == Uart) {
        fd = openUart(path.toLocal8Bit().constData(), baud);
        if (fd == kNoIo)
          emit logLine(QString("RC: cannot open UART %1 (%2) — neutral frames")
                           .arg(path, ioError()));
        else
          emit logLine(QString("RC: UART %1 @ %2 → %3")
                           .arg(path)
                           .arg(baud)
                           .arg(slave_path_));
      } else {
#ifdef _WIN32
        // A joystick number, polled with WinMM; "open" = it answers a poll.
        bool ok = false;
        const int idx = path.toInt(&ok);
        JOYINFOEX ji{};
        ji.dwSize = sizeof ji;
        ji.dwFlags = JOY_RETURNALL;
        if (ok && joyGetPosEx((UINT)idx, &ji) == JOYERR_NOERROR) {
          js_index = idx;
          emit logLine(QString("RC: joystick %1 → %2").arg(path, slave_path_));
        } else {
          emit logLine(QString("RC: no joystick %1 — neutral frames").arg(path));
        }
#else
        fd = ::open(path.toLocal8Bit().constData(), O_RDONLY | O_NONBLOCK);
        if (fd < 0)
          emit logLine(QString("RC: cannot open %1 (%2) — neutral frames")
                           .arg(path, ::strerror(errno)));
        else
          emit logLine(QString("RC: reading %1 → %2").arg(path, slave_path_));
#endif
        axis_.fill(0);
        axis_[2] = -32767;
        button_.fill(0);
      }
      cur_src = want;
    }

    const bool en = enabled_.load(std::memory_order_acquire);
    int roll, pitch, thr, yaw, arm;

    if (want == Uart) {
      // Drain available bytes, split on newlines, parse the latest frame.
      if (fd != kNoIo) {
        char b[256];
        long r;
        while ((r = readSome(fd, b, sizeof(b))) > 0) {
          for (long i = 0; i < r; ++i) {
            const char c = b[i];
            if (c == '\n' || c == '\r') {
              if (!uline.empty()) {
                parseCsvFrame(uline, &u_roll, &u_pitch, &u_thr, &u_yaw, &u_arm,
                              &u_ch6);
                uline.clear();
              }
            } else if (uline.size() < 128) {
              uline += c;
            }
          }
        }
      }
      roll = en ? u_roll : 1500;
      pitch = en ? u_pitch : 1500;
      thr = en ? u_thr : 1000;
      yaw = en ? u_yaw : 1500;
      arm = en ? u_arm : 1500;
      emit axesUpdated({}, {}); // no per-axis breakdown for a CSV source
    } else {
      // Joystick: pump events, then map axes/buttons to channels.
#ifdef _WIN32
      // WinMM reports state, not events: poll it. Axes X Y Z R U V come as
      // 0..65535 and are centred onto the js range (-32767..32767).
      if (js_index >= 0) {
        JOYINFOEX ji{};
        ji.dwSize = sizeof ji;
        ji.dwFlags = JOY_RETURNALL;
        if (joyGetPosEx((UINT)js_index, &ji) == JOYERR_NOERROR) {
          const DWORD pos[6] = {ji.dwXpos, ji.dwYpos, ji.dwZpos,
                                ji.dwRpos, ji.dwUpos, ji.dwVpos};
          for (int i = 0; i < 6; ++i)
            axis_[i] = std::clamp((int)pos[i] - 32767, -32767, 32767);
          for (int i = 0; i < (int)button_.size(); ++i)
            button_[i] = (ji.dwButtons >> i) & 1u;
        }
      }
#else
      if (fd >= 0) {
        js_event e;
        ssize_t r;
        while ((r = ::read(fd, &e, sizeof(e))) == (ssize_t)sizeof(e)) {
          if ((e.type & kJsAxis) && e.number < axis_.size())
            axis_[e.number] = e.value;
          else if ((e.type & kJsButton) && e.number < button_.size())
            button_[e.number] = e.value;
        }
      }
#endif
      auto chan = [&](int f) {
        const int src = mapAxis_[f].load(std::memory_order_relaxed);
        const bool inv = mapInv_[f].load(std::memory_order_relaxed) != 0;
        if (src >= kButtonBase) { // button source: pressed → 2000
          const int b = src - kButtonBase;
          bool on =
              (b >= 0 && b < (int)button_.size()) ? button_[b] != 0 : false;
          if (inv)
            on = !on;
          return on ? 2000 : 1000;
        }
        const int raw = (src >= 0 && src < (int)axis_.size()) ? axis_[src] : 0;
        return axisToUs(raw, inv);
      };
      roll = en ? chan(0) : 1500;
      pitch = en ? chan(1) : 1500;
      thr = en ? chan(2) : 1000; // idle when disabled
      yaw = en ? chan(3) : 1500;
      arm = en ? chan(4) : 1500;

      QVector<int> au;
      au.reserve(6);
      for (int i = 0; i < 6; ++i)
        au.push_back(axisToUs(axis_[i], false));
      QVector<int> bu;
      bu.reserve(4);
      for (int i = 0; i < 4; ++i)
        bu.push_back(button_[i] ? 1 : 0);
      emit axesUpdated(au, bu);
    }

    const int ov = armOverride_.load(std::memory_order_relaxed);
    if (ov >= 0)
      arm = ov ? 2000 : 1000; // software arm override

    // CSV frame: roll,pitch,throttle,yaw,arm,ch6  (feeder fills 7..13 = 1500).
    // ch6 carries the acro/angle flight-mode toggle (firmware: > 1500 = acro).
    // In UART mode the transmitter's physical ch6 switch is authoritative (just
    // like real RC); the Acro checkbox drives it only for the generated source.
    const int ch6 = (want == Uart)
                        ? (en ? u_ch6 : 1000)
                        : (acro_.load(std::memory_order_relaxed) ? 2000 : 1000);
    char buf[96];
    const int n = std::snprintf(buf, sizeof(buf), "%d,%d,%d,%d,%d,%d\n", roll,
                                pitch, thr, yaw, arm, ch6);
#ifdef _WIN32
    if (n > 0)
      writeOut(buf, n);
#else
    if (master_fd_ >= 0 && n > 0) {
      const ssize_t w = ::write(master_fd_, buf, (size_t)n);
      (void)w; // pty buffer full → drop frame; next one is along in 20 ms
    }
#endif
    emit channelsUpdated(roll, pitch, thr, yaw, arm, ch6);

    msleep(20); // ~50 Hz
  }

  close_src();
  emit logLine(QStringLiteral("RC: stopped"));
}
