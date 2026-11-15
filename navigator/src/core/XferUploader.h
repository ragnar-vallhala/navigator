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

#include <QByteArray>
#include <QElapsedTimer>
#include <QObject>
#include <QString>

class QTimer;

// Uploads one byte blob to a named path on the vehicle over the NavLink XFER
// substrate (XFER_OPEN / XFER_INFO / XFER_DATA / XFER_ACK / XFER_CLOSE). It has
// no widget and no codec dependency: frames leave through IXferLink and the
// vehicle's replies are fed in through onXferInfo / onXferAck / onCommandAck,
// so a test drives it with a fake peer and the GCS wires it to the real link.
//
// Protocol (follows the firmware, firmware/src/comm/xfer/navlink_xfer.c):
//  * XFER_OPEN(dir=upload, mode=file, service=0, arg=path) is re-sent with the
//    same req_seq every openRetryMs until the vehicle answers. The firmware
//    answers with a COMMAND_ACK (a refusal: DENIED / UNSUPPORTED / FAILED, or
//    TEMPORARILY_REJECTED when busy or when its clock is not yet synced) and,
//    on success, XFER_INFO{chunk_size}. A periodic XFER_ACK from the vehicle
//    also proves the session is open (INFO is only re-sent while nothing else
//    has been heard), so it ends the open phase too.
//  * XFER_DATA chunks are offset-addressed and contiguous-only on the vehicle:
//    a duplicate or a gap is dropped and the next XFER_ACK{next_offset} says
//    where to resume. The last chunk carries EOF.
//  * XFER_ACK{next_offset} (every ~200 ms from the vehicle) is cumulative. It is
//    behind the cursor is normal while chunks are in flight, so the cursor is
//    rewound only on an explicit NAK or when no progress is seen for stallMs.
//  * The vehicle ends the session with XFER_ACK{flags=DONE, result}. result is
//    ACCEPTED only once the bytes are on the card; anything else is a failure.
//  * Cancel: the firmware honours ABORT only as a flag on XFER_DATA for an
//    upload (it ignores XFER_ACK for uploads), so cancel sends that, then
//    XFER_CLOSE.
namespace xfer {

constexpr quint8 kFlagNone = 0;
constexpr quint8 kFlagDone = 1;
constexpr quint8 kFlagNak = 2;
constexpr quint8 kFlagAbort = 4;
constexpr quint8 kFlagEof = 8;

// COMMAND_ACK / XFER_INFO / XFER_ACK result values (navlink command_result).
constexpr quint8 kResAccepted = 0;
constexpr quint8 kResTempRejected = 1;
constexpr quint8 kResDenied = 2;
constexpr quint8 kResUnsupported = 3;
constexpr quint8 kResFailed = 4;
constexpr quint8 kResInProgress = 5;

constexpr quint32 kMsgidXferOpen = 8201;
constexpr quint32 kMsgidXferClose = 8202;
constexpr int kChunkMax = 247; // XFER_DATA payload
constexpr int kPathMax = 31;   // XFER_OPEN.arg is char[32], NUL included

} // namespace xfer

// The send half of the link. Return false when the frame could not be sent
// (link down, tx not allowed); the upload then fails at once.
class IXferLink {
public:
  virtual ~IXferLink() = default;
  virtual bool sendOpen(quint8 reqSeq, quint8 session,
                        const QByteArray &path) = 0;
  virtual bool sendData(quint8 session, quint8 flags, quint32 offset,
                        const QByteArray &payload) = 0;
  virtual bool sendClose(quint8 reqSeq, quint8 session, quint8 result) = 0;
};

class XferUploader : public QObject {
  Q_OBJECT
public:
  enum class Result {
    Ok,        // vehicle reported the file written
    Refused,   // vehicle refused the open (denied / unsupported / failed)
    Failed,    // vehicle aborted, or reported a failed write, or link refused
    Timeout,   // nothing heard in time
    Cancelled  // cancel() while running
  };
  Q_ENUM(Result)

  struct Config {
    quint8 session = 0;
    int openRetryMs = 500;      // XFER_OPEN re-send period
    int openTimeoutMs = 8000;   // give up if never opened
    int stallMs = 400;          // no progress this long: rewind to the acked byte
    int idleTimeoutMs = 5000;   // no progress this long, after open: give up
                                // (the firmware reaps an idle session at 5 s)
    int windowBytes = 4096;     // max unacked bytes in flight
    int maxRateBps = 38000;     // byte-rate cap (460800-baud link); 0 = none
    int maxChunk = xfer::kChunkMax;
    int tickMs = 5;
  };

  explicit XferUploader(IXferLink *link, QObject *parent = nullptr);
  XferUploader(IXferLink *link, const Config &cfg, QObject *parent = nullptr);
  ~XferUploader() override;

  // Begin uploading. False (with lastError set, and no finished() signal) when
  // busy or the input is unusable; otherwise finished() always follows.
  bool start(const QString &path, const QByteArray &blob);
  bool busy() const { return m_state != State::Idle; }
  QString lastError() const { return m_lastError; }

  const Config &config() const { return m_cfg; }

public slots:
  // Abort a running upload: tells the vehicle and emits finished(Cancelled).
  // No-op when idle.
  void cancel();

  // Vehicle -> GCS. Feed every decoded message; foreign sessions are ignored.
  void onXferInfo(quint8 session, quint8 result, quint16 chunkSize,
                  quint32 totalSize);
  void onXferAck(quint8 session, quint8 flags, quint8 result,
                 quint32 nextOffset);
  void onCommandAck(quint32 command, quint8 reqSeq, quint8 result);

signals:
  // Bytes the vehicle has confirmed so far, out of the blob size.
  void progress(quint32 acked, quint32 total);
  void finished(XferUploader::Result result, const QString &message);

private:
  enum class State { Idle, Opening, Sending };

  void tick();
  void pump();
  enum class Tell { No, Close, Abort };
  void finish(Result r, const QString &msg, Tell tell);
  qint64 nowMs() const { return m_clock.elapsed(); }

  IXferLink *m_link;
  Config m_cfg;
  QTimer *m_timer;
  QElapsedTimer m_clock;

  State m_state = State::Idle;
  QByteArray m_path;
  QByteArray m_blob;
  quint8 m_reqSeq = 0;
  int m_chunk = xfer::kChunkMax;
  quint32 m_acked = 0;  // vehicle's cumulative cursor
  quint32 m_cursor = 0; // next byte we will send
  bool m_pumping = false;
  bool m_openAccepted = false;
  int m_openSends = 0;
  quint8 m_lastOpenResult = xfer::kResAccepted;
  bool m_haveOpenResult = false;
  qint64 m_openStartMs = 0;
  qint64 m_lastOpenMs = 0;
  qint64 m_lastProgressMs = 0;
  qint64 m_lastRefillMs = 0;
  qint64 m_lastStallMs = 0;
  bool m_eofSent = false; // empty blob: the lone zero-length EOF chunk
  double m_tokens = 0.0;
  QString m_lastError;
};
