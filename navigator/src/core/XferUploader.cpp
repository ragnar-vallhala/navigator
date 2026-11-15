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
#include "XferUploader.h"

#include <QTimer>

#include <algorithm>

using xfer::kFlagAbort;
using xfer::kFlagDone;
using xfer::kFlagEof;
using xfer::kFlagNak;
using xfer::kFlagNone;

namespace {
QString resultName(quint8 r) {
  switch (r) {
  case xfer::kResAccepted:
    return QObject::tr("accepted");
  case xfer::kResTempRejected:
    return QObject::tr("busy or clock not synced");
  case xfer::kResDenied:
    return QObject::tr("denied");
  case xfer::kResUnsupported:
    return QObject::tr("unsupported");
  case xfer::kResFailed:
    return QObject::tr("failed");
  default:
    return QObject::tr("code %1").arg(r);
  }
}
} // namespace

XferUploader::XferUploader(IXferLink *link, QObject *parent)
    : XferUploader(link, Config(), parent) {}

XferUploader::XferUploader(IXferLink *link, const Config &cfg, QObject *parent)
    : QObject(parent), m_link(link), m_cfg(cfg), m_timer(new QTimer(this)) {
  m_cfg.maxChunk = std::clamp(m_cfg.maxChunk, 1, xfer::kChunkMax);
  m_cfg.tickMs = std::max(1, m_cfg.tickMs);
  m_timer->setInterval(m_cfg.tickMs);
  connect(m_timer, &QTimer::timeout, this, &XferUploader::tick);
}

XferUploader::~XferUploader() = default;

bool XferUploader::start(const QString &path, const QByteArray &blob) {
  if (m_state != State::Idle) {
    m_lastError = tr("an upload is already running");
    return false;
  }
  const QByteArray p = path.toLatin1();
  if (p.isEmpty() || p.size() > xfer::kPathMax || path != QString::fromLatin1(p)) {
    m_lastError = tr("path must be 1 to %1 Latin-1 characters").arg(xfer::kPathMax);
    return false;
  }
  if (blob.size() < 0 || qint64(blob.size()) > 0x7FFFFFFFLL) {
    m_lastError = tr("blob too large");
    return false;
  }
  m_lastError.clear();
  m_path = p;
  m_blob = blob;
  m_reqSeq = static_cast<quint8>(m_reqSeq + 1);
  m_chunk = m_cfg.maxChunk;
  m_acked = 0;
  m_cursor = 0;
  m_openAccepted = false;
  m_openSends = 0;
  m_haveOpenResult = false;
  m_lastOpenResult = xfer::kResAccepted;
  m_tokens = 0.0;
  m_clock.start();
  m_openStartMs = m_lastOpenMs = m_lastProgressMs = m_lastRefillMs = 0;
  m_lastStallMs = 0;
  m_eofSent = false;
  m_state = State::Opening;
  m_timer->start();
  emit progress(0, quint32(m_blob.size()));
  if (!m_link->sendOpen(m_reqSeq, m_cfg.session, m_path)) {
    finish(Result::Failed, tr("link not available"), Tell::No);
    return true; // started, and already finished
  }
  m_openSends = 1;
  return true;
}

void XferUploader::onCommandAck(quint32 command, quint8 reqSeq, quint8 result) {
  if (m_state != State::Opening || command != xfer::kMsgidXferOpen ||
      reqSeq != m_reqSeq)
    return;
  if (result == xfer::kResAccepted || result == xfer::kResInProgress) {
    m_openAccepted = true;
    return;
  }
  m_haveOpenResult = true;
  m_lastOpenResult = result;
  if (m_openAccepted)
    return; // stale refusal of a re-sent open after the first one was accepted
  if (result == xfer::kResTempRejected)
    return; // busy / not synced yet / our own re-send racing the first: keep
            // trying until the open timeout, then report it
  finish(Result::Refused,
         tr("vehicle refused the upload (%1)").arg(resultName(result)), Tell::No);
}

void XferUploader::onXferInfo(quint8 session, quint8 result, quint16 chunkSize,
                              quint32 totalSize) {
  Q_UNUSED(totalSize);
  if (session != m_cfg.session || m_state != State::Opening)
    return;
  if (result != xfer::kResAccepted) {
    finish(Result::Refused,
           tr("vehicle refused the upload (%1)").arg(resultName(result)),
           Tell::No);
    return;
  }
  if (chunkSize > 0)
    m_chunk = std::min<int>(m_cfg.maxChunk, chunkSize);
  m_openAccepted = true;
  m_state = State::Sending;
  m_lastProgressMs = nowMs();
  m_lastRefillMs = m_lastProgressMs;
  pump();
}

void XferUploader::onXferAck(quint8 session, quint8 flags, quint8 result,
                             quint32 nextOffset) {
  if (session != m_cfg.session || m_state == State::Idle)
    return;
  if (m_state == State::Opening) {
    // A periodic ack means the session is open and INFO went missing.
    m_openAccepted = true;
    m_state = State::Sending;
    m_lastRefillMs = nowMs();
    m_lastProgressMs = m_lastRefillMs;
  }
  const quint32 total = quint32(m_blob.size());
  if (flags & kFlagAbort) {
    finish(Result::Failed,
           tr("vehicle aborted the upload (%1)").arg(resultName(result)),
           Tell::No);
    return;
  }
  if (flags & kFlagDone) {
    if (result != xfer::kResAccepted) {
      finish(Result::Failed,
             tr("vehicle could not write the file (%1)").arg(resultName(result)),
             Tell::Close);
    } else if (nextOffset < total) {
      finish(Result::Failed,
             tr("vehicle finished at byte %1 of %2").arg(nextOffset).arg(total),
             Tell::Close);
    } else {
      m_acked = total;
      emit progress(total, total);
      finish(Result::Ok, tr("uploaded %1 bytes").arg(total), Tell::Close);
    }
    return;
  }
  if (nextOffset > m_acked && nextOffset <= total) {
    m_acked = nextOffset;
    m_lastProgressMs = nowMs();
    emit progress(m_acked, total);
  }
  // The cursor is only moved back on an explicit NAK; a cumulative ack behind
  // the cursor is just chunks still in flight.
  if ((flags & kFlagNak) && nextOffset <= total) {
    m_cursor = nextOffset;
    if (nextOffset > m_acked)
      m_acked = nextOffset;
  }
  if (m_cursor < m_acked)
    m_cursor = m_acked;
  pump();
}

void XferUploader::tick() {
  if (m_state == State::Idle)
    return;
  const qint64 t = nowMs();
  if (m_state == State::Opening) {
    if (t - m_openStartMs >= m_cfg.openTimeoutMs) {
      const QString why =
          m_haveOpenResult
              ? tr("vehicle did not accept the upload (%1)")
                    .arg(resultName(m_lastOpenResult))
              : tr("no answer from the vehicle");
      finish(Result::Timeout, why, Tell::Abort);
      return;
    }
    if (t - m_lastOpenMs >= m_cfg.openRetryMs) {
      m_lastOpenMs = t;
      ++m_openSends;
      if (!m_link->sendOpen(m_reqSeq, m_cfg.session, m_path))
        finish(Result::Failed, tr("link not available"), Tell::No);
    }
    return;
  }
  // Sending.
  if (t - m_lastProgressMs >= m_cfg.idleTimeoutMs) {
    finish(Result::Timeout,
           tr("vehicle stopped answering at byte %1 of %2")
               .arg(m_acked)
               .arg(m_blob.size()),
           Tell::Abort);
    return;
  }
  if (t - m_lastProgressMs >= m_cfg.stallMs &&
      t - m_lastStallMs >= m_cfg.stallMs) {
    // Stalled: resume from what the vehicle confirmed (covers a lost chunk or
    // a lost EOF). Rate-limited by stallMs, not by every tick.
    m_cursor = m_acked;
    m_lastStallMs = t;
    if (m_acked >= quint32(m_blob.size())) {
      // Everything is acked but DONE never came: the EOF marker may be lost.
      if (!m_link->sendData(m_cfg.session, kFlagEof, quint32(m_blob.size()),
                            QByteArray())) {
        finish(Result::Failed, tr("link not available"), Tell::No);
        return;
      }
    }
  }
  pump();
}

void XferUploader::pump() {
  if (m_pumping || m_state != State::Sending)
    return;
  m_pumping = true;
  const qint64 t = nowMs();
  if (m_cfg.maxRateBps > 0) {
    const double cap = std::max(double(m_cfg.windowBytes), double(m_chunk));
    m_tokens = std::min(cap, m_tokens + double(t - m_lastRefillMs) *
                                            double(m_cfg.maxRateBps) / 1000.0);
  }
  m_lastRefillMs = t;
  const quint32 total = quint32(m_blob.size());
  while (m_state == State::Sending) {
    const bool emptyEof = (total == 0 && !m_eofSent);
    if (m_cursor >= total && !emptyEof)
      break;
    if (m_cursor - m_acked >= quint32(std::max(m_cfg.windowBytes, 1)))
      break;
    if (m_cfg.maxRateBps > 0 && m_tokens < 1.0)
      break;
    const quint32 len = std::min<quint32>(quint32(m_chunk), total - m_cursor);
    const quint8 flags = (m_cursor + len >= total) ? kFlagEof : kFlagNone;
    const QByteArray payload = m_blob.mid(int(m_cursor), int(len));
    const quint32 off = m_cursor;
    m_cursor += len;
    m_tokens -= double(len);
    if (total == 0)
      m_eofSent = true;
    if (!m_link->sendData(m_cfg.session, flags, off, payload)) {
      m_pumping = false;
      finish(Result::Failed, tr("link not available"), Tell::No);
      return;
    }
  }
  m_pumping = false;
}

void XferUploader::finish(Result r, const QString &msg, Tell tell) {
  if (m_state == State::Idle)
    return;
  m_state = State::Idle;
  m_timer->stop();
  const quint32 total = quint32(m_blob.size());
  const quint32 acked = m_acked;
  if (tell == Tell::Abort)
    m_link->sendData(m_cfg.session, kFlagAbort, acked, QByteArray());
  if (tell != Tell::No) {
    m_reqSeq = static_cast<quint8>(m_reqSeq + 1);
    m_link->sendClose(m_reqSeq, m_cfg.session,
                      r == Result::Ok ? xfer::kResAccepted : xfer::kResFailed);
  }
  m_blob.clear();
  m_lastError = (r == Result::Ok) ? QString() : msg;
  Q_UNUSED(total);
  emit finished(r, msg);
}

void XferUploader::cancel() {
  finish(Result::Cancelled, tr("upload cancelled"), Tell::Abort);
}
