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
// XferUploader against a FAKE XFER peer that behaves like the firmware's
// upload side (firmware/src/comm/xfer/navlink_xfer.c): contiguous-only writes,
// a periodic cumulative XFER_ACK, EOF then a terminal XFER_ACK{DONE, result},
// COMMAND_ACK for open (a re-sent open on a live session is TEMPORARILY_
// REJECTED), XFER_INFO after the open, ABORT honoured on XFER_DATA only.
// Replies are delivered on the event loop, as they would be off a link.
// Also checks the wire encoders and the battery-calibration dialog's upload.

#include "BatteryCalib.h"
#include "XferUploader.h"
#include "CommandCodec.h"
#include "XferWireLink.h"
#include "ui/widgets/BatteryCalibDialog.h"

#include <QElapsedTimer>
#include <QPushButton>
#include <QTemporaryDir>
#include <QSet>
#include <QSignalSpy>
#include <QTimer>
#include <QtTest>
#include <algorithm>
#include <functional>

extern "C" {
#include "navlink_msgs.h"
}

Q_DECLARE_METATYPE(XferUploader::Result)

namespace {

struct Frame {
  quint8 flags = 0;
  quint32 offset = 0;
  int len = 0;
};

class FakePeer : public IXferLink {
public:
  explicit FakePeer(QObject *host) : m_host(host), m_ackTimer(host) {
    m_ackTimer.setInterval(15);
    QObject::connect(&m_ackTimer, &QTimer::timeout, host, [this] { periodic(); });
  }
  XferUploader *up = nullptr;

  // behaviour
  quint8 openResult = xfer::kResAccepted; // COMMAND_ACK result for the open
  bool sendInfo = true;
  bool sendAccept = true; // the COMMAND_ACK ACCEPTED for the open
  bool loseDone = false;  // the one terminal DONE never arrives
  quint16 chunk = 247;
  QSet<int> dropData; // 1-based index of the XFER_DATA frames to lose
  bool silent = false; // hears nothing, says nothing
  bool paused = false; // stops the periodic ack (and DONE)
  quint8 doneResult = xfer::kResAccepted;
  bool linkDown = false;

  // observed
  int opens = 0, datas = 0, closes = 0, aborts = 0, maxPayload = 0;
  QByteArray openPath;
  quint8 session = 0;
  QVector<Frame> frames;
  quint8 lastCloseResult = 255;
  QByteArray file;

  bool sendOpen(quint8 reqSeq, quint8 s, const QByteArray &path) override {
    if (linkDown)
      return false;
    ++opens;
    openPath = path;
    session = s;
    if (silent)
      return true;
    if (m_active) {
      reply([=, this] { up->onCommandAck(xfer::kMsgidXferOpen, reqSeq, xfer::kResTempRejected); });
      return true;
    }
    if (openResult != xfer::kResAccepted) {
      const quint8 r = openResult;
      reply([=, this] { up->onCommandAck(xfer::kMsgidXferOpen, reqSeq, r); });
      return true;
    }
    m_active = true;
    m_cursor = 0;
    m_eof = false;
    file.clear();
    m_doneSent = false;
    if (sendAccept)
      reply([=, this] { up->onCommandAck(xfer::kMsgidXferOpen, reqSeq, xfer::kResAccepted); });
    if (sendInfo) {
      const quint16 c = chunk;
      reply([=, this] { up->onXferInfo(s, xfer::kResAccepted, c, 0xFFFFFFFFu); });
    }
    m_ackTimer.start();
    return true;
  }

  bool sendData(quint8 s, quint8 flags, quint32 offset,
                const QByteArray &payload) override {
    if (linkDown)
      return false;
    ++datas;
    frames.push_back({flags, offset, int(payload.size())});
    maxPayload = std::max<int>(maxPayload, payload.size());
    if (silent || dropData.contains(datas))
      return true;
    if (flags & xfer::kFlagAbort) {
      ++aborts;
      m_active = false;
      m_ackTimer.stop();
      return true;
    }
    if (!m_active)
      return true;
    if (offset == m_cursor && !payload.isEmpty()) {
      file.append(payload);
      m_cursor += quint32(payload.size());
    }
    if ((flags & xfer::kFlagEof) && offset + quint32(payload.size()) <= m_cursor)
      m_eof = true;
    Q_UNUSED(s);
    return true;
  }

  bool sendClose(quint8, quint8, quint8 result) override {
    if (linkDown)
      return false;
    ++closes;
    lastCloseResult = result;
    m_active = false;
    m_ackTimer.stop();
    return true;
  }

private:
  void reply(std::function<void()> f) { QTimer::singleShot(0, m_host, std::move(f)); }
  void periodic() {
    if (paused || !m_active || silent)
      return;
    if (m_eof) {
      // Like the firmware: the terminal DONE goes out once, then the slot
      // lingers silently.
      if (!m_doneSent) {
        m_doneSent = true;
        if (!loseDone)
          up->onXferAck(session, xfer::kFlagDone, doneResult, m_cursor);
      }
      return;
    }
    up->onXferAck(session, xfer::kFlagNone, xfer::kResAccepted, m_cursor);
  }
  QObject *m_host;
  QTimer m_ackTimer;
  bool m_active = false;
  bool m_eof = false;
  bool m_doneSent = false;
  quint32 m_cursor = 0;
};

XferUploader::Config fastCfg() {
  XferUploader::Config c;
  c.openRetryMs = 30;
  c.openTimeoutMs = 400;
  c.stallMs = 60;
  c.idleTimeoutMs = 500;
  c.windowBytes = 4096;
  c.maxRateBps = 0;
  c.tickMs = 2;
  return c;
}

QByteArray blobOf(int n) {
  QByteArray b(n, 0);
  for (int i = 0; i < n; ++i)
    b[i] = char((i * 7 + 3) & 0xFF);
  return b;
}

constexpr int kWaitMs = 4000;

// Wait for the one finished() and return its arguments.
struct Outcome {
  XferUploader::Result r = XferUploader::Result::Failed;
  QString msg;
  bool got = false;
};
Outcome waitDone(QSignalSpy &done) {
  Outcome o;
  if (done.count() == 0 && !done.wait(kWaitMs))
    return o;
  o.got = true;
  o.r = qvariant_cast<XferUploader::Result>(done.last().at(0));
  o.msg = done.last().at(1).toString();
  return o;
}

// Decoded wire frames for the encoder test.
struct Seen {
  navlink_xfer_open_t open{};
  navlink_xfer_data_t data{};
  navlink_xfer_close_t close{};
  int nOpen = 0, nData = 0, nClose = 0;
} g_seen;
navlink_ack_t onOpen(void *, const navlink_frame_hdr_t *, const navlink_xfer_open_t *m) {
  g_seen.open = *m;
  ++g_seen.nOpen;
  return navlink_ack_t{};
}
navlink_ack_t onClose(void *, const navlink_frame_hdr_t *, const navlink_xfer_close_t *m) {
  g_seen.close = *m;
  ++g_seen.nClose;
  return navlink_ack_t{};
}
void onData(void *, const navlink_frame_hdr_t *, const navlink_xfer_data_t *m) {
  g_seen.data = *m;
  ++g_seen.nData;
}
void decode(const QByteArray &f) {
  g_seen = Seen();
  navlink_parser_t p;
  navlink_parser_init(&p);
  navlink_handlers_t h{};
  h.on_xfer_open = onOpen;
  h.on_xfer_close = onClose;
  h.on_xfer_data = onData;
  navlink_parser_push(&p, &h, reinterpret_cast<const uint8_t *>(f.constData()),
                      size_t(f.size()));
}

} // namespace

class TstXferUploader : public QObject {
  Q_OBJECT
private slots:
  void initTestCase() { qRegisterMetaType<XferUploader::Result>(); }
  void successSmallBlob();
  void blobLargerThanOneChunk();
  void windowLimitsFlight();
  void negotiatedChunkIsHonoured();
  void droppedChunkIsRetried();
  void droppedEofChunkIsRetried();
  void lostInfoFallsBackToDefaultChunk();
  void acceptAndInfoLostTimesOut();
  void staleAckWhileOpeningIsIgnored();
  void staleDoneWhileOpeningDoesNotGiveOk();
  void sessionSlotAlternates();
  void lostDoneIsUnconfirmed();
  void doneIsSentOnceAndStillOk();
  void refusalDenied();
  void refusalUnsupportedAndFailed();
  void busyPeerTimesOut();
  void silentPeerTimesOutOpening();
  void peerGoesSilentMidTransfer();
  void peerReportsFailedWrite();
  void cancelMidTransfer();
  void cancelWhileOpening();
  void cancelWhenIdleIsNoop();
  void emptyBlob();
  void badInputAndBusy();
  void linkRefusingTxFails();
  void rateCapStillCompletes();
  void wireEncoders();
  void dialogUploadsBlob();
  void dialogGatedByAvailability();
  void dialogCancelsWhenLinkLost();
  void dialogShowsFailure();
  void dialogShowsUnconfirmed();
};

void TstXferUploader::successSmallBlob() {
  FakePeer peer(this);
  XferUploader up(&peer, fastCfg());
  peer.up = &up;
  QSignalSpy done(&up, &XferUploader::finished);
  QSignalSpy prog(&up, &XferUploader::progress);
  const QByteArray b = blobOf(16);
  QVERIFY(up.start("0:batcal.bin", b));
  QVERIFY(up.busy());
  const Outcome o = waitDone(done);
  QVERIFY(o.got);
  QCOMPARE(o.r, XferUploader::Result::Ok);
  QCOMPARE(peer.file, b);
  QCOMPARE(peer.openPath, QByteArray("0:batcal.bin"));
  QCOMPARE(peer.frames.size(), 1);
  QCOMPARE(peer.frames[0].flags, xfer::kFlagEof);
  QCOMPARE(peer.closes, 1);
  QCOMPARE(peer.lastCloseResult, xfer::kResAccepted);
  QCOMPARE(peer.aborts, 0);
  QVERIFY(!up.busy());
  QCOMPARE(prog.last().at(0).toUInt(), 16u);
  QCOMPARE(prog.last().at(1).toUInt(), 16u);
  QCOMPARE(done.count(), 1);
}

void TstXferUploader::blobLargerThanOneChunk() {
  FakePeer peer(this);
  XferUploader up(&peer, fastCfg());
  peer.up = &up;
  QSignalSpy done(&up, &XferUploader::finished);
  const QByteArray b = blobOf(1000); // 247 + 247 + 247 + 247 + 12
  QVERIFY(up.start("0:big.bin", b));
  const Outcome o = waitDone(done);
  QCOMPARE(o.r, XferUploader::Result::Ok);
  QCOMPARE(peer.file, b);
  QCOMPARE(peer.frames.size(), 5);
  QVERIFY(peer.maxPayload <= 237); // capped below the firmware's 247
  quint32 off = 0;
  for (int i = 0; i < peer.frames.size(); ++i) {
    QCOMPARE(peer.frames[i].offset, off);
    off += quint32(peer.frames[i].len);
    // EOF on the last chunk and only there.
    QCOMPARE((peer.frames[i].flags & xfer::kFlagEof) != 0, i == 4);
  }
  QCOMPARE(off, 1000u);
  QCOMPARE(peer.frames[0].len, 237);
}

void TstXferUploader::windowLimitsFlight() {
  FakePeer peer(this);
  XferUploader::Config c = fastCfg();
  c.windowBytes = 300; // two chunks
  XferUploader up(&peer, c);
  peer.up = &up;
  peer.paused = true; // no acks: flight must stop at the window
  QSignalSpy done(&up, &XferUploader::finished);
  QVERIFY(up.start("0:w.bin", blobOf(3000)));
  QTRY_VERIFY_WITH_TIMEOUT(peer.datas >= 2, kWaitMs);
  QTest::qWait(30);
  QCOMPARE(peer.datas, 2); // 247 + 247 >= 300: nothing more until acked
  peer.paused = false;
  const Outcome o = waitDone(done);
  QCOMPARE(o.r, XferUploader::Result::Ok);
  QCOMPARE(peer.file, blobOf(3000));
}

void TstXferUploader::negotiatedChunkIsHonoured() {
  FakePeer peer(this);
  peer.chunk = 100;
  XferUploader up(&peer, fastCfg());
  peer.up = &up;
  QSignalSpy done(&up, &XferUploader::finished);
  QVERIFY(up.start("0:c.bin", blobOf(250)));
  QCOMPARE(waitDone(done).r, XferUploader::Result::Ok);
  QCOMPARE(peer.frames.size(), 3);
  QVERIFY(peer.maxPayload <= 100);
  QCOMPARE(peer.file, blobOf(250));
}

void TstXferUploader::droppedChunkIsRetried() {
  FakePeer peer(this);
  XferUploader up(&peer, fastCfg());
  peer.up = &up;
  peer.dropData = {2}; // lose the second chunk; the rest arrive as a gap
  QSignalSpy done(&up, &XferUploader::finished);
  const QByteArray b = blobOf(1000);
  QVERIFY(up.start("0:d.bin", b));
  const Outcome o = waitDone(done);
  QCOMPARE(o.r, XferUploader::Result::Ok);
  QCOMPARE(peer.file, b);
  QVERIFY2(peer.datas > 5, "the lost chunk must have been sent again");
}

void TstXferUploader::droppedEofChunkIsRetried() {
  FakePeer peer(this);
  XferUploader up(&peer, fastCfg());
  peer.up = &up;
  peer.dropData = {2}; // 300 B = 247 + 53: the second one is the EOF chunk
  QSignalSpy done(&up, &XferUploader::finished);
  const QByteArray b = blobOf(300);
  QVERIFY(up.start("0:e.bin", b));
  QCOMPARE(waitDone(done).r, XferUploader::Result::Ok);
  QCOMPARE(peer.file, b);
  QCOMPARE(peer.datas, 3);
}

void TstXferUploader::lostInfoFallsBackToDefaultChunk() {
  FakePeer peer(this);
  peer.sendInfo = false; // INFO lost; the COMMAND_ACK ACCEPTED still opened it
  XferUploader up(&peer, fastCfg());
  peer.up = &up;
  QSignalSpy done(&up, &XferUploader::finished);
  const QByteArray b = blobOf(500);
  QVERIFY(up.start("0:i.bin", b));
  QCOMPARE(waitDone(done).r, XferUploader::Result::Ok);
  QCOMPARE(peer.file, b);
  QVERIFY(peer.maxPayload <= 237);
  QCOMPARE(peer.opens, 1); // never re-sent once accepted
}

void TstXferUploader::acceptAndInfoLostTimesOut() {
  // Both open replies lost while the slot is live and acking: the re-sent open
  // is answered TEMPORARILY_REJECTED, and the periodic acks must NOT be taken
  // as the open. The upload ends as a timeout, with nothing written.
  FakePeer peer(this);
  peer.sendInfo = false;
  peer.sendAccept = false;
  XferUploader up(&peer, fastCfg());
  peer.up = &up;
  QSignalSpy done(&up, &XferUploader::finished);
  QVERIFY(up.start("0:i.bin", blobOf(500)));
  const Outcome o = waitDone(done);
  QCOMPARE(o.r, XferUploader::Result::Timeout);
  QVERIFY(peer.file.isEmpty());
  QCOMPARE(peer.datas, 1); // only the abort marker on the way out
  QCOMPARE(peer.aborts, 1);
}

void TstXferUploader::staleAckWhileOpeningIsIgnored() {
  FakePeer peer(this);
  peer.silent = true; // the open is never answered
  XferUploader up(&peer, fastCfg());
  peer.up = &up;
  QSignalSpy done(&up, &XferUploader::finished);
  QVERIFY(up.start("0:x.bin", blobOf(16)));
  // A late plain ack of some earlier upload on this slot.
  up.onXferAck(peer.session, xfer::kFlagNone, xfer::kResAccepted, 0);
  up.onXferAck(peer.session, xfer::kFlagNone, xfer::kResAccepted, 16);
  QTest::qWait(100);
  QCOMPARE(done.count(), 0);
  QCOMPARE(peer.datas, 0); // it did not skip the open and send chunks
  QVERIFY(up.busy());
  up.cancel();
}

void TstXferUploader::staleDoneWhileOpeningDoesNotGiveOk() {
  // 1) peer never opens: a stale DONE/ACCEPTED must not finish the upload.
  {
    FakePeer peer(this);
    peer.silent = true;
    XferUploader up(&peer, fastCfg());
    peer.up = &up;
    QSignalSpy done(&up, &XferUploader::finished);
    QVERIFY(up.start("0:batcal.bin", blobOf(16)));
    up.onXferAck(peer.session, xfer::kFlagDone, xfer::kResAccepted, 16);
    QCOMPARE(done.count(), 0);
    QVERIFY(up.busy());
    const Outcome o = waitDone(done); // ends as a timeout, never Ok
    QCOMPARE(o.r, XferUploader::Result::Timeout);
  }
  // 2) peer opens normally: the result is Ok only because the bytes arrived.
  {
    FakePeer peer(this);
    XferUploader up(&peer, fastCfg());
    peer.up = &up;
    QSignalSpy done(&up, &XferUploader::finished);
    const QByteArray b = blobOf(16);
    QVERIFY(up.start("0:batcal.bin", b));
    up.onXferAck(0, xfer::kFlagDone, xfer::kResAccepted, 16); // stale, right away
    QCOMPARE(done.count(), 0);
    QCOMPARE(waitDone(done).r, XferUploader::Result::Ok);
    QCOMPARE(peer.file, b);
  }
}

void TstXferUploader::sessionSlotAlternates() {
  FakePeer peer(this);
  XferUploader up(&peer, fastCfg());
  peer.up = &up;
  QSignalSpy done(&up, &XferUploader::finished);
  QVERIFY(up.start("0:a.bin", blobOf(8)));
  const quint8 first = peer.session;
  QCOMPARE(waitDone(done).r, XferUploader::Result::Ok);
  done.clear();
  peer.silent = true;
  QVERIFY(up.start("0:b.bin", blobOf(8)));
  QVERIFY(peer.session != first);
  QVERIFY(peer.session < 2); // the firmware has two slots
  // A late DONE from the first upload (its slot) is for another session.
  up.onXferAck(first, xfer::kFlagDone, xfer::kResAccepted, 8);
  QCOMPARE(done.count(), 0);
  up.cancel();
}

void TstXferUploader::lostDoneIsUnconfirmed() {
  FakePeer peer(this);
  peer.loseDone = true; // the single terminal DONE is lost
  XferUploader up(&peer, fastCfg());
  peer.up = &up;
  QSignalSpy done(&up, &XferUploader::finished);
  const QByteArray b = blobOf(16);
  QVERIFY(up.start("0:batcal.bin", b));
  const Outcome o = waitDone(done);
  QCOMPARE(o.r, XferUploader::Result::Unconfirmed);
  QVERIFY2(o.msg.contains("did not confirm"), qPrintable(o.msg));
  QCOMPARE(peer.file, b); // it did arrive
  QCOMPARE(peer.closes, 1);
  QCOMPARE(peer.aborts, 0); // not treated as a failed transfer
  QVERIFY(!up.busy());
}

void TstXferUploader::doneIsSentOnceAndStillOk() {
  FakePeer peer(this);
  XferUploader up(&peer, fastCfg());
  peer.up = &up;
  QSignalSpy done(&up, &XferUploader::finished);
  QVERIFY(up.start("0:batcal.bin", blobOf(16)));
  QCOMPARE(waitDone(done).r, XferUploader::Result::Ok);
  QCOMPARE(done.count(), 1);
}

void TstXferUploader::refusalDenied() {
  FakePeer peer(this);
  peer.openResult = xfer::kResDenied;
  XferUploader up(&peer, fastCfg());
  peer.up = &up;
  QSignalSpy done(&up, &XferUploader::finished);
  QVERIFY(up.start("0:batcal.bin", blobOf(16)));
  const Outcome o = waitDone(done);
  QCOMPARE(o.r, XferUploader::Result::Refused);
  QVERIFY2(o.msg.contains("denied"), qPrintable(o.msg));
  QCOMPARE(peer.datas, 0);
  QVERIFY(!up.busy());
  // A refusal is an answer: no abort or close for a session that never opened.
  QCOMPARE(peer.closes, 0);
}

void TstXferUploader::refusalUnsupportedAndFailed() {
  for (quint8 res : {xfer::kResUnsupported, xfer::kResFailed}) {
    FakePeer peer(this);
    peer.openResult = res;
    XferUploader up(&peer, fastCfg());
    peer.up = &up;
    QSignalSpy done(&up, &XferUploader::finished);
    QVERIFY(up.start("0:x.bin", blobOf(4)));
    QCOMPARE(waitDone(done).r, XferUploader::Result::Refused);
    QCOMPARE(peer.datas, 0);
  }
}

void TstXferUploader::busyPeerTimesOut() {
  // TEMPORARILY_REJECTED (busy, or clock not yet synced) is retried until the
  // open timeout, then reported as such.
  FakePeer peer(this);
  peer.openResult = xfer::kResTempRejected;
  XferUploader up(&peer, fastCfg());
  peer.up = &up;
  QSignalSpy done(&up, &XferUploader::finished);
  QVERIFY(up.start("0:x.bin", blobOf(4)));
  const Outcome o = waitDone(done);
  QCOMPARE(o.r, XferUploader::Result::Timeout);
  QVERIFY2(o.msg.contains("busy"), qPrintable(o.msg));
  QVERIFY(peer.opens >= 3); // re-sent while waiting
  QCOMPARE(peer.datas, 1);  // only the abort marker from the give-up
  QCOMPARE(peer.aborts, 1);
}

void TstXferUploader::silentPeerTimesOutOpening() {
  FakePeer peer(this);
  peer.silent = true;
  XferUploader up(&peer, fastCfg());
  peer.up = &up;
  QSignalSpy done(&up, &XferUploader::finished);
  QElapsedTimer t;
  t.start();
  QVERIFY(up.start("0:x.bin", blobOf(4)));
  const Outcome o = waitDone(done);
  QCOMPARE(o.r, XferUploader::Result::Timeout);
  QVERIFY2(o.msg.contains("no answer"), qPrintable(o.msg));
  QVERIFY(t.elapsed() >= 380);
  QVERIFY(peer.opens >= 5);
  QVERIFY(!up.busy());
}

void TstXferUploader::peerGoesSilentMidTransfer() {
  FakePeer peer(this);
  XferUploader::Config c = fastCfg();
  c.windowBytes = 300;
  c.idleTimeoutMs = 250;
  XferUploader up(&peer, c);
  peer.up = &up;
  QSignalSpy done(&up, &XferUploader::finished);
  QVERIFY(up.start("0:x.bin", blobOf(3000)));
  QTRY_VERIFY_WITH_TIMEOUT(peer.datas >= 1, kWaitMs);
  peer.paused = true; // vehicle vanishes
  const Outcome o = waitDone(done);
  QCOMPARE(o.r, XferUploader::Result::Timeout);
  QVERIFY2(o.msg.contains("stopped answering"), qPrintable(o.msg));
  QCOMPARE(peer.aborts, 1);
  QCOMPARE(peer.closes, 1); // told the vehicle on the way out
}

void TstXferUploader::peerReportsFailedWrite() {
  FakePeer peer(this);
  peer.doneResult = xfer::kResFailed;
  XferUploader up(&peer, fastCfg());
  peer.up = &up;
  QSignalSpy done(&up, &XferUploader::finished);
  QVERIFY(up.start("0:x.bin", blobOf(40)));
  const Outcome o = waitDone(done);
  QCOMPARE(o.r, XferUploader::Result::Failed);
  QVERIFY2(o.msg.contains("write"), qPrintable(o.msg));
  QCOMPARE(peer.lastCloseResult, xfer::kResFailed);
}

void TstXferUploader::cancelMidTransfer() {
  FakePeer peer(this);
  XferUploader::Config c = fastCfg();
  c.windowBytes = 300;
  XferUploader up(&peer, c);
  peer.up = &up;
  peer.paused = true;
  QSignalSpy done(&up, &XferUploader::finished);
  QVERIFY(up.start("0:x.bin", blobOf(3000)));
  QTRY_VERIFY_WITH_TIMEOUT(peer.datas >= 2, kWaitMs);
  up.cancel();
  QCOMPARE(done.count(), 1);
  QCOMPARE(qvariant_cast<XferUploader::Result>(done.at(0).at(0)),
           XferUploader::Result::Cancelled);
  QVERIFY(!up.busy());
  // ABORT goes out as a flag on XFER_DATA (the firmware ignores it elsewhere
  // for an upload), then XFER_CLOSE.
  QVERIFY(peer.frames.last().flags & xfer::kFlagAbort);
  QCOMPARE(peer.frames.last().len, 0);
  QCOMPARE(peer.aborts, 1);
  QCOMPARE(peer.closes, 1);
  const int frames = peer.datas;
  peer.paused = false;
  QTest::qWait(150); // nothing more is sent after a cancel
  QCOMPARE(peer.datas, frames);
  QCOMPARE(done.count(), 1);
  // And the uploader is reusable.
  QSignalSpy done2(&up, &XferUploader::finished);
  QVERIFY(up.start("0:y.bin", blobOf(10)));
  QCOMPARE(waitDone(done2).r, XferUploader::Result::Ok);
  QCOMPARE(peer.file, blobOf(10));
}

void TstXferUploader::cancelWhileOpening() {
  FakePeer peer(this);
  peer.silent = true;
  XferUploader up(&peer, fastCfg());
  peer.up = &up;
  QSignalSpy done(&up, &XferUploader::finished);
  QVERIFY(up.start("0:x.bin", blobOf(10)));
  up.cancel();
  QCOMPARE(done.count(), 1);
  QCOMPARE(qvariant_cast<XferUploader::Result>(done.at(0).at(0)),
           XferUploader::Result::Cancelled);
  QCOMPARE(peer.closes, 1);
  const int opens = peer.opens;
  QTest::qWait(120);
  QCOMPARE(peer.opens, opens); // no more re-sends
}

void TstXferUploader::cancelWhenIdleIsNoop() {
  FakePeer peer(this);
  XferUploader up(&peer, fastCfg());
  peer.up = &up;
  QSignalSpy done(&up, &XferUploader::finished);
  up.cancel();
  QCOMPARE(done.count(), 0);
  QCOMPARE(peer.closes, 0);
  QCOMPARE(peer.datas, 0);
}

void TstXferUploader::emptyBlob() {
  FakePeer peer(this);
  XferUploader up(&peer, fastCfg());
  peer.up = &up;
  QSignalSpy done(&up, &XferUploader::finished);
  QVERIFY(up.start("0:empty.bin", QByteArray()));
  QCOMPARE(waitDone(done).r, XferUploader::Result::Ok);
  QCOMPARE(peer.frames.size(), 1);
  QCOMPARE(peer.frames[0].len, 0);
  QVERIFY(peer.frames[0].flags & xfer::kFlagEof);
  QVERIFY(peer.file.isEmpty());
}

void TstXferUploader::badInputAndBusy() {
  FakePeer peer(this);
  XferUploader up(&peer, fastCfg());
  peer.up = &up;
  QSignalSpy done(&up, &XferUploader::finished);
  QVERIFY(!up.start(QString(), blobOf(4)));
  QVERIFY(!up.lastError().isEmpty());
  QVERIFY(!up.start(QString(32, QLatin1Char('a')), blobOf(4))); // > 31
  QVERIFY(!up.start(QString::fromUtf8("0:\u00e9\u4e2d.bin"), blobOf(4)));
  QVERIFY(!up.busy());
  QCOMPARE(peer.opens, 0);
  QCOMPARE(done.count(), 0);
  QVERIFY(up.start(QString(31, QLatin1Char('a')), blobOf(4))); // exactly fits
  QVERIFY(!up.start("0:b.bin", blobOf(4)));                   // busy
  QVERIFY(up.lastError().contains("already"));
  QCOMPARE(waitDone(done).r, XferUploader::Result::Ok);
  QCOMPARE(peer.openPath.size(), 31);
}

void TstXferUploader::linkRefusingTxFails() {
  FakePeer peer(this);
  peer.linkDown = true;
  XferUploader up(&peer, fastCfg());
  peer.up = &up;
  QSignalSpy done(&up, &XferUploader::finished);
  QVERIFY(up.start("0:x.bin", blobOf(4)));
  QCOMPARE(done.count(), 1);
  QCOMPARE(qvariant_cast<XferUploader::Result>(done.at(0).at(0)),
           XferUploader::Result::Failed);
  QVERIFY(!up.busy());
  QVERIFY(done.at(0).at(1).toString().contains("link"));
}

void TstXferUploader::rateCapStillCompletes() {
  FakePeer peer(this);
  XferUploader::Config c = fastCfg();
  c.maxRateBps = 20000; // 600 B takes ~30 ms
  XferUploader up(&peer, c);
  peer.up = &up;
  QSignalSpy done(&up, &XferUploader::finished);
  const QByteArray b = blobOf(600);
  QVERIFY(up.start("0:r.bin", b));
  QCOMPARE(waitDone(done).r, XferUploader::Result::Ok);
  QCOMPARE(peer.file, b);
}

void TstXferUploader::wireEncoders() {
  decode(CommandCodec::encodeXferOpen(3, 9, "0:batcal.bin", 42));
  QCOMPARE(g_seen.nOpen, 1);
  QCOMPARE(int(g_seen.open.session), 3);
  QCOMPARE(int(g_seen.open.req_seq), 9);
  QCOMPARE(int(g_seen.open.dir), 1);  // upload
  QCOMPARE(int(g_seen.open.mode), 0); // file
  QCOMPARE(int(g_seen.open.service_id), 0);
  QCOMPARE(int(g_seen.open.target_sys), 42);
  QCOMPARE(QByteArray(g_seen.open.arg), QByteArray("0:batcal.bin"));

  const QByteArray p = blobOf(247);
  decode(CommandCodec::encodeXferData(1, xfer::kFlagEof, 494, p));
  QCOMPARE(g_seen.nData, 1);
  QCOMPARE(int(g_seen.data.session), 1);
  QCOMPARE(int(g_seen.data.flags), int(xfer::kFlagEof));
  QCOMPARE(int(g_seen.data.len), 247);
  QCOMPARE(g_seen.data.offset, 494u);
  QCOMPARE(QByteArray(reinterpret_cast<const char *>(g_seen.data.data), 247), p);

  // A short chunk: trailing zeros truncate on the wire, len stays authoritative.
  const QByteArray z = QByteArray::fromHex("0102030000");
  decode(CommandCodec::encodeXferData(0, 0, 0, z));
  QCOMPARE(int(g_seen.data.len), 5);
  QCOMPARE(QByteArray(reinterpret_cast<const char *>(g_seen.data.data), 5), z);

  decode(CommandCodec::encodeXferClose(2, 4, xfer::kResFailed, 7));
  QCOMPARE(g_seen.nClose, 1);
  QCOMPARE(int(g_seen.close.session), 2);
  QCOMPARE(int(g_seen.close.req_seq), 4);
  QCOMPARE(int(g_seen.close.result), int(xfer::kResFailed));

  // The wire link feeds the sink one frame per call and reports a refusal.
  QVector<QByteArray> sent;
  bool allow = true;
  XferWireLink wl([&](const QByteArray &f) {
    if (allow)
      sent.push_back(f);
    return allow;
  });
  QVERIFY(wl.sendOpen(1, 0, "0:a.bin"));
  QVERIFY(wl.sendData(0, 0, 0, QByteArray("x")));
  QVERIFY(wl.sendClose(2, 0, 0));
  QCOMPARE(sent.size(), 3);
  allow = false;
  QVERIFY(!wl.sendData(0, 0, 0, QByteArray("x")));
}

namespace {
// A dialog with a fit ready: 16-byte blob from the bench points.
void fitBench(BatteryCalibDialog &d) {
  d.setLive(1091, true);
  d.setVolts(0, 12.46);
  QVERIFY(d.capture(0));
  d.setLive(698, true);
  d.setVolts(1, 7.95);
  QVERIFY(d.capture(1));
}
} // namespace

void TstXferUploader::dialogUploadsBlob() {
  FakePeer peer(this);
  XferUploader up(&peer, fastCfg());
  peer.up = &up;
  BatteryCalibDialog d;
  d.setUploader(&up);
  d.setUploadAvailable(true);
  fitBench(d);
  QVERIFY(d.fit().ok);
  QVERIFY(d.uploadEnabled());
  QSignalSpy done(&up, &XferUploader::finished);
  QVERIFY(d.startUpload());
  QVERIFY(d.uploading());
  QVERIFY(waitDone(done).got);
  QTRY_VERIFY_WITH_TIMEOUT(!d.uploading(), kWaitMs);
  QCOMPARE(peer.openPath, QByteArray(batcal::kCardPath));
  QCOMPARE(peer.file, d.blob());
  QCOMPARE(peer.file.size(), batcal::kBlobSize);
  QVERIFY2(d.uploadStatus().contains("Reboot"), qPrintable(d.uploadStatus()));
  QVERIFY(d.uploadEnabled()); // can be repeated
}

void TstXferUploader::dialogGatedByAvailability() {
  FakePeer peer(this);
  XferUploader up(&peer, fastCfg());
  peer.up = &up;
  BatteryCalibDialog d;
  // No uploader, no fit, no link: disabled, and start refuses.
  QVERIFY(!d.uploadEnabled());
  QVERIFY(!d.startUpload());
  d.setUploader(&up);
  fitBench(d);
  QVERIFY(!d.uploadEnabled()); // fit ok but no link allowed yet
  QVERIFY(!d.startUpload());
  QCOMPARE(peer.opens, 0);
  d.setUploadAvailable(false, "Read-only while replaying a log.");
  QVERIFY(!d.uploadEnabled());
  QVERIFY(!d.startUpload());
  QCOMPARE(peer.opens, 0);
  d.setUploadAvailable(true);
  QVERIFY(d.uploadEnabled());
  // Fit goes bad: disabled again.
  d.setVolts(1, 12.46); // equal volts at different counts: refused
  d.capture(1);
  QVERIFY(!d.fit().ok);
  QVERIFY(!d.uploadEnabled());
  QVERIFY(!d.startUpload());
  QCOMPARE(peer.opens, 0);
}

void TstXferUploader::dialogCancelsWhenLinkLost() {
  FakePeer peer(this);
  XferUploader::Config c = fastCfg();
  c.windowBytes = 300;
  XferUploader up(&peer, c);
  peer.up = &up;
  peer.silent = true; // stays in the open phase
  BatteryCalibDialog d;
  d.setUploader(&up);
  d.setUploadAvailable(true);
  fitBench(d);
  QVERIFY(d.startUpload());
  QVERIFY(d.uploading());
  QVERIFY(up.busy());
  d.setUploadAvailable(false, "Connect to the vehicle first.");
  QVERIFY(!up.busy());
  QVERIFY(!d.uploading());
  QVERIFY(d.uploadStatus().contains("cancelled"));
  QVERIFY(!d.uploadEnabled());
}

void TstXferUploader::dialogShowsUnconfirmed() {
  FakePeer peer(this);
  peer.loseDone = true;
  XferUploader up(&peer, fastCfg());
  peer.up = &up;
  BatteryCalibDialog d;
  d.setUploader(&up);
  d.setUploadAvailable(true);
  fitBench(d);
  QSignalSpy done(&up, &XferUploader::finished);
  QVERIFY(d.startUpload());
  QVERIFY(waitDone(done).got);
  QTRY_VERIFY_WITH_TIMEOUT(!d.uploading(), kWaitMs);
  const QString t = d.uploadStatus();
  QVERIFY2(t.contains("did not confirm"), qPrintable(t));
  QVERIFY(t.contains("Reboot"));
  QVERIFY(!t.contains("failed"));
  QCOMPARE(peer.file, d.blob());
}

void TstXferUploader::dialogShowsFailure() {
  FakePeer peer(this);
  peer.openResult = xfer::kResDenied;
  XferUploader up(&peer, fastCfg());
  peer.up = &up;
  BatteryCalibDialog d;
  d.setUploader(&up);
  d.setUploadAvailable(true);
  fitBench(d);
  QSignalSpy done(&up, &XferUploader::finished);
  QVERIFY(d.startUpload());
  QVERIFY(waitDone(done).got);
  QTRY_VERIFY_WITH_TIMEOUT(!d.uploading(), kWaitMs);
  QVERIFY2(d.uploadStatus().contains("failed"), qPrintable(d.uploadStatus()));
  QVERIFY(d.uploadStatus().contains("denied"));
  QVERIFY(d.uploadEnabled()); // retry is possible
  // Save-to-file still works alongside.
  QTemporaryDir dir;
  QVERIFY(dir.isValid());
  QVERIFY(d.saveTo(dir.filePath("batcal.bin")));
}

QTEST_MAIN(TstXferUploader)
#include "tst_xfer_uploader.moc"
