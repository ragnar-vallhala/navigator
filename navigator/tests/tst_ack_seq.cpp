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
// NavLink 3.0: the COMMAND_ACK the parser emits takes its frame seq from
// handlers.next_seq. The GCS wires that to the same counter every frame it
// transmits uses (TxSeq), so ACKs and own frames form one sequence.
#include "CommandCodec.h"
#include "NavlinkRouter.h"
#include "TxSeq.h"

#include <QtTest>
#include <atomic>
#include <thread>
#include <vector>

extern "C" {
#include "navlink_msgs.h"
}

namespace {
struct Seen {
  uint32_t msgid = 0;
  uint8_t seq = 0;
  bool ok = false;
};

// Decode one whole frame and report its msgid and frame seq (any message).
Seen inspect(const QByteArray &f) {
  struct Ctx {
    Seen s;
  } c;
  navlink_handlers_t h{};
  h.ctx = &c;
  h.on_default = [](void *ctx, const navlink_frame_hdr_t *hdr, uint32_t msgid,
                    const uint8_t *, size_t) {
    auto *x = static_cast<Ctx *>(ctx);
    x->s.msgid = msgid;
    x->s.seq = hdr->seq;
    x->s.ok = true;
  };
  navlink_parser_t p;
  navlink_parser_init(&p);
  navlink_parser_push(&p, &h, reinterpret_cast<const uint8_t *>(f.constData()),
                      static_cast<size_t>(f.size()));
  return c.s;
}

// A CMD_ARM request from the vehicle side, to be answered by the GCS parser.
// The router has no on_cmd_arm slot, so the parser answers UNSUPPORTED itself.
QByteArray peerRequest(uint8_t reqSeq, uint8_t frameSeq) {
  navlink_cmd_arm_t m{};
  m.target_sys = 0xFF;
  m.target_comp = 1;
  m.req_seq = reqSeq;
  uint8_t buf[NAVLINK_MAX_FRAME];
  const size_t n = navlink_cmd_arm_encode(buf, &m, frameSeq, 1, 1);
  return QByteArray(reinterpret_cast<const char *>(buf), static_cast<int>(n));
}
} // namespace

class TstAckSeq : public QObject {
  Q_OBJECT
private slots:
  void acksAndOwnFramesShareOneCounter();
  void noSinkMeansNoAckAndNoSeqConsumed();
  void counterIsSafeAcrossThreads();
};

void TstAckSeq::acksAndOwnFramesShareOneCounter() {
  NavlinkRouter router;
  QList<QByteArray> sent;
  router.setSendFrame([&](const QByteArray &f) { sent.append(f); });

  TxSeq::setNext(253);
  // 253 and 254: frames the GCS transmits itself.
  const Seen own1 = inspect(CommandCodec::encodeTaskNameRequest(3));
  const Seen own2 = inspect(CommandCodec::encodeDisarm());
  QVERIFY(own1.ok && own2.ok);
  QCOMPARE(int(own1.seq), 253);
  QCOMPARE(int(own2.seq), 254);

  // The vehicle's request carries req_seq 77 in its payload and frame seq 5.
  // The ACK must take 255 from the shared counter, not echo 77 or 5.
  router.feed(peerRequest(77, 5));
  QCOMPARE(sent.size(), 1);
  const Seen ack = inspect(sent.at(0));
  QVERIFY(ack.ok);
  QCOMPARE(ack.msgid, uint32_t(NAVLINK_MSGID_COMMAND_ACK));
  QCOMPARE(int(ack.seq), 255);

  // req_seq still correlates request and ack, in the payload.
  struct Got {
    bool ok = false;
    uint8_t reqSeq = 0;
  } got;
  navlink_handlers_t h{};
  h.ctx = &got;
  h.on_command_ack = [](void *ctx, const navlink_frame_hdr_t *,
                        const navlink_command_ack_t *m) {
    auto *g = static_cast<Got *>(ctx);
    g->ok = true;
    g->reqSeq = m->req_seq;
  };
  navlink_parser_t p;
  navlink_parser_init(&p);
  navlink_parser_push(&p, &h,
                      reinterpret_cast<const uint8_t *>(sent.at(0).constData()),
                      static_cast<size_t>(sent.at(0).size()));
  QVERIFY(got.ok);
  QCOMPARE(int(got.reqSeq), 77);

  // The next own frame continues the same sequence and wraps to 0.
  const Seen own3 = inspect(CommandCodec::encodeArm());
  QVERIFY(own3.ok);
  QCOMPARE(int(own3.seq), 0);
}

void TstAckSeq::noSinkMeansNoAckAndNoSeqConsumed() {
  NavlinkRouter router; // no setSendFrame: parser emits nothing
  TxSeq::setNext(10);
  router.feed(peerRequest(1, 2));
  const Seen own = inspect(CommandCodec::encodeArm());
  QVERIFY(own.ok);
  QCOMPARE(int(own.seq), 10);
}

// next_seq runs on the thread that feeds the parser while other threads encode
// commands: hammer the counter from several threads; every value must be handed
// out equally often (no duplicate, none skipped).
void TstAckSeq::counterIsSafeAcrossThreads() {
  constexpr int kThreads = 4;
  constexpr int kRounds = 256 * 40; // calls per thread, a multiple of 256
  TxSeq::setNext(0);
  std::vector<std::vector<uint8_t>> out(kThreads);
  std::vector<std::thread> th;
  std::atomic<bool> go{false};
  for (int t = 0; t < kThreads; ++t) {
    out[t].reserve(kRounds);
    th.emplace_back([&, t] {
      while (!go.load()) {
      }
      for (int i = 0; i < kRounds; ++i)
        out[t].push_back(TxSeq::next());
    });
  }
  go = true;
  for (auto &x : th)
    x.join();
  std::vector<int> count(256, 0);
  for (const auto &v : out)
    for (uint8_t s : v)
      ++count[s];
  for (int s = 0; s < 256; ++s)
    QCOMPARE(count[s], kThreads * kRounds / 256);
}

QTEST_APPLESS_MAIN(TstAckSeq)
#include "tst_ack_seq.moc"
