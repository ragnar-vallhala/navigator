// GainPushTracker: the pending / retry / acknowledge state machine behind the
// simulator's airframe-gain push. Pure (no Qt event loop, no clock of its own):
// the caller passes `nowMs` and drives tick(), so it is unit-testable headless.
//
// Why it exists: the firmware drops (TEMPORARILY_REJECTED) every command until
// its clock is disciplined by the time-sync handshake, so a CMD_SET_PID sent
// right at "online" can be lost and the sim then flies stock gains. The push is
// therefore re-sent until the firmware ACKs it.
//
// Acknowledgement rule (honest, not per-slot): CommandCodec::encodeSetPid does
// not expose the req_seq it used, so acks cannot be matched to individual
// slots. A round counts as accepted when `expected` ACK-OK results for
// CMD_SET_PID have arrived since the LAST (re)send. A concurrent SET_PID from
// elsewhere (e.g. an autotune Apply) could be counted; the gate is all-or-
// nothing, so in practice either every slot of a round is accepted or none is.
#pragma once

#include <QString>
#include <QtGlobal>

class GainPushTracker {
public:
  static constexpr qint64 kRetryMs = 1000;  // re-send period while pending
  static constexpr qint64 kGiveUpMs = 30000; // stop retrying after this long
  static constexpr int kResultOk = 0;        // NAVLINK_COMMAND_RESULT_ACCEPTED

  enum class Action { None, Resend, GaveUp };

  // A (re-)armed round for preset `id` whose gain set has `expected` slots.
  // Empty id (custom vehicle) or expected<=0 disarms: nothing is ever pending.
  void begin(const QString &id, int expected, qint64 nowMs) {
    cancel();
    m_accepted = m_failed = false;
    if (id.isEmpty() || expected <= 0)
      return;
    m_id = id;
    m_expected = expected;
    m_pending = true;
    m_startMs = m_lastSendMs = nowMs;
  }

  // The caller sent the gain set outside tick() (e.g. a belt-and-suspenders
  // re-send): acks are counted from here.
  void noteSent(qint64 nowMs) {
    if (!m_pending)
      return;
    m_lastSendMs = nowMs;
    m_ok = 0;
  }

  // Sim stopped, custom vehicle, or preset changed. Does not clear failed():
  // callers use it to log the give-up once.
  void cancel() {
    m_pending = false;
    m_ok = m_rejected = m_retries = 0;
    m_id.clear();
    m_expected = 0;
  }

  // One CMD_SET_PID ack. Returns true exactly once per round, when the round
  // becomes fully accepted (so the caller logs it once).
  bool onAck(int result, qint64 nowMs) {
    Q_UNUSED(nowMs);
    if (!m_pending)
      return false;
    if (result != kResultOk) {
      ++m_rejected;
      return false;
    }
    if (++m_ok < m_expected)
      return false;
    m_pending = false;
    m_accepted = true;
    return true;
  }

  // Poll (about once a second). Resend: the caller must send the gain set
  // again. GaveUp is returned once, after which nothing is pending.
  Action tick(qint64 nowMs) {
    if (!m_pending)
      return Action::None;
    if (nowMs - m_startMs >= kGiveUpMs) {
      m_pending = false;
      m_failed = true;
      return Action::GaveUp;
    }
    if (nowMs - m_lastSendMs >= kRetryMs) {
      m_lastSendMs = nowMs;
      m_ok = 0; // count acks since the last send
      ++m_retries;
      return Action::Resend;
    }
    return Action::None;
  }

  bool pending() const { return m_pending; }
  bool accepted() const { return m_accepted; }
  bool failed() const { return m_failed; }
  int okCount() const { return m_ok; }
  int rejectedCount() const { return m_rejected; }
  int retries() const { return m_retries; }
  int expected() const { return m_expected; }
  const QString &id() const { return m_id; }

private:
  QString m_id;
  int m_expected = 0, m_ok = 0, m_rejected = 0, m_retries = 0;
  bool m_pending = false, m_accepted = false, m_failed = false;
  qint64 m_startMs = 0, m_lastSendMs = 0;
};
