// PendingCommand: the pending / retry / acknowledge state for ONE command (or
// one batch of identical-command frames) sent to the firmware. Pure: no event
// loop, no clock; the caller passes `nowMs` and drives tick(), so it is
// unit-testable headless. CommandRetrier owns a set of these.
//
// Why: the firmware answers TEMPORARILY_REJECTED to EVERY command until the
// GCS completes the time-sync handshake, and never queues or replays it
// (router_command_gate, SYS-SAFE-007). A command sent before sync is simply
// lost, so it must be re-issued until the firmware ACKs it.
//
// Acknowledgement rule (honest, not per-frame): the codec does not expose the
// req_seq it used, so acks cannot be matched to individual frames. A round
// counts as accepted when `expected` ACK-OK results for the command arrived
// since the LAST (re)send. The gate is all-or-nothing, so in practice every
// frame of a round is accepted or none is.
#pragma once

#include <QString>
#include <QtGlobal>

class PendingCommand {
public:
  static constexpr qint64 kRetryMs = 1000;   // re-send period while pending
  static constexpr qint64 kGiveUpMs = 30000; // stop retrying after this long
  static constexpr int kResultOk = 0;        // NAVLINK_COMMAND_RESULT_ACCEPTED
  static constexpr int kResultTempRejected = 1; // the pre-sync gate

  enum class Action { None, Resend, GaveUp };
  enum class Ack { None, Accepted, Failed };

  // (Re-)arm a round expecting `expected` ACK-OK results. expected<=0 disarms:
  // nothing is ever pending.
  void begin(int expected, qint64 nowMs) {
    cancel();
    m_accepted = m_failed = false;
    if (expected <= 0)
      return;
    m_expected = expected;
    m_pending = true;
    m_startMs = m_lastSendMs = nowMs;
  }

  // Drop the round (sim stopped, preset changed, ...). Does not clear
  // accepted()/failed() so callers can still read the last outcome.
  void cancel() {
    m_pending = false;
    m_ok = m_rejected = m_retries = 0;
    m_expected = 0;
  }

  // The caller sent the frames outside tick(): acks are counted from here.
  void noteSent(qint64 nowMs) {
    if (!m_pending)
      return;
    m_lastSendMs = nowMs;
    m_ok = 0;
  }

  // One ack for this command. TEMPORARILY_REJECTED (1): not applied, stays
  // pending (the caller re-sends). IN_PROGRESS (5): ignored. DENIED /
  // UNSUPPORTED / FAILED (2..4): the round fails. Accepted / Failed are each
  // returned once per round, so the caller logs once.
  Ack onAck(int result) {
    if (!m_pending)
      return Ack::None;
    if (result == kResultOk) {
      if (++m_ok < m_expected)
        return Ack::None;
      m_pending = false;
      m_accepted = true;
      return Ack::Accepted;
    }
    if (result == kResultTempRejected) {
      ++m_rejected;
      return Ack::None;
    }
    if (result >= 2 && result <= 4) {
      m_pending = false;
      m_failed = true;
      m_lastError = result;
      return Ack::Failed;
    }
    return Ack::None;
  }

  // Poll (about once a second). Resend: the caller must send again. GaveUp is
  // returned once, after which nothing is pending.
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
  int lastError() const { return m_lastError; }

private:
  int m_expected = 0, m_lastError = 0, m_ok = 0, m_rejected = 0, m_retries = 0;
  bool m_pending = false, m_accepted = false, m_failed = false;
  qint64 m_startMs = 0, m_lastSendMs = 0;
};
