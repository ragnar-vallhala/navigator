// CommandRetrier: keeps the simulator's Start-time commands reliable.
//
// The in-app firmware drops every command (TEMPORARILY_REJECTED) until the
// time-sync handshake completes, so what Start pushes (the motor-mix geometry
// 8197, flight mode 8198, and whatever else registers: e.g. an airframe gain
// set, CMD_SET_PID 8195) can silently not land. Each push is registered here
// under a key with the command's msgid and a send function. The retrier
//   - sends it when the link is synced (arm() while unsynced defers; the
//     synced edge sends everything pending),
//   - re-sends every PendingCommand::kRetryMs while pending (the safety net: it
//     does not trust the sync flag, which can be stale across a sim restart),
//   - ends on ACK OK, treats TEMPORARILY_REJECTED as "not applied, still
//     pending", logs a refusal (DENIED/UNSUPPORTED/FAILED) once,
//   - gives up after PendingCommand::kGiveUpMs with ONE visible warning.
// One pending entry per command msgid (acks carry the msgid, not the key).
// Pure and clock-free: the caller passes `nowMs` and drives tick().
#pragma once

#include "PendingCommand.h"

#include <QMap>
#include <QString>
#include <functional>

class CommandRetrier {
public:
  using Send = std::function<void()>;
  using Log = std::function<void(const QString &)>;

  void setLog(Log log) { m_log = std::move(log); }

  // Register a push of `expected` ACK-OK frames of command `msgid`. `label`
  // names it in the log; `consequence` finishes the give-up warning. Re-arming
  // an existing key starts a fresh round. Sends now only if synced.
  // Returns pending() for the key (false when expected<=0: nothing to wait for).
  bool arm(const QString &key, quint32 msgid, int expected, const QString &label,
           const QString &consequence, Send send, qint64 nowMs) {
    Entry &e = m_entries[key];
    e.msgid = msgid;
    e.label = label;
    e.consequence = consequence;
    e.send = std::move(send);
    e.st.begin(expected, nowMs);
    if (e.st.pending() && m_synced && e.send)
      e.send();
    return e.st.pending();
  }

  // The time-sync state. On the rising edge every pending entry is sent now.
  void setSynced(bool synced, qint64 nowMs) {
    const bool rose = synced && !m_synced;
    m_synced = synced;
    if (!rose)
      return;
    for (Entry &e : m_entries)
      if (e.st.pending() && e.send) {
        e.send();
        e.st.noteSent(nowMs);
      }
  }

  // An ack for command `msgid` arrived.
  void onAck(quint32 msgid, int result) {
    for (Entry &e : m_entries) {
      if (e.msgid != msgid || !e.st.pending())
        continue;
      const int retries = e.st.retries(), expected = e.st.expected();
      switch (e.st.onAck(result)) {
      case PendingCommand::Ack::Accepted:
        log(QStringLiteral("firmware accepted %1 (%2/%2 ACK OK%3)")
                .arg(e.label)
                .arg(expected)
                .arg(retries ? QStringLiteral(", after %1 resend(s)").arg(retries)
                             : QString()));
        break;
      case PendingCommand::Ack::Failed:
        log(QStringLiteral("firmware refused %1 (result %2); %3")
                .arg(e.label)
                .arg(e.st.lastError())
                .arg(e.consequence));
        break;
      case PendingCommand::Ack::None:
        break;
      }
      return;
    }
  }

  // ~1 Hz. Re-sends what is still pending; logs a give-up once.
  void tick(qint64 nowMs) {
    for (Entry &e : m_entries) {
      switch (e.st.tick(nowMs)) {
      case PendingCommand::Action::Resend:
        if (e.send)
          e.send(); // quiet: one log line per round, not per retry
        break;
      case PendingCommand::Action::GaveUp:
        log(QStringLiteral("firmware did not acknowledge %1; %2")
                .arg(e.label)
                .arg(e.consequence));
        break;
      case PendingCommand::Action::None:
        break;
      }
    }
  }

  void cancel(const QString &key) {
    auto it = m_entries.find(key);
    if (it != m_entries.end())
      it->st.cancel();
  }
  void cancelAll() {
    for (Entry &e : m_entries)
      e.st.cancel();
  }

  bool pending(const QString &key) const {
    auto it = m_entries.constFind(key);
    return it != m_entries.constEnd() && it->st.pending();
  }
  bool anyPending() const {
    for (const Entry &e : m_entries)
      if (e.st.pending())
        return true;
    return false;
  }
  const PendingCommand *state(const QString &key) const {
    auto it = m_entries.constFind(key);
    return it == m_entries.constEnd() ? nullptr : &it->st;
  }
  bool synced() const { return m_synced; }

private:
  struct Entry {
    PendingCommand st;
    quint32 msgid = 0;
    QString label, consequence;
    Send send;
  };
  void log(const QString &s) {
    if (m_log)
      m_log(s);
  }
  QMap<QString, Entry> m_entries;
  Log m_log;
  bool m_synced = false;
};
