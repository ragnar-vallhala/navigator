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
// AckText.h - what to tell the pilot about a COMMAND_ACK.
//
// The ACK stream is only logged as "cmd 8192 #5 -> DENIED". For the commands a
// pilot waits on (ARM, DISARM, flight mode) a refusal must be seen, not found in
// the log: ackFailure() returns the sentence for a refusal and an empty string
// when there is nothing to say.
#pragma once

#include <QString>
#include <QtGlobal>

extern "C" {
#include "navlink_msgs.h"
}

namespace acktext {

// COMMAND_ACK.result (spec section 17): 0 accepted, 1 temporarily rejected,
// 2 denied, 3 unsupported, 4 failed, 5 in progress.
inline QString resultName(quint8 result) {
  static const char *const kRes[] = {"accepted", "temporarily rejected", "denied",
                                     "unsupported", "failed", "in progress"};
  return result < 6 ? QString::fromLatin1(kRes[result])
                    : QStringLiteral("result %1").arg(result);
}

// "ARM refused by the vehicle (denied)" for a refused ARM / DISARM / flight
// mode command; empty for accepted / in progress and for other commands.
inline QString ackFailure(quint32 command, quint8 result) {
  if (result == 0 || result == 5)
    return {};
  const char *what = nullptr;
  switch (command) {
  case NAVLINK_MSGID_CMD_ARM:
    what = "ARM";
    break;
  case NAVLINK_MSGID_CMD_DISARM:
    what = "DISARM";
    break;
  case NAVLINK_MSGID_CMD_SET_FLIGHT_MODE:
    what = "Flight mode change";
    break;
  default:
    return {};
  }
  return QStringLiteral("%1 refused by the vehicle (%2)")
      .arg(QString::fromLatin1(what), resultName(result));
}

} // namespace acktext
