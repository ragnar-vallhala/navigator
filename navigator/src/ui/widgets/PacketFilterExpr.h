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

#include <QString>
#include <functional>

/**
 * Per-row context the expression is evaluated against (filled from a
 * PacketEntry by the proxy).
 */
struct FilterCtx {
  int type = 0xFF;  // type nibble, 0xFF = RAW
  QString typeName; // lower-cased, e.g. "attitude"
  bool tx = false;
  int dev = 0;
  int len = 0;
  int origin = -1; // SYSTEM_STATUS origin, else -1
  QString info;    // lower-cased one-line summary
};

/**
 * Wireshark-style display-filter: `type==attitude && dev==42 && size>50`.
 * Fields: type, dir (rx/tx), dev, size/len, origin, info.
 * Ops: == != < <= > >= contains, combined with && || ! ( ).
 * Compiles to a predicate; fails safe (an invalid filter is inert, not fatal).
 */
class PacketFilterExpr {
public:
  bool compile(const QString &src); // true if usable (empty counts as usable)
  bool empty() const { return m_empty; }
  bool valid() const { return m_valid; }
  QString error() const { return m_err; }
  bool eval(const FilterCtx &c) const {
    return m_empty || (m_valid && m_pred && m_pred(c));
  }

private:
  std::function<bool(const FilterCtx &)> m_pred;
  bool m_valid = true;
  bool m_empty = true;
  QString m_err;
};
