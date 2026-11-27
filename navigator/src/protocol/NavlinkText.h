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
// NavlinkText.h - the one way to turn a NavLink char[N] field into a QString.
//
// The fixed-size text fields (STATUSTEXT.text[50], FAULT_STATUS.text[50],
// PERF_TASKNAME.name[32], PARAM_*.param_id[16], ...) are NUL-padded, not
// NUL-terminated: a field that fills its array has no terminator at all (a
// 16-character parameter id uses all 16 bytes). strlen(), fromUtf8(ptr) and
// fromLatin1(ptr) would read on into whatever follows. The length bound here is
// taken from the array type itself, so it cannot be the wrong constant.
#pragma once

#include <QString>
#include <QtGlobal>

#include <cstddef>

namespace navtext {

// Characters of `f` before the first NUL, at most N.
template <std::size_t N> inline int fieldLength(const char (&f)[N]) {
  return static_cast<int>(qstrnlen(f, static_cast<qsizetype>(N)));
}

// The field as text (Latin-1, as the firmware writes it), never more than N
// characters.
template <std::size_t N> inline QString fieldLatin1(const char (&f)[N]) {
  return QString::fromLatin1(f, fieldLength(f));
}

} // namespace navtext
