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

namespace vsim {

// Where the simulator writes its per-run ground-truth logs when the user has
// not chosen a directory: $VAYU_REPO/logs if VAYU_REPO is set, otherwise a
// stable per-user location (<AppData>/logs, e.g. ~/.local/share/<app>/logs).
// Deliberately NOT derived from the executable's location: that resolved to
// whichever navigator checkout or git worktree happened to be built.
QString defaultLogDir();

} // namespace vsim
