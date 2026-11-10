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
// Where the in-process firmware keeps its host VFS (persisted gains, 0:pid.bin).
// The firmware reads $VAYU_VFS_DIR, defaulting to the machine-shared
// /tmp/vayu_vfs; the app gives itself a private per-user directory instead so
// other processes' (or tests') files can never change how the sim flies.
#include <QString>

namespace vsim {

struct VfsChoice {
  QString dir;           // directory the firmware will use
  bool fromUser = false; // VAYU_VFS_DIR was already set by the user
  bool ok = true;        // directory exists / was created
};

// Pure resolver: a non-empty userEnv wins untouched; otherwise
// <appDataRoot>/vfs, which is created.
VfsChoice resolveVfsDir(const QString &userEnv, const QString &appDataRoot);

// Reads VAYU_VFS_DIR, resolves against QStandardPaths::AppDataLocation, exports
// the result into the environment and logs one line. Call before the engine
// module is loaded. Returns the choice made.
VfsChoice applyVfsEnvironment();

// The app-chosen private dir (<AppData>/vfs), regardless of the environment.
QString privateVfsDir();

// Deletes the contents of `dir` (not the dir itself). Refuses, returning false
// and filling *why, if the user chose the dir (userSet), or if `dir` is not
// strictly inside appDataRoot after resolving symlinks.
bool resetPrivateVfs(const QString &dir, const QString &appDataRoot,
                     bool userSet, QString *why);

} // namespace vsim
