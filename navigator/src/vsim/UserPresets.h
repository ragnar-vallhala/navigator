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

#include <QJsonObject>
#include <QString>
#include <QStringList>

namespace vsim {

// User-created named presets: one file per preset, holding exactly the portable
// JSON the editors already export (.vveh for a vehicle, .vworld for a world),
// so a preset file can also be loaded through Load Vehicle / Load. The preset's
// name is the file's base name.
//
//   <root>/vehicles/<name>.vveh
//   <root>/worlds/<name>.vworld
//
// Pure Qt core: no widgets, no engine.
class UserPresetStore {
public:
  enum class Kind { Vehicle, World };
  enum class SaveResult {
    Ok,
    InvalidName, // empty, too long, separators, '..', reserved characters
    Reserved,    // equals a built-in preset id or name (never shadowed)
    Exists,      // a preset of that name (any case) exists and !overwrite
    WriteFailed,
  };

  static constexpr int kMaxNameLength = 48;

  // Default root: QStandardPaths::AppDataLocation/presets.
  static QString defaultRoot();

  explicit UserPresetStore(const QString &root = defaultRoot());

  // Built-in ids and display names a user preset may not take (compared case
  // insensitively). The store rejects, it does not suffix.
  void setReserved(Kind kind, const QStringList &idsAndNames);

  // Trimmed name, or an empty string with *err set when the name is unusable.
  static QString sanitize(const QString &name, QString *err = nullptr);

  // Saves (or, with overwrite, replaces the same-named preset in any case).
  // *savedName gets the name as stored.
  SaveResult save(Kind kind, const QString &name, const QJsonObject &data,
                  bool overwrite = false, QString *savedName = nullptr);
  // Names, sorted case-insensitively. Unreadable / corrupt / badly named files
  // are skipped with a log line.
  QStringList list(Kind kind) const;
  // Case-insensitive name match. False when missing or the file is corrupt.
  bool load(Kind kind, const QString &name, QJsonObject *out) const;
  bool exists(Kind kind, const QString &name) const;
  bool remove(Kind kind, const QString &name);

  static QString saveResultText(SaveResult r);

private:
  QString dirFor(Kind kind) const;
  static const char *extFor(Kind kind);
  QString findFile(Kind kind, const QString &name) const; // "" if none
  bool isReserved(Kind kind, const QString &name) const;
  bool readFile(const QString &path, QJsonObject *out) const;

  QString root_;
  QStringList reservedVehicle_;
  QStringList reservedWorld_;
};

} // namespace vsim
