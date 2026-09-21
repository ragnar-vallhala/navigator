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
#include "SettingsManager.h"

const QString SettingsManager::FileName = "vayu_settings.dat";

bool SettingsManager::saveToPath(const QString &path, const GcsSettings &s) {
  QFile file(path);
  if (!file.open(QIODevice::WriteOnly)) {
    return false;
  }

  QDataStream out(&file);
  out << Magic;
  out << Version;
  out << s;

  return true;
}

bool SettingsManager::loadFromPath(const QString &path, GcsSettings &s) {
  QFile file(path);
  if (!file.exists() || !file.open(QIODevice::ReadOnly)) {
    return false;
  }

  QDataStream in(&file);
  quint32 magic;
  int version;
  in >> magic;
  in >> version;

  if (magic != Magic || version > Version) {
    return false;
  }

  in >> s;
  return true;
}

bool SettingsManager::save(const GcsSettings &s) {
  return saveToPath(FileName, s);
}

bool SettingsManager::load(GcsSettings &s) { return loadFromPath(FileName, s); }
