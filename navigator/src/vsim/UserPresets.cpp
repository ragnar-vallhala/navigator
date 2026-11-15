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
#include "UserPresets.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QSaveFile>
#include <QStandardPaths>
#include <QDebug>

namespace vsim {

namespace {
using Kind = UserPresetStore::Kind;

bool sameName(const QString &a, const QString &b) {
  return a.compare(b, Qt::CaseInsensitive) == 0;
}

bool isWindowsDeviceName(const QString &n) {
  static const QStringList dev = {"con", "prn", "aux", "nul", "com1", "com2",
                                  "com3", "com4", "com5", "com6", "com7",
                                  "com8", "com9", "lpt1", "lpt2", "lpt3",
                                  "lpt4", "lpt5", "lpt6", "lpt7", "lpt8",
                                  "lpt9"};
  return dev.contains(n.toLower());
}
} // namespace

QString UserPresetStore::defaultRoot() {
  return QDir(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation))
      .filePath(QStringLiteral("presets"));
}

UserPresetStore::UserPresetStore(const QString &root) : root_(root) {}

void UserPresetStore::setReserved(Kind kind, const QStringList &idsAndNames) {
  (kind == Kind::Vehicle ? reservedVehicle_ : reservedWorld_) = idsAndNames;
}

QString UserPresetStore::sanitize(const QString &name, QString *err) {
  auto fail = [err](const QString &why) {
    if (err)
      *err = why;
    return QString();
  };
  const QString n = name.trimmed();
  if (n.isEmpty())
    return fail(QStringLiteral("The name is empty."));
  if (n.size() > kMaxNameLength)
    return fail(QStringLiteral("The name is longer than %1 characters.")
                    .arg(kMaxNameLength));
  if (n.contains(QStringLiteral("..")))
    return fail(QStringLiteral("The name may not contain '..'."));
  if (n.startsWith('.') || n.endsWith('.'))
    return fail(QStringLiteral("The name may not start or end with '.'."));
  static const QString bad = QStringLiteral("/\\:*?\"<>|");
  for (const QChar c : n) {
    if (bad.contains(c) || c.unicode() < 0x20 || c.unicode() == 0x7f)
      return fail(QStringLiteral("The name contains a character that is not "
                                 "allowed in a file name (/ \\ : * ? \" < > |)."));
  }
  if (isWindowsDeviceName(n))
    return fail(QStringLiteral("'%1' is a reserved system name.").arg(n));
  return n;
}

QString UserPresetStore::dirFor(Kind kind) const {
  return QDir(root_).filePath(kind == Kind::Vehicle ? QStringLiteral("vehicles")
                                                    : QStringLiteral("worlds"));
}

const char *UserPresetStore::extFor(Kind kind) {
  return kind == Kind::Vehicle ? "vveh" : "vworld";
}

bool UserPresetStore::isReserved(Kind kind, const QString &name) const {
  const QStringList &r = kind == Kind::Vehicle ? reservedVehicle_ : reservedWorld_;
  for (const QString &s : r)
    if (sameName(s, name))
      return true;
  // "Custom" is the combo's own first entry in both pages.
  return sameName(name, QStringLiteral("Custom"));
}

bool UserPresetStore::readFile(const QString &path, QJsonObject *out) const {
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly)) {
    qWarning().noquote() << "[presets] skipping unreadable file" << path << ":"
                         << f.errorString();
    return false;
  }
  QJsonParseError pe;
  const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &pe);
  if (pe.error != QJsonParseError::NoError || !doc.isObject()) {
    qWarning().noquote() << "[presets] skipping corrupt file" << path;
    return false;
  }
  if (out)
    *out = doc.object();
  return true;
}

QStringList UserPresetStore::list(Kind kind) const {
  QStringList names;
  const QDir d(dirFor(kind));
  const QString pat = QStringLiteral("*.") + QLatin1String(extFor(kind));
  const auto files = d.entryInfoList({pat}, QDir::Files | QDir::Readable);
  for (const QFileInfo &fi : files) {
    const QString name = fi.completeBaseName();
    QString why;
    if (sanitize(name, &why) != name) {
      qWarning().noquote() << "[presets] skipping" << fi.filePath() << ":" << why;
      continue;
    }
    if (isReserved(kind, name)) {
      qWarning().noquote() << "[presets] skipping" << fi.filePath()
                           << ": name clashes with a built-in preset";
      continue;
    }
    if (!readFile(fi.filePath(), nullptr))
      continue;
    bool dup = false;
    for (const QString &n : names)
      dup = dup || sameName(n, name);
    if (dup) {
      qWarning().noquote() << "[presets] skipping" << fi.filePath()
                           << ": duplicate name (case)";
      continue;
    }
    names << name;
  }
  names.sort(Qt::CaseInsensitive);
  return names;
}

QString UserPresetStore::findFile(Kind kind, const QString &name) const {
  const QString n = name.trimmed();
  const QDir d(dirFor(kind));
  const auto files = d.entryInfoList(
      {QStringLiteral("*.") + QLatin1String(extFor(kind))}, QDir::Files);
  for (const QFileInfo &fi : files)
    if (sameName(fi.completeBaseName(), n))
      return fi.filePath();
  return QString();
}

bool UserPresetStore::exists(Kind kind, const QString &name) const {
  return !findFile(kind, name).isEmpty();
}

bool UserPresetStore::load(Kind kind, const QString &name,
                           QJsonObject *out) const {
  if (sanitize(name).isEmpty())
    return false;
  const QString path = findFile(kind, name);
  return !path.isEmpty() && readFile(path, out);
}

UserPresetStore::SaveResult UserPresetStore::save(Kind kind, const QString &name,
                                                  const QJsonObject &data,
                                                  bool overwrite,
                                                  QString *savedName) {
  const QString n = sanitize(name);
  if (n.isEmpty())
    return SaveResult::InvalidName;
  if (isReserved(kind, n))
    return SaveResult::Reserved;
  const QString old = findFile(kind, n);
  if (!old.isEmpty() && !overwrite)
    return SaveResult::Exists;
  if (!QDir().mkpath(dirFor(kind)))
    return SaveResult::WriteFailed;
  const QString path =
      QDir(dirFor(kind)).filePath(n + QLatin1Char('.') + QLatin1String(extFor(kind)));
  QSaveFile f(path);
  if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate))
    return SaveResult::WriteFailed;
  f.write(QJsonDocument(data).toJson(QJsonDocument::Indented));
  if (!f.commit())
    return SaveResult::WriteFailed;
  // A same-name file in another case is replaced, not left beside the new one.
  if (!old.isEmpty() && QFileInfo(old).fileName() != QFileInfo(path).fileName())
    QFile::remove(old);
  if (savedName)
    *savedName = n;
  return SaveResult::Ok;
}

bool UserPresetStore::remove(Kind kind, const QString &name) {
  const QString path = findFile(kind, name);
  return !path.isEmpty() && QFile::remove(path);
}

QString UserPresetStore::saveResultText(SaveResult r) {
  switch (r) {
  case SaveResult::Ok:
    return QStringLiteral("Saved.");
  case SaveResult::InvalidName:
    return QStringLiteral("That name cannot be used.");
  case SaveResult::Reserved:
    return QStringLiteral("That name belongs to a built-in preset.");
  case SaveResult::Exists:
    return QStringLiteral("A preset with that name already exists.");
  case SaveResult::WriteFailed:
    return QStringLiteral("The preset could not be written.");
  }
  return QString();
}

} // namespace vsim
