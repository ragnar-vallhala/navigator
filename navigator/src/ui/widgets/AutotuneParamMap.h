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

#include "AutotuneGains.h"
#include "ParamApplier.h"
#include "ParamCatalog.h"

#include <QString>
#include <QVector>

// The autotuner's Apply expressed as parameters: each PidSetCmd (controller,
// axis, gains) becomes the catalog entries the firmware's pid_config table is
// built from. Pure (no widgets), so the mapping is unit-testable against the
// real catalog.
namespace autotune_params {

// controller 1 (rate): RATE_<AXIS>_{KP,KI,KD,KFF}; controller 0 (angle):
// ANG_<AXIS>_KP, the only angle gain the table has (an angle slot with I, D or
// FF gains cannot be expressed and is refused, not silently dropped).
// False, with the reason, when a slot has no parameters; `out` is then empty.
inline bool pidCmdsToItems(const QVector<PidSetCmd> &cmds, const params::Catalog &cat,
                           QVector<params::ParamApplier::Item> &out, QString *why = nullptr) {
  static const char *kAxis[3] = {"ROLL", "PITCH", "YAW"};
  out.clear();
  auto no = [&](const QString &m) {
    out.clear();
    if (why)
      *why = m;
    return false;
  };
  auto add = [&](const QString &id, double v) {
    if (!cat.byId(id))
      return false;
    out.push_back({id, v});
    return true;
  };
  for (const PidSetCmd &c : cmds) {
    if (c.axis < 0 || c.axis > 2 || (c.controller != 0 && c.controller != 1))
      return no(QStringLiteral("gain slot %1/%2 does not exist").arg(c.controller).arg(c.axis));
    const QString a = QString::fromLatin1(kAxis[c.axis]);
    if (c.controller == 1) {
      const QString p = QStringLiteral("RATE_%1_").arg(a);
      if (!add(p + QStringLiteral("KP"), double(c.kp)) || !add(p + QStringLiteral("KI"), double(c.ki)) ||
          !add(p + QStringLiteral("KD"), double(c.kd)) || !add(p + QStringLiteral("KFF"), double(c.kff)))
        return no(QStringLiteral("the vehicle has no parameters for the %1 rate gains").arg(a.toLower()));
    } else {
      if (c.ki != 0.0f || c.kd != 0.0f || c.kff != 0.0f)
        return no(QStringLiteral("the %1 angle gains have I, D or FF parts, which the vehicle has no "
                                 "parameters for")
                      .arg(a.toLower()));
      if (!add(QStringLiteral("ANG_%1_KP").arg(a), double(c.kp)))
        return no(QStringLiteral("the vehicle has no parameter for the %1 angle gain").arg(a.toLower()));
    }
  }
  return true;
}

// Which way an apply goes, from what the client knows of the vehicle.
//  Params:   the table is Ready: the values go through ParamApplier.
//  Legacy:   the vehicle does not offer PARAMS (older firmware): CMD_SET_PID.
//  NotReady: it offers them but the table is not usable now (still loading, a
//            different table, ...): nothing is sent, the reason is shown. The
//            old command would bypass the staging and checks this path has.
enum class Route { Params, Legacy, NotReady };
inline Route routeFor(params::Client::State s) {
  switch (s) {
  case params::Client::State::Ready: return Route::Params;
  case params::Client::State::NoCaps: return Route::Legacy;
  default: return Route::NotReady;
  }
}

} // namespace autotune_params
