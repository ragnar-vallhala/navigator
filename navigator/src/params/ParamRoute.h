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

#include "ParamClient.h"

namespace params {

// Which way a tuning screen's Apply goes, from what the client knows of the
// vehicle.
//  Params:   the table is Ready: the values go through ParamApplier.
//  Legacy:   the vehicle does not offer PARAMS (older firmware): the old command.
//  NotReady: it offers them but the table is not usable now (still loading, a
//            different table, ...): nothing is sent and the reason is shown. The
//            old command would bypass the staging and checks the parameter path
//            has.
enum class Route { Params, Legacy, NotReady };
inline Route routeFor(Client::State s) {
  switch (s) {
  case Client::State::Ready: return Route::Params;
  case Client::State::NoCaps: return Route::Legacy;
  default: return Route::NotReady;
  }
}

} // namespace params
