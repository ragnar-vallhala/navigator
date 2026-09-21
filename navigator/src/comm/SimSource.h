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

#include "ITelemetrySource.h"

// Wraps the in-app simulator's telemetry as a first-class ITelemetrySource so
// Sim plugs into the same engine.setSource() seam as Replay
// (gcs-source-state-machine.md). MainWindow connects SimulatorWidget::dataReceived
// to feed(); the controller attaches/detaches this source as the Sim state is
// entered/left, so stale sim bytes can't reach the parser once Sim is torn down.
class SimSource : public ITelemetrySource {
  Q_OBJECT

public:
  using ITelemetrySource::ITelemetrySource;

public slots:
  void feed(const QByteArray &bytes) { emit bytesReceived(bytes); }
};
