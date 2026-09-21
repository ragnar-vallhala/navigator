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

#include <QByteArray>
#include <QObject>

// Abstract source of inbound telemetry bytes feeding DroneProtocol's parser.
//
// MainWindow connects the *active* source's bytesReceived() into the engine's
// feedBytes(), so the entire decode -> typed-signal -> widget chain is identical
// regardless of where the bytes came from. SimSource wraps the in-app sim;
// ReplaySource replays a recorded .bin; the live serial/UDP feed is internal to
// TelemetryEngine (toggled via setLiveFeed). The SourceController picks exactly
// one at a time (docs/roadmap/gcs-source-state-machine.md).
class ITelemetrySource : public QObject {
  Q_OBJECT

public:
  using QObject::QObject;
  ~ITelemetrySource() override = default;

signals:
  // Raw inbound bytes — a live stream chunk or a replayed frame — to hand to
  // the protocol parser verbatim. The parser owns reassembly/framing, so the
  // source never has to.
  void bytesReceived(const QByteArray &data);
};
