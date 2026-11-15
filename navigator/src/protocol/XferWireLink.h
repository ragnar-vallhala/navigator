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

#include "CommandCodec.h"
#include "core/XferUploader.h"

#include <QByteArray>
#include <functional>
#include <utility>

// IXferLink over NavLink v2 frames: each call is encoded by CommandCodec and
// handed to `sink` (MainWindow::sendToFc on the live link). The sink returns
// false when nothing may be transmitted.
class XferWireLink : public IXferLink {
public:
  using Sink = std::function<bool(const QByteArray &frame)>;
  explicit XferWireLink(Sink sink, quint8 devId = 42)
      : m_sink(std::move(sink)), m_dev(devId) {}

  bool sendOpen(quint8 reqSeq, quint8 session, const QByteArray &path) override {
    return m_sink(CommandCodec::encodeXferOpen(session, reqSeq, path, m_dev));
  }
  bool sendData(quint8 session, quint8 flags, quint32 offset,
                const QByteArray &payload) override {
    return m_sink(CommandCodec::encodeXferData(session, flags, offset, payload));
  }
  bool sendClose(quint8 reqSeq, quint8 session, quint8 result) override {
    return m_sink(CommandCodec::encodeXferClose(session, reqSeq, result, m_dev));
  }

private:
  Sink m_sink;
  quint8 m_dev;
};
