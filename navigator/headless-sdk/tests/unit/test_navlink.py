# Copyright (C) 2026 NAVRobotec Pvt Ltd
# Author: Ragnar Vallhala
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Unit tests for telemetry decode (uses the generated codec, no daemon)."""
from vayu_headless.transport import navlink


def test_codec_loaded():
    assert navlink.MSG_BY_ID, "no NavLink message classes discovered"
    # Heartbeat (msgid 0) must be present — the harness relies on nav_state.
    assert 0 in navlink.MSG_BY_ID


def _heartbeat_frame():
    """A full on-wire Heartbeat frame via the generated codec (class.pack() +
    frame.encode), matching exactly what the FC emits."""
    hb_cls = navlink.MSG_BY_ID[0]
    msg = hb_cls()
    if hasattr(msg, "nav_state"):
        msg.nav_state = 4
    return navlink.nlframe.encode(hb_cls.MSGID, msg.pack())


def test_parse_frames_decodes_encoded_message():
    telem, counts = {}, {}
    buf = bytearray(_heartbeat_frame())
    navlink.parse_frames(buf, telem, counts)
    assert counts.get("Heartbeat", 0) == 1
    assert "Heartbeat" in telem
    assert len(buf) == 0                      # frame fully consumed


def test_parse_frames_resyncs_on_garbage():
    telem, counts = {}, {}
    buf = bytearray(b"\x00\xff\x12" + bytes(_heartbeat_frame()))   # leading junk
    navlink.parse_frames(buf, telem, counts)
    assert counts.get("Heartbeat", 0) == 1               # found after resync
