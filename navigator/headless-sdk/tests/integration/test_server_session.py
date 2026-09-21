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
"""Phase 2/3 integration: drive a live session through SessionServer + client,
exercising both the text and JSON protocols over the control socket.
"""
import json
import threading
import time

import pytest

from vayu_headless import client
from vayu_headless.autopilot import Pilot
from vayu_headless.server import SessionServer
from vayu_headless.session import SitlSession


@pytest.mark.integration
def test_serve_text_and_json(require_binaries, gcs_conf, tmp_path):
    sock = str(tmp_path / "sess.sock")
    lab = SitlSession(gcs=False, conf=gcs_conf)
    pilot = Pilot(lab, alt=-5.0)
    srv = SessionServer(lab, pilot)
    t = threading.Thread(target=srv.serve_forever, args=(sock,), daemon=True)
    t.start()
    try:
        # wait for the socket to come up
        for _ in range(100):
            try:
                client.send_command("status", sock_path=sock)
                break
            except ConnectionError:
                time.sleep(0.05)

        # text protocol: takeoff then status. nav is ARMED (4) on the ground or
        # IN_AIR (5) once the baro-driven takeoff detector fires (Phase 2) —
        # accept either, like test_golden_flight.py.
        assert client.send_command("takeoff -5", sock_path=sock).startswith("ok")
        time.sleep(4.0)
        txt = client.send_command("status", sock_path=sock)
        assert ("nav=4" in txt or "nav=5" in txt), txt

        # JSON protocol: structured request → structured reply
        reply = client.send_command(
            json.dumps({"v": 1, "cmd": "status"}), sock_path=sock)
        obj = json.loads(reply)
        assert obj["ok"] is True
        assert obj["status"]["nav"] in (4, 5)  # ARMED or IN_AIR
        assert obj["status"]["alt"] > 3.0

        # shutdown
        assert client.send_command("quit", sock_path=sock) == "BYE"
    finally:
        lab._stop.set()
        t.join(timeout=5)
