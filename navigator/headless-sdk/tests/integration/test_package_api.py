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
"""Phase 1 integration: drive a flight through the PUBLIC package API.

The golden (test_golden_flight) drives the legacy sitl_lab.py shim to prove the
carve-out is regression-free; this test proves the SDK stands on its own —
`from vayu_headless import SitlSession, Pilot` boots, flies, and lands. Concise
(takeoff + hold + land) so it doesn't double the full-course runtime.
"""
import time

import pytest

# Armed and flying: ARMED on the ground, or IN_AIR once the baro-driven takeoff
# detector fires (Phase 2). Both mean "armed" — see test_golden_flight.py.
NAV_FLYING = (4, 5)  # ARMED, IN_AIR


@pytest.mark.integration
def test_public_api_takeoff_hold_land(require_binaries, gcs_conf):
    from vayu_headless import SitlSession, Pilot

    with SitlSession(gcs=False, conf=gcs_conf) as sess:
        pilot = Pilot(sess, alt=-5.0)
        pilot.arm_takeoff(alt=-5.0)
        time.sleep(4.0)

        tr = sess.truth()
        assert tr is not None
        # Wide band: the SITL outer loop is known-wobbly and settles anywhere
        # ~5-7 m for a 5 m target (host-scheduling-sensitive) — same band as
        # test_vertical_sitl.py.
        assert 3.5 < -tr["pos"][2] < 8.0, "altitude not held near target"

        hb = sess.telem.get("Heartbeat")
        assert hb is not None and hb.nav_state in NAV_FLYING
        assert sess.telem_counts.get("ImuCompressed", 0) > 0

        pilot.land()
        time.sleep(0.5)
        assert pilot.armed is False
