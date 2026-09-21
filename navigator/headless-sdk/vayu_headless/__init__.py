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
"""vayu_headless — SDK for driving the REAL Vayu flight-controller logic headlessly.

Drives the actual firmware (estimator → angle/rate cascade → mixer → arming →
telemetry) against the in-process physics engine, with no hardware and no human on
the sticks. See PLAN.md for the standardisation roadmap.

Phase 0: package skeleton only. The public API surface (SitlSession, Pilot, …)
is carved out of sim/host/sitl_lab.py in Phase 1 and re-exported here.
"""

__version__ = "0.0.1"

from .session import SitlSession, SitlLab    # noqa: E402
from .autopilot import Pilot, DEFAULT_GAINS  # noqa: E402

__all__ = [
    "__version__",
    "SitlSession", "SitlLab",   # SitlLab is a back-compat alias of SitlSession
    "Pilot", "DEFAULT_GAINS",
]
