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
"""Phase 4 integration: the sysID/trajectory examples run and produce data."""
import os
import sys

import pytest

_EXAMPLES = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "examples"))


@pytest.fixture(autouse=True)
def _examples_on_path():
    if _EXAMPLES not in sys.path:
        sys.path.insert(0, _EXAMPLES)


@pytest.mark.integration
def test_step_response_records_and_responds(require_binaries):
    import step_response
    rows = step_response.run(axis="roll", amp=0.4, pre=0.3, hold=0.7, post=0.5)
    assert len(rows) > 20, "no samples recorded"
    # the airframe actually rolled in response to the step (col 2 = roll)
    peak = max(abs(r[2]) for r in rows)
    assert peak > 1.0, f"no roll response to the step (peak {peak:.2f}°)"


@pytest.mark.integration
def test_figure8_flies_the_path(require_binaries):
    import figure8
    rows = figure8.run(size=6.0, laps=0.5, lap_secs=6.0)
    assert len(rows) > 20
    # the craft translated along the lemniscate (north excursion)
    assert max(r[3] for r in rows) > 1.0, "craft did not track the figure-8"
