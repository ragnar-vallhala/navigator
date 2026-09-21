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
"""Unit tests for RC channel encoding."""
from vayu_headless.transport import rc


def test_stick_centre_and_extremes():
    assert rc.stick_us(0.0) == 1500
    assert rc.stick_us(1.0) == 2000
    assert rc.stick_us(-1.0) == 1000
    assert rc.stick_us(2.0) == 2000     # clamped
    assert rc.stick_us(-9.0) == 1000    # clamped


def test_throttle_range():
    assert rc.throttle_us(0.0) == 1000
    assert rc.throttle_us(1.0) == 2000
    assert rc.throttle_us(0.5) == 1500
    assert rc.throttle_us(-1.0) == 1000  # clamped
    assert rc.throttle_us(5.0) == 2000   # clamped


def test_rc_csv():
    assert rc.rc_csv([1500, 1500, 1000, 1500, 1000, 1500]) == \
        "1500,1500,1000,1500,1000,1500\n"
