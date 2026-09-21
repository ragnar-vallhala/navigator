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
"""Unit tests for the session command protocol (pure parse/format, no daemon)."""
import pytest

from vayu_headless import server


# --- course_from --------------------------------------------------------------
def test_course_from_text():
    assert server.course_from("8,0;8,8;0,8") == [(8, 0), (8, 8), (0, 8)]


def test_course_from_list():
    assert server.course_from([[8, 0], [0, 8]]) == [(8.0, 0.0), (0.0, 8.0)]


# --- parse_command (text) -----------------------------------------------------
def test_parse_text_status_default():
    assert server.parse_command("") == {"cmd": "status"}
    assert server.parse_command("st") == {"cmd": "status"}


def test_parse_text_fly_full():
    c = server.parse_command("fly 8,0;8,8 -5 40")
    assert c == {"cmd": "fly", "course": "8,0;8,8", "alt": -5.0, "timeout": 40.0}


def test_parse_text_takeoff_and_alt():
    assert server.parse_command("takeoff -6") == {"cmd": "takeoff", "alt": -6.0}
    assert server.parse_command("alt -4") == {"cmd": "alt", "alt": -4.0}


def test_parse_text_rc():
    assert server.parse_command("rc 0 0.2 0.5 0")["rc"] == [0.0, 0.2, 0.5, 0.0]


# --- parse_command (JSON) -----------------------------------------------------
def test_parse_json_passthrough():
    c = server.parse_command('{"v":1,"cmd":"fly","course":[[8,0]],"alt":-5}')
    assert c["cmd"] == "fly" and c["course"] == [[8, 0]] and c["alt"] == -5


def test_parse_json_missing_cmd_raises():
    with pytest.raises(ValueError):
        server.parse_command('{"alt":-5}')


# --- status_text formatting ---------------------------------------------------
def test_status_text_renders_fields():
    s = {"nav": 4, "armed": True, "active": True, "pos": [1.0, 2.0], "alt": 5.0,
         "vD": 0.1, "att": [0, -3, 12], "wp": [2, 3], "thr": 0.34,
         "telem": {"Heartbeat": 7}}
    txt = server.status_text(s)
    assert "nav=4" in txt and "wp=2/3" in txt and "Heartbeat×7" in txt
    assert "alt=+5.00m" in txt
