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
"""Phase 5 drift guard: the Python vsim wire layer must match vsim_proto.h.

Parses the C static_assert sizes and asserts the Python struct formats agree, so
a change to the C protocol fails loudly here instead of silently corrupting
frames at runtime.
"""
import os
import re
import struct

import pytest

from vayu_headless._repo import repo_root
from vayu_headless.transport import vsim

PROTO = os.path.join(repo_root(), "sim", "vsim", "include", "vsim_proto.h")


def _proto_sizes():
    txt = open(PROTO).read()
    sizes = {}
    for m in re.finditer(r"static_assert\(sizeof\((\w+)\)\s*==\s*([0-9 +]+)", txt):
        sizes[m.group(1)] = eval(m.group(2))   # noqa: S307 — trusted, digits/+ only
    return sizes


@pytest.fixture(scope="module")
def sizes():
    s = _proto_sizes()
    assert s, "no static_assert sizes parsed from vsim_proto.h"
    return s


def test_header_and_pose_match(sizes):
    assert sizes["vsim_hdr_t"] == vsim.HDR.size == 16
    assert sizes["vsim_pose_frame_t"] == vsim.HDR.size + vsim.POSE.size


def test_ctl_frame_total_matches(sizes):
    # ctl frame on the wire == hdr + subtype(4) + reserved(4) + body(256)
    assert sizes["vsim_ctl_frame_t"] == len(vsim.world())


def test_ctl_bodies_match_python_formats(sizes):
    assert sizes["vsim_ctl_world_t"] == struct.calcsize("<7f")        # gravity..damp
    assert sizes["vsim_ctl_wind_t"] == struct.calcsize("<3f4fi")      # steady..enable
    geometry_fmt = "<f" + "9f" + "3f3f5f" * 4                          # mass+I+4 motors
    assert sizes["vsim_ctl_geometry_t"] == struct.calcsize(geometry_fmt)
