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
"""Unit tests for the vsim wire layer (no I/O, no daemon)."""
import struct

import pytest

from vayu_headless.transport import vsim


def test_struct_sizes_match_proto():
    # vsim_hdr_t == 16, pose body == 140 at proto v5 (vsim_proto.h
    # static_asserts); 128 is the oldest layout parse_pose still decodes.
    assert vsim.HDR.size == 16
    assert vsim.POSE.size == 140
    assert vsim.POSE_V3.size == 128
    assert vsim.VERSION == 5


def test_ctl_frame_shape():
    # CTL frame = 16-byte hdr + subtype(4) + reserved(4) + body(256) = 280.
    f = vsim.world()
    assert len(f) == 16 + 8 + 256
    magic, ver, typ, plen, seq = vsim.HDR.unpack_from(f, 0)
    assert magic == vsim.MAGIC and ver == vsim.VERSION
    assert typ == 4                      # CTL frame type
    assert plen == 8 + 256
    subtype, reserved = struct.unpack_from("<II", f, 16)
    assert subtype == vsim.CTL_SET_WORLD and reserved == 0


@pytest.mark.parametrize("builder,subtype", [
    (lambda: vsim.reset(), vsim.CTL_RESET),
    (lambda: vsim.world(), vsim.CTL_SET_WORLD),
    (lambda: vsim.testrig(True), vsim.CTL_SET_TESTRIG),
    (lambda: vsim.wind((1, 0, 0), 0.2), vsim.CTL_SET_WIND),
])
def test_builders_carry_their_subtype(builder, subtype):
    f = builder()
    assert len(f) == 280
    got, _ = struct.unpack_from("<II", f, 16)
    assert got == subtype


def test_world_mesh_counts_and_path():
    f = vsim.world_mesh("/tmp/x.bin", 17202, 5734, 2047, 0.3, True)
    nverts, ntris, nodes, flags, rest, plen = struct.unpack_from("<IIIIfI", f, 24)
    assert (nverts, ntris, nodes) == (17202, 5734, 2047)
    assert flags == 1                    # double-sided
    assert abs(rest - 0.3) < 1e-6
    path = f[24 + 24: 24 + 24 + plen].decode()
    assert path == "/tmp/x.bin"


def test_quat_to_euler_identity_and_axes():
    assert vsim.quat_to_euler(1, 0, 0, 0) == pytest.approx([0, 0, 0])
    # 90° about body X (roll): q = (cos45, sin45, 0, 0)
    import math
    r, p, y = vsim.quat_to_euler(math.cos(math.pi / 4), math.sin(math.pi / 4), 0, 0)
    assert r == pytest.approx(90.0, abs=1e-3)
    assert p == pytest.approx(0.0, abs=1e-3)


def test_parse_pose_roundtrip():
    # Build a POSE frame, confirm parse_pose recovers pos/quat/vel and trims buf.
    body = struct.pack(
        "<II3f4f3f3f4f4f3fffffff",
        0, 0,                       # two leading u32 (seq/flags slot)
        1.0, 2.0, -3.0,             # pos
        1.0, 0.0, 0.0, 0.0,         # quat
        0.1, 0.2, 0.3,              # vel
        0.0, 0.0, 0.0,              # omega
        10, 20, 30, 40,             # motor_omega
        0, 0, 0, 0,                 # (4f)
        0.0, 0.0, 0.0,              # wind
        0, 0, 0, 0, 0, 0)           # trailing 6f
    # A v3 body (128 B): the oldest layout parse_pose still decodes.
    frame = vsim.vhdr(vsim.FRAME_POSE, len(body)) + body
    buf = bytearray(frame)
    truth = vsim.parse_pose(buf)
    assert truth is not None
    assert truth["pos"] == pytest.approx((1.0, 2.0, -3.0))
    assert truth["quat"] == pytest.approx((1.0, 0.0, 0.0, 0.0))
    assert truth["vel"] == pytest.approx((0.1, 0.2, 0.3))
    assert truth["motor_omega"] == pytest.approx((10, 20, 30, 40))
    assert len(buf) == 0             # fully consumed


def test_parse_pose_partial_frame_kept():
    frame = vsim.vhdr(vsim.FRAME_POSE, vsim.POSE.size) + b"\x00" * vsim.POSE.size
    buf = bytearray(frame[:20])       # header + a few body bytes only
    assert vsim.parse_pose(buf) is None
    assert len(buf) == 20             # incomplete frame retained for next read


def _pose_body(crash_flags=None, peak=None, with_v4=True):
    base = struct.pack(
        "<II3f4f3f3f4f4f3fffffff",
        0, 0, 1.0, 2.0, -3.0, 1.0, 0.0, 0.0, 0.0, 0.1, 0.2, 0.3,
        0.0, 0.0, 0.0, 10, 20, 30, 40, 0, 0, 0, 0, 0.0, 0.0, 0.0,
        0, 0, 0, 0, 0, 0)
    if with_v4:
        base += struct.pack("<f", 1.0)          # air_density_ratio
    if crash_flags is not None:
        base += struct.pack("<If", crash_flags, peak)
    return base


def test_parse_pose_reads_the_crash_tail():
    body = _pose_body(vsim.POSE_CRASHED, 9.5)
    assert len(body) == vsim.POSE.size
    truth = vsim.parse_pose(bytearray(vsim.vhdr(vsim.FRAME_POSE, len(body)) + body))
    assert truth["crashed"] is True
    assert truth["crash_flags"] == vsim.POSE_CRASHED
    assert truth["peak_impact"] == pytest.approx(9.5)
    assert truth["pos"] == pytest.approx((1.0, 2.0, -3.0))   # the prefix still decodes


def test_parse_pose_hard_contact_without_crash():
    body = _pose_body(0, 7.5)       # a peak, but under the limit: not a crash
    truth = vsim.parse_pose(bytearray(vsim.vhdr(vsim.FRAME_POSE, len(body)) + body))
    assert truth["crashed"] is False
    assert truth["peak_impact"] == pytest.approx(7.5)


@pytest.mark.parametrize("with_v4", [False, True])
def test_parse_pose_older_sim_has_no_crash_tail(with_v4):
    # A v3 (128 B) or v4 (132 B) sim sends no crash tail: read as not crashed.
    body = _pose_body(with_v4=with_v4)
    assert len(body) == (132 if with_v4 else 128)
    truth = vsim.parse_pose(bytearray(vsim.vhdr(vsim.FRAME_POSE, len(body)) + body))
    assert truth["crashed"] is False
    assert truth["crash_flags"] == 0 and truth["peak_impact"] == 0.0
    assert truth["quat"] == pytest.approx((1.0, 0.0, 0.0, 0.0))


def test_parse_pose_newer_sim_longer_tail_is_ignored():
    # Append-only: a later version adds fields after ours; the known prefix
    # (including the crash fields) must still decode.
    body = _pose_body(vsim.POSE_CRASHED, 12.0) + struct.pack("<II", 7, 8)
    truth = vsim.parse_pose(bytearray(vsim.vhdr(vsim.FRAME_POSE, len(body)) + body))
    assert truth["crashed"] is True and truth["peak_impact"] == pytest.approx(12.0)


def test_set_crash_limit_frame():
    frame = vsim.set_crash_limit(8.0)
    magic, ver, typ, plen, _ = vsim.HDR.unpack_from(frame, 0)
    sub, _res = struct.unpack_from("<II", frame, vsim.HDR.size)
    (limit,) = struct.unpack_from("<f", frame, vsim.HDR.size + 8)
    assert (magic, ver) == (vsim.MAGIC, vsim.VERSION)
    assert sub == vsim.CTL_SET_CRASH_LIMIT == 20
    assert limit == pytest.approx(8.0)
    assert len(frame) == vsim.HDR.size + 8 + 256
