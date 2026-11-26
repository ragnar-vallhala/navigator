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
"""vsim wire protocol: control-frame builders + pose decoding.

Mirrors sim/host/sdk/vsim_proto.h. These are pure functions (no I/O), so
they unit-test without booting anything. Frame layout (carved verbatim from the
original sim/host/sitl_lab.py so behaviour is identical):

    16-byte header (vsim_hdr_t)  +  payload
    CTL frames: subtype(u32) + reserved(u32) + body[256]
"""
import math
import struct

MAGIC, VERSION = 0x4D495356, 6
FRAME_POSE = 3

# vsim_ctl_* subtypes (see vsim_proto.h).
CTL_RESET = 1
CTL_SET_GEOMETRY = 5
CTL_SET_WORLD = 6
CTL_SET_WORLD_MESH = 10
CTL_SET_TESTRIG = 12
CTL_SET_WIND = 14
CTL_SET_CRASH_LIMIT = 20

HDR = struct.Struct("<IHHII")
# The pose body is APPEND-ONLY (vsim_proto.h): a field is added at the tail and
# an existing one never moves, so a reader decodes the prefix it knows and
# ignores the rest. POSE_V3 is the oldest layout this decoder accepts; POSE is
# the current one (v4 air_density_ratio, v5 crash_flags + peak_impact,
# v6 ctl_caps + ctl_unknown).
_POSE_V3_FMT = "II3f4f3f3f4f4f3fffffff"                  # 128 B
POSE_V3 = struct.Struct("<" + _POSE_V3_FMT)
POSE = struct.Struct("<" + _POSE_V3_FMT + "f" + "If" + "II")  # v6 body, 148 B
POSE_V4_SIZE = POSE_V3.size + 4
POSE_V5 = struct.Struct("<" + _POSE_V3_FMT + "f" + "If")      # 140 B
assert (HDR.size == 16 and POSE_V3.size == 128 and POSE_V5.size == 140
        and POSE.size == 148)
POSE_CRASHED = 0x1      # crash_flags bit 0


def vhdr(typ, plen, seq=0):
    return HDR.pack(MAGIC, VERSION, typ, plen, seq)


def ctl(subtype, body):
    body = body[:256].ljust(256, b"\x00")
    payload = struct.pack("<II", subtype, 0) + body
    return vhdr(4, len(payload)) + payload


def reset(pos=(0, 0, -0.05), seed=1):
    return ctl(CTL_RESET, struct.pack("<3f4f3f3fI", *pos, 1, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0, seed))


def world(gravity=9.81, ground_z=50.0, lin_drag=0.10):
    # ground_z far below so a rig/airborne craft never clamps unexpectedly.
    return ctl(CTL_SET_WORLD, struct.pack("<7f", gravity, ground_z, 0.0,
                                          lin_drag, 0.005, 40.0, 6.0))


def testrig(enable, pos=(0, 0, -1.0), tether_k=0.0):
    return ctl(CTL_SET_TESTRIG, struct.pack("<i3ff", 1 if enable else 0,
                                            pos[0], pos[1], pos[2], tether_k))


def wind(steady=(0, 0, 0), turb=0.0, enable=True):
    return ctl(CTL_SET_WIND, struct.pack("<3f4fi", steady[0], steady[1],
               steady[2], 0.0, 0.0, turb, 1.0, 1 if enable else 0))


def set_crash_limit(mps):
    """vsim_ctl_crash_limit_t: a contact whose normal speed exceeds `mps` [m/s]
    crashes the vehicle (motors cut until a reset). 0, negative or non-finite
    turns the model off, which is the engine's own default."""
    return ctl(CTL_SET_CRASH_LIMIT, struct.pack("<f", float(mps)))


def world_mesh(path, nverts, ntris, nodes, restitution, double_sided):
    """vsim_ctl_world_mesh_t: counts + flags + restitution + the BVH file path
    the daemon mmaps. Counts must match the blob header (daemon cross-checks)."""
    pb = path.encode()[:215]
    body = struct.pack("<IIIIfI", nverts, ntris, nodes,
                       1 if double_sided else 0, float(restitution), len(pb))
    body += pb + b"\x00" * (216 - len(pb))
    return ctl(CTL_SET_WORLD_MESH, body)


def quat_to_euler(w, x, y, z):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(s)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [math.degrees(v) for v in (roll, pitch, yaw)]


def parse_pose(buf):
    """Consume vsim frames from `buf` (a bytearray, trimmed in place) and return
    the LATEST decoded pose dict, or None if no complete pose frame was present.
    Matches the original SitlLab._parse_pose semantics exactly."""
    truth = None
    pos = 0
    while pos + HDR.size <= len(buf):
        magic, ver, t, plen, seq = HDR.unpack_from(buf, pos)
        if magic != MAGIC:
            pos += 1
            continue
        if pos + HDR.size + plen > len(buf):
            break
        if t == FRAME_POSE and plen >= POSE_V3.size:
            body = pos + HDR.size
            p = POSE_V3.unpack_from(buf, body)
            # Tail fields exist only when the sim is new enough to send them;
            # a missing tail reads as "not crashed", never as garbage.
            crash_flags, peak = 0, 0.0
            ctl_caps, ctl_unknown = 0, 0
            if plen >= POSE_V5.size:
                crash_flags, peak = struct.unpack_from(
                    "<If", buf, body + POSE_V4_SIZE)
            if plen >= POSE.size:
                ctl_caps, ctl_unknown = struct.unpack_from(
                    "<II", buf, body + POSE_V5.size)
            truth = {
                "pos": p[2:5], "quat": p[5:9], "vel": p[9:12],
                "omega": p[12:15], "motor_omega": p[15:19],
                "wind": p[23:26],
                "crash_flags": crash_flags, "peak_impact": peak,
                "crashed": bool(crash_flags & POSE_CRASHED),
                "ctl_caps": ctl_caps, "ctl_unknown": ctl_unknown,
            }
        pos += HDR.size + plen
    del buf[:pos]
    return truth
