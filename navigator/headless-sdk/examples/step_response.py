#!/usr/bin/env python3
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
"""Attitude STEP response (system-ID) on a tuning rig.

Arms the real FC on a rig (translation pinned, attitude free), commands a step
on one axis (roll/pitch/yaw), and records the TRUE attitude + body rate
(ground truth) alongside the FC's ControlTrace — so you can see the real
angle/rate cascade track the setpoint.

    python examples/step_response.py --axis roll --amp 0.4 --hold 1.5 --csv step.csv
"""
import argparse
import time

from vayu_headless import SitlSession

from _common import Recorder, arm_on_rig, gcs_conf, quat_to_euler

AXES = {"roll": 0, "pitch": 1, "yaw": 2}


def run(axis="roll", amp=0.4, pre=1.0, hold=1.5, post=2.0, thr=0.5, csv=None):
    rec = Recorder(csv, ["t", "cmd", "roll", "pitch", "yaw", "wx", "wy", "wz", "nav"])
    with SitlSession(rig=True, conf=gcs_conf()) as s:
        arm_on_rig(s, thr=thr)
        t0 = time.time()
        total = pre + hold + post
        ai = AXES[axis]
        while time.time() - t0 < total:
            t = time.time() - t0
            cmd = amp if pre <= t < pre + hold else 0.0       # step up then back
            kw = {"thr": thr, axis: cmd}
            s.stick(**kw)
            s.set_rc(swa=2000)
            tr = s.truth()
            hb = s.telem.get("Heartbeat")
            nav = getattr(hb, "nav_state", -1) if hb else -1
            if tr:
                r, p, y = quat_to_euler(*tr["quat"])
                wx, wy, wz = tr["omega"]
                rec.add(t, cmd, r, p, y, wx, wy, wz, nav)
            time.sleep(0.02)
    rec.close()
    if rec.rows:
        col = 2 + ai                                          # roll/pitch/yaw column
        peak = max(rec.rows, key=lambda r: abs(r[col]))
        print(f"step {axis} amp={amp} (~{amp*30:.0f}° at 30°/full): "
              f"peak true {axis}={peak[col]:+.1f}° at t={peak[0]:.2f}s, "
              f"{len(rec.rows)} samples")
    return rec.rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--axis", choices=list(AXES), default="roll")
    ap.add_argument("--amp", type=float, default=0.4)
    ap.add_argument("--pre", type=float, default=1.0)
    ap.add_argument("--hold", type=float, default=1.5)
    ap.add_argument("--post", type=float, default=2.0)
    ap.add_argument("--thr", type=float, default=0.5)
    ap.add_argument("--csv", default="")
    a = ap.parse_args()
    run(a.axis, a.amp, a.pre, a.hold, a.post, a.thr, a.csv or None)
