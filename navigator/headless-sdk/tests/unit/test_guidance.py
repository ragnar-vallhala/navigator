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
"""Unit tests for the pure cascade-guidance math."""
from vayu_headless.autopilot import DEFAULT_GAINS, guidance_outputs

GP = DEFAULT_GAINS


def test_hover_at_target_holds_hover_throttle():
    # at target altitude, zero errors, zero velocities → ~hover throttle, level,
    # all integrators untouched.
    thr, roll, pitch, iz, iN, iE = guidance_outputs(
        GP, ez=0.0, iz=0.0, vD=0.0, eN=0.0, eE=0.0, vN=0.0, vE=0.0, dt=0.02)
    assert thr == GP["hover"]
    assert roll == 0.0 and pitch == 0.0
    assert iz == 0.0 and iN == 0.0 and iE == 0.0


def test_below_target_increases_throttle():
    # ez>0 means below the target (NED z larger than target) → climb → more thrust
    thr, *_ = guidance_outputs(GP, ez=2.0, iz=0.0, vD=0.0,
                               eN=0, eE=0, vN=0, vE=0, dt=0.02)
    assert thr > GP["hover"]


def test_throttle_clamped_unit_interval():
    hi, *_ = guidance_outputs(GP, ez=100.0, iz=0.3, vD=0.0, eN=0, eE=0,
                              vN=0, vE=0, dt=0.02)
    lo, *_ = guidance_outputs(GP, ez=-100.0, iz=-0.3, vD=0.0, eN=0, eE=0,
                              vN=0, vE=0, dt=0.02)
    assert hi == 1.0 and lo == 0.0


def test_tilt_clamped_to_limit():
    # huge north error → pitch saturates at +tilt; roll stays 0 (no east error)
    _, roll, pitch, *_ = guidance_outputs(
        GP, ez=0.0, iz=0.0, vD=0.0, eN=1000.0, eE=0.0, vN=0.0, vE=0.0, dt=0.02)
    assert pitch == GP["tilt"]
    assert roll == 0.0


def test_alt_integrator_accumulates_and_clamps():
    _, _, _, iz, _, _ = guidance_outputs(GP, ez=5.0, iz=0.29, vD=0.0, eN=0, eE=0,
                                         vN=0, vE=0, dt=0.02)
    assert iz == 0.3            # clamped at +0.3


def test_velocity_feedforward_tilts_for_moving_setpoint():
    # zero position error but a commanded path velocity → a non-zero tilt, so a
    # moving setpoint no longer just lags behind position error.
    _, _, pitch, *_ = guidance_outputs(
        GP, ez=0.0, iz=0.0, vD=0.0, eN=0.0, eE=0.0, vN=0.0, vE=0.0, dt=0.02,
        vff_n=2.0)
    assert pitch > 0.0


def test_horizontal_antiwindup_does_not_grow_when_saturated():
    # saturating north error must not let the integrator wind past one step.
    _, _, _, _, iN, _ = guidance_outputs(
        GP, ez=0.0, iz=0.0, vD=0.0, eN=1000.0, eE=0.0, vN=0.0, vE=0.0, dt=0.02,
        iN=GP["i_lim"])
    assert abs(iN) <= GP["i_lim"]      # clamped, not runaway
