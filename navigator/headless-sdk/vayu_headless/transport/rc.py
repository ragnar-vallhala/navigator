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
"""RC channel encoding for the firmware host's RC feeder.

The host reads a CSV line of channel microseconds (centres 1500) over a pty.
These are the pure conversions (normalised stick → µs, channel list → CSV line);
the streaming feeder + channel state live on the session (it owns the pty fd).
"""

RC_CENTRE = 1500


def stick_us(channel):
    """Normalised stick in [-1, 1] → microseconds (clamped)."""
    return int(RC_CENTRE + 500 * max(-1.0, min(1.0, channel)))


def throttle_us(thr):
    """Normalised throttle in [0, 1] → microseconds (1000–2000, clamped)."""
    return int(1000 + 1000 * max(0.0, min(1.0, thr)))


def rc_csv(channels):
    """Channel-µs iterable → the CSV line the host RC feeder parses."""
    return ",".join(str(int(v)) for v in channels) + "\n"
