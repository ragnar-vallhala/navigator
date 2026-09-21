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
"""Shared test fixtures for the headless SDK.

Locates the repo root, the SITL binary, and the GCS config, and skips
integration tests cleanly when the binary is missing (so unit tests still run on
a machine that hasn't built the firmware host).
"""
import os
import sys

import pytest

# repo root = .../vayu (this file is navigator/headless-sdk/tests/conftest.py)
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))

# build_sitl_rtos/ at the repo root is the shared convention: vayu.sh build,
# vtest's pytest adapter and tools/autotune all produce and consume it there.
RTOS_BIN = os.environ.get("VAYU_SITL_RTOS_BIN",
                          os.path.join(ROOT, "build_sitl_rtos/vayu_sitl_rtos"))
GCS_CONF = os.environ.get("VAYU_GCS_CONF",
                          os.path.expanduser("~/.config/Vayu/Vayu GCS.conf"))


@pytest.fixture(scope="session")
def repo_root():
    return ROOT


@pytest.fixture(scope="session")
def gcs_conf():
    return GCS_CONF if os.path.exists(GCS_CONF) else None


@pytest.fixture
def require_binaries():
    """Skip an integration test unless the SITL binary exists."""
    if not os.path.exists(RTOS_BIN):
        pytest.skip(f"missing SITL binary: {RTOS_BIN} (build it to run integration tests)")
    os.environ["VAYU_SITL_RTOS_BIN"] = RTOS_BIN
    return RTOS_BIN
