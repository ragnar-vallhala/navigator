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
"""Phase 2/6 integration: the `vayu-headless run` one-shot path via cli.main."""
import pytest

from vayu_headless import cli


@pytest.mark.integration
def test_cli_run_oneshot(require_binaries, gcs_conf, monkeypatch):
    monkeypatch.setenv("VAYU_GCS_CONF", gcs_conf or "/no/conf")
    rc = cli.main(["run", "--box", "6", "--alt", "-5", "--secs", "8"])
    assert rc == 0
