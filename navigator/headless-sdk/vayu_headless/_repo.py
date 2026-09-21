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
"""Locate the Vayu repo root (the dir containing navlink/, tools/, navigator/).

Shared by the modules that need to resolve binary/codec paths. Phase 2's
paths.py will build the full path-resolution policy on top of this.
"""
import os


def repo_root():
    env = os.environ.get("VAYU_REPO_ROOT")
    if env and os.path.isdir(os.path.join(env, "navlink")):
        return env
    d = os.path.abspath(os.path.dirname(__file__))
    while True:
        if os.path.isdir(os.path.join(d, "navlink")) and \
           os.path.isdir(os.path.join(d, "tools")):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            raise RuntimeError("could not locate Vayu repo root")
        d = parent
