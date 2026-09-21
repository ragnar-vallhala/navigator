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
"""Thin client: send one command to a running `serve` session, print the reply."""
import socket
import sys

from . import paths


def send_command(line, sock_path=None, timeout=120.0):
    """Connect, send one command line, return the reply string (no trailing NL).
    Raises ConnectionError if no session is listening."""
    sock_path = sock_path or paths.SOCK_PATH
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect(sock_path)
    except (FileNotFoundError, ConnectionRefusedError) as e:
        raise ConnectionError(f"no session at {sock_path}") from e
    try:
        s.sendall((line + "\n").encode())
        buf = b""
        while b"\n" not in buf:
            chunk = s.recv(4096)
            if not chunk:
                break
            buf += chunk
    finally:
        s.close()
    return buf.decode(errors="replace").strip()


def main(line, sock_path=None):
    try:
        print(send_command(line, sock_path))
        return 0
    except ConnectionError as e:
        print(f"err: {e} — start one with: vayu-headless serve", file=sys.stderr)
        return 1
