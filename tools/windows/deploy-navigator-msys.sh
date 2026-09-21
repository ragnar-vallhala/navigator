#!/usr/bin/env bash
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
# Bundle a standalone Navigator.exe folder on Windows/MSYS2 (UCRT64).
#
# MSYS2's windeployqt6 copies the Qt6 DLLs + plugins but NOT the MinGW C++
# runtime (libgcc/libstdc++/libwinpthread) nor Qt's third-party deps (ICU,
# harfbuzz, pcre2, ...) -- it leans on the MSYS PATH for those, so the deployed
# folder fails to start on a clean Windows box. This finishes the job: it runs
# windeployqt6, copies the runtime DLLs, then resolves the full transitive DLL
# closure with ldd so dist/ runs anywhere.
#
# Usage (from an MSYS2 UCRT64 shell, or: bash deploy-navigator-msys.sh):
#   bash tools/windows/deploy-navigator-msys.sh <path-to-build-dir>
# Default build dir: navigator/build (relative to repo root).
set -o pipefail
export CHERE_INVOKING=1
export MSYSTEM=UCRT64
source /etc/profile

here="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"   # repo root
BUILD="${1:-$here/navigator/build}"
EXE="$BUILD/Navigator.exe"
DIST="$BUILD/dist"

[ -f "$EXE" ] || { echo "Navigator.exe not found at $EXE -- build first"; exit 1; }

rm -rf "$DIST"; mkdir -p "$DIST"
cp "$EXE" "$DIST/"
windeployqt6 --release --no-translations --no-system-d3d-compiler "$DIST/Navigator.exe" || exit 1

# MinGW C++ runtime (not copied by windeployqt6)
for d in libgcc_s_seh-1.dll libstdc++-6.dll libwinpthread-1.dll; do
  cp -n "/ucrt64/bin/$d" "$DIST/" 2>/dev/null || true
done
# offscreen platform plugin (headless smoke-testing)
cp -n /ucrt64/share/qt6/plugins/platforms/qoffscreen.dll "$DIST/platforms/" 2>/dev/null || true

# Resolve the full transitive DLL closure to a fixed point.
cd "$DIST" || exit 1
for pass in 1 2 3 4 5; do
  before=$(ls -1 *.dll 2>/dev/null | wc -l)
  for b in Navigator.exe *.dll platforms/*.dll styles/*.dll tls/*.dll; do
    [ -f "$b" ] && ldd "$b" 2>/dev/null
  done | grep -io '/ucrt64/bin/[^ ]*\.dll' | sort -u | while read -r f; do
    [ -f "$f" ] && cp -n "$f" .
  done
  after=$(ls -1 *.dll 2>/dev/null | wc -l)
  echo "pass $pass: $before -> $after dlls"
  [ "$before" = "$after" ] && break
done
echo "standalone bundle ready: $DIST ($(ls -1 *.dll | wc -l) dlls)"
