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
# Which files should clang-tidy look at for a change set?
#
# Usage: tidy_changed_files.sh <base-commit> [head]     (head defaults to HEAD)
#
# Prints either the single word FULL (analyse everything), or a possibly empty
# list of repo-relative paths, one per line, for run_clang_tidy.sh to intersect
# with each compile database (TIDY_FILES=<file holding this list>).
#
# FULL when the change can alter what every file is analysed with: any CMake
# file (flags, defines, which sources exist), .clang-tidy (the checks), the
# analysis driver itself, this script, the workflow and its local actions. A
# partial pass would then say "clean" about code it never re-read.
#
# Otherwise: the changed C/C++ sources, plus every source that includes a
# changed header. "Includes" follows the textual #include chain to a fixed
# point by file basename, so a header edited under a header that three .cpp
# files reach is still found. It is a textual match, not the preprocessor: it
# can over-select (same basename in two directories), which only costs time,
# and it does not see includes made through a macro or a generated header. The
# full pass on every push to main is the backstop for what this misses.
set -euo pipefail

base="${1:?usage: tidy_changed_files.sh <base-commit> [head]}"
head="${2:-HEAD}"

# No usable base (new branch, force push, shallow clone): do everything.
if ! git cat-file -e "${base}^{commit}" 2>/dev/null; then
  echo FULL; exit 0
fi

changed="$(git diff --name-only --diff-filter=d "$base" "$head")"

if grep -qE '(^|/)CMakeLists\.txt$|\.cmake$|(^|/)\.clang-tidy$|^tools/dev/(run_clang_tidy|tidy_changed_files)\.sh$|^\.github/(workflows/ci\.yml|actions/)' <<<"$changed"; then
  echo FULL; exit 0
fi

src_re='\.(c|cc|cpp|cxx)$'
hdr_re='\.(h|hh|hpp|hxx)$'

result="$(grep -E "$src_re" <<<"$changed" | grep -v '^extern/' || true)"
names="$(grep -E "$hdr_re" <<<"$changed" | grep -v '^extern/' | sed 's|.*/||' | sort -u || true)"

# Fixed point over header basenames: files that include a name in the set; any
# of those that are themselves headers add their own basename.
seen=""
while [ -n "$names" ]; do
  seen="$(printf '%s\n%s\n' "$seen" "$names" | grep . | sort -u)"
  alt="$(sed 's/[][\.^$*+?(){}|]/\\&/g' <<<"$names" | paste -sd'|')"
  hits="$(git grep -lE "^[[:space:]]*#[[:space:]]*include[[:space:]]*[\"<]([^\">]*/)?(${alt})[\">]" \
            -- '*.c' '*.cc' '*.cpp' '*.cxx' '*.h' '*.hh' '*.hpp' '*.hxx' \
            ':!extern' || true)"
  result="$(printf '%s\n%s\n' "$result" "$(grep -E "$src_re" <<<"$hits" || true)")"
  newnames="$(grep -E "$hdr_re" <<<"$hits" | sed 's|.*/||' | sort -u || true)"
  names="$(comm -13 <(sort -u <<<"$seen") <(sort -u <<<"$newnames") | grep . || true)"
done

grep . <<<"$result" | sort -u || true
