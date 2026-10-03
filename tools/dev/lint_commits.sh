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
# Lint the commit messages in a range, the same rules NavHAL enforces:
#
#   <type>[(<scope>)][!]: <subject>    subject line <= 72 chars
#   types: feat fix docs style refactor perf test build ci chore revert
#   line 2 blank; no AI attribution trailer (Co-Authored-By: Claude, ...)
#
# Usage:  tools/dev/lint_commits.sh [<base>..<head>]   (default origin/main..HEAD)
#         tools/dev/lint_commits.sh --msg-file <file>   one message, as the
#                                                       commit-msg hook does
#
# CI runs the range form on every PR. .githooks/commit-msg runs the --msg-file
# form at commit time, so the same rules reject a message before it is made;
# enable it once per clone with `git config core.hooksPath .githooks`.
#
# Only commits NEW to the range are checked. A release PR (main -> stable)
# re-walks history that was already gated when it landed on main, and the
# history from before this check existed does not all conform and cannot be
# rewritten -- so anything already reachable from main or stable is skipped.
# Merge commits and fixup!/squash! commits are skipped too.
set -eu
cd "$(git rev-parse --show-toplevel)"

TYPES='feat|fix|docs|style|refactor|perf|test|build|ci|chore|revert'
PATTERN="^(${TYPES})(\([a-z0-9][a-z0-9_.,-]*\))?!?: .+$"
# Anchored on trailer syntax, so prose that mentions Claude and a genuine
# human Co-Authored-By both pass.
AI_TRAILER='co-authored-by:.*(claude|anthropic)|generated with .*claude'

# check_msg <label> <message>: print one line per problem, return 1 if any.
# fixup!/squash! messages pass: they are folded into a checked commit.
check_msg() {
  local label=$1 msg=$2 subject rc=0
  subject=$(printf '%s\n' "$msg" | sed -n 1p)
  case "$subject" in "fixup! "*|"squash! "*) return 0 ;; esac
  bad() { echo "${PREFIX}${label}\"$subject\": $1"; rc=1; }
  printf '%s' "$subject" | grep -qE "$PATTERN" \
    || bad "not <type>[(<scope>)][!]: <subject>, type one of ${TYPES//|/ }"
  [ "${#subject}" -le 72 ] || bad "subject is ${#subject} chars, max 72"
  [ -z "$(printf '%s\n' "$msg" | sed -n 2p)" ] || bad "line 2 must be blank"
  ! printf '%s\n' "$msg" | grep -Eqi "$AI_TRAILER" || bad "carries an AI attribution trailer"
  return "$rc"
}

if [ "${1:-}" = --msg-file ]; then
  PREFIX="commit-msg: "
  # The file still holds the editor's # comments; git strips them afterwards.
  check_msg "" "$(git stripspace --strip-comments < "$2")" && exit 0
  echo "commit-msg: fix the message, or see tools/dev/lint_commits.sh" >&2
  exit 1
fi

PREFIX="::error::"
RANGE="${1:-origin/main..HEAD}"
for br in main stable; do   # the CI checkout may not have both
  git rev-parse -q --verify "origin/$br" >/dev/null \
    || git fetch -q origin "$br:refs/remotes/origin/$br" 2>/dev/null || true
done
landed() {
  for br in main stable; do
    git rev-parse -q --verify "origin/$br" >/dev/null || continue
    git merge-base --is-ancestor "$1" "origin/$br" && return 0
  done
  return 1
}

fail=0 checked=0
for c in $(git rev-list --no-merges --reverse "$RANGE"); do
  landed "$c" && continue
  checked=$((checked + 1))
  check_msg "$(git log -1 --format=%h "$c") " "$(git log -1 --format=%B "$c")" || fail=1
done

[ "$fail" = 0 ] && echo "commit lint: $checked new commit(s) in $RANGE ok"
exit "$fail"
