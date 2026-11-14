#!/usr/bin/env bash
# One SHA256SUMS for every artifact in <dir> (top level files only, sorted,
# the SHA256SUMS file itself excluded). Verify with: sha256sum -c SHA256SUMS
set -euo pipefail
dir=${1:?dir}
cd "$dir"
find . -maxdepth 1 -type f ! -name SHA256SUMS -printf '%f\n' | LC_ALL=C sort \
  | xargs -d '\n' sha256sum > SHA256SUMS
cat SHA256SUMS
sha256sum -c SHA256SUMS >/dev/null
