#!/usr/bin/env bash
# Fetch a pinned `nav` release (public repo ragnar-vallhala/nav), verify it
# against the release's own SHA256SUMS, and unpack it into <outdir>/{bin,share}.
#
#   fetch-nav.sh <linux|windows> <nav-version, e.g. 1.3.2> <outdir>
#
# Linux uses the .deb (unpacked with dpkg-deb -x, nothing is installed);
# Windows uses the win64.zip. Both ship bin/nav[.exe] and share/nav/fonts.
#
# Fails, with a message, when the tag does not carry the expected assets or no
# SHA256SUMS: nav releases before the one that adds SHA256SUMS (v1.3.1 and
# older) are refused on purpose. Local testing against files you already hold:
#   NAV_LOCAL_DIR=<dir with the assets> NAV_UNVERIFIED=1 fetch-nav.sh ...
# NAV_UNVERIFIED skips the checksum and prints that it did; CI never sets it.
set -euo pipefail
platform=${1:?platform linux|windows}; ver=${2:?nav version}; out=${3:?outdir}
ver=${ver#v}
base="https://github.com/ragnar-vallhala/nav/releases/download/v${ver}"
case "$platform" in
  linux)   asset="nav-${ver}-Linux.deb" ;;
  windows) asset="nav-${ver}-win64.zip" ;;
  *) echo "unknown platform $platform" >&2; exit 2 ;;
esac
work=$(mktemp -d); trap 'rm -rf "$work"' EXIT
if [ -n "${NAV_LOCAL_DIR:-}" ]; then
  cp "$NAV_LOCAL_DIR/$asset" "$work/"
else
  curl -fsSL --retry 3 -o "$work/$asset" "$base/$asset" || {
    echo "::error::nav v${ver} has no asset $asset: NAV_VERSION points at a tag that does not carry the release layout (nav-<ver>-Linux.deb, -win64.zip, SHA256SUMS). Bump NAV_VERSION." >&2; exit 1; }
  curl -fsSL --retry 3 -o "$work/SHA256SUMS" "$base/SHA256SUMS" || {
    if [ "${NAV_UNVERIFIED:-}" != 1 ]; then
      echo "::error::nav v${ver} has no SHA256SUMS: refusing to bundle an unverifiable binary. Bump NAV_VERSION to a tag cut after nav PR #11." >&2; exit 1; fi; }
fi
if [ "${NAV_UNVERIFIED:-}" = 1 ]; then
  echo "WARNING: nav checksum NOT verified (NAV_UNVERIFIED=1, local use only)" >&2
else
  (cd "$work" && grep -F " $asset" SHA256SUMS | sha256sum -c -)
fi
rm -rf "$out"; mkdir -p "$out"
if [ "$platform" = linux ]; then
  dpkg-deb -x "$work/$asset" "$work/x"
  mkdir -p "$out/bin" "$out/share"
  cp "$work/x/usr/bin/nav" "$out/bin/nav"
  cp -r "$work/x/usr/share/nav" "$out/share/nav"
else
  unzip -q "$work/$asset" -d "$work/x"
  d=$(echo "$work"/x/nav-*)
  mkdir -p "$out/bin" "$out/share"
  cp "$d/bin/nav.exe" "$out/bin/"; cp -r "$d/share/nav" "$out/share/nav"
fi
echo "nav ${ver} ($platform) unpacked to $out"
