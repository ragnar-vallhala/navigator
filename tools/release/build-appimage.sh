#!/usr/bin/env bash
# Package a Release build of Navigator as an AppImage (Linux x86_64).
#
# Layout inside the AppImage mirrors the installed layout of
# ~/.local/opt/navigator: the binary and the SITL module are found by
# SitlModule beside/above the executable (usr/bin/Navigator, usr/lib/
# libvayu_sitl.so), and linuxdeploy rewrites RUNPATH to $ORIGIN/../lib so the
# bundled Qt, Qt Advanced Docking, assimp, ... load from usr/lib. `nav` and
# the flashing tools sit in usr/bin, which AppRun puts first on PATH.
#
# Required environment:
#   BUILD_DIR   CMake build dir holding Navigator (Release, SITL ON)
#   SDK_DIR     unpacked vayu-sitl-sdk (has lib/libvayu_sitl.so)
#   VERSION     e.g. 0.3.0 or 0.3.0-beta.1 (no leading v)
#   CHANNEL     stable | beta
#   OUT_DIR     where navigator-<version>-<channel>-linux-x86_64.AppImage goes
#   NAV_DIR     output of fetch-nav.sh linux (bin/nav, share/nav)
# Optional:
#   TOOLS_DIR            cache for linuxdeploy/appimagetool (default $OUT_DIR/.tools)
#   FLASH_TOOLS          executables to bundle (default "st-flash openocd avrdude")
#   REQUIRE_FLASH_TOOLS  1 = a missing flash tool is an error (CI); default 0 warns
#   WORK_DIR             AppDir parent (default $OUT_DIR/.work)
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
: "${BUILD_DIR:?}" "${SDK_DIR:?}" "${VERSION:?}" "${CHANNEL:?}" "${OUT_DIR:?}" "${NAV_DIR:?}"
TOOLS_DIR=${TOOLS_DIR:-$OUT_DIR/.tools}; WORK_DIR=${WORK_DIR:-$OUT_DIR/.work}
FLASH_TOOLS=${FLASH_TOOLS:-"st-flash openocd avrdude"}
mkdir -p "$OUT_DIR" "$TOOLS_DIR" "$WORK_DIR"
export APPIMAGE_EXTRACT_AND_RUN=1   # no FUSE on CI runners
export NO_STRIP=1                   # linuxdeploy's bundled strip chokes on newer .relr sections

# --- pinned packaging tools (version tag + sha256 of the file) ---------------
fetch() { # name url sha256
  local f="$TOOLS_DIR/$1"
  if [ ! -x "$f" ]; then curl -fsSL --retry 3 -o "$f" "$2"; chmod +x "$f"; fi
  echo "$3  $f" | sha256sum -c - >/dev/null || { echo "checksum mismatch for $1" >&2; rm -f "$f"; exit 1; }
}
fetch linuxdeploy-x86_64.AppImage \
  https://github.com/linuxdeploy/linuxdeploy/releases/download/1-alpha-20250213-2/linuxdeploy-x86_64.AppImage \
  4648f278ab3ef31f819e67c30d50f462640e5365a77637d7e6f2ad9fd0b4522a
fetch linuxdeploy-plugin-qt-x86_64.AppImage \
  https://github.com/linuxdeploy/linuxdeploy-plugin-qt/releases/download/1-alpha-20250213-1/linuxdeploy-plugin-qt-x86_64.AppImage \
  15106be885c1c48a021198e7e1e9a48ce9d02a86dd0a1848f00bdbf3c1c92724
fetch appimagetool-x86_64.AppImage \
  https://github.com/AppImage/appimagetool/releases/download/1.9.1/appimagetool-x86_64.AppImage \
  ed4ce84f0d9caff66f50bcca6ff6f35aae54ce8135408b3fa33abfc3cb384eb0

AD="$WORK_DIR/AppDir"
rm -rf "$AD"; mkdir -p "$AD/usr/bin" "$AD/usr/lib" "$AD/usr/share/doc/navigator"
test -x "$BUILD_DIR/Navigator"; test -f "$SDK_DIR/lib/libvayu_sitl.so"
cp "$BUILD_DIR/Navigator" "$AD/usr/bin/Navigator"
cp "$SDK_DIR/lib/libvayu_sitl.so" "$AD/usr/lib/libvayu_sitl.so"
cp "$NAV_DIR/bin/nav" "$AD/usr/bin/nav"; cp -r "$NAV_DIR/share/nav" "$AD/usr/share/nav"
echo "$VERSION $CHANNEL" > "$AD/usr/share/doc/navigator/VERSION"

# --- flashing tools: executables via linuxdeploy (pulls their libs too) ------
exe_args=()
for t in $FLASH_TOOLS; do
  if p=$(command -v "$t"); then exe_args+=(--executable "$p")
  elif [ "${REQUIRE_FLASH_TOOLS:-0}" = 1 ]; then echo "flash tool $t not installed" >&2; exit 1
  else echo "WARNING: $t not installed here, not bundled" >&2; fi
done
# data the tools look for relative to their own location
if [ -d /usr/share/openocd ] && command -v openocd >/dev/null; then
  mkdir -p "$AD/usr/share"; cp -r /usr/share/openocd "$AD/usr/share/openocd"; fi
if command -v avrdude >/dev/null && [ -f /etc/avrdude.conf ]; then
  mkdir -p "$AD/usr/etc"; cp /etc/avrdude.conf "$AD/usr/etc/avrdude.conf"; fi
# ST-Link udev rules: an AppImage cannot install them, ship them for the user
mkdir -p "$AD/usr/share/navigator/udev"
cp /usr/lib/udev/rules.d/49-stlink*.rules "$AD/usr/share/navigator/udev/" 2>/dev/null || true

# --- bundle libraries + Qt plugins --------------------------------------------
adslib=$(dirname "$(find "$BUILD_DIR" -name 'libqtadvanceddocking-qt6.so' | head -1)")
export QMAKE=${QMAKE:-$(command -v qmake6)}
export LD_LIBRARY_PATH="$adslib:${LD_LIBRARY_PATH:-}"
PATH="$TOOLS_DIR:$PATH" "$TOOLS_DIR/linuxdeploy-x86_64.AppImage" \
  --appdir "$AD" \
  --executable "$AD/usr/bin/Navigator" \
  --library "$AD/usr/lib/libvayu_sitl.so" \
  "${exe_args[@]}" \
  --desktop-file "$here/packaging/linux/navigator.desktop" \
  --icon-file "$here/packaging/linux/navigator.svg" --icon-filename navigator \
  --custom-apprun "$here/packaging/linux/AppRun" \
  --plugin qt

# --- licences ------------------------------------------------------------------
doc="$AD/usr/share/doc/navigator"
cp "$here/LICENSE.md" "$doc/LICENSE.md"
cp "$here/navigator/third_party/ads/NOTICE.md" "$doc/NOTICE-ads.md"
cp "$here/navigator/third_party/ads/LICENSE-LGPL-2.1.txt" "$doc/"
"$here/tools/release/make-notices.sh" linux "$doc/licenses" "$doc/THIRD-PARTY-NOTICES.md" "$AD"

# --- the image ----------------------------------------------------------------
out="$OUT_DIR/navigator-${VERSION}-${CHANNEL}-linux-x86_64.AppImage"
ARCH=x86_64 "$TOOLS_DIR/appimagetool-x86_64.AppImage" --no-appstream "$AD" "$out"
ls -l "$out"
echo "built $out"
