#!/usr/bin/env bash
# Generate THIRD-PARTY-NOTICES.md plus the licence texts for a release bundle.
#
#   make-notices.sh linux   <licenses-out-dir> <notices-file> <AppDir>
#   make-notices.sh windows <licenses-out-dir> <notices-file> <dist-dir>
#
# The "Components in this build" table is derived from the bundle itself, not
# from a hand-kept list: every shared library / plugin in the bundle is mapped
# to the package that owns it on the build machine (dpkg on Linux, pacman on
# MSYS2), and that package's copyright / licence files are copied. Anything it
# cannot map is listed as UNMAPPED so a human sees it; it is never guessed.
set -euo pipefail
platform=${1:?linux|windows}; lic=${2:?licenses dir}; notices=${3:?notices file}; root=${4:?bundle root}
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
mkdir -p "$lic/third-party"
cp "$here"/packaging/licenses/*.txt "$lic/"
rows=$(mktemp); trap 'rm -f "$rows"' EXIT
unmapped=0

if [ "$platform" = linux ]; then
  qtver=$(qmake6 -query QT_VERSION 2>/dev/null || echo unknown)
  ldc=$(ldconfig -p)   # once; not piped into an early-exiting awk (SIGPIPE + pipefail)
  declare -A seen owner
  syslist=$(mktemp); trap 'rm -f "$rows" "$syslist"' EXIT
  # 1. system path of every bundled library / plugin / tool (the AppDir holds copies)
  while IFS= read -r f; do
    b=$(basename "$f")
    case "$b" in
      Navigator|nav|qt.conf|libvayu_sitl.so*|libqtadvanceddocking-qt6.so*) continue ;;
    esac
    sys=""
    if [[ "$f" == */plugins/* ]]; then
      rel=${f#*/plugins/}
      for d in /usr/lib/x86_64-linux-gnu/qt6/plugins /usr/lib/qt6/plugins; do
        [ -e "$d/$rel" ] && sys="$d/$rel" && break
      done
    elif [[ "$f" == */share/openocd/* || "$f" == */etc/avrdude.conf ]]; then
      continue
    else
      sys=$(awk -v n="$b" '$1==n {print $NF; exit}' <<<"$ldc")
      [ -z "$sys" ] && [ -e "/usr/bin/$b" ] && sys="/usr/bin/$b"
      # libraries in a subdirectory of the multiarch lib dir (pulseaudio/, libproxy/, ...)
      [ -z "$sys" ] && sys=$(find /usr/lib/x86_64-linux-gnu -name "$b" -print -quit 2>/dev/null)
    fi
    if [ -z "$sys" ]; then echo "UNMAPPED: $f" >&2; unmapped=$((unmapped+1)); continue; fi
    printf '%s\n' "$(readlink -f "$sys")" >> "$syslist"
  done < <(find "$root" \( -name '*.so' -o -name '*.so.*' -o -path '*/bin/*' \) -type f -print | sort)
  # 2. ONE dpkg -S for all of them (per-file calls take minutes)
  while IFS= read -r line; do
    for pkg in $(echo "${line%%: *}" | tr -d ','); do pkg=${pkg%%:*}; seen[$pkg]=1; done
  done < <(xargs -d '\n' dpkg -S -- < "$syslist" 2>/dev/null | sort -u)
  for pkg in $(printf '%s\n' "${!seen[@]}" | sort); do
    ver=$(dpkg-query -W -f='${Version}\n' "$pkg" | head -1)
    src=$(dpkg-query -W -f='${source:Package} ${source:Version}\n' "$pkg" | head -1)
    printf '| %s | %s | source package `%s` | `licenses/third-party/%s.copyright` |\n' \
      "$pkg" "$ver" "$src" "$pkg" >> "$rows"
    cp "/usr/share/doc/$pkg/copyright" "$lic/third-party/$pkg.copyright" 2>/dev/null \
      || echo "no copyright file for $pkg" > "$lic/third-party/$pkg.copyright"
  done
  srcnote="Ubuntu $(. /etc/os-release; echo "$VERSION_ID"): \`apt-get source <source package>\` or \`https://packages.ubuntu.com/source/$(. /etc/os-release; echo "$VERSION_CODENAME")/<source package>\`."
else
  qtver=$(pacman -Q mingw-w64-ucrt-x86_64-qt6-base 2>/dev/null | awk '{print $2}' || true)
  qtver=${qtver%-*}   # drop the pkgrel; empty means unknown
  qtver=${qtver:-unknown}
  declare -A seen
  while IFS= read -r f; do
    b=$(basename "$f")
    [ "$b" = Navigator.exe ] && continue
    if [ "$(dirname "$f")" = "$root" ]; then sys="/ucrt64/bin/$b"          # plain DLL
    else sys="/ucrt64/share/qt6/plugins/${f#"$root"/}"; fi               # Qt plugin (platforms/, styles/, ...)
    pkg=$(pacman -Qoq "$sys" 2>/dev/null | head -1 || true)
    if [ -z "$pkg" ]; then echo "UNMAPPED: $f" >&2; unmapped=$((unmapped+1)); continue; fi
    seen[$pkg]=1
  done < <(find "$root" -name '*.dll' -type f -print | sort)
  for pkg in $(printf '%s\n' "${!seen[@]}" | sort); do
    ver=$(pacman -Q "$pkg" | awk '{print $2}')
    licn=$(pacman -Qi "$pkg" | sed -n 's/^Licenses *: *//p')
    printf '| %s | %s | MSYS2 package, source via its PKGBUILD | %s (`licenses/third-party/%s/`) |\n' \
      "$pkg" "$ver" "$licn" "$pkg" >> "$rows"
    mkdir -p "$lic/third-party/$pkg"
    cp -r "/ucrt64/share/licenses/${pkg#mingw-w64-ucrt-x86_64-}/." "$lic/third-party/$pkg/" 2>/dev/null \
      || echo "no licence directory for $pkg" > "$lic/third-party/$pkg/NONE.txt"
  done
  srcnote="MSYS2: \`https://github.com/msys2/MINGW-packages/tree/master/<package without the mingw-w64- prefix>\` (PKGBUILD with the upstream source URL and checksums)."
fi

{
  cat "$here/packaging/licenses/THIRD-PARTY-NOTICES.header.md"
  echo
  echo "## Components in this build (generated)"
  echo
  echo "Platform: $platform. **Qt version: $qtver.** Qt source for exactly this version:"
  echo "\`https://download.qt.io/official_releases/qt/${qtver%.*}/$qtver/submodules/\` ; also via the distribution source package below."
  echo
  echo "Source for the packages below: $srcnote"
  echo
  echo "| Package | Version | Source | Licence / copyright file |"
  echo "|---|---|---|---|"
  cat "$rows"
  echo
  echo "Built here from this repository (Apache-2.0 unless noted): Navigator, libvayu_sitl (firmware simulator module, Linux only), Qt Advanced Docking System (LGPL-2.1+, above)."
  [ "$unmapped" -eq 0 ] || echo "UNMAPPED files (not attributed to a package): $unmapped, see the build log."
} > "$notices"
echo "wrote $notices ($(grep -c '^| ' "$rows") packages, $unmapped unmapped)"
[ "${NOTICES_STRICT:-0}" = 1 ] && [ "$unmapped" -ne 0 ] && exit 1
exit 0
