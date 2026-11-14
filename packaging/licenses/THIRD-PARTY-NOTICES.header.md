# Third-party notices

Navigator is licensed under the Apache License 2.0 (`LICENSE.md`). It is
distributed together with the third-party components below. The section
"Components in this build" at the end of this file is generated from the
package manager of the machine that built the bundle, so it lists what is
really inside, with exact versions. Full licence texts are in `licenses/`.

## Qt 6 (LGPL-3.0)

Navigator uses the Qt 6 toolkit (Core, Gui, Widgets, Network, OpenGL,
OpenGLWidgets, SerialPort, Concurrent, and the platform and image plugins that
go with them) under the GNU Lesser General Public License, version 3
(`licenses/LGPL-3.0.txt`, which incorporates `licenses/GPL-3.0.txt`).
Copyright (C) The Qt Company Ltd. and other contributors. Some Qt modules or
third-party code inside Qt carry other licences; see the per-package copyright
files in `licenses/third-party/`.

- Qt is linked **dynamically**: it is shipped as separate shared libraries
  (`.so` on Linux, `.dll` on Windows) and is **not modified**.
- You may replace those libraries with your own build of a compatible Qt
  (same major.minor series, ABI compatible). On Linux, put your libraries in
  front with `LD_LIBRARY_PATH` or replace the files in the unpacked AppImage
  (`./navigator-*.AppImage --appimage-extract`, then run
  `squashfs-root/AppRun`); on Windows replace the DLLs in the install folder.
  This is the "relinking" the LGPL asks for: Navigator's own object code is not
  statically bound to Qt.
- Qt version in this build and where to get the exact source: see the
  generated section at the end. Qt source is published at
  `https://download.qt.io/official_releases/qt/<major.minor>/<version>/submodules/`
  (for example `.../qt/6.4/6.4.2/submodules/qtbase-everywhere-src-6.4.2.tar.xz`).
  Linux builds use the distribution's packages: the source is available with
  `apt-get source qt6-base` on the same Ubuntu release, or from
  `https://packages.ubuntu.com/source/<release>/qt6-base`. Windows builds use
  the MSYS2 packages: the build recipe and source references are at
  `https://github.com/msys2/MINGW-packages/tree/master/mingw-w64-qt6-base`.

## Qt Advanced Docking System (LGPL-2.1-or-later)

The dockable window layout uses Qt Advanced Docking System 5.1.1
(`https://github.com/githubuser0xFFFF/Qt-Advanced-Docking-System`, tag
`v5.1.1`, commit `023ce95934fecfd4cf5c672c9c404fabe0f54923`), Copyright (C)
2017 Uwe Kindler and contributors, under the GNU Lesser General Public
License, version 2.1 or later (`licenses/LGPL-2.1.txt`). It is built from the
unmodified upstream source as a **shared library** (`libqtadvanceddocking-qt6`)
and may be replaced by your own build in the same way as Qt above.

## Open Asset Import Library, assimp (BSD-3-Clause)

The simulator imports airframe meshes with assimp, Copyright (C) 2006-2022
the assimp development team, under the 3-clause BSD licence (text in
`licenses/third-party/` as part of the assimp package copyright file; the
Linux AppImage includes it). The Windows build does not include the simulator
and does not ship assimp.

## nav (Apache-2.0)

The `nav` command-line tool (`https://github.com/ragnar-vallhala/nav`),
Copyright 2026 NAVRobotec Pvt Ltd, is shipped beside Navigator under the
Apache License 2.0 (`licenses/Apache-2.0.txt`). It bundles the DejaVu Sans
Mono font under the licence in `share/nav/fonts/LICENSE.DejaVu`.

## Flashing tools (Linux AppImage only)

The AppImage ships `st-flash` (stlink-tools, BSD-3-Clause), `openocd`
(GPL-2.0-or-later) and `avrdude` (GPL-2.0-or-later) with their shared library
dependencies, unmodified, as the distribution built them. Their licences and
versions are in the generated section below and in `licenses/third-party/`.

## Source offer

For every component above that is licensed under the LGPL or GPL, the
corresponding source code is the unmodified upstream or distribution source at
the version listed in the generated section. It is available at the locations
given above. In addition, NAVRobotec Pvt Ltd will, for three years from the
date of the release you received, send you the corresponding source code of
any of these components on request, for no more than the cost of physically
performing the distribution: write to `support@navrobotec.com`, naming the
release file and the component.

This file is a notice, not legal advice.
