# Qt Advanced Docking System (ADS)

| | |
|---|---|
| Project | Qt Advanced Docking System, <https://github.com/githubuser0xFFFF/Qt-Advanced-Docking-System> |
| Version | 5.1.1 (tag `v5.1.1`, commit `023ce95934fecfd4cf5c672c9c404fabe0f54923`) |
| Copyright | (C) 2017 Uwe Kindler and contributors |
| Licence | GNU Lesser General Public License, version 2.1 or later; full text in [LICENSE-LGPL-2.1.txt](LICENSE-LGPL-2.1.txt) |

## How Navigator uses it

ADS is not stored in this repository. `tools/workspace_mock/CMakeLists.txt`
fetches that exact commit when configured with `-DNAVIGATOR_FETCH_ADS=ON`, and
builds it, unmodified, as a **shared** library (`libqtadvanceddocking-qt6.so`,
`.dll` on Windows). It is linked by the workspace mock and the workspace tests
only; the `Navigator` application does not link it at this time.

## Keeping the LGPL conditions

- ADS is built as a shared library, never statically, so a user can replace it
  with their own build of the same or a modified ADS (`BUILD_STATIC` is forced
  OFF in the CMake file; do not change that).
- The ADS sources are used as they are upstream. If a patch is ever needed it
  must be published (LGPL section 2) and listed here.
- If a binary release ever contains ADS (the mock, or `Navigator` once it
  adopts the docking layer), ship with it: this notice, the licence text, the
  shared library as a separate file, and a pointer to the source above at the
  pinned commit. The About dialog (`src/ui/widgets/AboutDialog.cpp`) and the
  Windows bundle (`tools/windows/`) are the places to list it; neither lists
  third-party libraries yet (not even Qt or assimp), so nothing was added there
  for a library the application does not ship.

This file is a record for developers, not legal advice.
