# Workspace mock: try it

A stand-alone app (no engine, no SITL SDK, no flight controller) for trying
dockable panels and named workspaces before they go into the real window. The
docking layer it uses is `src/ui/workspace/`; the panels are placeholders,
except `Simulator view`, which is a live OpenGL animation.

It needs Qt Advanced Docking System (ADS, LGPL-2.1, see
`third_party/ads/NOTICE.md`). CMake fetches a pinned release and builds it as a
shared library when asked to, which needs git and network access once.

## Build

Linux (Debian/Ubuntu). `qt6-base-private-dev` is required by ADS:

    sudo apt-get install cmake ninja-build qt6-base-dev qt6-base-private-dev \
        qt6-serialport-dev libgl1-mesa-dev libassimp-dev libpulse-dev python3-numpy
    cd navigator
    cmake -S . -B build-ws -G Ninja -DNAVIGATOR_SITL=OFF -DNAVIGATOR_FETCH_ADS=ON
    cmake --build build-ws --target workspace_mock

Offline, or with ADS already installed or checked out:

    # a local checkout of ADS v5.1.1 (unmodified)
    cmake ... -DNAVIGATOR_FETCH_ADS=ON -DFETCHCONTENT_SOURCE_DIR_ADS=/path/to/ads
    # or an installed copy
    cmake ... -DCMAKE_PREFIX_PATH=/path/to/ads/prefix

Without either, the mock and its tests are skipped and the rest of the Navigator
build is unaffected. Windows (MSYS2 UCRT64) uses the same options; it has not
been tried there by the author.

## Run

    build-ws/tools/workspace_mock/workspace_mock

State (the last layout and your saved workspaces) is kept in the usual Qt
settings file for organisation "NAVRobotec", application
"navigator-workspace-mock". Delete it, or run with `XDG_CONFIG_HOME=$(mktemp -d)`,
to start clean.

## What to try

1. **Workspaces menu**: switch between Fly (the default), Sim, Tune and Debug
   link. Each rebuilds the whole layout.
2. **Drag a panel tab** out of the window to float it; drop it on the edges of
   another panel or onto its tab bar to dock or tab it. Float `Simulator view`
   and watch the frame counter: it keeps animating, and `init` stays at 1.
3. **Workspaces > Save layout as...** with a name, change the layout, switch to
   another workspace, and come back. Rename and Delete are submenus of the same menu.
4. **Workspaces > Reset layout** returns the current workspace to its built-in form.
5. **Panels menu**: hide and show panels; the layout is kept.
6. **Close and start again**: the last layout comes back, including floating
   windows; a floating window that would be off-screen (a monitor was
   unplugged) is pulled back on.

`NAV_MOCK_GL=widget` swaps the 3D view for a `QOpenGLWidget`, to see the problem
the default avoids: under ADS 5.1.1 on Qt 6.4 it re-initialises on every move
between windows and the main window stops repainting. The default hosts the view
in a `QOpenGLWindow` through `QWidget::createWindowContainer`, which does not.

## Scripted run and screenshots

`--script "step;step"` drives it without a mouse. Steps: `workspace:NAME`,
`float:ID`, `tab:ID+ONTO`, `hide:ID`, `show:ID`, `save:NAME`, `rename:A>B`,
`delete:NAME`, `reset`, `geom:ID,x,y,w,h`, `shot:PATH`, `quit`. Panel ids are
listed in `MockPanels.h`.

    QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1 \
      workspace_mock --script "workspace:Sim;float:sim.view;shot:/tmp/sim.png;quit"

Use a real X server or a private Xvfb (`Xvfb :93 -screen 0 1280x800x24
+extension GLX`, then `DISPLAY=:93`); `QT_QPA_PLATFORM=offscreen` cannot do GL.

## Tests

    ctest --test-dir build-ws -R tst_workspace_manager       # 19 cases, no display
    QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1 DISPLAY=:93 \
      ctest --test-dir build-ws -R tst_workspace_gl          # needs an X server

`tst_workspace_gl` skips itself under `offscreen`. One of its cases is an
expected failure (a `QOpenGLWidget` under ADS): it passes while the problem
exists and fails when it is fixed, as a prompt to remove the workaround.

## The same layer in the real window

`src/ui/main/DockShell` hosts every Navigator page as a panel on this layer
(ids `view3d imu logs rc packets calibration motors control_loop gyro_notch perf
sim settings`; workspaces Fly, Tune, Debug link, Sim, RC channels, Motors, Control loop,
Packets, Kernel perf). It is the window's only layout (there is no switch and
no View menu): Workspaces and Panels menus, and Ctrl+1..6, Ctrl+8 switch workspace (Fly, RC channels, Motors,
Control loop, Packets, Sim, Kernel perf).

GL decision: the attitude panel (`view3d`) holds two `QOpenGLWidget`s, which
cannot move between windows under ADS 5.1.1 on Qt 6.4, so that panel is pinned
to the main window (`DockBackend::setPanelFixed`): it can be hidden, shown and
split next to, but not floated. The simulator's 3D view is already a
`QOpenGLWindow` in a window container, so the `sim` panel floats freely.
