# Unit tests (QtTest + ctest). Each test compiles the specific core sources
# it exercises directly — the core classes aren't a shared lib yet, so this
# keeps the test target self-contained without duplicating the GUI target.
#
# Tests run headless: ctest sets QT_QPA_PLATFORM=offscreen so the QApplication
# QTEST_MAIN spins up needs no display.

find_package(Qt6 REQUIRED COMPONENTS Test Widgets Core Network)

set(SRC ${CMAKE_SOURCE_DIR}/src)
set(REPLAY ${SRC}/replay)

# navigator_test(<name> <sources...>) — build + register one QtTest binary.
function(navigator_test name)
  qt_add_executable(${name} ${ARGN})
  target_include_directories(${name} PRIVATE
      ${SRC} ${SRC}/core ${SRC}/core/ui ${SRC}/comm ${SRC}/protocol)
  target_link_libraries(${name} PRIVATE
      Qt6::Test Qt6::Widgets Qt6::Core Qt6::Network Qt6::SerialPort)
  # qt_add_executable makes a GUI-subsystem exe on Windows, whose stdout is
  # lost: a failing QtTest then prints nothing under --output-on-failure.
  set_target_properties(${name} PROPERTIES WIN32_EXECUTABLE FALSE)
  add_test(NAME ${name} COMMAND ${name})
  set_tests_properties(${name} PROPERTIES
      ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
endfunction()

# ---- Start-command retry / ACK state machine (header only) -----------------
navigator_test(tst_command_retrier tst_command_retrier.cpp)
target_include_directories(tst_command_retrier PRIVATE ${SRC}/ui/widgets)

# ---- Time-sync estimator (NTP offset/delay, jitter filter, skew) ----------
navigator_test(tst_time_sync_estimator
    tst_time_sync_estimator.cpp
    ${SRC}/protocol/TimeSyncEstimator.cpp)
target_include_directories(tst_time_sync_estimator PRIVATE ${SRC}/protocol)

# ---- Battery: is it a pack, how many cells, what percent -------------------
navigator_test(tst_battery_estimator tst_battery_estimator.cpp)
target_include_directories(tst_battery_estimator PRIVATE ${SRC}/core)
target_link_libraries(tst_battery_estimator PRIVATE navlink_codec)

# ---- RC bridge failsafe policy (NO_SIGNAL on link loss, arm gating) --------
navigator_test(tst_rc_frame tst_rc_frame.cpp)
target_include_directories(tst_rc_frame PRIVATE ${SRC}/vsim)

# ---- CMD_SET_PID command codec (AT-1 foundation) --------------------------
navigator_test(tst_command_codec
    tst_command_codec.cpp
    ${SRC}/protocol/CommandCodec.cpp
    ${SRC}/core/crc.cpp)
target_include_directories(tst_command_codec PRIVATE ${SRC}/protocol ${SRC}/core)
# CommandCodec emits NavLink v2 (CMD_SET_PID); link the generated codec (its
# PUBLIC SYSTEM include carries navlink_msgs.h).
target_link_libraries(tst_command_codec PRIVATE navlink_codec)

# ---- AT-1: autotune gains -> CMD_SET_PID mapping --------------------------
navigator_test(tst_autotune_gains
    tst_autotune_gains.cpp)
target_include_directories(tst_autotune_gains PRIVATE ${SRC}/ui/widgets)

# ---- C++ autotune optimizer port ------------------------------------------
# Space.cpp: the all-optimizer convergence test runs on the real param bounds.
navigator_test(tst_optimizer
    tst_optimizer.cpp
    ${SRC}/autotune/Optimizer.cpp
    ${SRC}/autotune/Space.cpp)
target_include_directories(tst_optimizer PRIVATE ${SRC}/autotune)

# ---- C++ autotune cost functions ------------------------------------------
navigator_test(tst_cost
    tst_cost.cpp
    ${SRC}/autotune/Cost.cpp)
target_include_directories(tst_cost PRIVATE ${SRC}/autotune)

# ---- System-ID plant fit + analytic gain design ---------------------------
navigator_test(tst_sysid
    tst_sysid.cpp
    ${SRC}/autotune/SysId.cpp)
target_include_directories(tst_sysid PRIVATE ${SRC}/autotune)

# ---- System-ID end-to-end vs the real vayu_sitl_rtos binary (opt-in) -------
navigator_test(tst_sysid_rtos
    tst_sysid_rtos.cpp
    ${SRC}/autotune/SysId.cpp
    ${SRC}/autotune/RtosEval.cpp
    ${SRC}/autotune/Cost.cpp)
target_include_directories(tst_sysid_rtos PRIVATE ${SRC}/autotune)

# ---- Private firmware VFS dir: resolver + reset safety rule ----------------
navigator_test(tst_vfs_dir tst_vfs_dir.cpp ${SRC}/vsim/VfsDir.cpp)
target_include_directories(tst_vfs_dir PRIVATE ${SRC}/vsim ${CMAKE_CURRENT_SOURCE_DIR})

# ---- C++ autotune engine (Space + orchestration, synthetic rollout) -------
navigator_test(tst_autotune_engine
    tst_autotune_engine.cpp
    ${SRC}/autotune/AutotuneEngine.cpp
    ${SRC}/autotune/Space.cpp
    ${SRC}/autotune/Optimizer.cpp
    ${SRC}/autotune/Cost.cpp)
target_include_directories(tst_autotune_engine PRIVATE ${SRC}/autotune)

# ---- Phase 1A: command registry -------------------------------------------
navigator_test(tst_command_registry
    tst_command_registry.cpp
    ${SRC}/core/CommandRegistry.cpp)

# ---- Phase 1C: record format round-trip -----------------------------------
navigator_test(tst_record_sink
    tst_record_sink.cpp
    ${REPLAY}/RecordSink.cpp)
target_include_directories(tst_record_sink PRIVATE ${REPLAY})

# ---- Phase C: telemetry-source state machine (full transition matrix) ------
navigator_test(tst_source_controller
    tst_source_controller.cpp
    ${SRC}/core/SourceController.cpp)
target_include_directories(tst_source_controller PRIVATE ${SRC}/core)

# ---- Phase 2A: shortcuts manager (overrides + conflicts + persistence) -----
navigator_test(tst_shortcuts_manager
    tst_shortcuts_manager.cpp
    ${SRC}/core/ShortcutsManager.cpp
    ${SRC}/core/CommandRegistry.cpp)

# ---- Phase 2A: shortcuts editor dialog (GUI smoke) ------------------------
navigator_test(tst_shortcuts_editor
    tst_shortcuts_editor.cpp
    ${SRC}/ui/widgets/ShortcutsEditorDialog.cpp
    ${SRC}/core/ShortcutsManager.cpp
    ${SRC}/core/CommandRegistry.cpp
    ${SRC}/core/Notify.cpp
    ${SRC}/core/ToastOverlay.cpp)
target_include_directories(tst_shortcuts_editor PRIVATE ${SRC}/ui/widgets)

# ---- Phase 2B: command palette (fuzzy matcher + popup smoke) ---------------
navigator_test(tst_command_palette
    tst_command_palette.cpp
    ${SRC}/ui/widgets/CommandPalette.cpp
    ${SRC}/core/CommandRegistry.cpp)
target_include_directories(tst_command_palette PRIVATE ${SRC}/ui/widgets)

# ---- Phase 2C: view history (MRU logic) -----------------------------------
navigator_test(tst_view_history
    tst_view_history.cpp
    ${SRC}/core/ViewHistory.cpp)

# ---- Calibration wizard step machine --------------------------------------
navigator_test(tst_calibration_wizard
    tst_calibration_wizard.cpp
    ${SRC}/core/CalibrationWizard.cpp)
target_include_directories(tst_calibration_wizard PRIVATE ${SRC}/core)

# ---- Phase 2D: replay source (playback clock, crop, loop, seek) ------------
navigator_test(tst_replay_source
    tst_replay_source.cpp
    ${SRC}/comm/ITelemetrySource.h  # header-only Q_OBJECT base — needs AUTOMOC
    ${REPLAY}/ReplaySource.cpp
    ${REPLAY}/RecordSink.cpp)
target_include_directories(tst_replay_source PRIVATE ${REPLAY})

# ---- Phase 2E: replay bar (transport GUI smoke) ---------------------------
navigator_test(tst_replay_bar
    tst_replay_bar.cpp
    ${SRC}/comm/ITelemetrySource.h
    ${SRC}/ui/widgets/ReplayBar.cpp
    ${REPLAY}/ReplaySource.cpp
    ${REPLAY}/RecordSink.cpp)
target_include_directories(tst_replay_bar PRIVATE ${REPLAY} ${SRC}/ui/widgets)

# ---- Phase 3A/3B: about dialog + docs path --------------------------------
navigator_test(tst_about_dialog
    tst_about_dialog.cpp
    ${SRC}/ui/widgets/AboutDialog.cpp)
target_include_directories(tst_about_dialog PRIVATE ${SRC}/ui/widgets)

# ---- Procedural world generation (Phase 0): noise/heightfield/terrain ------
# Qt-free core; the test just needs QtTest as a harness. Include dir is
# ${SRC}/vsim so the "procgen/..." paths resolve.
navigator_test(tst_procgen
    tst_procgen.cpp
    ${SRC}/vsim/procgen/Noise.cpp
    ${SRC}/vsim/procgen/TerrainGen.cpp
    ${SRC}/vsim/procgen/TerrainField.cpp
    ${SRC}/vsim/procgen/WorldPresets.cpp
    ${SRC}/vsim/procgen/Flora.cpp
    ${SRC}/vsim/ChunkStreamer.cpp)
target_include_directories(tst_procgen PRIVATE ${SRC}/vsim)

# ---- Procedural world presets (Qt-free core, headless) ----------------------
navigator_test(tst_worldpresets
    tst_worldpresets.cpp
    ${SRC}/vsim/procgen/Noise.cpp
    ${SRC}/vsim/procgen/TerrainField.cpp
    ${SRC}/vsim/procgen/WorldPresets.cpp)
target_include_directories(tst_worldpresets PRIVATE ${SRC}/vsim)

# ---- Procedural drone model generator (Qt-free core, headless) -------------
navigator_test(tst_dronemodel
    tst_dronemodel.cpp
    ${SRC}/vsim/DroneModel.cpp)
target_include_directories(tst_dronemodel PRIVATE ${SRC}/vsim)

# ---- 3D sim view survives its GL context being recreated ------------------
# Needs a real GL platform (xcb on an Xvfb, LIBGL_ALWAYS_SOFTWARE=1): it skips on
# the offscreen platform ctest uses, which crashes QOpenGLWidget/QOpenGLWindow.
if(NAVIGATOR_SITL AND EXISTS "${VAYU_SITL_SDK}/vayu_sitl_abi.h")
  find_package(Qt6 REQUIRED COMPONENTS OpenGL OpenGLWidgets)
  navigator_test(tst_sim_gl_context
      tst_sim_gl_context.cpp
      ${SRC}/vsim/SimRendererWidget.cpp ${SRC}/vsim/SimRendererWidget.h
      ${SRC}/vsim/DroneModel.cpp)
  target_include_directories(tst_sim_gl_context PRIVATE ${SRC}/vsim ${VAYU_SITL_SDK})
  target_link_libraries(tst_sim_gl_context PRIVATE Qt6::OpenGL Qt6::OpenGLWidgets)
endif()

# ---- World tab preset wiring (headless widget test) ------------------------
if(NAVIGATOR_SITL)
  navigator_test(tst_worldeditor
      tst_worldeditor.cpp
      ${SRC}/ui/widgets/WorldEditorWidget.cpp
      ${SRC}/ui/widgets/CollapsibleSection.cpp
      ${SRC}/core/Theme.cpp
      ${SRC}/core/ui/Buttons.h # Q_OBJECT header: moc must see it
      ${SRC}/vsim/ProceduralWorld.cpp
      ${SRC}/vsim/procgen/Noise.cpp
      ${SRC}/vsim/procgen/TerrainGen.cpp
      ${SRC}/vsim/procgen/TerrainField.cpp
      ${SRC}/vsim/procgen/WorldPresets.cpp)
  target_include_directories(tst_worldeditor PRIVATE ${SRC}/vsim ${SRC}/ui/widgets)
endif()

# ---- Side-panel sizing helpers (no splitter may crop a panel) -------------
navigator_test(tst_panel_widgets tst_panel_widgets.cpp)

# ---- In-app simulator end to end (SITL builds only) -----------------------
# Navigator's SitlModule + SimWorker + RcBridge against the real engine module:
# it loads, boots and steps, telemetry reaches the sink, and RcBridge's RC gets
# through the pty (a named pipe on Windows) into the firmware. It needs the
# module itself, so VAYU_SITL_MODULE points at the build root, where CI and a
# developer put it; without one the test skips and says why.
if(NAVIGATOR_SITL AND EXISTS "${VAYU_SITL_SDK}/vayu_sitl_abi.h")
  navigator_test(tst_sitl_engine
      tst_sitl_engine.cpp
      ${SRC}/vsim/SitlModule.cpp ${SRC}/vsim/SitlModule.h
      ${SRC}/vsim/VfsDir.cpp ${SRC}/vsim/VfsDir.h
      ${SRC}/vsim/SimWorker.cpp ${SRC}/vsim/SimWorker.h
      ${SRC}/vsim/RcBridge.cpp ${SRC}/vsim/RcBridge.h)
  target_include_directories(tst_sitl_engine PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${SRC}/vsim ${VAYU_SITL_SDK})
  target_link_libraries(tst_sitl_engine PRIVATE navlink_codec)
  if(WIN32)
    target_link_libraries(tst_sitl_engine PRIVATE winmm)
  endif()
  set_tests_properties(tst_sitl_engine PROPERTIES
      ENVIRONMENT "QT_QPA_PLATFORM=offscreen;VAYU_SITL_MODULE=${CMAKE_BINARY_DIR}/libvayu_sitl${CMAKE_SHARED_MODULE_SUFFIX}"
      TIMEOUT 60)
endif()

# ---- Airframe presets: data checks (no engine needed) ---------------------
navigator_test(tst_airframe_presets
    tst_airframe_presets.cpp
    ${SRC}/vsim/AirframePresets.cpp
    ${SRC}/vsim/VehicleJson.cpp)
target_include_directories(tst_airframe_presets PRIVATE ${SRC}/vsim ${SRC}/ui/widgets)
target_compile_definitions(tst_airframe_presets PRIVATE
    AIRFRAME_VEHICLE_DIR="${CMAKE_SOURCE_DIR}/docs/vehicles")

# ---- Airframe presets flown in the real firmware (SITL builds only) -------
# Hover + yaw-sign per preset. Needs the engine module like tst_sitl_engine and
# skips, saying why, without one. About a minute of wall-clock-paced sim.
if(NAVIGATOR_SITL AND EXISTS "${VAYU_SITL_SDK}/vayu_sitl_abi.h")
  navigator_test(tst_airframe_flight
      tst_airframe_flight.cpp
      ${SRC}/vsim/SitlModule.cpp ${SRC}/vsim/SitlModule.h
      ${SRC}/vsim/VfsDir.cpp ${SRC}/vsim/VfsDir.h
      ${SRC}/vsim/SimWorker.cpp ${SRC}/vsim/SimWorker.h
      ${SRC}/vsim/AirframePresets.cpp
      ${SRC}/protocol/CommandCodec.cpp
      ${SRC}/core/crc.cpp)
  target_include_directories(tst_airframe_flight PRIVATE
      ${SRC}/vsim ${SRC}/ui/widgets ${SRC}/protocol ${SRC}/core ${VAYU_SITL_SDK})
  target_link_libraries(tst_airframe_flight PRIVATE navlink_codec)
  set_tests_properties(tst_airframe_flight PROPERTIES
      ENVIRONMENT "QT_QPA_PLATFORM=offscreen;VAYU_SITL_MODULE=${CMAKE_BINARY_DIR}/libvayu_sitl${CMAKE_SHARED_MODULE_SUFFIX}"
      TIMEOUT 300)

  # The Vehicle page's Preset combo: picking a preset keeps it, any manual edit
  # of the airframe form flips it back to Custom. Needs the editor widget, so
  # it builds with the mesh loader (assimp) like the app does.
  find_package(assimp REQUIRED)
  navigator_test(tst_geometry_editor_preset
      tst_geometry_editor_preset.cpp
      ${SRC}/ui/widgets/GeometryEditorWidget.cpp ${SRC}/ui/widgets/GeometryEditorWidget.h
      ${SRC}/ui/widgets/CollapsibleSection.cpp ${SRC}/ui/widgets/CollapsibleSection.h
      ${SRC}/core/Theme.cpp ${SRC}/core/ui/Buttons.h
      ${SRC}/vsim/MeshLoader.cpp ${SRC}/vsim/MassProperties.cpp
      ${SRC}/vsim/AirframePresets.cpp ${SRC}/vsim/VehicleJson.cpp)
  target_include_directories(tst_geometry_editor_preset PRIVATE
      ${SRC}/vsim ${SRC}/ui/widgets)
  target_link_libraries(tst_geometry_editor_preset PRIVATE assimp::assimp)
endif()

# ---- Simulator Stop -> Start twice against the real engine ------------------
# Spawn pose, Start commands ACK OK, a single RC feeder, a steady arm state.
# Needs the engine module like tst_sitl_engine; skips, saying why, without one.
if(NAVIGATOR_SITL AND EXISTS "${VAYU_SITL_SDK}/vayu_sitl_abi.h")
  navigator_test(tst_sim_restart
      tst_sim_restart.cpp
      ${SRC}/vsim/SitlModule.cpp ${SRC}/vsim/SitlModule.h
      ${SRC}/vsim/VfsDir.cpp ${SRC}/vsim/VfsDir.h
      ${SRC}/vsim/SimWorker.cpp ${SRC}/vsim/SimWorker.h
      ${SRC}/protocol/CommandCodec.cpp ${SRC}/protocol/CommandCodec.h
      ${SRC}/core/crc.cpp)
  target_include_directories(tst_sim_restart PRIVATE
      ${SRC}/vsim ${SRC}/ui/widgets ${VAYU_SITL_SDK})
  target_link_libraries(tst_sim_restart PRIVATE navlink_codec)
  if(WIN32)
    target_link_libraries(tst_sim_restart PRIVATE winmm)
  endif()
  set_tests_properties(tst_sim_restart PROPERTIES
      ENVIRONMENT "QT_QPA_PLATFORM=offscreen;VAYU_SITL_MODULE=${CMAKE_BINARY_DIR}/libvayu_sitl${CMAKE_SHARED_MODULE_SUFFIX}"
      TIMEOUT 120)
endif()
