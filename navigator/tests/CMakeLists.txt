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
  # The dock library is a shared library; Windows finds a DLL only beside the
  # exe (or on PATH), so put a copy next to each test binary.
  if(WIN32 AND TARGET ads::qtadvanceddocking-qt6)
    add_custom_command(TARGET ${name} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              $<TARGET_FILE:ads::qtadvanceddocking-qt6> $<TARGET_FILE_DIR:${name}>)
  endif()
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

# ---- Battery warn/crit alert tracker (thresholds, hysteresis, debounce) ----
navigator_test(tst_battery_alert tst_battery_alert.cpp)
target_include_directories(tst_battery_alert PRIVATE ${SRC}/core)
target_link_libraries(tst_battery_alert PRIVATE navlink_codec)

# ---- FAILSAFE_STATUS display rules (unknown until seen, refusal only disarmed) ----
navigator_test(tst_failsafe_status tst_failsafe_status.cpp)
target_include_directories(tst_failsafe_status PRIVATE ${SRC}/core)
target_link_libraries(tst_failsafe_status PRIVATE navlink_codec)

# ---- Battery calibration: fit, band, 16-byte file, dialog ------------------
navigator_test(tst_battery_calib
    tst_battery_calib.cpp
    ${SRC}/core/BatteryCalib.cpp
    ${SRC}/core/XferUploader.cpp
    ${SRC}/ui/widgets/BatteryCalibDialog.cpp)
target_include_directories(tst_battery_calib PRIVATE ${SRC}/core)

# ---- XFER upload client vs a fake peer, wire encoders, calib dialog upload --
navigator_test(tst_xfer_uploader
    tst_xfer_uploader.cpp
    ${SRC}/core/XferUploader.cpp
    ${SRC}/core/BatteryCalib.cpp
    ${SRC}/ui/widgets/BatteryCalibDialog.cpp
    ${SRC}/protocol/CommandCodec.cpp
    ${SRC}/core/crc.cpp)
target_include_directories(tst_xfer_uploader PRIVATE ${SRC}/core ${SRC}/protocol)
target_link_libraries(tst_xfer_uploader PRIVATE navlink_codec)

# ---- RC bridge failsafe policy (NO_SIGNAL on link loss, arm gating) --------
navigator_test(tst_rc_frame tst_rc_frame.cpp)
target_include_directories(tst_rc_frame PRIVATE ${SRC}/vsim)
# RcBridge's UART reader end to end over a pty (POSIX only).
if(NOT WIN32)
  navigator_test(tst_rc_uart_bridge
      tst_rc_uart_bridge.cpp ${SRC}/vsim/RcBridge.cpp ${SRC}/vsim/RcBridge.h)
  target_include_directories(tst_rc_uart_bridge PRIVATE ${SRC}/vsim)
endif()

# ---- CMD_SET_PID command codec (AT-1 foundation) --------------------------
navigator_test(tst_command_codec
    tst_command_codec.cpp
    ${SRC}/protocol/CommandCodec.cpp
    ${SRC}/core/crc.cpp)
target_include_directories(tst_command_codec PRIVATE ${SRC}/protocol ${SRC}/core)
# CommandCodec emits NavLink v2 (CMD_SET_PID); link the generated codec (its
# PUBLIC SYSTEM include carries navlink_msgs.h).
target_link_libraries(tst_command_codec PRIVATE navlink_codec)

# ---- Parameter service: catalog, codec and client against a fake FC -------
navigator_test(tst_param_client
    tst_param_client.cpp
    ${SRC}/params/ParamCatalog.cpp
    ${SRC}/params/ParamCodec.cpp
    ${SRC}/params/ParamClient.cpp)
target_include_directories(tst_param_client PRIVATE ${SRC}/params ${SRC}/protocol)
target_compile_definitions(tst_param_client PRIVATE NAVLINK_GEN_DIR="${NAVLINK_GEN}")
target_link_libraries(tst_param_client PRIVATE navlink_codec)

# ---- Parameter service: heartbeat, sync and router thread hop -> client -----
navigator_test(tst_param_service
    tst_param_service.cpp
    ${SRC}/params/ParamCatalog.cpp
    ${SRC}/params/ParamCodec.cpp
    ${SRC}/params/ParamClient.cpp
    ${SRC}/params/ParamService.cpp
    ${SRC}/protocol/NavlinkRouter.cpp)
target_include_directories(tst_param_service PRIVATE ${SRC}/params ${SRC}/protocol)
target_link_libraries(tst_param_service PRIVATE navlink_codec)
target_sources(tst_param_service PRIVATE ${PARAMS_CATALOG_QRC})

navigator_test(tst_params_panel
    tst_params_panel.cpp
    ${SRC}/params/ParamCatalog.cpp
    ${SRC}/params/ParamCodec.cpp
    ${SRC}/params/ParamClient.cpp
    ${SRC}/params/ParamService.cpp
    ${SRC}/params/ParamApplier.cpp
    ${SRC}/params/ParamFile.cpp
    ${SRC}/ui/widgets/ParamsPanel.cpp
    ${SRC}/ui/widgets/ParamFileDialog.cpp
    ${SRC}/protocol/NavlinkRouter.cpp)
target_include_directories(tst_params_panel PRIVATE ${SRC}/params ${SRC}/protocol ${SRC}/ui/widgets)
target_link_libraries(tst_params_panel PRIVATE navlink_codec)
target_sources(tst_params_panel PRIVATE ${PARAMS_CATALOG_QRC})

navigator_test(tst_gyro_notch_widget
    tst_gyro_notch_widget.cpp
    ${SRC}/ui/widgets/GyroNotchWidget.cpp
    ${SRC}/protocol/DroneProtocol.cpp
    ${SRC}/protocol/NavlinkRouter.cpp
    ${SRC}/protocol/TelemetryEngine.cpp
    ${SRC}/protocol/CommandCodec.cpp
    ${SRC}/params/ParamCatalog.cpp
    ${SRC}/params/ParamCodec.cpp
    ${SRC}/params/ParamClient.cpp
    ${SRC}/params/ParamService.cpp
    ${SRC}/params/ParamApplier.cpp
    ${SRC}/core/Notify.cpp
    ${SRC}/core/ToastOverlay.cpp
    ${SRC}/replay/RecordSink.cpp
    ${SRC}/comm/SerialManager.cpp
    ${SRC}/comm/UdpManager.cpp)
target_include_directories(tst_gyro_notch_widget PRIVATE ${SRC}/params ${SRC}/protocol ${SRC}/ui/widgets ${SRC}/ui/main ${SRC}/core ${SRC}/replay ${SRC}/comm)
target_link_libraries(tst_gyro_notch_widget PRIVATE navlink_codec)
target_sources(tst_gyro_notch_widget PRIVATE ${PARAMS_CATALOG_QRC})

navigator_test(tst_param_file
    tst_param_file.cpp
    ${SRC}/params/ParamCatalog.cpp
    ${SRC}/params/ParamFile.cpp)
target_include_directories(tst_param_file PRIVATE ${SRC}/params ${SRC}/protocol)
target_compile_definitions(tst_param_file PRIVATE NAVLINK_GEN_DIR="${NAVLINK_GEN}")
target_link_libraries(tst_param_file PRIVATE navlink_codec)

navigator_test(tst_param_applier
    tst_param_applier.cpp
    ${SRC}/params/ParamCatalog.cpp
    ${SRC}/params/ParamCodec.cpp
    ${SRC}/params/ParamClient.cpp
    ${SRC}/params/ParamService.cpp
    ${SRC}/params/ParamApplier.cpp
    ${SRC}/protocol/NavlinkRouter.cpp)
target_include_directories(tst_param_applier PRIVATE ${SRC}/params ${SRC}/protocol)
target_link_libraries(tst_param_applier PRIVATE navlink_codec)
target_sources(tst_param_applier PRIVATE ${PARAMS_CATALOG_QRC})

# ---- AT-1: autotune gains -> CMD_SET_PID mapping --------------------------
navigator_test(tst_autotune_gains
    tst_autotune_gains.cpp)
target_include_directories(tst_autotune_gains PRIVATE ${SRC}/ui/widgets)

navigator_test(tst_autotune_param_map
    tst_autotune_param_map.cpp
    ${SRC}/params/ParamCatalog.cpp)
target_include_directories(tst_autotune_param_map PRIVATE ${SRC}/ui/widgets ${SRC}/params ${SRC}/protocol)
target_compile_definitions(tst_autotune_param_map PRIVATE NAVLINK_GEN_DIR="${NAVLINK_GEN}")
target_link_libraries(tst_autotune_param_map PRIVATE navlink_codec)

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

# ---- Simulator default paths (log dir, stale world tmp sweep) --------------
navigator_test(tst_sim_paths tst_sim_paths.cpp ${SRC}/vsim/SimPaths.cpp)

# ---- User presets store (pure core, also in the GCS-only build) ------------
navigator_test(tst_user_presets tst_user_presets.cpp ${SRC}/vsim/UserPresets.cpp)
target_include_directories(tst_user_presets PRIVATE ${SRC}/vsim)
target_include_directories(tst_sim_paths PRIVATE ${SRC}/vsim)

# ---- Per-run flight event log (ev-*.jsonl) + run retention ----------------
navigator_test(tst_flight_event_log
    tst_flight_event_log.cpp ${SRC}/vsim/FlightEventLog.cpp)
target_include_directories(tst_flight_event_log PRIVATE ${SRC}/vsim)

# ---- crash status: pose read zeroes first, banner text (pose proto v5) -----
if(NAVIGATOR_SITL AND EXISTS "${VAYU_SITL_SDK}/vsim_proto.h")
  navigator_test(tst_crash_status tst_crash_status.cpp)
  target_include_directories(tst_crash_status PRIVATE ${SRC}/vsim ${VAYU_SITL_SDK})
endif()

# ---- SitlModule load-failure text (needs only the ABI header) ---------------
if(NAVIGATOR_SITL AND EXISTS "${VAYU_SITL_SDK}/vayu_sitl_abi.h")
  navigator_test(tst_sitl_module_error
      tst_sitl_module_error.cpp
      ${SRC}/vsim/SitlModule.cpp ${SRC}/vsim/SitlModule.h
      ${SRC}/vsim/VfsDir.cpp ${SRC}/vsim/VfsDir.h)
  target_include_directories(tst_sitl_module_error PRIVATE
      ${SRC}/vsim ${VAYU_SITL_SDK})
endif()

# ---- Start button latch after a firmware settings reset (header only) -------
navigator_test(tst_start_gate tst_start_gate.cpp)
target_include_directories(tst_start_gate PRIVATE ${SRC}/ui/widgets)

# ---- Custom-vehicle untuned warning logic (header only) ---------------------
navigator_test(tst_custom_vehicle_warning tst_custom_vehicle_warning.cpp)
target_include_directories(tst_custom_vehicle_warning PRIVATE ${SRC}/ui/widgets)

# ---- Prop audio: stale-feed gate (silence after Stop) ----------------------
navigator_test(tst_prop_audio_gate
    tst_prop_audio_gate.cpp
    ${SRC}/audio/PropAudio.cpp)
target_include_directories(tst_prop_audio_gate PRIVATE ${SRC}/audio)

# ---- C++ autotune engine (Space + orchestration, synthetic rollout) -------
# Where the autotune backend is looked for (installed app has no repo to walk up to).
navigator_test(tst_autotune_backend_locator
    tst_autotune_backend_locator.cpp
    ${SRC}/autotune/BackendLocator.cpp ${SRC}/autotune/BackendLocator.h)
target_include_directories(tst_autotune_backend_locator PRIVATE ${SRC}/autotune)

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
    ${SRC}/vsim/procgen/TerrainField.cpp ${SRC}/vsim/procgen/WaterField.cpp
    ${SRC}/vsim/procgen/GroundCover.cpp
    ${SRC}/vsim/procgen/WorldPresets.cpp
    ${SRC}/vsim/procgen/Flora.cpp
    ${SRC}/vsim/procgen/Helipads.cpp
    ${SRC}/vsim/ChunkStreamer.cpp)
target_include_directories(tst_procgen PRIVATE ${SRC}/vsim)

# ---- Water basins: WaterField carve, shore, spawn exclusion, helipad veto ---
navigator_test(tst_waterfield
    tst_waterfield.cpp
    ${SRC}/vsim/procgen/Noise.cpp
    ${SRC}/vsim/procgen/TerrainField.cpp ${SRC}/vsim/procgen/WaterField.cpp
    ${SRC}/vsim/procgen/GroundCover.cpp
    ${SRC}/vsim/procgen/Helipads.cpp)
target_include_directories(tst_waterfield PRIVATE ${SRC}/vsim)

# ---- Prop-wash ripple sim (Qt-free, headless) -------------------------------
navigator_test(tst_waterripples tst_waterripples.cpp ${SRC}/vsim/WaterRipples.cpp
    ${SRC}/vsim/procgen/WaterField.cpp)
target_include_directories(tst_waterripples PRIVATE ${SRC}/vsim)

# ---- Terrain material: splat rule + quality default (header only) ----------
navigator_test(tst_terrainmaterial tst_terrainmaterial.cpp)
target_include_directories(tst_terrainmaterial PRIVATE ${SRC}/vsim)

# ---- Procedural vegetation placement (Qt-free core, headless) ---------------
navigator_test(tst_noise_ridged
    tst_noise_ridged.cpp
    ${SRC}/vsim/procgen/Noise.cpp)
target_include_directories(tst_noise_ridged PRIVATE ${SRC}/vsim)

navigator_test(tst_vegetation
    tst_vegetation.cpp
    ${SRC}/vsim/procgen/Noise.cpp
    ${SRC}/vsim/procgen/TerrainField.cpp ${SRC}/vsim/procgen/WaterField.cpp
    ${SRC}/vsim/procgen/GroundCover.cpp
    ${SRC}/vsim/procgen/Vegetation.cpp)
target_include_directories(tst_vegetation PRIVATE ${SRC}/vsim)

# ---- Collider window around the drone (Qt-free, engine-free) ----------------
navigator_test(tst_treecolliderwindow
    tst_treecolliderwindow.cpp
    ${SRC}/vsim/procgen/TreeColliderWindow.cpp
    ${SRC}/vsim/procgen/Noise.cpp
    ${SRC}/vsim/procgen/TerrainField.cpp ${SRC}/vsim/procgen/WaterField.cpp
    ${SRC}/vsim/procgen/GroundCover.cpp
    ${SRC}/vsim/procgen/Vegetation.cpp)
target_include_directories(tst_treecolliderwindow PRIVATE ${SRC}/vsim/procgen ${SRC}/vsim)

# ---- Procedural world presets (Qt-free core, headless) ----------------------
navigator_test(tst_worldpresets
    tst_worldpresets.cpp
    ${SRC}/vsim/procgen/Noise.cpp
    ${SRC}/vsim/procgen/TerrainField.cpp ${SRC}/vsim/procgen/WaterField.cpp
    ${SRC}/vsim/procgen/GroundCover.cpp
    ${SRC}/vsim/procgen/WorldPresets.cpp)
target_include_directories(tst_worldpresets PRIVATE ${SRC}/vsim)

# ---- Procedural drone model generator (Qt-free core, headless) -------------
navigator_test(tst_dronemodel
    tst_dronemodel.cpp
    ${SRC}/vsim/DroneModel.cpp)
target_include_directories(tst_dronemodel PRIVATE ${SRC}/vsim)

# ---- Cascaded shadow-map fitting (pure maths, headless) --------------------
navigator_test(tst_shadow_cascades tst_shadow_cascades.cpp)
target_include_directories(tst_shadow_cascades PRIVATE ${SRC}/vsim)

# ---- Vegetation assets load from the embedded resource (no GL) ----------------
navigator_test(tst_vegetation_assets tst_vegetation_assets.cpp)
target_include_directories(tst_vegetation_assets PRIVATE ${SRC}/vsim)
target_link_libraries(tst_vegetation_assets PRIVATE navigator_vegetation)

# ---- 3D sim view survives its GL context being recreated ------------------
# Needs a real GL platform (xcb on an Xvfb, LIBGL_ALWAYS_SOFTWARE=1): it skips on
# the offscreen platform ctest uses, which crashes QOpenGLWidget/QOpenGLWindow.
if(NAVIGATOR_SITL AND EXISTS "${VAYU_SITL_SDK}/vayu_sitl_abi.h")
  find_package(Qt6 REQUIRED COMPONENTS OpenGL OpenGLWidgets)
  navigator_test(tst_sim_gl_context
      tst_sim_gl_context.cpp
      ${SRC}/vsim/SimRendererWidget.cpp ${SRC}/vsim/SimRendererWidget.h
      ${SRC}/vsim/FarTerrain.cpp ${SRC}/vsim/FarTerrain.h
      ${SRC}/vsim/TerrainMaterial.cpp ${SRC}/vsim/TerrainTextures.h
      ${CMAKE_SOURCE_DIR}/resources/resources.qrc
      ${SRC}/vsim/procgen/Noise.cpp ${SRC}/vsim/procgen/TerrainField.cpp ${SRC}/vsim/procgen/WaterField.cpp
      ${SRC}/vsim/procgen/GroundCover.cpp
      ${SRC}/ui/widgets/SimHudWidget.cpp ${SRC}/ui/widgets/SimHudWidget.h
      ${SRC}/core/Units.cpp
      ${SRC}/vsim/DroneModel.cpp)
  target_include_directories(tst_sim_gl_context PRIVATE ${SRC}/vsim ${SRC}/ui/widgets ${VAYU_SITL_SDK})
  target_link_libraries(tst_sim_gl_context PRIVATE Qt6::OpenGL Qt6::OpenGLWidgets)
  if(NAVIGATOR_SIM_GRASS) # same renderer configuration as the app
    target_sources(tst_sim_gl_context PRIVATE ${SRC}/vsim/GpuGrass.cpp)
    target_compile_definitions(tst_sim_gl_context PRIVATE VAYU_SIM_GRASS=1)
  endif()
  target_sources(tst_sim_gl_context PRIVATE ${SRC}/vsim/VegetationRenderer.cpp ${SRC}/vsim/WaterRenderer.cpp ${SRC}/vsim/WaterRipples.cpp
      ${SRC}/vsim/procgen/Vegetation.cpp ${SRC}/vsim/procgen/Noise.cpp
      ${SRC}/vsim/procgen/TerrainField.cpp ${SRC}/vsim/procgen/WaterField.cpp ${SRC}/vsim/procgen/GroundCover.cpp)
  target_link_libraries(tst_sim_gl_context PRIVATE navigator_vegetation)

  # Colliders match the rendered size (no GL context needed).
  navigator_test(tst_vegetation_scale
      tst_vegetation_scale.cpp
      ${SRC}/vsim/VegetationRenderer.cpp
      ${SRC}/vsim/procgen/Vegetation.cpp ${SRC}/vsim/procgen/Noise.cpp
      ${SRC}/vsim/procgen/TerrainField.cpp ${SRC}/vsim/procgen/WaterField.cpp ${SRC}/vsim/procgen/GroundCover.cpp)
  target_include_directories(tst_vegetation_scale PRIVATE ${SRC}/vsim)
  target_link_libraries(tst_vegetation_scale PRIVATE Qt6::OpenGL navigator_vegetation)

  # ---- Vegetation renderer in the real 3D view (xvfb software GL) -----------
  # Forest scene with vegetation on vs off, instance counts per palette, LOD
  # selection, billboards, GL teardown. Skips on the offscreen platform.
  navigator_test(tst_vegetation_render
      tst_vegetation_render.cpp
      ${SRC}/vsim/FarTerrain.cpp ${SRC}/vsim/FarTerrain.h
      ${SRC}/vsim/SimRendererWidget.cpp ${SRC}/vsim/SimRendererWidget.h
      ${SRC}/vsim/VegetationRenderer.cpp ${SRC}/vsim/WaterRenderer.cpp ${SRC}/vsim/WaterRipples.cpp
      ${SRC}/vsim/TerrainMaterial.cpp ${SRC}/vsim/TerrainTextures.h
      ${CMAKE_SOURCE_DIR}/resources/resources.qrc
      ${SRC}/ui/widgets/SimHudWidget.cpp ${SRC}/ui/widgets/SimHudWidget.h
      ${SRC}/core/Units.cpp
      ${SRC}/vsim/DroneModel.cpp
      ${SRC}/vsim/ChunkStreamer.cpp
      ${SRC}/vsim/procgen/Noise.cpp ${SRC}/vsim/procgen/TerrainGen.cpp
      ${SRC}/vsim/procgen/TerrainField.cpp ${SRC}/vsim/procgen/WaterField.cpp ${SRC}/vsim/procgen/GroundCover.cpp
      ${SRC}/vsim/procgen/Vegetation.cpp)
  target_include_directories(tst_vegetation_render PRIVATE ${SRC}/vsim
      ${SRC}/ui/widgets ${VAYU_SITL_SDK})
  target_link_libraries(tst_vegetation_render PRIVATE Qt6::OpenGL
      Qt6::OpenGLWidgets navigator_vegetation)
  if(NAVIGATOR_SIM_GRASS)
    target_sources(tst_vegetation_render PRIVATE ${SRC}/vsim/GpuGrass.cpp
        ${SRC}/vsim/procgen/Flora.cpp)
    target_compile_definitions(tst_vegetation_render PRIVATE VAYU_SIM_GRASS=1)
  endif()
endif()

# ---- No OpenGL context: the 3D views show a notice instead of crashing ----
# Sets QT_XCB_GL_INTEGRATION=none itself; a real check on xcb (an Xvfb).
find_package(Qt6 REQUIRED COMPONENTS OpenGL OpenGLWidgets)
navigator_test(tst_gl_unavailable
    tst_gl_unavailable.cpp
    ${SRC}/ui/widgets/AttitudeWidget.cpp
    ${SRC}/ui/widgets/Drone3DWidget.cpp
    ${SRC}/core/Theme.cpp)
target_include_directories(tst_gl_unavailable PRIVATE ${SRC}/ui/widgets ${SRC}/core)
target_link_libraries(tst_gl_unavailable PRIVATE Qt6::OpenGL Qt6::OpenGLWidgets)

# ---- GPU grass respects the ground-cover mask (grass builds only) ----------
# The mask data checks run anywhere; the blade readback needs a real GL platform
# (xcb on an Xvfb + LIBGL_ALWAYS_SOFTWARE=1) and skips on offscreen.
if(NAVIGATOR_SIM_GRASS)
  find_package(Qt6 REQUIRED COMPONENTS OpenGL)
  navigator_test(tst_gpu_grass
      tst_gpu_grass.cpp
      ${SRC}/vsim/GpuGrass.cpp
      ${SRC}/vsim/procgen/Noise.cpp
      ${SRC}/vsim/procgen/TerrainField.cpp ${SRC}/vsim/procgen/WaterField.cpp
      ${SRC}/vsim/procgen/GroundCover.cpp)
  target_include_directories(tst_gpu_grass PRIVATE ${SRC}/vsim)
  target_link_libraries(tst_gpu_grass PRIVATE Qt6::OpenGL)
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
      ${SRC}/vsim/procgen/TerrainField.cpp ${SRC}/vsim/procgen/WaterField.cpp
      ${SRC}/vsim/procgen/GroundCover.cpp
      ${SRC}/vsim/UserPresets.cpp
      ${SRC}/vsim/procgen/WorldPresets.cpp)
  target_include_directories(tst_worldeditor PRIVATE ${SRC}/vsim ${SRC}/ui/widgets)
endif()

# ---- Side-panel sizing helpers (no splitter may crop a panel) -------------
navigator_test(tst_panel_widgets tst_panel_widgets.cpp)

# ---- Native window containers are clipped to their dock area --------------
navigator_test(tst_native_clip tst_native_clip.cpp)
target_include_directories(tst_native_clip PRIVATE ${SRC}/ui/widgets)
target_link_libraries(tst_native_clip PRIVATE Qt6::Gui Qt6::OpenGL)

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
      ${SRC}/vsim/procgen/TreeColliderWindow.cpp
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

# ---- Crash path end to end against the real module (SITL builds only) ------
# SimWorker sends the crash limit, a 20 m fall reports CRASHED in the pose tail,
# a reset clears it. Skips without a module, like tst_sitl_engine.
if(NAVIGATOR_SITL AND EXISTS "${VAYU_SITL_SDK}/vsim_proto.h")
  navigator_test(tst_sitl_crash
      tst_sitl_crash.cpp
      ${SRC}/vsim/SitlModule.cpp ${SRC}/vsim/SitlModule.h
      ${SRC}/vsim/VfsDir.cpp ${SRC}/vsim/VfsDir.h
      ${SRC}/vsim/SimWorker.cpp ${SRC}/vsim/SimWorker.h
      ${SRC}/vsim/procgen/TreeColliderWindow.cpp)
  target_include_directories(tst_sitl_crash PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${SRC}/vsim ${VAYU_SITL_SDK})
  target_link_libraries(tst_sitl_crash PRIVATE navlink_codec)
  if(WIN32)
    target_link_libraries(tst_sitl_crash PRIVATE winmm)
  endif()
  set_tests_properties(tst_sitl_crash PROPERTIES
      ENVIRONMENT "QT_QPA_PLATFORM=offscreen;VAYU_SITL_MODULE=${CMAKE_BINARY_DIR}/libvayu_sitl${CMAKE_SHARED_MODULE_SUFFIX}"
      TIMEOUT 90)
endif()

# ---- Tree colliders against the real engine (SITL builds only) --------------
# The ENV frames SimWorker sends make a trunk solid, leave a line beside it free
# and go away on a clear. Skips, saying why, without the engine module or on an
# SDK that has no ENV messages (older than v0.3.0; the pinned one has them).
if(NAVIGATOR_SITL AND EXISTS "${VAYU_SITL_SDK}/vayu_sitl_abi.h")
  navigator_test(tst_treecolliders_engine
      tst_treecolliders_engine.cpp
      ${SRC}/vsim/SitlModule.cpp ${SRC}/vsim/SitlModule.h
      ${SRC}/vsim/VfsDir.cpp ${SRC}/vsim/VfsDir.h
      ${SRC}/vsim/SimWorker.cpp ${SRC}/vsim/SimWorker.h
      ${SRC}/vsim/procgen/TreeColliderWindow.cpp
      ${SRC}/vsim/RcBridge.cpp ${SRC}/vsim/RcBridge.h)
  target_include_directories(tst_treecolliders_engine PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${SRC}/vsim ${SRC}/vsim/procgen ${VAYU_SITL_SDK})
  target_link_libraries(tst_treecolliders_engine PRIVATE navlink_codec)
  if(WIN32)
    target_link_libraries(tst_treecolliders_engine PRIVATE winmm)
  endif()
  set_tests_properties(tst_treecolliders_engine PROPERTIES
      ENVIRONMENT "QT_QPA_PLATFORM=offscreen;VAYU_SITL_MODULE=${CMAKE_BINARY_DIR}/libvayu_sitl${CMAKE_SHARED_MODULE_SUFFIX}"
      TIMEOUT 120)
endif()

# ---- Airframe presets: data checks (no engine needed) ---------------------
navigator_test(tst_airframe_presets
    tst_airframe_presets.cpp
    ${SRC}/vsim/AirframePresets.cpp
    ${SRC}/vsim/VehicleJson.cpp)
target_include_directories(tst_airframe_presets PRIVATE ${SRC}/vsim ${SRC}/ui/widgets)
target_compile_definitions(tst_airframe_presets PRIVATE
    AIRFRAME_VEHICLE_DIR="${CMAKE_SOURCE_DIR}/docs/vehicles")

# ---- Box/arms inertia formula (pure, no engine, builds in the GCS-only build) --
navigator_test(tst_box_inertia
    tst_box_inertia.cpp
    ${SRC}/vsim/MassProperties.cpp
    ${SRC}/vsim/AirframePresets.cpp)
target_include_directories(tst_box_inertia PRIVATE ${SRC}/vsim)

# ---- Stopping a stuck worker thread without terminate() (pure, no engine) ---
navigator_test(tst_thread_retire tst_thread_retire.cpp)
target_include_directories(tst_thread_retire PRIVATE ${SRC}/vsim)

# ---- Main-window size fitted to the screen (pure, no widget) ---------------
navigator_test(tst_window_fit tst_window_fit.cpp)
target_include_directories(tst_window_fit PRIVATE ${SRC}/ui/main)

# ---- Airframe presets flown in the real firmware (SITL builds only) -------
# Hover + yaw-sign per preset. Needs the engine module like tst_sitl_engine and
# skips, saying why, without one. About a minute of wall-clock-paced sim.
if(NAVIGATOR_SITL AND EXISTS "${VAYU_SITL_SDK}/vayu_sitl_abi.h")
  navigator_test(tst_airframe_flight
      tst_airframe_flight.cpp
      ${SRC}/vsim/SitlModule.cpp ${SRC}/vsim/SitlModule.h
      ${SRC}/vsim/VfsDir.cpp ${SRC}/vsim/VfsDir.h
      ${SRC}/vsim/SimWorker.cpp ${SRC}/vsim/SimWorker.h
      ${SRC}/vsim/procgen/TreeColliderWindow.cpp
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
      ${SRC}/vsim/AirframePresets.cpp ${SRC}/vsim/VehicleJson.cpp
      ${SRC}/vsim/UserPresets.cpp)
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
      ${SRC}/vsim/procgen/TreeColliderWindow.cpp
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

# ---- Workspace manager (docking layer behind the workspace mock) -----------
# Needs Qt Advanced Docking System; skipped, like the mock, when it is absent.
if(TARGET navigator_workspace)
    navigator_test(tst_workspace_manager tst_workspace_manager.cpp)
    target_link_libraries(tst_workspace_manager PRIVATE navigator_workspace)
endif()

# The dock host of the main window (DockShell): ids, built-in workspaces, the
# switch, showPanel, the pinned attitude panel. Offscreen, no real pages.
if(TARGET navigator_workspace)
    navigator_test(tst_dock_shell tst_dock_shell.cpp
        ${SRC}/ui/main/DockShell.cpp ${SRC}/ui/main/DockShell.h
        ${SRC}/core/Theme.cpp)
    target_include_directories(tst_dock_shell PRIVATE ${SRC}/ui/main)
    target_compile_definitions(tst_dock_shell PRIVATE
        DARK_QSS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../resources/styles/dark.qss")
    target_link_libraries(tst_dock_shell PRIVATE navigator_workspace)
endif()

# The dock host's child windows (drop overlays, floating windows) can paint.
if(TARGET navigator_workspace)
    navigator_test(tst_dock_overlay tst_dock_overlay.cpp
        ${SRC}/ui/main/DockShell.cpp ${SRC}/ui/main/DockShell.h
        ${SRC}/core/Theme.cpp)
    target_include_directories(tst_dock_overlay PRIVATE ${SRC}/ui/main
        ${SRC}/core)
    target_link_libraries(tst_dock_overlay PRIVATE navigator_workspace)
endif()

# Panels hosted by AdsBackend keep a content minimum size (PanelFrame).
if(TARGET navigator_workspace)
    navigator_test(tst_panel_min_size tst_panel_min_size.cpp)
    target_link_libraries(tst_panel_min_size PRIVATE navigator_workspace)
endif()

# A floated panel is a Qt::Tool window of the main window.
if(TARGET navigator_workspace)
    navigator_test(tst_floating_tool tst_floating_tool.cpp)
    target_link_libraries(tst_floating_tool PRIVATE navigator_workspace)
endif()

# QOpenGLWidget inside the dock layer: skips on "offscreen" (see the file).
if(TARGET navigator_workspace)
    navigator_test(tst_workspace_gl tst_workspace_gl.cpp
        ${CMAKE_SOURCE_DIR}/tools/workspace_mock/GlPanel.h
        ${SRC}/ui/widgets/AttitudeWidget.cpp
        ${SRC}/ui/widgets/Drone3DWidget.cpp
        ${SRC}/ui/widgets/AttitudePanel.cpp
        ${SRC}/core/Theme.cpp)
    target_include_directories(tst_workspace_gl PRIVATE
        ${CMAKE_SOURCE_DIR}/tools/workspace_mock
        ${SRC}/ui/widgets ${SRC}/core)
    target_link_libraries(tst_workspace_gl PRIVATE navigator_workspace
        Qt6::OpenGLWidgets)
endif()

# ---- FAULT_STATUS: faults model, router decode, panel badge ---------------
navigator_test(tst_faults
    tst_faults.cpp
    ${SRC}/core/FaultsModel.cpp
    ${SRC}/protocol/NavlinkRouter.cpp
    ${SRC}/ui/widgets/FaultsPanel.cpp)
target_include_directories(tst_faults PRIVATE ${SRC}/ui/widgets)
target_link_libraries(tst_faults PRIVATE navlink_codec)

# ---- STATUSTEXT: severity prefix + multi-chunk reassembly -----------------
navigator_test(tst_statustext
    tst_statustext.cpp
    ${SRC}/protocol/NavlinkRouter.cpp)
target_link_libraries(tst_statustext PRIVATE navlink_codec)

# ---- FAILSAFE_STATUS decode: a real frame through the router ---------------
navigator_test(tst_failsafe_decode
    tst_failsafe_decode.cpp
    ${SRC}/protocol/NavlinkRouter.cpp)
target_link_libraries(tst_failsafe_decode PRIVATE navlink_codec)

# ---- NavLink 3: COMMAND_ACK frame seq shares the GCS tx counter -----------
navigator_test(tst_ack_seq
    tst_ack_seq.cpp
    ${SRC}/protocol/NavlinkRouter.cpp
    ${SRC}/protocol/CommandCodec.cpp)
target_link_libraries(tst_ack_seq PRIVATE navlink_codec)

# ---- World change leaves the vehicle on the ground (SITL builds only) -------
# Drives the real SimulatorWidget (the Navigator sources minus main.cpp, built
# a second time here: the widget is not in a library) through World-tab changes,
# sim Stopped and Running, and checks the vehicle sits on the new terrain.
# Skips on "offscreen" (it owns GL views) and, for the Running part, without an
# engine module; run it under QT_QPA_PLATFORM=xcb on an Xvfb.
if(TARGET Navigator AND NAVIGATOR_SITL AND EXISTS "${VAYU_SITL_SDK}/vayu_sitl_abi.h")
  get_target_property(_nav_srcs Navigator SOURCES)
  set(_wf_srcs "")
  foreach(_s ${_nav_srcs})
    if(_s STREQUAL "${PARAMS_CATALOG_QRC}")
      continue() # absolute, in the build tree: added as it is below
    endif()
    if(NOT _s MATCHES "(^|/)app/main\\.cpp$")
      list(APPEND _wf_srcs "${CMAKE_SOURCE_DIR}/${_s}")
    endif()
  endforeach()
  navigator_test(tst_world_change_floor tst_world_change_floor.cpp ${_wf_srcs})
  target_sources(tst_world_change_floor PRIVATE ${PARAMS_CATALOG_QRC})
  get_target_property(_nav_inc Navigator INCLUDE_DIRECTORIES)
  get_target_property(_nav_def Navigator COMPILE_DEFINITIONS)
  get_target_property(_nav_lib Navigator LINK_LIBRARIES)
  target_include_directories(tst_world_change_floor PRIVATE ${_nav_inc}
      ${CMAKE_SOURCE_DIR}/src/ui/widgets ${CMAKE_SOURCE_DIR}/src/vsim)
  target_compile_definitions(tst_world_change_floor PRIVATE ${_nav_def})
  target_link_libraries(tst_world_change_floor PRIVATE ${_nav_lib})
  set_tests_properties(tst_world_change_floor PROPERTIES
      ENVIRONMENT "QT_QPA_PLATFORM=offscreen;VAYU_SITL_MODULE=${CMAKE_BINARY_DIR}/libvayu_sitl${CMAKE_SHARED_MODULE_SUFFIX}"
      TIMEOUT 600)


  # ---- Per-run log files of the real SimulatorWidget ------------------------
  # One file pair per Start (unique stamp), none left by a failed Start. Same
  # widget-source build as above; each case runs in its own process because the
  # engine module loads once per process. Needs a GL platform plugin (xcb on an
  # Xvfb): both cases skip on "offscreen".
  navigator_test(tst_sim_runlog tst_sim_runlog.cpp ${_wf_srcs})
  target_include_directories(tst_sim_runlog PRIVATE ${_nav_inc}
      ${CMAKE_SOURCE_DIR}/src/ui/widgets ${CMAKE_SOURCE_DIR}/src/vsim)
  target_compile_definitions(tst_sim_runlog PRIVATE ${_nav_def})
  target_link_libraries(tst_sim_runlog PRIVATE ${_nav_lib})
  add_test(NAME tst_sim_runlog_failed_start
      COMMAND tst_sim_runlog missingModuleLeavesNoLogs)
  set_tests_properties(tst_sim_runlog_failed_start PROPERTIES
      ENVIRONMENT "QT_QPA_PLATFORM=offscreen;VAYU_SITL_MODULE=/nonexistent/libvayu_sitl.so"
      TIMEOUT 120)
  add_test(NAME tst_sim_runlog_rapid
      COMMAND tst_sim_runlog rapidStartStopOneRunPerStart)
  set_tests_properties(tst_sim_runlog_rapid PROPERTIES
      ENVIRONMENT "QT_QPA_PLATFORM=offscreen;VAYU_SITL_MODULE=${CMAKE_BINARY_DIR}/libvayu_sitl${CMAKE_SHARED_MODULE_SUFFIX}"
      TIMEOUT 300)
endif()

# ---- Simulator page minimum width (SITL builds only) -----------------------
# Builds the real SimulatorWidget (sources compiled a second time, as above) and
# checks its minimum size fits a laptop screen. Offscreen; own process.
if(TARGET Navigator AND NAVIGATOR_SITL AND EXISTS "${VAYU_SITL_SDK}/vayu_sitl_abi.h")
  get_target_property(_nav_srcs Navigator SOURCES)
  set(_wf_srcs "")
  foreach(_s ${_nav_srcs})
    if(_s STREQUAL "${PARAMS_CATALOG_QRC}")
      continue() # absolute, in the build tree: added as it is below
    endif()
    if(NOT _s MATCHES "(^|/)app/main\\.cpp$")
      list(APPEND _wf_srcs "${CMAKE_SOURCE_DIR}/${_s}")
    endif()
  endforeach()
  navigator_test(tst_sim_page_size tst_sim_page_size.cpp ${_wf_srcs})
  get_target_property(_nav_inc Navigator INCLUDE_DIRECTORIES)
  get_target_property(_nav_def Navigator COMPILE_DEFINITIONS)
  get_target_property(_nav_lib Navigator LINK_LIBRARIES)
  target_include_directories(tst_sim_page_size PRIVATE ${_nav_inc}
      ${CMAKE_SOURCE_DIR}/src/ui/widgets ${CMAKE_SOURCE_DIR}/src/vsim)
  target_compile_definitions(tst_sim_page_size PRIVATE ${_nav_def})
  target_link_libraries(tst_sim_page_size PRIVATE ${_nav_lib})
  set_tests_properties(tst_sim_page_size PROPERTIES
      ENVIRONMENT "QT_QPA_PLATFORM=offscreen;VAYU_SITL_MODULE=${CMAKE_BINARY_DIR}/libvayu_sitl${CMAKE_SHARED_MODULE_SUFFIX}"
      TIMEOUT 120)
endif()

# ---- Real panels: content minimum, no overlapping rows, main window fit -----
# Builds the real MainWindow (Navigator's sources compiled a second time, as
# above; GL-free: nothing is shown) and checks the dock host with the real
# panels. Offscreen, own process, private XDG_*. Valid in the GCS-only build.
if(TARGET Navigator AND TARGET navigator_workspace)
  get_target_property(_pr_srcs Navigator SOURCES)
  set(_pr_wf "")
  foreach(_s ${_pr_srcs})
    if(_s STREQUAL "${PARAMS_CATALOG_QRC}")
      continue() # absolute, in the build tree: added as it is below
    endif()
    if(NOT _s MATCHES "(^|/)app/main\\.cpp$")
      list(APPEND _pr_wf "${CMAKE_SOURCE_DIR}/${_s}")
    endif()
  endforeach()
  navigator_test(tst_panel_real_min tst_panel_real_min.cpp ${_pr_wf})
  target_sources(tst_panel_real_min PRIVATE ${PARAMS_CATALOG_QRC})
  get_target_property(_pr_inc Navigator INCLUDE_DIRECTORIES)
  get_target_property(_pr_def Navigator COMPILE_DEFINITIONS)
  get_target_property(_pr_lib Navigator LINK_LIBRARIES)
  target_include_directories(tst_panel_real_min PRIVATE ${_pr_inc}
      ${CMAKE_SOURCE_DIR}/src/ui/widgets ${CMAKE_SOURCE_DIR}/src/ui/main
      ${CMAKE_SOURCE_DIR}/src/ui/workspace ${CMAKE_SOURCE_DIR}/src/vsim)
  target_compile_definitions(tst_panel_real_min PRIVATE ${_pr_def})
  target_link_libraries(tst_panel_real_min PRIVATE ${_pr_lib})
  set_tests_properties(tst_panel_real_min PROPERTIES TIMEOUT 120)
endif()
