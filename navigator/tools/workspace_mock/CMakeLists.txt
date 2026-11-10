# Workspace mock + the reusable docking layer (src/ui/workspace). Standalone:
# no engine, no SITL SDK. Needs Qt Advanced Docking System built as a SHARED
# library (LGPL-2.1); when it is not found this directory configures nothing and
# the rest of the build is unaffected. Point CMAKE_PREFIX_PATH at its install
# prefix, e.g. -DCMAKE_PREFIX_PATH=/opt/ads.
find_package(qtadvanceddocking-qt6 QUIET)
if(NOT qtadvanceddocking-qt6_FOUND)
    message(STATUS "Workspace mock: Qt Advanced Docking System not found -- skipped")
    return()
endif()
# The tests/ directory links navigator_workspace too: make the imported target
# visible there.
set_target_properties(ads::qtadvanceddocking-qt6 PROPERTIES IMPORTED_GLOBAL TRUE)
message(STATUS "Workspace mock: ADS found (${qtadvanceddocking-qt6_DIR})")

set(WS_DIR ${CMAKE_SOURCE_DIR}/src/ui/workspace)
add_library(navigator_workspace STATIC
    ${WS_DIR}/AdsBackend.cpp
    ${WS_DIR}/WorkspaceManager.cpp
    ${WS_DIR}/AdsBackend.h
    ${WS_DIR}/WorkspaceManager.h
    ${WS_DIR}/DockBackend.h
    ${WS_DIR}/LayoutSpec.h
    ${WS_DIR}/PanelRegistry.h
    ${WS_DIR}/ScreenClamp.h)
target_include_directories(navigator_workspace PUBLIC ${WS_DIR})
target_link_libraries(navigator_workspace PUBLIC Qt6::Widgets
    ads::qtadvanceddocking-qt6)

qt_add_executable(workspace_mock main.cpp MockPanels.h)
target_link_libraries(workspace_mock PRIVATE navigator_workspace)
