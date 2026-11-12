# Workspace mock + the reusable docking layer (src/ui/workspace). Standalone:
# no engine, no SITL SDK. Needs Qt Advanced Docking System as a SHARED library
# (LGPL-2.1): see below for where it comes from.

# Where Qt Advanced Docking System (ADS, LGPL-2.1, see third_party/ads/NOTICE.md)
# comes from, in this order:
#   1. an installed copy: find_package(qtadvanceddocking-qt6), e.g. with
#      -DCMAKE_PREFIX_PATH=/opt/ads;
#   2. NAVIGATOR_FETCH_ADS=ON: FetchContent of the pinned release below, built
#      by this project as a SHARED library (the LGPL relinking requirement);
#      FETCHCONTENT_SOURCE_DIR_ADS=<dir> uses a local checkout instead and
#      needs no network.
# With neither, this directory configures nothing and the Navigator target
# stops the configure (its docked layout needs the library). The default is ON;
# use FETCHCONTENT_SOURCE_DIR_ADS or an installed copy to stay offline.
option(NAVIGATOR_FETCH_ADS "Fetch and build Qt Advanced Docking System (pinned)" ON)
# Escape hatch: directories holding the Qt private headers (QtGui/<ver>, ...)
# for a machine that cannot install qt6-base-private-dev. Normally empty.
set(NAVIGATOR_QT_PRIVATE_INCLUDES "" CACHE STRING
    "Qt private header directories (only if qt6-base-private-dev cannot be installed)")

if(NOT TARGET ads::qtadvanceddocking-qt6)
    find_package(qtadvanceddocking-qt6 QUIET)
endif()
if(NOT TARGET ads::qtadvanceddocking-qt6 AND NAVIGATOR_FETCH_ADS)
    include(FetchContent)
    # Pinned to the commit of tag v5.1.1; a commit cannot be moved, a tag can.
    # The ADS sources are used unmodified.
    FetchContent_Declare(ads
        GIT_REPOSITORY https://github.com/githubuser0xFFFF/Qt-Advanced-Docking-System.git
        GIT_TAG 023ce95934fecfd4cf5c672c9c404fabe0f54923)
    set(BUILD_STATIC OFF CACHE BOOL "" FORCE)    # LGPL: keep it a shared library
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(QT_VERSION_MAJOR 6)
    # ADS 5.1.1 links Qt6::GuiPrivate on Linux (it includes
    # <qpa/qplatformnativeinterface.h>), so the Qt private headers must be
    # installed: qt6-base-private-dev on Debian/Ubuntu. A developer who cannot
    # install it can point this at an unpacked copy of the headers instead.
    if(UNIX AND NOT APPLE AND NAVIGATOR_QT_PRIVATE_INCLUDES AND TARGET Qt6::GuiPrivate)
        foreach(t Qt6::GuiPrivate Qt6::CorePrivate)
            if(TARGET ${t})
                set_property(TARGET ${t} PROPERTY INTERFACE_INCLUDE_DIRECTORIES
                    "${NAVIGATOR_QT_PRIVATE_INCLUDES}")
            endif()
        endforeach()
    endif()
    FetchContent_MakeAvailable(ads)
    set(ADS_BUILT_HERE TRUE)
endif()
if(NOT TARGET ads::qtadvanceddocking-qt6)
    message(STATUS "Workspace mock: Qt Advanced Docking System not found -- skipped "
                   "(configure with -DNAVIGATOR_FETCH_ADS=ON, see README.md)")
    return()
endif()
# The tests/ directory links navigator_workspace too: an imported target needs
# GLOBAL for that; a target built here is visible already.
if(NOT ADS_BUILT_HERE)
    set_target_properties(ads::qtadvanceddocking-qt6 PROPERTIES IMPORTED_GLOBAL TRUE)
endif()
if(ADS_BUILT_HERE)
    message(STATUS "Workspace mock: ADS built here (FetchContent, shared)")
else()
    message(STATUS "Workspace mock: ADS found (${qtadvanceddocking-qt6_DIR})")
endif()

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

# The library above is always built (Navigator's dock host needs it); only the
# demo app is optional.
if(NAVIGATOR_WORKSPACE_MOCK)
    qt_add_executable(workspace_mock main.cpp MockPanels.h GlPanel.h)
    target_link_libraries(workspace_mock PRIVATE navigator_workspace Qt6::OpenGLWidgets)
endif()
