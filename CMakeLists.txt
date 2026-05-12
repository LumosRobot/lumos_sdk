cmake_minimum_required(VERSION 3.10)
project(lumos_sdk VERSION 1.0.0 LANGUAGES CXX)

# ── C++ standard ──────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)  # 禁用 GNU 扩展，保持标准可移植性

# ── Build type default ─────────────────────────────────────────
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# RPATH for locally-built glog
set(CMAKE_INSTALL_RPATH /tmp/glog-local/lib)
set(CMAKE_BUILD_RPATH /tmp/glog-local/lib)

# ── Dependencies ───────────────────────────────────────────────
# Use local glog 0.7.1 (system glog 0.6.0 crashes with GCC 13 on exit)
set(glog_DIR /tmp/glog-local/lib/cmake/glog)
find_package(glog REQUIRED)
find_package(lcm REQUIRED)
find_package(gflags REQUIRED)

# ── Shared source (compiled once, linked into all targets) ─────
add_library(sdk_core OBJECT
    src/sdk_robot_manager.cpp
)
target_include_directories(sdk_core PUBLIC
    src
    lcm_typedef
)
# glog 0.7.1 requires these defines when building shared
target_compile_definitions(sdk_core PUBLIC
    GLOG_USE_GLOG_EXPORT
    GOOGLE_GLOG_IS_A_DLL
)
target_link_libraries(sdk_core PUBLIC
    glog::glog
    gflags
    lcm
)

# ── Example executables ────────────────────────────────────────
file(GLOB EXAMPLE_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/example/*.cpp")

foreach(example_file ${EXAMPLE_SOURCES})
    get_filename_component(target_name ${example_file} NAME_WE)
    add_executable(${target_name} ${example_file})
    target_link_libraries(${target_name} PRIVATE sdk_core)
endforeach()
