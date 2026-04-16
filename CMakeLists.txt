cmake_minimum_required(VERSION 3.10)
project(lumos_sdk)

# Use C++20 as requested
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(glog REQUIRED)
find_package(lcm REQUIRED)
find_package(gflags REQUIRED)

# Update include paths: added ./src and ./lcm_typedef
include_directories(
    src
    lcm_typedef
)

# Compile all example programs in the example directory: XXX.cpp -> XXX
file(GLOB EXAMPLE_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/example/*.cpp")

foreach(example_file ${EXAMPLE_SOURCE})
    get_filename_component(target_name ${example_file} NAME_WE)

    add_executable(${target_name}
        src/sdk_robot_manager.cpp  # Path updated to reflect the new 'src' folder
        ${example_file}
    )
    
    # Using modern CMake target linking where possible
    target_link_libraries(${target_name} 
        glog::glog 
        gflags 
        lcm
    )
endforeach()
