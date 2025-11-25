cmake_minimum_required(VERSION 3.10)
project(lumos_sdk)
set(CMAKE_CXX_STANDARD 20)


find_package (glog REQUIRED)
find_package(lcm REQUIRED)

include_directories(./
    ./lcm_typedef
)

# 编译example目录下的所有示例程序，XXX.cpp -> XXX
file(GLOB EXAMPLE_SOURCE ./example/*.cpp)

foreach(example_file ${EXAMPLE_SOURCE})
    get_filename_component(target_name ${example_file} NAME_WE)

    add_executable(${target_name}
        ./sdk_robot_manager.cpp
        ${example_file}
    )
    target_link_libraries(${target_name} glog gflags lcm)
endforeach()
