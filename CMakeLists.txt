cmake_minimum_required(VERSION 3.14)
project(LightRay)

set(CONAN_SYSTEM_INCLUDES ON) # suppress compiler warnings in header-only libs

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

add_executable(main main.cpp)

add_executable(test test.cpp)

target_link_libraries(main ${CONAN_LIBS})

if(DEFINED ENV{PROFILE})
    find_library(PROFILER profiler PATHS ENV GPERFTOOLS_LIB_DIR REQUIRED)
    target_link_libraries(main ${PROFILER})
endif()

set(CMAKE_CXX_FLAGS "-std=c++20 -Wall -Wextra -Wpedantic -fdiagnostics-show-template-tree")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -ffast-math -g -DNDEBUG")

set(CMAKE_POSITION_INDEPENDENT_CODE OFF)

set(CMAKE_BUILD_TYPE RelWithDebInfo)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_VERBOSE_MAKEFILE ON)

