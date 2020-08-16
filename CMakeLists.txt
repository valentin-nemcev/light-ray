cmake_minimum_required(VERSION 3.12)

include("cmake/HunterGate.cmake")
# https://github.com/ruslo/hunter/releases
HunterGate(
    URL "https://github.com/cpp-pm/hunter/archive/v0.23.263.tar.gz"
    SHA1 "9b96c376bf3526163e81b7adb8db5a0ea8c7e1bf"
)

project (light-ray)

hunter_add_package(SDL2)
find_package(SDL2 CONFIG REQUIRED)

hunter_add_package(SDL_ttf)
find_package(SDL_ttf CONFIG REQUIRED)

hunter_add_package(Boost)
find_package(Boost CONFIG REQUIRED)

hunter_add_package(Eigen)
find_package(Eigen3 CONFIG REQUIRED)


add_executable(main main.cpp)

add_executable(test test.cpp)

target_link_libraries(main SDL2::SDL2main SDL2::SDL2 SDL_ttf::SDL_ttf Boost::boost Eigen3::Eigen)

if(DEFINED ENV{PROFILE})
    find_library(PROFILER profiler PATHS ENV GPERFTOOLS_LIB_DIR REQUIRED)
    target_link_libraries(main ${PROFILER})
endif()

set(CMAKE_CXX_FLAGS "-std=c++20 -Wall -Wextra -Wpedantic -fdiagnostics-show-template-tree")
set(CMAKE_POSITION_INDEPENDENT_CODE OFF)

set(CMAKE_BUILD_TYPE RelWithDebInfo)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_VERBOSE_MAKEFILE ON)

