cmake_minimum_required(VERSION 3.12)

include("cmake/HunterGate.cmake")
# https://github.com/ruslo/hunter/releases
HunterGate(
    URL "https://github.com/ruslo/hunter/archive/v0.23.14.tar.gz"
    SHA1 "f58bcf274f29097ac380bfe4955d2a66f077e741"
)

project (light-ray)

hunter_add_package(SDL2)
find_package(SDL2 CONFIG REQUIRED)

hunter_add_package(Boost)
# Header-only libraries
find_package(Boost CONFIG REQUIRED)

hunter_add_package(Eigen)
find_package(Eigen3 CONFIG REQUIRED)

hunter_add_package(range-v3)
find_package(range-v3 CONFIG REQUIRED)

add_executable(main main.cpp)

target_link_libraries(main SDL2::SDL2main SDL2::SDL2 Boost::boost Eigen3::Eigen range-v3::range-v3)

set(CMAKE_CXX_FLAGS "-std=c++20 -Wall -fdiagnostics-show-template-tree")
set(CMAKE_BUILD_TYPE Debug)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
