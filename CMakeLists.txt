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

add_executable(main main.cpp)

target_link_libraries(main SDL2::SDL2main SDL2::SDL2)

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall")


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
