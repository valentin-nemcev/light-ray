cmake_minimum_required(VERSION 2.8.9)
project (light-ray)
add_executable(main main.cpp)

set(CMAKE_CXX_COMPILER             "/usr/local/opt/llvm/bin/clang++")
set(CMAKE_CXX_FLAGS                "-std=c++17 -Wall")
