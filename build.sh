#!/usr/bin/env bash

export CC="$(brew --prefix llvm)/bin/clang"
export CXX="$(brew --prefix llvm)/bin/clang++"

cd build

cmake ..
cmake --build .
