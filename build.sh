#!/usr/bin/env bash

# Exit on error
set -e

export CC="$(brew --prefix llvm)/bin/clang"
export CXX="$(brew --prefix llvm)/bin/clang++"

cd build

# cmake ..
cmake --build .

./main
