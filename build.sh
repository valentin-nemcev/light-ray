#!/usr/bin/env bash

# Exit on error
set -e


TARGET=main

for i in "$@"
do
case $i in
    --setup)
    SETUP=YES
    ;;
    --debug)
    DEBUG=YES
    ;;
    --target=*)
    TARGET="${i#*=}"
    ;;
    *)
            # unknown option
    ;;
esac
done


PREFIX="$(brew --prefix llvm)"

export CC="$PREFIX/bin/clang"
export CXX="$PREFIX/bin/clang++"
export LDFLAGS="-L$PREFIX/lib -Wl,-rpath,$PREFIX/lib"

cd build

if [ ! -z $SETUP ]
then
  rm -rf ./*
  cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_BUILD_TYPE=Debug ..
  mv compile_commands.json ..
fi

cmake --build . --target $TARGET

if [ ! -z $DEBUG ]
then
$PREFIX/bin/lldb -o run ./$TARGET
else
./$TARGET
fi
