#!/usr/bin/env bash

# Exit on error
set -e

for i in "$@"
do
case $i in
    --setup)
    SETUP=YES
    ;;
    --debug)
    DEBUG=YES
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
  cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_BUILD_TYPE=Debug ..
  mv compile_commands.json ..
fi

cmake --build .

if [ ! -z $DEBUG ]
then
$PREFIX/bin/lldb -o run ./main
else
./main
fi
