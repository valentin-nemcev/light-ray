#!/usr/bin/env bash

# Exit on error
set -e

for i in "$@"
do
case $i in
    --setup)
    SETUP=YES
    ;;
    *)
            # unknown option
    ;;
esac
done


export CC="$(brew --prefix llvm)/bin/clang"
export CXX="$(brew --prefix llvm)/bin/clang++"
export LDFLAGS="-L/usr/local/opt/llvm/lib -Wl,-rpath,/usr/local/opt/llvm/lib"

cd build

if [ ! -z $SETUP ]
then
  cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_BUILD_TYPE=Debug ..
  mv compile_commands.json ..
fi

cmake --build .

./main
# $(brew --prefix llvm)/bin/lldb -o run ./main
