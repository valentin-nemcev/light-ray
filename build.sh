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
    --profile)
    export PROFILE=YES
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
export CPPFLAGS="-I$PREFIX/include"
export GPERFTOOLS_LIB_DIR="$(brew --prefix gperftools)/lib"
# export CMAKE_CXX_CLANG_TIDY="$(brew --prefix llvm)/bin/clang-tidy;-header-filter=.*"

cd build

if [ ! -z $SETUP ]
then
  rm -rf ./*
  conan profile new default --detect --force
  conan profile update settings.compiler.libcxx=libc++ default
  conan install -pr default --build=missing ..
  cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_CXX_CLANG_TIDY="$CMAKE_CXX_CLANG_TIDY" ..
  mv compile_commands.json ..
fi

cmake --build . --target $TARGET

if [ ! -z $DEBUG ]
then
  $PREFIX/bin/lldb -o run bin/$TARGET
elif [ ! -z $PROFILE ]
then
  export CPUPROFILE=profile.prof
  bin/$TARGET
  pprof bin/$TARGET $CPUPROFILE
else
  bin/$TARGET
fi
