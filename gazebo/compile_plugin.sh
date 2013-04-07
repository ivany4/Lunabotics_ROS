#!/bin/bash 

echo "Cleaning targets"
rm -rf build
echo "Compiling targets"
mkdir -p build
cd build
cmake -DCMAKE_INSTALL_PREFIX=~/local ..
make
make install
