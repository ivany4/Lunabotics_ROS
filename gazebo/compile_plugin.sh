#!/bin/bash 

cd build
echo "Cleaning targets"
make clean
echo "Compiling targets"
cmake -DCMAKE_INSTALL_PREFIX=~/local ..
make
make install
