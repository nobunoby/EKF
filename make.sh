#!/bin/bash

source ~/.bashrc
mkdir -p build && cd build
cmake ..
make
#./main
cd ..
