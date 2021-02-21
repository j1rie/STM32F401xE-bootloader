#! /bin/bash
mkdir ./binaries &>/dev/null
make clean
make -j3
cp usbdfu.bin ./binaries/boot.F401.3k.bin
make clean
