#!/bin/bash
set -e
echo "Start build"
find . -type f -exec touch {} +
make CONFIG_SENSORS_TMF8806=m
echo "Build done"
cp -v tmf8806.ko ./install
cp -v ./arch/arm/boot/dts/tmf8806-overlay.dtbo ./install
cp -v ./arch/arm/boot/dts/tmf8806-overlay-fpc.dtbo ./install
