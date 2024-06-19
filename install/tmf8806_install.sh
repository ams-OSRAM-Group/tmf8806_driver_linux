#!/bin/bash
#
#
# make all the following files are in the current directory
# tof.ko  tmf8806-overlay-fpc.dtbo  
# this script run with su privilege.
set -e

if grep -Fxq "dtoverlay=i2c0,pins_28_29" /boot/config.txt
then
    echo "2. No need to modify /boot/config.txt"
else
    echo "2. Append dtoverlay=i2c0,pins_28_29 to /boot/config.txt"
    echo "dtoverlay=i2c0,pins_28_29" >> /boot/config.txt
fi

if grep -Fxq "dtparam=i2c0_baudrate=400000" /boot/config.txt
then
    echo "3. No need to modify /boot/config.txt"
else
    echo "3. Append dtparam=i2c0_baudrate=400000 to /boot/config.txt"
    echo "dtparam=i2c0_baudrate=400000" >> /boot/config.txt
fi

if grep -Fxq "dtoverlay=tmf8806-overlay-fpc" /boot/config.txt
then
    echo "1. No need to modify /boot/config.txt"
else
    echo "1. Append dtoverlay=tmf8806-overlay-fpc to /boot/config.txt"
    echo "dtoverlay=tmf8806-overlay-fpc" >> /boot/config.txt
fi

cp tmf8806-overlay-fpc.dtbo /boot/overlays/
#cp main_app.hex /lib/firmware/tmf8806_firmware.bin   # no patch download for now
cp tmf8806.ko /opt/USBSensorBridgeRuntime/modules/
echo "modules/tmf8806.ko" > /opt/USBSensorBridgeRuntime/config/default_driver.config
