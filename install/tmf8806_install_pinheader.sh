#!/bin/bash
#
#
# make all the following files are in the current directory
# tof.ko  tmf8806-overlay.dtbo  
# this script run with su privilege.
set -e
if grep -Fxq "dtoverlay=tmf8806-overlay" /boot/config.txt
then
    echo "No need to modify /boot/config.txt"
else
    echo "Append dtoverlay=tmf8806-overlay to /boot/config.txt"
    echo "dtoverlay=tmf8806-overlay" >> /boot/config.txt
fi
cp tmf8806-overlay.dtbo /boot/overlays/
#cp main_app.hex /lib/firmware/tmf8806_firmware.bin   # no patch download for now
cp tmf8806.ko /opt/USBSensorBridgeRuntime/modules/
echo "modules/tmf8806.ko" > /opt/USBSensorBridgeRuntime/config/default_driver.config
