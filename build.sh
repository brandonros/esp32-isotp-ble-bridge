#!/bin/bash
set -e

# git clone -b v4.4.7 --recursive https://github.com/espressif/esp-idf.git esp-idf-v4.4.7

export IDF_PATH="/Users/brandon/Desktop/esp-idf-v4.4.7"
export PIP_BREAK_SYSTEM_PACKAGES=1
cd ~/Desktop/esp-idf-v4.4.7
./install.sh
#python tools/idf_tools.py install --help
#python tools/idf_tools.py install-python-env
. export.sh

cd ~/Desktop/esp32-isotp-ble-bridge
idf.py -p /dev/tty.usbserial-023109E4 build flash monitor
