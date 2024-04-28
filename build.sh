#!/bin/bash
set -e
cd ~/Desktop/esp-idf-v5.1.3
/opt/homebrew/opt/python@3.12/bin/python3.12 /Users/brandon/Desktop/esp-idf-v5.1.3/tools/idf_tools.py install
/opt/homebrew/opt/python@3.12/bin/python3.12 /Users/brandon/Desktop/esp-idf-v5.1.3/tools/idf_tools.py install-python-env
cd ~/Desktop/esp32-isotp-ble-bridge
. /Users/brandon/Desktop/esp-idf-v5.1.3/export.sh
idf.py build flash monitor
