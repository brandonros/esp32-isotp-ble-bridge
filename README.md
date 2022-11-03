# esp32-isotp-ble-bridge
Macchina A0 ESP32 Bluetooth Low Energy ISO15765 flasher

## How to use

```shell
# have v4.4.2 esp-idf in $IDF_PATH already 
git clone --recursive git@github.com:brandonros/esp32-isotp-ble-bridge.git
./build.sh
```

## How to distribute

```shell
python -m esptool \
  --chip esp32 \
  --port {{COM_PORT}} \
  --baud 460800 \
  --before default_reset \
  --after hard_reset \
  write_flash \
  -z \
  --flash_mode dio \
  --flash_freq 40m \
  --flash_size 2MB \
  0x1000 ./build/bootloader/bootloader.bin \
  0x10000 ./build/esp32_isotp_ble_bridge.bin \
  0x8000 ./build/partition_table/partition-table.bin
```
