#ifndef BLE_H
#define BLE_H

#include <mutex>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define SERVICE_UUID "0000abf0-0000-1000-8000-00805f9b34fb"
#define DATA_NOTIFY_CHARACTERISTIC_UUID "0000abf2-0000-1000-8000-00805f9b34fb"
#define COMMAND_WRITE_CHARACTERISTIC_UUID "0000abf3-0000-1000-8000-00805f9b34fb"
#define DEVICE_NAME "BLE_TO_ISOTP"

enum ble_states {
  WAITING_FOR_CLIENT,
  HAVE_CLIENT,
  CLIENT_NOTIFIED
};
enum ble_command_ids {
  UPLOAD_ISOTP_CHUNK = 0x02,
  FLUSH_ISOTP_PAYLOAD = 0x03,
  START_PERIODIC_MESSAGE = 0x04,
  STOP_PERIODIC_MESSAGE = 0x05,
  CONFIGURE_ISOTP_LINK = 0x06
};
extern BLEServer *pServer;
extern BLEService *pService;
extern BLECharacteristic *pDataNotifyCharacteristic;
extern BLECharacteristic *pCommandWriteCharacteristic;
extern uint8_t *ble_tx_command_buf;
extern uint8_t *ble_rx_command_buf;
extern ble_states ble_state;
extern std::mutex ble_command_mtx;

void ble_setup();
void process_ble_command(uint8_t *data, size_t data_length);

#endif
