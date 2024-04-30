#include <mutex>
#include <Arduino.h>
#include <isotp.h>
#include "isotp_link_containers.h"
#include "utilities.h"
#include "ble.h"
#include "ble_isotp.h"
#include "periodic_messages.h"
#include "protocol.h"

std::mutex ble_command_mtx;

class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    Serial.printf("onConnect\n");
    assert(ble_state == WAITING_FOR_CLIENT);
    ble_state = HAVE_CLIENT;
  };

  void onDisconnect(BLEServer* pServer) {
    Serial.printf("onDisconnect\n");
    assert(ble_state == HAVE_CLIENT);
    ble_state = WAITING_FOR_CLIENT;
    // restart?
    ESP.restart();
  }
};

class CommandWriteCharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic, esp_ble_gatts_cb_param_t* param) {
    // lock
    ble_command_mtx.lock();
    // process
    uint8_t *data = pCharacteristic->getData();
    size_t data_length = pCharacteristic->getLength();
    process_ble_command(data, data_length);
    // unlock
    ble_command_mtx.unlock();
  }
};

void ble_setup() {
  ble_state = WAITING_FOR_CLIENT;

  ble_tx_command_buf = (uint8_t*)malloc(ISOTP_BUFSIZE + PROTOCOL_HEADER_SIZE);
  assert(ble_tx_command_buf != NULL);
  memset(ble_tx_command_buf, 0, ISOTP_BUFSIZE + PROTOCOL_HEADER_SIZE);

  ble_rx_command_buf = (uint8_t*)malloc(ISOTP_BUFSIZE + PROTOCOL_HEADER_SIZE);
  assert(ble_rx_command_buf != NULL);
  memset(ble_rx_command_buf, 0, ISOTP_BUFSIZE + PROTOCOL_HEADER_SIZE);

  BLEDevice::init(DEVICE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  pService = pServer->createService(SERVICE_UUID);
  // BLE data notify
  pDataNotifyCharacteristic = pService->createCharacteristic(
    DATA_NOTIFY_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pDataNotifyCharacteristic->addDescriptor(new BLE2902());
  // BLE command write
  pCommandWriteCharacteristic = pService->createCharacteristic(
    COMMAND_WRITE_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE_NR // TODO: PROPERTY_WRITE instead of PROPERTY_WRITE_NR?
  );
  pCommandWriteCharacteristic->setCallbacks(new CommandWriteCharacteristicCallbacks());
  // BLE start
  pService->start();
  // BLE advertising
  pServer->getAdvertising()->addServiceUUID(pService->getUUID());
  pServer->getAdvertising()->start();
}

void process_ble_command(uint8_t *data, size_t data_length) {
  ble_command_ids ble_command_id = (ble_command_ids)data[0];
  if (ble_command_id == CONFIGURE_ISOTP_LINK) {
    // parse
    size_t pointer = 1;
    uint32_t link_index = read_uint32_be(data + pointer);
    pointer += 4;
    uint32_t request_arbitration_id = read_uint32_be(data + pointer);
    pointer += 4;
    uint32_t reply_arbitration_id = read_uint32_be(data + pointer);
    pointer += 4;
    uint32_t name_len = read_uint32_be(data + pointer);
    pointer += 4;
    char *name = (char*)(data + pointer); // should include \0
    // log
    Serial.printf("CONFIGURE_ISOTP_LINK: link_index = %02x request_arbitration_id = %08x reply_arbitration_id = %08x name_len = %08x name = %s\n", link_index, request_arbitration_id, reply_arbitration_id, name_len, name);
    // validate
    assert(link_index < NUM_LINK_CONTAINERS);
    // configure
    isotp_init_link(&link_containers[link_index].isotp_link, request_arbitration_id, link_containers[link_index].isotp_tx_buffer, ISOTP_BUFSIZE, link_containers[link_index].isotp_rx_buffer, ISOTP_BUFSIZE);
    link_containers[link_index].initialized = true;
    link_containers[link_index].request_arbitration_id = request_arbitration_id;
    link_containers[link_index].reply_arbitration_id = reply_arbitration_id;
    // TODO: send command success?
  } else if (ble_command_id == UPLOAD_ISOTP_CHUNK) {
    // parse
    uint32_t chunk_offset = read_uint16_be(data + 1);
    uint32_t chunk_length = read_uint16_be(data + 3);
    uint8_t *bytes = data + 5;
    // log
    Serial.printf("UPLOAD_ISOTP_CHUNK: chunk_offset = %04x chunk_length = %04x\n", chunk_offset, chunk_length);
    // copy
    memcpy(ble_rx_command_buf + chunk_offset, bytes, chunk_length);
    // TODO: send command success?
  } else if (ble_command_id == FLUSH_ISOTP_PAYLOAD) {
    // parse
    uint16_t payload_length = read_uint16_be(data + 1);
    uint32_t request_arbitration_id = read_uint32_be(ble_rx_command_buf);
    uint32_t reply_arbitration_id = read_uint32_be(ble_rx_command_buf + 4);
    uint16_t msg_length = payload_length - 8;
    uint8_t *msg = ble_rx_command_buf + 8;
    // log
    Serial.printf("FLUSH_ISOTP_PAYLOAD: payload_length = %04x request_arbitration_id = %08x reply_arbitration_id = %08x msg_length = %04x\n", payload_length, request_arbitration_id, reply_arbitration_id, msg_length);
    // dispatch
    int ret_val = tx_isotp_on_ble_rx(request_arbitration_id, reply_arbitration_id, msg, msg_length);
    if (ret_val == ISOTP_SEND_STATUS_ERROR) {
      Serial.printf("ISOTP_SEND_STATUS_ERROR\n");
    }
    // TODO: send command success?
  } else if (ble_command_id == START_PERIODIC_MESSAGE) {
    uint8_t periodic_message_index = data[1];
    assert(periodic_message_index < NUM_LINK_CONTAINERS);
    uint16_t interval_ms = read_uint16_be(data + 2);
    uint32_t rx_address = read_uint32_be(data + 4);
    uint32_t tx_address = read_uint32_be(data + 8);
    uint16_t num_msgs = read_uint16_be(data + 12);
    assert(num_msgs == 1);
    Serial.printf("START_PERIODIC_MESSAGE: periodic_message_index = %d interval_ms = %d rx_address = %08x tx_address = %08x num_msgs = %08x\n", periodic_message_index, interval_ms, rx_address, tx_address, num_msgs);
    IsoTpLinkContainer *isotp_link_container = find_link_container_by_request_arbitration_id(rx_address);
    assert(isotp_link_container != NULL);
    int pointer = 14;
    periodic_message_containers[periodic_message_index].request_arbitration_id = rx_address;
    periodic_message_containers[periodic_message_index].reply_arbitration_id = tx_address;
    periodic_message_containers[periodic_message_index].interval_ms = interval_ms;
    // TODO: num_msgs is really only ever 1
    for (int i = 0; i < num_msgs; ++i) {
      uint16_t msg_length = read_uint16_be(data + pointer);
      assert(msg_length < MAX_PERIODIC_MESSAGE_SIZE);
      pointer += 2;
      periodic_message_containers[periodic_message_index].msg_length = msg_length;
      memcpy(periodic_message_containers[periodic_message_index].msg, data + pointer, msg_length);
    }
    periodic_message_containers[periodic_message_index].active = true;
    // TODO: send command success?
  } else if (ble_command_id == STOP_PERIODIC_MESSAGE) {
    uint8_t periodic_message_index = data[1];
    assert(periodic_message_index < NUM_LINK_CONTAINERS);
    uint32_t rx_address = read_uint32_be(data + 2);
    uint32_t tx_address = read_uint32_be(data + 6);
    Serial.printf("STOP_PERIODIC_MESSAGE: periodic_message_index = %d rx_address = %08x tx_address = %08x\n", periodic_message_index, rx_address, tx_address);
    IsoTpLinkContainer *isotp_link_container = find_link_container_by_request_arbitration_id(rx_address);
    assert(isotp_link_container != NULL);
    periodic_message_containers[periodic_message_index].active = false;
    // TODO: send command success?
  } else {
    Serial.printf("unknown command ID: %02x\n", ble_command_id);
  }
}
