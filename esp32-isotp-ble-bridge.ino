// ble
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
// esp32 twai
#include "driver/twai.h"
// iso-tp
#include <isotp.h> // need to add https://github.com/SimonCahill/isotp-c as a library in a .zip file and then manually uncomment #define ISO_TP_FRAME_PADDING
// task scheduler
#include <TaskScheduler.h> // need to add https://github.com/arkhipenko/TaskScheduler library

// protocol
#define PROTOCOL_HEADER_SIZE 8

// BLE
#define SERVICE_UUID "0000abf0-0000-1000-8000-00805f9b34fb"
#define DATA_NOTIFY_CHARACTERISTIC_UUID "0000abf2-0000-1000-8000-00805f9b34fb"
#define COMMAND_WRITE_CHARACTERISTIC_UUID "0000abf3-0000-1000-8000-00805f9b34fb"
#define DEVICE_NAME "BLE_TO_ISOTP"

// ISOTP
#define ISOTP_BUFSIZE 4096
#define ISO_TP_DEFAULT_ST_MIN_US 0 // TODO: higher
#define ISO_TP_DEFAULT_RESPONSE_TIMEOUT_US 100000
#define ISO_TP_DEFAULT_BLOCK_SIZE 8 // TODO: larger
#define ISO_TP_FRAME_PADDING
#define ISO_TP_FRAME_PADDING_VALUE 0xAA

// TWAI (Macchina A0)
#define RX_PIN GPIO_NUM_4
#define TX_PIN GPIO_NUM_5

// ISO-TP
typedef struct {
  bool initialized;
  uint16_t request_arbitration_id;
  uint16_t reply_arbitration_id;
  IsoTpLink isotp_link;
  uint8_t isotp_rx_buffer[ISOTP_BUFSIZE];
  uint8_t isotp_tx_buffer[ISOTP_BUFSIZE];
  uint8_t isotp_payload_buffer[ISOTP_BUFSIZE];
} IsoTpLinkContainer;

IsoTpLinkContainer link_containers[4];

// periodic messages
typedef struct {
  bool active;
  uint16_t request_arbitration_id;
  uint16_t reply_arbitration_id;
  uint32_t interval_ms;
  uint8_t msg[128];
  uint16_t msg_length;
} PeriodicMessageContainer;

PeriodicMessageContainer periodic_message_containers[4];

// math
void write_uint32_be(uint32_t value, uint8_t *output) {
  output[3] = value & 0xFF;
  output[2] = (value >> 8) & 0xFF;
  output[1] = (value >> 16) & 0xFF;
  output[0] = (value >> 24) & 0xFF;
}

uint32_t read_uint32_be(const uint8_t *data) {
  return data[3] | (data[2] << 8) | (data[1] << 16) | (data[0] << 24);
}

uint16_t read_uint16_be(const uint8_t *data) {
  return data[1] | (data[0] << 8);
}

// ISOTP user functions
void isotp_user_debug(const char* format, ...) {
  // TODO: no Serial.vprintf
}

int isotp_user_send_can(uint32_t arbitration_id, const uint8_t* data, uint8_t size) {
  /*Serial.printf("isotp_user_send_can ");
  for (int i = 0; i < size; ++i) {
    Serial.printf("%02x", data[i]);
  }
  Serial.printf("\n");*/
  int ret_val = can_send(arbitration_id, data, size);
  if (ret_val != ESP_OK) {
    Serial.printf("isotp_user_send_can: can_send ret_val = %08x\n", ret_val);
    // we need to stop -> start the TWAI driver or else we'll have 0 success flashing after a timeout has occurred
    //can_reset();
    return ISOTP_RET_ERROR;
  }
  return ISOTP_RET_OK;
}

uint32_t isotp_user_get_us() {
  return micros();
}

// BLE
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
BLEServer *pServer;
BLEService *pService;
BLECharacteristic *pDataNotifyCharacteristic;
BLECharacteristic *pCommandWriteCharacteristic;
static uint8_t ble_tx_command_buf[ISOTP_BUFSIZE + PROTOCOL_HEADER_SIZE] = {0};
static uint8_t ble_rx_command_buf[ISOTP_BUFSIZE + PROTOCOL_HEADER_SIZE] = {0};
ble_states ble_state = WAITING_FOR_CLIENT;

class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    Serial.printf("onConnect\n");
    if (ble_state == WAITING_FOR_CLIENT) {
      ble_state = HAVE_CLIENT;
    }
  };

  void onDisconnect(BLEServer* pServer) {
    Serial.printf("onDisconnect\n");
    ble_state = WAITING_FOR_CLIENT;
    // restart?
    ESP.restart();
  }
};

int tx_isotp_on_ble_rx(uint16_t request_arbitration_id, uint16_t reply_arbitration_id, uint8_t *msg, uint16_t msg_length) {
  Serial.printf("tx_isotp_on_ble_rx: sending to request_arbitration_id = %04x reply_arbitration_id = %04x msg_length = %04x...\n", request_arbitration_id, reply_arbitration_id, msg_length);
  IsoTpLinkContainer *link_container = find_link_container_by_request_arbitration_id(request_arbitration_id);
  // check if link is currently sending?
  for (;;) {
    if (link_container->isotp_link.send_status != ISOTP_SEND_STATUS_INPROGRESS) {
      break;
    }
    //delay(1);
    yield();
  }
  // start sending
  int ret_val = isotp_send_with_id(&link_container->isotp_link, request_arbitration_id, msg, msg_length);
  if (ret_val != ISOTP_RET_OK) {
    Serial.printf("isotp_send_with_id: ret_val = %08x\n", ret_val);
  }
  // wait for sending all frames to finish?
  for (;;) {
    if (link_container->isotp_link.send_status != ISOTP_SEND_STATUS_INPROGRESS) {
      break;
    }
    //delay(1);
    yield();
  }
  // check result
  return link_container->isotp_link.send_protocol_result;
}

void tx_ble_on_isotp_rx(uint16_t rx_id, uint16_t tx_id, uint8_t *buffer, uint16_t len) {
  Serial.printf("tx_ble_on_isotp_rx rx_id = %04x tx_id = %04x len = %04x\n", rx_id, tx_id, len);
  // make sure we have a client to send to?
  assert(ble_state != WAITING_FOR_CLIENT);
  // build message
  write_uint32_be(tx_id, ble_tx_command_buf); // flipped?
  write_uint32_be(rx_id, ble_tx_command_buf + 4); // flipped?
  memcpy(ble_tx_command_buf + 8, buffer, len);
  // set value + notify
  pDataNotifyCharacteristic->setValue(ble_tx_command_buf, len + 8);
  pDataNotifyCharacteristic->notify();
  // sleep to prevent bluetooth congestion?
  delay(10);
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
    char *name = (char*)(data + pointer);
    // log
    Serial.printf("CONFIGURE_ISOTP_LINK: link_index = %02x request_arbitration_id = %08x reply_arbitration_id = %08x name_len = %08x name = %s\n", link_index, request_arbitration_id, reply_arbitration_id, name_len, name);
    // validate
    assert(link_index < 4);
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
    Serial.printf("UPLOAD_ISOTP_CHUNK chunk_offset = %04x chunk_length = %04x\n", chunk_offset, chunk_length);
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
    Serial.printf("FLUSH_ISOTP_PAYLOAD payload_length = %04x request_arbitration_id = %08x reply_arbitration_id = %08x msg_length = %04x\n", payload_length, request_arbitration_id, reply_arbitration_id, msg_length);
    // dispatch
    int ret_val = tx_isotp_on_ble_rx(request_arbitration_id, reply_arbitration_id, msg, msg_length);
    if (ret_val == ISOTP_SEND_STATUS_ERROR) {
      Serial.printf("ISOTP_SEND_STATUS_ERROR\n");
    }
    // TODO: send command success?
  } else if (ble_command_id == START_PERIODIC_MESSAGE) {
    uint8_t periodic_message_index = data[1];
    assert(periodic_message_index < 4);
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
      assert(msg_length < 128);
      pointer += 2;
      periodic_message_containers[periodic_message_index].msg_length = msg_length;
      memcpy(periodic_message_containers[periodic_message_index].msg, data + pointer, msg_length);
    }
    periodic_message_containers[periodic_message_index].active = true;
    // TODO: send command success?
  } else if (ble_command_id == STOP_PERIODIC_MESSAGE) {
    uint8_t periodic_message_index = data[1];
    assert(periodic_message_index < 4);
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

class CommandWriteCharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic, esp_ble_gatts_cb_param_t* param) {
    // lock
    //ble_command_mtx.lock();
    // process
    uint8_t *data = pCharacteristic->getData();
    size_t data_length = pCharacteristic->getLength();
    process_ble_command(data, data_length);
    // unlock
    //ble_command_mtx.unlock();
  }
};

// link containers
IsoTpLinkContainer* find_link_container_by_request_arbitration_id(uint16_t request_arbitration_id) {
  for (int i = 0; i < 4; ++i) {
    if (link_containers[i].initialized == false) {
      continue;
    }
    if (link_containers[i].request_arbitration_id == request_arbitration_id) {
      return &link_containers[i];
    }
  }
  return NULL;
}

IsoTpLinkContainer* find_link_container_by_reply_arbitration_id(uint16_t reply_arbitration_id) {
  for (int i = 0; i < 4; ++i) {
    if (link_containers[i].initialized == false) {
      continue;
    }
    if (link_containers[i].reply_arbitration_id == reply_arbitration_id) {
      return &link_containers[i];
    }
  }
  return NULL;
}

// isotp
void isotp_poll_task_callback() {
  for (int i = 0; i < 4; ++i) {
    // skip uninitialized links
    if (link_containers[i].initialized == false) {
      continue;
    }
    // poll
    isotp_poll(&link_containers[i].isotp_link);
  }
}

void isotp_receive_task_callback() {
  for (int i = 0; i < 4; ++i) {
    // skip uninitialized links
    if (link_containers[i].initialized == false) {
      continue;
    }
    // receive
    uint16_t out_size = 0;
    int isotp_receive_ret_val = isotp_receive(&link_containers[i].isotp_link, link_containers[i].isotp_payload_buffer, ISOTP_BUFSIZE, &out_size);
    if (isotp_receive_ret_val == ISOTP_RET_OK) {
      tx_ble_on_isotp_rx(link_containers[i].request_arbitration_id, link_containers[i].reply_arbitration_id, link_containers[i].isotp_payload_buffer, out_size);
    } else if (isotp_receive_ret_val == ISOTP_RET_NO_DATA) {
      // expected timeout trying to read (no data available?)
    } else {
      Serial.printf("isotp_receive_ret_val = %08x\n", isotp_receive_ret_val);
    }
  }
}

void isotp_setup() {
  for (int i = 0; i < 4; ++i) {
    link_containers[i].initialized = false;
    memset(link_containers[i].isotp_payload_buffer, 0, ISOTP_BUFSIZE);
    memset(link_containers[i].isotp_rx_buffer, 0, ISOTP_BUFSIZE);
    memset(link_containers[i].isotp_tx_buffer, 0, ISOTP_BUFSIZE);
  }
}

// periodic
void periodic_messages_task_callback() {
  for (int i = 0; i < 4; ++i) {
    PeriodicMessageContainer periodic_message_container = periodic_message_containers[i];
    if (periodic_message_container.active == false) {
      continue;
    }
    tx_isotp_on_ble_rx(periodic_message_container.request_arbitration_id, periodic_message_container.reply_arbitration_id, periodic_message_container.msg, periodic_message_container.msg_length);
    delay(periodic_message_container.interval_ms);
  }
}

void periodic_setup() {
  for (int i = 0; i < 4; ++i) {
    periodic_message_containers[i].active = false;
    memset(periodic_message_containers[i].msg, 0, 128);
  }
}

// twai
void twai_setup() {
  // transceiver silence workaround
  pinMode(GPIO_NUM_21, OUTPUT);
  digitalWrite(GPIO_NUM_21, LOW);

  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  //Look in the api-reference for other speed sets.
  //twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_filter_config_t f_config = {
    .acceptance_code = (0x7e8 << 21),
    .acceptance_mask = ~(0x7FF << 21),
    .single_filter = true
	};

  // Install TWAI driver
  assert(twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK);

  // Start TWAI driver
  assert(twai_start() == ESP_OK);
  
  // Reconfigure alerts to detect TX alerts and Bus-Off errors
  uint32_t alerts_to_enable = TWAI_ALERT_ABOVE_ERR_WARN |
    TWAI_ALERT_ARB_LOST |
    TWAI_ALERT_BELOW_ERR_WARN |
    TWAI_ALERT_BUS_ERROR |
    TWAI_ALERT_BUS_OFF |
    TWAI_ALERT_BUS_RECOVERED |
    TWAI_ALERT_ERR_ACTIVE |
    TWAI_ALERT_ERR_PASS |
    TWAI_ALERT_RECOVERY_IN_PROGRESS |
    TWAI_ALERT_RX_DATA |
    TWAI_ALERT_RX_QUEUE_FULL |
    TWAI_ALERT_TX_FAILED |
    TWAI_ALERT_TX_IDLE |
    TWAI_ALERT_TX_SUCCESS;
  assert(twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK);
}

// ble
void ble_setup() {
  // BLE
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

// can
static int can_send(uint16_t arbitration_id, const uint8_t *buf, size_t size) {
  assert(size == 8);
  twai_message_t message;
  message.identifier = arbitration_id;
  message.data_length_code = size;
  for (int i = 0; i < size; ++i) {
    message.data[i] = buf[i];
  }
  return twai_transmit(&message, pdMS_TO_TICKS(10));
}

static int can_recv(uint16_t *arbitration_id, uint8_t *buf, size_t *size)
{
  twai_message_t rx_message;
  int ret_val = twai_receive(&rx_message, pdMS_TO_TICKS(10));
  if (ret_val == ESP_OK) {
    *arbitration_id = rx_message.identifier;
    *size = rx_message.data_length_code;
    assert(*size == 8);
    memcpy(buf, rx_message.data, rx_message.data_length_code);
  }
  return ret_val;
}

void can_rx_task_callback() {
  uint16_t arbitration_id = 0;
  size_t size = 0;
  uint8_t can_rx_buf[8] = {0};
  int ret_val = can_recv(&arbitration_id, can_rx_buf, &size);
  if (ret_val == ESP_OK) {
    if (arbitration_id == 0x7e8) {
      //Serial.printf("Received CAN message: identifier = %08x data_length_code = %02x data=%02x%02x%02x%02x%02x%02x%02x%02x\n", arbitration_id, size, can_rx_buf[0], can_rx_buf[1], can_rx_buf[2], can_rx_buf[3], can_rx_buf[4], can_rx_buf[5], can_rx_buf[6], can_rx_buf[7]);
      IsoTpLinkContainer *link_container = find_link_container_by_reply_arbitration_id(arbitration_id);
      assert(link_container != NULL);
      isotp_on_can_message(&link_container->isotp_link, can_rx_buf, size);
    } else {
      Serial.printf("dropping RX'd CAN message identifier = %08x\n", arbitration_id);
    }
  } else if (ret_val == ESP_ERR_TIMEOUT) {
    // expected?
  } else {
    Serial.printf("Failed to receive message ret_val = %08x\n", ret_val);
  }
}

void can_alerts_task_callback() {
  // Check if alert happened
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(10));
  twai_status_info_t twaistatus;
  twai_get_status_info(&twaistatus);

  // Handle alerts
  if (alerts_triggered & TWAI_ALERT_ABOVE_ERR_WARN) {
    Serial.printf("TWAI_ALERT_ABOVE_ERR_WARN\n");
  }
  if (alerts_triggered & TWAI_ALERT_ARB_LOST) {
    Serial.printf("TWAI_ALERT_ARB_LOST\n");
  }
  if (alerts_triggered & TWAI_ALERT_BELOW_ERR_WARN) {
    Serial.printf("TWAI_ALERT_BELOW_ERR_WARN\n");
  }
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
    Serial.printf("TWAI_ALERT_BUS_ERROR\n");
    Serial.printf("Bus error count: %d\n", twaistatus.bus_error_count);
    twai_initiate_recovery();
  }
  if (alerts_triggered & TWAI_ALERT_BUS_OFF) {
    Serial.printf("TWAI_ALERT_BUS_OFF\n");
    twai_initiate_recovery();
  }
  if (alerts_triggered & TWAI_ALERT_BUS_RECOVERED) {
    Serial.printf("TWAI_ALERT_BUS_RECOVERED\n");
    twai_clear_transmit_queue();
    twai_clear_receive_queue();
  }
  if (alerts_triggered & TWAI_ALERT_ERR_ACTIVE) {
    Serial.printf("TWAI_ALERT_ERR_ACTIVE\n");
  }
  if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
    Serial.printf("TWAI_ALERT_ERR_PASS\n");
  }
  if (alerts_triggered & TWAI_ALERT_RECOVERY_IN_PROGRESS) {
    Serial.printf("TWAI_ALERT_RECOVERY_IN_PROGRESS\n");
  }
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    //Serial.printf("TWAI_ALERT_RX_DATA\n");
  }
  if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
    Serial.printf("TWAI_ALERT_RX_QUEUE_FULL\n");
  }
  if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
    Serial.printf("TWAI_ALERT_TX_FAILED\n");
    Serial.printf("TX buffered: %d\n", twaistatus.msgs_to_tx);
    Serial.printf("TX error: %d\n", twaistatus.tx_error_counter);
    Serial.printf("TX failed: %d\n", twaistatus.tx_failed_count);
  }
  if (alerts_triggered & TWAI_ALERT_TX_IDLE) {
    //Serial.printf("TWAI_ALERT_TX_IDLE\n");
  }
  if (alerts_triggered & TWAI_ALERT_TX_SUCCESS) {
    //Serial.printf("TWAI_ALERT_TX_SUCCESS\n");
    //Serial.printf("TX buffered: %d\n", twaistatus.msgs_to_tx);
  }
}

// scheduler + tasks
Scheduler ts;
Task can_alerts_task(TASK_IMMEDIATE, TASK_FOREVER, &can_alerts_task_callback, &ts, true);
Task can_rx_task(TASK_IMMEDIATE, TASK_FOREVER, &can_rx_task_callback, &ts, true);
Task isotp_poll_task(TASK_IMMEDIATE, TASK_FOREVER, &isotp_poll_task_callback, &ts, true);
Task isotp_receive_task(TASK_IMMEDIATE, TASK_FOREVER, &isotp_receive_task_callback, &ts, true);
Task periodic_messages_task(TASK_IMMEDIATE, TASK_FOREVER, &periodic_messages_task_callback, &ts, true);

void setup() {
  Serial.begin(115200);
  twai_setup();
  isotp_setup();
  ble_setup();
  periodic_setup();
}

void loop() {
  ts.execute();
}
