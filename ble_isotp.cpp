#include <Arduino.h>
#include "isotp-c/isotp.h"
#include "ble_isotp.h"
#include "isotp_link_containers.h"
#include "ble.h"
#include "utilities.h"

int tx_isotp_on_ble_rx(uint16_t request_arbitration_id, uint16_t reply_arbitration_id, uint8_t *msg, uint16_t msg_length, Task *task) {
  Serial.printf("tx_isotp_on_ble_rx: sending to request_arbitration_id = %04x reply_arbitration_id = %04x msg_length = %04x...\n", request_arbitration_id, reply_arbitration_id, msg_length);
  IsoTpLinkContainer *link_container = find_link_container_by_request_arbitration_id(request_arbitration_id);
  assert(link_container != NULL);
  // check if link is currently sending?
  for (;;) {
    if (link_container->isotp_link.send_status != ISOTP_SEND_STATUS_INPROGRESS) {
      break;
    }
    if (task == NULL) { // bluetooth callback?
      delay(1);
    } else {
      task->delay(1);
    }
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
    if (task == NULL) { // bluetooth callback?
      delay(1);
    } else {
      task->delay(1);
    }
  }
  // check result
  return link_container->isotp_link.send_protocol_result;
}

void tx_ble_on_isotp_rx(uint16_t rx_id, uint16_t tx_id, uint8_t *buffer, uint16_t len, Task *task) {
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
  task->delay(10);
}