#include <Arduino.h>
#include "periodic_messages.h"
#include "ble_isotp.h"

PeriodicMessageContainer periodic_message_containers[NUM_PERIODIC_MESSAGE_CONTAINERS];

void periodic_messages_task_callback() {
  for (int i = 0; i < NUM_PERIODIC_MESSAGE_CONTAINERS; ++i) {
    PeriodicMessageContainer periodic_message_container = periodic_message_containers[i];
    if (periodic_message_container.active == false) {
      continue;
    }
    tx_isotp_on_ble_rx(periodic_message_container.request_arbitration_id, periodic_message_container.reply_arbitration_id, periodic_message_container.msg, periodic_message_container.msg_length);
    delay(periodic_message_container.interval_ms);
  }
}

void periodic_setup() {
  for (int i = 0; i < NUM_PERIODIC_MESSAGE_CONTAINERS; ++i) {
    periodic_message_containers[i].active = false;
    periodic_message_containers[i].msg = (uint8_t*)malloc(MAX_PERIODIC_MESSAGE_SIZE);
    assert(periodic_message_containers[i].msg != NULL);
    memset(periodic_message_containers[i].msg, 0, MAX_PERIODIC_MESSAGE_SIZE);
  }
}
