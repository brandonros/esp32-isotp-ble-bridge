#include <Arduino.h>
#include "periodic_messages.h"
#include "ble_isotp.h"

PeriodicMessageContainer periodic_message_containers[4];

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
    periodic_message_containers[i].msg = (uint8_t*)malloc(128); // TODO: support periodic messages higher than 128?
    assert(periodic_message_containers[i].msg != NULL);
    memset(periodic_message_containers[i].msg, 0, 128);
  }
}
