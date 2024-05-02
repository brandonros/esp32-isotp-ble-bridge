#include <Arduino.h>
#include "periodic_messages.h"

PeriodicMessageContainer periodic_message_containers[NUM_PERIODIC_MESSAGE_CONTAINERS];

void periodic_setup() {
  for (int i = 0; i < NUM_PERIODIC_MESSAGE_CONTAINERS; ++i) {
    periodic_message_containers[i].active = false;
    periodic_message_containers[i].msg = (uint8_t*)malloc(MAX_PERIODIC_MESSAGE_SIZE);
    assert(periodic_message_containers[i].msg != NULL);
    memset(periodic_message_containers[i].msg, 0, MAX_PERIODIC_MESSAGE_SIZE);
  }
}
