#ifndef PERIODIC_MESSAGES_H
#define PERIODIC_MESSAGES_H

#include <stdint.h>

#define NUM_PERIODIC_MESSAGE_CONTAINERS 4
#define MAX_PERIODIC_MESSAGE_SIZE 128 // TODO: support periodic messages higher than 128?

typedef struct {
  bool active;
  uint16_t request_arbitration_id;
  uint16_t reply_arbitration_id;
  uint32_t interval_ms;
  uint8_t *msg;
  uint16_t msg_length;
} PeriodicMessageContainer;

extern PeriodicMessageContainer periodic_message_containers[NUM_PERIODIC_MESSAGE_CONTAINERS];

void periodic_setup();

#endif
