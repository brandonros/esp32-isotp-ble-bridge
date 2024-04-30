#ifndef PERIODIC_MESSAGES_H
#define PERIODIC_MESSAGES_H

#include <stdint.h>

typedef struct {
  bool active;
  uint16_t request_arbitration_id;
  uint16_t reply_arbitration_id;
  uint32_t interval_ms;
  uint8_t *msg;
  uint16_t msg_length;
} PeriodicMessageContainer;

extern PeriodicMessageContainer periodic_message_containers[4];

void periodic_setup();
void periodic_messages_task_callback();

#endif
