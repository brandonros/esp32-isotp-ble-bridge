#ifndef TWAI_H
#define TWAI_H

#include <stdint.h>
#include "driver/twai.h"

#define RX_PIN GPIO_NUM_4
#define TX_PIN GPIO_NUM_5

void twai_setup();
int twai_recv(uint16_t *arbitration_id, uint8_t *buf, size_t *size);
void twai_rx_task_callback();
int twai_send(uint16_t arbitration_id, const uint8_t *buf, size_t size);
void twai_alerts_task_callback();

static uint8_t twai_rx_buf[8];

static twai_message_t rx_message;
static twai_message_t tx_message;

#endif
