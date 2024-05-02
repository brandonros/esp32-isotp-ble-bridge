#ifndef TWAI_H
#define TWAI_H

#include <stdint.h>
#include "driver/twai.h"

#define RX_PIN GPIO_NUM_4
#define TX_PIN GPIO_NUM_5

#define CAN_FRAME_SIZE 8

#define TWAI_SEND_MS 0 // TODO: really not sure what to do here
#define TWAI_RECEIVE_MS 0 // TODO: really not sure what to do here
#define TWAI_READ_ALERTS_MS 1 // TODO: really not sure what to do here

#define REPLY_ARBITRATION_ID 0x7E8 // TODO: support more than just 1 arbitration ID through filters

void twai_setup();
int twai_recv(uint16_t *arbitration_id, uint8_t *buf, size_t *size);
int twai_send(uint16_t arbitration_id, const uint8_t *buf, size_t size);

static uint8_t twai_rx_buf[CAN_FRAME_SIZE];

static twai_message_t rx_message;
static twai_message_t tx_message;

#endif
