#ifndef BLE_ISOTP_H
#define BLE_ISOTP_H

#include <stdint.h>

void tx_ble_on_isotp_rx(uint16_t rx_id, uint16_t tx_id, uint8_t *buffer, uint16_t len);
int tx_isotp_on_ble_rx(uint16_t request_arbitration_id, uint16_t reply_arbitration_id, uint8_t *msg, uint16_t msg_length);

#endif
