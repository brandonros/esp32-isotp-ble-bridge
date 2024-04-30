#include <Arduino.h>
#include "isotp.h"
#include "isotp_link_containers.h"
#include "ble_isotp.h"

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
    int isotp_receive_ret_val = isotp_receive(&link_containers[i].isotp_link, isotp_payload_buffer, ISOTP_BUFSIZE, &out_size);
    if (isotp_receive_ret_val == ISOTP_RET_OK) {
      tx_ble_on_isotp_rx(link_containers[i].request_arbitration_id, link_containers[i].reply_arbitration_id, isotp_payload_buffer, out_size);
    } else if (isotp_receive_ret_val == ISOTP_RET_NO_DATA) {
      // expected timeout trying to read (no data available?)
    } else {
      Serial.printf("isotp_receive_ret_val = %08x\n", isotp_receive_ret_val);
    }
  }
}

void isotp_setup() {
  isotp_payload_buffer = (uint8_t*)malloc(ISOTP_BUFSIZE);
  assert(isotp_payload_buffer != NULL);
  memset(isotp_payload_buffer, 0, ISOTP_BUFSIZE);

  for (int i = 0; i < 4; ++i) {
    link_containers[i].initialized = false;

    link_containers[i].isotp_rx_buffer = (uint8_t*)malloc(ISOTP_BUFSIZE);
    assert(link_containers[i].isotp_rx_buffer != NULL);
    memset(link_containers[i].isotp_rx_buffer, 0, ISOTP_BUFSIZE);    

    link_containers[i].isotp_tx_buffer = (uint8_t*)malloc(ISOTP_BUFSIZE);
    assert(link_containers[i].isotp_tx_buffer != NULL);    
    memset(link_containers[i].isotp_tx_buffer, 0, ISOTP_BUFSIZE);
  }
}