#include <Arduino.h>
#include "isotp.h"
#include "isotp_link_containers.h"
#include "ble_isotp.h"

uint8_t *isotp_payload_buffer;

void isotp_setup() {
  isotp_payload_buffer = (uint8_t*)malloc(ISOTP_BUFSIZE);
  assert(isotp_payload_buffer != NULL);
  memset(isotp_payload_buffer, 0, ISOTP_BUFSIZE);

  for (int i = 0; i < NUM_LINK_CONTAINERS; ++i) {
    link_containers[i].initialized = false;

    link_containers[i].isotp_rx_buffer = (uint8_t*)malloc(ISOTP_BUFSIZE);
    assert(link_containers[i].isotp_rx_buffer != NULL);
    memset(link_containers[i].isotp_rx_buffer, 0, ISOTP_BUFSIZE);    

    link_containers[i].isotp_tx_buffer = (uint8_t*)malloc(ISOTP_BUFSIZE);
    assert(link_containers[i].isotp_tx_buffer != NULL);    
    memset(link_containers[i].isotp_tx_buffer, 0, ISOTP_BUFSIZE);
  }
}