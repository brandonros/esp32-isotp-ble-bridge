#ifndef ISOTP_LINK_CONTAINERS_H
#define ISOTP_LINK_CONTAINERS_H

#include <stdint.h>
#include "isotp-c/isotp.h"
#include "isotp.h"

#define NUM_LINK_CONTAINERS 4

typedef struct {
  bool initialized;
  uint16_t request_arbitration_id;
  uint16_t reply_arbitration_id;
  IsoTpLink isotp_link;
  uint8_t *isotp_rx_buffer;
  uint8_t *isotp_tx_buffer;
} IsoTpLinkContainer;

extern IsoTpLinkContainer link_containers[NUM_LINK_CONTAINERS];

IsoTpLinkContainer* find_link_container_by_request_arbitration_id(uint16_t request_arbitration_id);
IsoTpLinkContainer* find_link_container_by_reply_arbitration_id(uint16_t reply_arbitration_id);

#endif
