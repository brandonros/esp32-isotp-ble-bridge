#include <stdint.h>
#include "isotp_link_containers.h"

IsoTpLinkContainer* find_link_container_by_request_arbitration_id(uint16_t request_arbitration_id) {
  for (int i = 0; i < 4; ++i) {
    if (link_containers[i].initialized == false) {
      continue;
    }
    if (link_containers[i].request_arbitration_id == request_arbitration_id) {
      return &link_containers[i];
    }
  }
  return NULL;
}

IsoTpLinkContainer* find_link_container_by_reply_arbitration_id(uint16_t reply_arbitration_id) {
  for (int i = 0; i < 4; ++i) {
    if (link_containers[i].initialized == false) {
      continue;
    }
    if (link_containers[i].reply_arbitration_id == reply_arbitration_id) {
      return &link_containers[i];
    }
  }
  return NULL;
}