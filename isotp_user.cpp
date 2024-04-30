#include <Arduino.h>
#include <stdint.h>
#include <isotp.h>
#include "twai.h"

void isotp_user_debug(const char* format, ...) {
  // TODO: no Serial.vprintf
}

int isotp_user_send_can(uint32_t arbitration_id, const uint8_t* data, uint8_t size) {
  /*Serial.printf("isotp_user_send_can ");
  for (int i = 0; i < size; ++i) {
    Serial.printf("%02x", data[i]);
  }
  Serial.printf("\n");*/
  int ret_val = twai_send(arbitration_id, data, size);
  if (ret_val != ESP_OK) {
    Serial.printf("isotp_user_send_can: can_send ret_val = %08x\n", ret_val);
    // we need to stop -> start the TWAI driver or else we'll have 0 success flashing after a timeout has occurred
    //can_reset();
    return ISOTP_RET_ERROR;
  }
  return ISOTP_RET_OK;
}

uint32_t isotp_user_get_us() {
  return micros();
}
