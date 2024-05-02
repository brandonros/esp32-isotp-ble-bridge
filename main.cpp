#include <Arduino.h>
#include <TaskScheduler.h>
#include "protocol.h"
#include "ble.h"
#include "twai.h"
#include "periodic_messages.h"
#include "isotp_link_containers.h"
#include "utilities.h"
#include "isotp.h"
#include "ble_isotp.h"
#include "tasks.h"

#define DEBUG

// ISOTP
#define ISO_TP_DEFAULT_ST_MIN_US 0 // TODO: higher
#define ISO_TP_DEFAULT_RESPONSE_TIMEOUT_US 100000
#define ISO_TP_DEFAULT_BLOCK_SIZE 8 // TODO: larger
#define ISO_TP_FRAME_PADDING
#define ISO_TP_FRAME_PADDING_VALUE 0xAA

void setup() {
  Serial.begin(115200);
  twai_setup();
  isotp_setup();
  ble_setup();
  periodic_setup();
}

void loop() {
  ts.execute();
}
