#include <TaskScheduler.h>
#include "protocol.h"
#include "ble.h"
#include "twai.h"
#include "periodic_messages.h"
#include "isotp_link_containers.h"
#include "math.h"
#include "isotp.h"
#include "ble_isotp.h"

// ISOTP
#define ISO_TP_DEFAULT_ST_MIN_US 0 // TODO: higher
#define ISO_TP_DEFAULT_RESPONSE_TIMEOUT_US 100000
#define ISO_TP_DEFAULT_BLOCK_SIZE 8 // TODO: larger
#define ISO_TP_FRAME_PADDING
#define ISO_TP_FRAME_PADDING_VALUE 0xAA

// scheduler + tasks
Scheduler ts;
Task twai_alerts_task(TASK_IMMEDIATE, TASK_FOREVER, &twai_alerts_task_callback, &ts, true);
Task twai_rx_task(TASK_IMMEDIATE, TASK_FOREVER, &twai_rx_task_callback, &ts, true);

Task isotp_poll_task(TASK_IMMEDIATE, TASK_FOREVER, &isotp_poll_task_callback, &ts, true);
Task isotp_receive_task(TASK_IMMEDIATE, TASK_FOREVER, &isotp_receive_task_callback, &ts, true);

Task periodic_messages_task(TASK_IMMEDIATE, TASK_FOREVER, &periodic_messages_task_callback, &ts, true);

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
