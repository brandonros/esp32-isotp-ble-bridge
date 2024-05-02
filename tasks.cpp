#include <Arduino.h>
#include "driver/twai.h"
#include "periodic_messages.h"
#include "isotp_link_containers.h"
#include "tasks.h"
#include "twai.h"
#include "ble_isotp.h"

Scheduler ts;
Task twai_alerts_task(TASK_IMMEDIATE, TASK_FOREVER, &twai_alerts_task_callback, &ts, true);
Task twai_rx_task(TASK_IMMEDIATE, TASK_FOREVER, &twai_rx_task_callback, &ts, true);
Task isotp_poll_task(TASK_IMMEDIATE, TASK_FOREVER, &isotp_poll_task_callback, &ts, true);
Task isotp_receive_task(TASK_IMMEDIATE, TASK_FOREVER, &isotp_receive_task_callback, &ts, true);
Task periodic_messages_task(TASK_IMMEDIATE, TASK_FOREVER, &periodic_messages_task_callback, &ts, true);

void periodic_messages_task_callback() {
  for (int i = 0; i < NUM_PERIODIC_MESSAGE_CONTAINERS; ++i) {
    PeriodicMessageContainer periodic_message_container = periodic_message_containers[i];
    if (periodic_message_container.active == false) {
      continue;
    }
    unsigned long start_us = micros();
    tx_isotp_on_ble_rx(periodic_message_container.request_arbitration_id, periodic_message_container.reply_arbitration_id, periodic_message_container.msg, periodic_message_container.msg_length, &periodic_messages_task);
    unsigned long end_us = micros();
    unsigned long time_spent_ms = (end_us - start_us) / 1000;
#ifdef DEBUG
    Serial.printf("periodic_messages_task_callback: tx_isotp_on_ble_rx time_spent_ms = %lu\n", time_spent_ms);
#endif
    periodic_messages_task.delay(periodic_message_container.interval_ms - time_spent_ms);
  }
}

void isotp_poll_task_callback() {
  for (int i = 0; i < NUM_LINK_CONTAINERS; ++i) {
    // skip uninitialized links
    if (link_containers[i].initialized == false) {
      continue;
    }
    // poll
    unsigned long start_us = micros();
    isotp_poll(&link_containers[i].isotp_link);
    unsigned long end_us = micros();
    unsigned long time_spent_ms = (end_us - start_us) / 1000;
#ifdef DEBUG
    Serial.printf("isotp_poll_task_callback: isotp_poll time_spent_ms = %lu\n", time_spent_ms);
#endif
  }
}

void isotp_receive_task_callback() {
  for (int i = 0; i < NUM_LINK_CONTAINERS; ++i) {
    // skip uninitialized links
    if (link_containers[i].initialized == false) {
      continue;
    }
    // receive
    unsigned long start_us = micros();
    uint16_t out_size = 0;
    int isotp_receive_ret_val = isotp_receive(&link_containers[i].isotp_link, isotp_payload_buffer, ISOTP_BUFSIZE, &out_size);
    unsigned long end_us = micros();
    unsigned long time_spent_ms = (end_us - start_us) / 1000;
#ifdef DEBUG
    Serial.printf("isotp_receive_task_callback: isotp_receive time_spent_ms = %lu\n", time_spent_ms);
#endif        
    if (isotp_receive_ret_val == ISOTP_RET_OK) {
      unsigned long start_us = micros();
      tx_ble_on_isotp_rx(link_containers[i].request_arbitration_id, link_containers[i].reply_arbitration_id, isotp_payload_buffer, out_size, &isotp_receive_task);
      unsigned long end_us = micros();
      unsigned long time_spent_ms = (end_us - start_us) / 1000;
#ifdef DEBUG
      Serial.printf("isotp_receive_task_callback: tx_ble_on_isotp_rx time_spent_ms = %lu\n", time_spent_ms);
#endif         
    } else if (isotp_receive_ret_val == ISOTP_RET_NO_DATA) {
      // expected timeout trying to read (no data available?)
    } else {
      Serial.printf("isotp_receive_ret_val = %08x\n", isotp_receive_ret_val);
    }
  }
}

void twai_rx_task_callback() {
  uint16_t arbitration_id = 0;
  size_t size = 0;
  int ret_val = twai_recv(&arbitration_id, twai_rx_buf, &size);
  if (ret_val == ESP_OK) {
    if (arbitration_id == REPLY_ARBITRATION_ID) {
#ifdef DEBUG      
      Serial.printf("Received CAN message: identifier = %08x data_length_code = %02x data=%02x%02x%02x%02x%02x%02x%02x%02x\n", arbitration_id, size, twai_rx_buf[0], twai_rx_buf[1], twai_rx_buf[2], twai_rx_buf[3], twai_rx_buf[4], twai_rx_buf[5], twai_rx_buf[6], twai_rx_buf[7]);
#endif      
      IsoTpLinkContainer *link_container = find_link_container_by_reply_arbitration_id(arbitration_id);
      assert(link_container != NULL);
      isotp_on_can_message(&link_container->isotp_link, twai_rx_buf, size);
    } else {
      Serial.printf("dropping RX'd CAN message identifier = %08x\n", arbitration_id);
    }
  } else if (ret_val == ESP_ERR_TIMEOUT) {
    // expected?
  } else {
    Serial.printf("Failed to receive message ret_val = %08x\n", ret_val);
  }
}

void twai_alerts_task_callback() {
  // Check if alert happened
  uint32_t alerts_triggered;
  int ret_val = twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(TWAI_READ_ALERTS_MS));
  if (ret_val == ESP_ERR_TIMEOUT) {
    return;
  }

  twai_status_info_t twaistatus;
  twai_get_status_info(&twaistatus);

  // Handle alerts
  if (alerts_triggered & TWAI_ALERT_ABOVE_ERR_WARN) {
    Serial.printf("TWAI_ALERT_ABOVE_ERR_WARN\n");
  }
  if (alerts_triggered & TWAI_ALERT_ARB_LOST) {
    Serial.printf("TWAI_ALERT_ARB_LOST\n");
  }
  if (alerts_triggered & TWAI_ALERT_BELOW_ERR_WARN) {
    Serial.printf("TWAI_ALERT_BELOW_ERR_WARN\n");
  }
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
    Serial.printf("TWAI_ALERT_BUS_ERROR\n");
    Serial.printf("Bus error count: %d\n", twaistatus.bus_error_count);
    // TODO: not sure what to do here
    /*if (twaistatus.bus_error_count > 256) {
      twai_clear_transmit_queue(); // TODO: not sure if this is right
      twai_clear_receive_queue(); // TODO: not sure if this is right
      twai_driver_uninstall();
      twai_stop();
      twai_initiate_recovery();
    }*/
  }
  if (alerts_triggered & TWAI_ALERT_BUS_OFF) {
    Serial.printf("TWAI_ALERT_BUS_OFF\n");
    // TODO: twai_driver_uninstall()?
    // TODO: twai_stop()?
    twai_initiate_recovery(); // TODO: not sure if this is right
  }
  if (alerts_triggered & TWAI_ALERT_BUS_RECOVERED) {
    Serial.printf("TWAI_ALERT_BUS_RECOVERED\n");
    // TODO: twai_driver_install()?;
    // TODO: twai_start()?
    twai_clear_transmit_queue(); // TODO: not sure if this is right
    twai_clear_receive_queue(); // TODO: not sure if this is right
  }
  if (alerts_triggered & TWAI_ALERT_ERR_ACTIVE) {
    Serial.printf("TWAI_ALERT_ERR_ACTIVE\n");
  }
  if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
    Serial.printf("TWAI_ALERT_ERR_PASS\n");
  }
  if (alerts_triggered & TWAI_ALERT_RECOVERY_IN_PROGRESS) {
    Serial.printf("TWAI_ALERT_RECOVERY_IN_PROGRESS\n");
  }
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
#ifdef DEBUG
    Serial.printf("TWAI_ALERT_RX_DATA\n");
#endif
  }
  if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
    Serial.printf("TWAI_ALERT_RX_QUEUE_FULL\n");
  }
  if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
    Serial.printf("TWAI_ALERT_TX_FAILED\n");
    Serial.printf("TX buffered: %d\n", twaistatus.msgs_to_tx);
    Serial.printf("TX error: %d\n", twaistatus.tx_error_counter);
    Serial.printf("TX failed: %d\n", twaistatus.tx_failed_count);
  }
  if (alerts_triggered & TWAI_ALERT_TX_IDLE) {
#ifdef DEBUG    
    Serial.printf("TWAI_ALERT_TX_IDLE\n");
#endif
  }
  if (alerts_triggered & TWAI_ALERT_TX_SUCCESS) {
#ifdef DEBUG    
    Serial.printf("TWAI_ALERT_TX_SUCCESS\n");
    Serial.printf("TX buffered: %d\n", twaistatus.msgs_to_tx);
#endif    
  }
}
