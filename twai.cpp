#include <Arduino.h>
#include "driver/twai.h"
#include "twai.h"
#include "isotp_link_containers.h"

void twai_setup() {
  // transceiver silence workaround
  pinMode(GPIO_NUM_21, OUTPUT);
  digitalWrite(GPIO_NUM_21, LOW);

  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  //Look in the api-reference for other speed sets.
  //twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_filter_config_t f_config = {
    .acceptance_code = (0x7E8 << 21),
    .acceptance_mask = ~(0x7FF << 21),
    .single_filter = true
	};

  // Install TWAI driver
  assert(twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK);

  // Start TWAI driver
  assert(twai_start() == ESP_OK);
  
  // Reconfigure alerts to detect TX alerts and Bus-Off errors
  uint32_t alerts_to_enable = TWAI_ALERT_ABOVE_ERR_WARN |
    TWAI_ALERT_ARB_LOST |
    TWAI_ALERT_BELOW_ERR_WARN |
    TWAI_ALERT_BUS_ERROR |
    TWAI_ALERT_BUS_OFF |
    TWAI_ALERT_BUS_RECOVERED |
    TWAI_ALERT_ERR_ACTIVE |
    TWAI_ALERT_ERR_PASS |
    TWAI_ALERT_RECOVERY_IN_PROGRESS |
    TWAI_ALERT_RX_DATA |
    TWAI_ALERT_RX_QUEUE_FULL |
    TWAI_ALERT_TX_FAILED |
    TWAI_ALERT_TX_IDLE |
    TWAI_ALERT_TX_SUCCESS;
  assert(twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK);
}

int twai_send(uint16_t arbitration_id, const uint8_t *buf, size_t size) {
  assert(size == 8);
  tx_message.identifier = arbitration_id;
  tx_message.data_length_code = size;
  for (int i = 0; i < size; ++i) {
    tx_message.data[i] = buf[i];
  }
  return twai_transmit(&tx_message, pdMS_TO_TICKS(10));
}

int twai_recv(uint16_t *arbitration_id, uint8_t *buf, size_t *size)
{
  int ret_val = twai_receive(&rx_message, pdMS_TO_TICKS(10));
  if (ret_val == ESP_OK) {
    *arbitration_id = rx_message.identifier;
    *size = rx_message.data_length_code;
    assert(*size == 8);
    memcpy(buf, rx_message.data, rx_message.data_length_code);
  }
  return ret_val;
}

void twai_rx_task_callback() {
  uint16_t arbitration_id = 0;
  size_t size = 0;
  int ret_val = twai_recv(&arbitration_id, twai_rx_buf, &size);
  if (ret_val == ESP_OK) {
    if (arbitration_id == 0x7E8) {
      //Serial.printf("Received CAN message: identifier = %08x data_length_code = %02x data=%02x%02x%02x%02x%02x%02x%02x%02x\n", arbitration_id, size, twai_rx_buf[0], twai_rx_buf[1], twai_rx_buf[2], twai_rx_buf[3], twai_rx_buf[4], twai_rx_buf[5], twai_rx_buf[6], twai_rx_buf[7]);
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
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(10));
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
    twai_initiate_recovery();
  }
  if (alerts_triggered & TWAI_ALERT_BUS_OFF) {
    Serial.printf("TWAI_ALERT_BUS_OFF\n");
    twai_initiate_recovery();
  }
  if (alerts_triggered & TWAI_ALERT_BUS_RECOVERED) {
    Serial.printf("TWAI_ALERT_BUS_RECOVERED\n");
    twai_clear_transmit_queue();
    twai_clear_receive_queue();
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
    //Serial.printf("TWAI_ALERT_RX_DATA\n");
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
    //Serial.printf("TWAI_ALERT_TX_IDLE\n");
  }
  if (alerts_triggered & TWAI_ALERT_TX_SUCCESS) {
    //Serial.printf("TWAI_ALERT_TX_SUCCESS\n");
    //Serial.printf("TX buffered: %d\n", twaistatus.msgs_to_tx);
  }
}
