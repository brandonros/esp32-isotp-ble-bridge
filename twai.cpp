#include <Arduino.h>
#include "driver/twai.h"
#include "twai.h"
#include "isotp_link_containers.h"

std::mutex can_mtx;

uint8_t twai_rx_buf[CAN_FRAME_SIZE];

twai_message_t rx_message;
twai_message_t tx_message;

void twai_setup() {
  // transceiver silence workaround
  pinMode(GPIO_NUM_21, OUTPUT);
  digitalWrite(GPIO_NUM_21, LOW);

  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)TX_PIN, 
    (gpio_num_t)RX_PIN, 
    TWAI_MODE_NORMAL
  );
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  //Look in the api-reference for other speed sets.
  //twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_filter_config_t f_config = {
    .acceptance_code = (REPLY_ARBITRATION_ID << 21),
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
  can_mtx.lock();
  assert(size == CAN_FRAME_SIZE);
  tx_message.identifier = arbitration_id;
  tx_message.data_length_code = size;
  for (int i = 0; i < size; ++i) {
    tx_message.data[i] = buf[i];
  }
  int ret_val = twai_transmit(&tx_message, TWAI_SEND_MS > 0 ? pdMS_TO_TICKS(TWAI_SEND_MS) : 0);
  can_mtx.unlock();
  return ret_val;
}

int twai_recv(uint16_t *arbitration_id, uint8_t *buf, size_t *size)
{
  can_mtx.lock();
  int ret_val = twai_receive(&rx_message, TWAI_RECEIVE_MS > 0 ? pdMS_TO_TICKS(TWAI_RECEIVE_MS) : 0);
  if (ret_val == ESP_OK) {
    *arbitration_id = rx_message.identifier;
    *size = rx_message.data_length_code;
    assert(*size == CAN_FRAME_SIZE);
    memcpy(buf, rx_message.data, rx_message.data_length_code);
  }
  can_mtx.unlock();
  return ret_val;
}
