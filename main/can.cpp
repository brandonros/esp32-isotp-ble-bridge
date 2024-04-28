#include <mutex>
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "util.h"

static std::mutex can_mtx;
static twai_message_t tx_message;
static twai_message_t rx_message;

int can_init()
{
    twai_general_config_t g_config = {
      .mode = TWAI_MODE_NORMAL,
      .tx_io = GPIO_NUM_5,
      .rx_io = GPIO_NUM_4,
      .clkout_io = TWAI_IO_UNUSED,
      .bus_off_io = TWAI_IO_UNUSED,
      .tx_queue_len = 32, // setting this above 5 yields poor results?
      .rx_queue_len = 32, // setting this above 5 yields poor results?
      .alerts_enabled = TWAI_ALERT_RX_DATA | TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_BUS_OFF | TWAI_ALERT_TX_FAILED | TWAI_ALERT_RX_QUEUE_FULL,
      .clkout_divider = 0,
      .intr_flags = ESP_INTR_FLAG_LEVEL1
    };
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = { // TODO: how to accept only everything above like 0x400?
      .acceptance_code = 0, 
      .acceptance_mask = 0xFFFFFFFF, 
      .single_filter = true
    };
    int install_ret_val = twai_driver_install(&g_config, &t_config, &f_config);
    if (install_ret_val != ESP_OK) {
      return install_ret_val;
    }
    int start_ret_val = twai_start();
    if (start_ret_val != ESP_OK) {
      return start_ret_val;
    }
    return ESP_OK;
}

int can_send(uint16_t arbitration_id, const uint8_t *buf, size_t size)
{
  wait_for_lock(can_mtx);
  tx_message.identifier = arbitration_id;
  tx_message.flags = TWAI_MSG_FLAG_NONE;
  tx_message.data_length_code = size;
  memcpy(tx_message.data, buf, size);
  int ret_val = twai_transmit(&tx_message, pdMS_TO_TICKS(1));
  can_mtx.unlock();
  return ret_val;
}

int can_recv(uint16_t *arbitration_id, uint8_t *buf, size_t *size)
{
    wait_for_lock(can_mtx);
    int ret_val = twai_receive(&rx_message, pdMS_TO_TICKS(1));
    can_mtx.unlock();
    if (ret_val == ESP_OK) {
      *arbitration_id = rx_message.identifier;
      *size = rx_message.data_length_code;
      memcpy(buf, rx_message.data, rx_message.data_length_code);
    }
    return ret_val;
}

void can_reset()
{
  // lock
  wait_for_lock(can_mtx);
  // stop
  twai_stop();
  //twai_driver_uninstall();
  //periph_module_reset(PERIPH_TWAI_MODULE);
  // wait for it to get back to running?
  for (;;)
  {
    twai_status_info_t status_info;
    twai_get_status_info(&status_info);
    if (status_info.state == TWAI_STATE_BUS_OFF)
    {
      Serial.printf("TWAI_STATE_BUS_OFF\n");
      twai_initiate_recovery();
    }
    else if (status_info.state == TWAI_STATE_STOPPED)
    {
      Serial.printf("TWAI_STATE_STOPPED\n");
      //twai_driver_install();
      twai_start();
    }
    else if (status_info.state == TWAI_STATE_RUNNING)
    {
        break;
    }
    else if (status_info.state == TWAI_STATE_RECOVERING)
    {
      Serial.printf("TWAI_STATE_RECOVERING\n");
    }
    // sleep?
    delay(1);
  }
  // unlock
  can_mtx.unlock();
}
