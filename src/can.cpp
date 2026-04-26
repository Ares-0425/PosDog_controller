#include "can.hpp"

twai_message_t rx_message;
twai_message_t tx_message;

void CAN_setup() {
  // CAN初期化設定
  //twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CTX, CRX, TWAI_MODE_NORMAL);
  twai_general_config_t g_config = {.mode = TWAI_MODE_NORMAL,
                                   .tx_io = CTX,
                                   .rx_io = CRX,
                                   .clkout_io = TWAI_IO_UNUSED,
                                   .bus_off_io = TWAI_IO_UNUSED,
                                   .tx_queue_len = 20,
                                   .rx_queue_len = 20,
                                   .alerts_enabled = TWAI_ALERT_NONE,
                                   .clkout_divider = 0,
                                   .intr_flags = 0};
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();  // 1Mbps
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("CAN driver install failed");
    return;
  }

  if (twai_start() != ESP_OK) {
    Serial.println("CAN start failed");
    return;
  }

  Serial.println("CAN Started");
}