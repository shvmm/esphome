#ifdef USE_ESP32
#include "esp32_can.h"
#include "esphome/core/log.h"

#include <esp_twai.h>

namespace esphome::esp32_can {

static const char *const TAG = "esp32_can";

static bool get_bitrate(canbus::CanSpeed bitrate, twai_timing_basic_config_t *t_config) {
  switch (bitrate) {
#if defined(USE_ESP32_VARIANT_ESP32C3) || \
    defined(USE_ESP32_VARIANT_ESP32C6) || \
    defined(USE_ESP32_VARIANT_ESP32C61) || \
    defined(USE_ESP32_VARIANT_ESP32H2) || \
    defined(USE_ESP32_VARIANT_ESP32P4) || \
    defined(USE_ESP32_VARIANT_ESP32S2) || \
    defined(USE_ESP32_VARIANT_ESP32S3)
    case canbus::CAN_1KBPS:
      *t_config = TWAI_TIMING_CONFIG_1KBITS();
      return true;
    case canbus::CAN_5KBPS:
      *t_config = TWAI_TIMING_CONFIG_5KBITS();
      return true;
    case canbus::CAN_10KBPS:
      *t_config = TWAI_TIMING_CONFIG_10KBITS();
      return true;
    case canbus::CAN_12K5BPS:
      *t_config = TWAI_TIMING_CONFIG_12_5KBITS();
      return true;
    case canbus::CAN_16KBPS:
      *t_config = TWAI_TIMING_CONFIG_16KBITS();
      return true;
    case canbus::CAN_20KBPS:
      *t_config = TWAI_TIMING_CONFIG_20KBITS();
      return true;
#endif
    case canbus::CAN_25KBPS:
      *t_config = TWAI_TIMING_CONFIG_25KBITS();
      return true;
    case canbus::CAN_50KBPS:
      *t_config = TWAI_TIMING_CONFIG_50KBITS();
      return true;
    case canbus::CAN_100KBPS:
      *t_config = TWAI_TIMING_CONFIG_100KBITS();
      return true;
    case canbus::CAN_125KBPS:
      *t_config = TWAI_TIMING_CONFIG_125KBITS();
      return true;
    case canbus::CAN_250KBPS:
      *t_config = TWAI_TIMING_CONFIG_250KBITS();
      return true;
    case canbus::CAN_500KBPS:
      *t_config = TWAI_TIMING_CONFIG_500KBITS();
      return true;
    case canbus::CAN_800KBPS:
      *t_config = TWAI_TIMING_CONFIG_800KBITS();
      return true;
    case canbus::CAN_1000KBPS:
      *t_config = TWAI_TIMING_CONFIG_1MBITS();
      return true;
    default:
      return false;
  }
}

bool ESP32Can::setup_internal() {
  // Select TWAI mode based on configuration
  twai_mode_t twai_mode = (this->mode_ == CAN_MODE_LISTEN_ONLY) ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL;

  if (this->mode_ == CAN_MODE_LISTEN_ONLY) {
    ESP_LOGI(TAG, "CAN bus configured in LISTEN_ONLY mode (passive, no ACKs)");
  }

  // Create general configuration
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
      static_cast<gpio_num_t>(this->tx_),
      static_cast<gpio_num_t>(this->rx_),
      twai_mode
  );

  // Apply optional queue length settings
  if (this->tx_queue_len_.has_value()) {
    g_config.tx_queue_len = this->tx_queue_len_.value();
  }
  if (this->rx_queue_len_.has_value()) {
    g_config.rx_queue_len = this->rx_queue_len_.value();
  }

  // Create filter and timing configurations
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_timing_config_t t_config;

  if (!get_bitrate(this->bit_rate_, &t_config)) {
    ESP_LOGE(TAG, "Invalid bit rate configuration");
    this->mark_failed();
    return false;
  }

  // Install TWAI driver using modern API
  esp_err_t install_err = twai_driver_install(&g_config, &t_config, &f_config, &(this->twai_handle_));
  if (install_err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to install TWAI driver: %s", esp_err_to_name(install_err));
    this->mark_failed();
    return false;
  }

  // Start TWAI driver
  esp_err_t start_err = twai_start(this->twai_handle_);
  if (start_err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start TWAI driver: %s", esp_err_to_name(start_err));
    twai_driver_uninstall(this->twai_handle_);
    this->twai_handle_ = nullptr;
    this->mark_failed();
    return false;
  }

  ESP_LOGI(TAG, "TWAI driver initialized successfully");
  return true;
}

canbus::Error ESP32Can::send_message(struct canbus::CanFrame *frame) {
  // In listen-only mode, we cannot transmit
  if (this->mode_ == CAN_MODE_LISTEN_ONLY) {
    ESP_LOGW(TAG, "Cannot send messages in LISTEN_ONLY mode");
    return canbus::ERROR_FAIL;
  }

  if (this->twai_handle_ == nullptr) {
    ESP_LOGW(TAG, "TWAI driver not initialized");
    return canbus::ERROR_FAIL;
  }

  if (frame->can_data_length_code > canbus::CAN_MAX_DATA_LENGTH) {
    ESP_LOGW(TAG, "CAN frame data length %d exceeds maximum %d", 
             frame->can_data_length_code, canbus::CAN_MAX_DATA_LENGTH);
    return canbus::ERROR_FAILTX;
  }

  uint32_t flags = TWAI_MSG_FLAG_NONE;
  if (frame->use_extended_id) {
    flags |= TWAI_MSG_FLAG_EXTD;
  }
  if (frame->remote_transmission_request) {
    flags |= TWAI_MSG_FLAG_RTR;
  }

  twai_message_t message = {
      .flags = flags,
      .identifier = frame->can_id,
      .data_length_code = frame->can_data_length_code,
      .data = {},
  };
  if (!frame->remote_transmission_request) {
    memcpy(message.data, frame->data, frame->can_data_length_code);
  }

  esp_err_t tx_err = twai_transmit(this->twai_handle_, &message, pdMS_TO_TICKS(this->tx_enqueue_timeout_ms_));
  if (tx_err == ESP_OK) {
    return canbus::ERROR_OK;
  } else if (tx_err == ESP_ERR_TIMEOUT) {
    ESP_LOGW(TAG, "TWAI TX queue full, timeout waiting");
    return canbus::ERROR_ALLTXBUSY;
  } else {
    ESP_LOGE(TAG, "TWAI transmit failed: %s", esp_err_to_name(tx_err));
    return canbus::ERROR_FAIL;
  }
}

canbus::Error ESP32Can::read_message(struct canbus::CanFrame *frame) {
  if (this->twai_handle_ == nullptr) {
    return canbus::ERROR_FAIL;
  }

  twai_message_t message;

  esp_err_t rx_err = twai_receive(this->twai_handle_, &message, 0);
  if (rx_err != ESP_OK) {
    if (rx_err != ESP_ERR_TIMEOUT) {
      ESP_LOGW(TAG, "TWAI receive error: %s", esp_err_to_name(rx_err));
    }
    return canbus::ERROR_NOMSG;
  }

  frame->can_id = message.identifier;
  frame->use_extended_id = message.flags & TWAI_MSG_FLAG_EXTD;
  frame->remote_transmission_request = message.flags & TWAI_MSG_FLAG_RTR;
  frame->can_data_length_code = message.data_length_code;

  if (!frame->remote_transmission_request) {
    size_t dlc = (message.data_length_code < canbus::CAN_MAX_DATA_LENGTH) 
                 ? message.data_length_code 
                 : canbus::CAN_MAX_DATA_LENGTH;
    memcpy(frame->data, message.data, dlc);
  }

  return canbus::ERROR_OK;
}

}  // namespace esphome::esp32_can

#endif
