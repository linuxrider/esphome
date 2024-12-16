
#include "esphome/core/defines.h"
#if defined(USE_ESP32_VARIANT_ESP32S2) || (USE_ESP32_VARIANT_ESP32S3)
#include <stdio.h>
#include <string.h>

#include "usb_vcp.h"
#include "esphome/core/log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "usb/cdc_acm_host.h"
#include "usb/vcp_ch34x.hpp"
#include "usb/vcp_cp210x.hpp"
#include "usb/vcp_ftdi.hpp"
#include "usb/vcp.hpp"
#include "usb/usb_host.h"
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

using namespace esp_usb;


namespace esphome {
namespace usb_vcp {

static const char *const TAG = "usb_vcp";

bool usb_configured = false;


static SemaphoreHandle_t device_disconnected_sem;

static bool handle_rx(const uint8_t *data, size_t data_len, void *arg)
{
  // ESP_LOGI(TAG, "callback");
  auto *self = reinterpret_cast<UsbVcp *>(arg);
  self->concat_received_bytes(data, data_len);

  ESP_LOGI(TAG, "%.*s", data_len, data);
    // printf("%.*s", data_len, data);
    return true;
}

static void handle_event(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    switch (event->type) {
    case CDC_ACM_HOST_ERROR:
        ESP_LOGE(TAG, "CDC-ACM error has occurred, err_no = %d", event->data.error);
        break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGI(TAG, "Device suddenly disconnected");
        xSemaphoreGive(device_disconnected_sem);
        usb_configured = false;
        break;
    case CDC_ACM_HOST_SERIAL_STATE:
        ESP_LOGI(TAG, "Serial state notif 0x%04X", event->data.serial_state.val);
        break;
    case CDC_ACM_HOST_NETWORK_CONNECTION:
    default: break;
    }
}

static void usb_lib_task(void *arg)
{
    while (1) {
        // Start handling system events
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_ERROR_CHECK(usb_host_device_free_all());
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "USB: All devices freed");
            // Continue handling USB events to allow device reconnection
        }
    }
}


void UsbVcp::loop() {
    if (usb_configured == false) {


    ESP_LOGI(TAG, "Opening any VCP device...");
    const cdc_acm_host_device_config_t dev_config = {
        .connection_timeout_ms = 500,
        .out_buffer_size = this->tx_buffer_size_,
        .in_buffer_size = this->rx_buffer_size_,
        .event_cb = handle_event,
        .data_cb = handle_rx,
        .user_arg = this,
    };
 
    CdcAcmDevice *vcp = VCP::open(&dev_config);
    if (vcp == nullptr) {
        ESP_LOGI(TAG, "Failed to open VCP device");
        return;
    }
    vTaskDelay(10);
    ESP_LOGI(TAG, "Setting up line coding");
    cdc_acm_line_coding_t line_coding = {
        .dwDTERate = baud_rate_,
        .bCharFormat = usb_vcp::VCPStopBitsOptions::VCP_CONFIG_STOP_BIT_ONE,
        .bParityType = usb_vcp::VCPParityOptions::VCP_CONFIG_PARITY_NONE,
        .bDataBits = data_bits_,
    };
    
    ESP_ERROR_CHECK(vcp->line_coding_set(&line_coding));

    ESP_ERROR_CHECK(vcp->set_control_line_state(false, true));
    this->vcp = vcp;
    usb_configured = true;
    }
    
    // std::array[this->rx_buffer_size_];

}

int UsbVcp::available() { 
    return 0;
}

void UsbVcp::dump_config() { 
    ESP_LOGCONFIG(TAG, "USB VCP:");
    // ESP_LOGCONFIG(TAG, "  VID: 0x%04x", this->vcp->vid_);
    ESP_LOGCONFIG(TAG, "  Baud Rate: %" PRIu32, this->baud_rate_);
    ESP_LOGCONFIG(TAG, "  Parity: %d", VCPParityOptions(this->parity_));
    ESP_LOGCONFIG(TAG, "  Stop Bits: %d", this->stop_bits_);
    ESP_LOGCONFIG(TAG, "  Data Bits: %d", this->data_bits_);
}
void UsbVcp::setup() {

    device_disconnected_sem = xSemaphoreCreateBinary();
    assert(device_disconnected_sem);

    // // Install USB Host driver. Should only be called once in entire application
    ESP_LOGI(TAG, "Installing USB Host");
    usb_host_config_t host_config = {};
    host_config.skip_phy_setup = false;
    host_config.intr_flags = ESP_INTR_FLAG_LEVEL1;
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    ESP_LOGI(TAG, "Registered USB callback...");

    // // // Create a task that will handle USB library events
    BaseType_t task_created = xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 10, NULL);
    assert(task_created == pdTRUE);

    ESP_LOGI(TAG, "Installing CDC-ACM driver");
    ESP_ERROR_CHECK(cdc_acm_host_install(NULL));

    // Register VCP drivers to VCP service
    VCP::register_driver<CH34x>();
    VCP::register_driver<CP210x>();
    VCP::register_driver<FT23x>();
    this->rx_buffer_ = new uint8_t[rx_buffer_size_];

}

// bool UsbVcp::read_array(uint8_t *data, size_t len) {
//   size_t length_to_read = len;
//   if (!this->check_read_timeout_(len))
//     return false;
//   xSemaphoreTake(this->lock_, portMAX_DELAY);
//   if (this->has_peek_) {
//     length_to_read--;
//     *data = this->peek_byte_;
//     data++;
//     this->has_peek_ = false;
//   }
//   if (length_to_read > 0)
//     uart_read_bytes(this->uart_num_, data, length_to_read, 20 / portTICK_PERIOD_MS);
//   xSemaphoreGive(this->lock_);
// #ifdef USE_UART_DEBUGGER
//   for (size_t i = 0; i < len; i++) {
//     this->debug_callback_.call(UART_DIRECTION_RX, data[i]);
//   }
// #endif
//   return true;
// }
void UsbVcp::concat_received_bytes(const uint8_t *data, size_t data_len) {
  int last_position = this->rx_buffer_data_len_;
  for (size_t i = 0; i < data_len; i++){
    this->rx_buffer_[last_position+i] = data[i];
  }
  ESP_LOGI(TAG, "before");
  this->rx_buffer_data_len_ = last_position+data_len;
  ESP_LOGI(TAG, "%.*s", this->rx_buffer_data_len_, this->rx_buffer_);
  ESP_LOGI(TAG, "%d\n", this->ends_with(this->rx_buffer_, this->rx_buffer_data_len_, "_\r\n\r\n"));
  if (this->ends_with(this->rx_buffer_, this->rx_buffer_data_len_, "_\r\n\r\n")) {
    ESP_LOGI(TAG, "match");
    this->receive_message(rx_buffer_, rx_buffer_data_len_);
    this->rx_buffer_data_len_ = 0;
  }
  ESP_LOGI(TAG, "after");
};


bool UsbVcp::ends_with(const uint8_t *buffer, size_t buffer_len, const char *suffix) {
    // Länge der Suffix-Zeichenkette berechnen
    size_t suffix_len = strlen(suffix);

    // Wenn der Puffer kürzer als das Suffix ist, kann es nicht übereinstimmen
    if (buffer_len < suffix_len) {
        return false;
    }

    // Pointer auf die Position im Puffer setzen, an der das Suffix beginnen sollte
    const uint8_t *buffer_suffix_start = buffer + (buffer_len - suffix_len);

    // Überprüfen, ob der relevante Teil des Puffers mit dem Suffix übereinstimmt
    return memcmp(buffer_suffix_start, suffix, suffix_len) == 0;
}

void UsbVcp::receive_message(const uint8_t *data, size_t data_len) {
  // ESP_LOGI(TAG, "%.*s", data_len, data);
    // uint8_t buffer[this->rx_buffer_size_];
    // if (int bytes_to_read = available(); bytes_to_read > 0) {
      // read_array(buffer, data_len);
      // taken from UARTDebug::log_string
      std::string res;

      char buf[5];
      for (size_t i = 0; i < data_len; i++) {
        if (data[i] == 7) {
          res += "\\a";
        } else if (data[i] == 8) {
          res += "\\b";
        } else if (data[i] == 9) {
          res += "\\t";
        } else if (data[i] == 10) {
          res += "\\n";
        } else if (data[i] == 11) {
          res += "\\v";
        } else if (data[i] == 12) {
          res += "\\f";
        } else if (data[i] == 13) {
          res += "\\r";
        } else if (data[i] == 27) {
          res += "\\e";
        } else if (data[i] == 34) {
          res += "\\\"";
        } else if (data[i] == 39) {
          res += "\\'";
        } else if (data[i] == 92) {
          res += "\\\\";
        } else if (data[i] < 32 || data[i] > 127) {
          sprintf(buf, "\\x%02X", data[i]);
          res += buf;
        } else {
          res += data[i];
        }
      }
      received_->publish_state(res);
    // }
};

void UsbVcp::set_received_text_sensor(text_sensor::TextSensor *sensor) { received_ = sensor; };
bool UsbVcp::get_configured_() {

  return usb_configured;


}
}  // namespace usb_vcp
}  // namespace esphome
#endif
