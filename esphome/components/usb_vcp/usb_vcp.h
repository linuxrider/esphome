#pragma once

#include "esphome/core/defines.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include <freertos/FreeRTOS.h>
#include "usb/cdc_acm_host.h"
#include "usb/vcp.hpp"
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

namespace esphome {
namespace usb_vcp {

enum VCPStopBitsOptions {
  VCP_CONFIG_STOP_BIT_ONE = 0,
  VCP_CONFIG_STOP_BIT_ONE_FIVE = 1,
  VCP_CONFIG_STOP_BIT_TWO = 2,
};

enum VCPParityOptions {
  VCP_CONFIG_PARITY_NONE,
  VCP_CONFIG_PARITY_ODD,
  VCP_CONFIG_PARITY_EVEN,
  VCP_CONFIG_PARITY_MARK,
  VCP_CONFIG_PARITY_SPACE,

};

enum VCPHandshakeOptions {
  VCP_CONFIG_HANDSHAKE_NONE,
  VCP_CONFIG_HANDSHAKE_RTSCTS,
  VCP_CONFIG_HANDSHAKE_DTRDSR,
};

class UsbVcp : public Component {
 public:
   // Writes an array of bytes to the UART bus.
  // @param data A vector of bytes to be written.
  void write_array(const std::vector<uint8_t> &data) { this->write_array(&data[0], data.size()); }

  // Writes a single byte to the UART bus.
  // @param data The byte to be written.
  void write_byte(uint8_t data) { this->write_array(&data, 1); };

  // Writes a null-terminated string to the UART bus.
  // @param str Pointer to the null-terminated string.
  void write_str(const char *str) {
    const auto *data = reinterpret_cast<const uint8_t *>(str);
    this->write_array(data, strlen(str));
  };

  // Pure virtual method to write an array of bytes to the UART bus.
  // @param data Pointer to the array of bytes.
  // @param len Length of the array.
  void write_array(const uint8_t *data, size_t len);

  // Reads a single byte from the UART bus.
  // @param data Pointer to the byte where the read data will be stored.
  // @return True if a byte was successfully read, false otherwise.
  bool read_byte(uint8_t *data) { return this->read_array(data, 1); };

  // Pure virtual method to peek the next byte in the UART buffer without removing it.
  // @param data Pointer to the byte where the peeked data will be stored.
  // @return True if a byte is available to peek, false otherwise.
  bool peek_byte(uint8_t *data);

  // Pure virtual method to read an array of bytes from the UART bus.
  // @param data Pointer to the array where the read data will be stored.
  // @param len Number of bytes to read.
  // @return True if the specified number of bytes were successfully read, false otherwise.
  bool read_array(uint8_t *data, size_t len);
  // bool read_array(uint8_t *data, size_t len) { return this->read_array(data, len); }
  //   bool read_byte(uint8_t *data) { return this->read_byte(data); }
  // bool peek_byte(uint8_t *data) { return this->peek_byte(data); }
  template<size_t N> optional<std::array<uint8_t, N>> read_array() {  // NOLINT
    std::array<uint8_t, N> res;
    if (!this->read_array(res.data(), N)) {
      return {};
    }
    return res;
  }
  // Pure virtual method to return the number of bytes available for reading.
  // @return Number of available bytes.
  int available();

  // Pure virtual method to block until all bytes have been written to the UART bus.
  void flush();

  // Sets the TX (transmit) pin for the UART bus.
  // @param tx_pin Pointer to the internal GPIO pin used for transmission.
  void set_tx_pin(InternalGPIOPin *tx_pin) { this->tx_pin_ = tx_pin; }

  // Sets the RX (receive) pin for the UART bus.
  // @param rx_pin Pointer to the internal GPIO pin used for reception.
  void set_rx_pin(InternalGPIOPin *rx_pin) { this->rx_pin_ = rx_pin; }

  // Sets the size of the RX buffer.
  // @param rx_buffer_size Size of the RX buffer in bytes.
  void set_rx_buffer_size(size_t rx_buffer_size) { this->rx_buffer_size_ = rx_buffer_size; }

  // Gets the size of the RX buffer.
  // @return Size of the RX buffer in bytes.
  size_t get_rx_buffer_size() { return this->rx_buffer_size_; }

  // Sets the number of stop bits used in UART communication.
  // @param stop_bits Number of stop bits.
  void set_stop_bits(uint8_t stop_bits) { this->stop_bits_ = stop_bits; }

  // Gets the number of stop bits used in UART communication.
  // @return Number of stop bits.
  uint8_t get_stop_bits() const { return this->stop_bits_; }

  // Set the number of data bits used in UART communication.
  // @param data_bits Number of data bits.
  void set_data_bits(uint8_t data_bits) { this->data_bits_ = data_bits; }

  // Get the number of data bits used in UART communication.
  // @return Number of data bits.
  uint8_t get_data_bits() const { return this->data_bits_; }

  // Set the parity used in UART communication.
  // @param parity Parity option.
  void set_parity(VCPParityOptions parity) { this->parity_ = parity; }

  // Get the parity used in UART communication.
  // @return Parity option.
  VCPParityOptions get_parity() const { return this->parity_; }

  // Set the baud rate for UART communication.
  // @param baud_rate Baud rate in bits per second.
  void set_baud_rate(uint32_t baud_rate) { baud_rate_ = baud_rate; }

  // Get the baud rate for UART communication.
  // @return Baud rate in bits per second.
  uint32_t get_baud_rate() const { return baud_rate_; }

  void setup() override;
  void dump_config() override;
  // void update() override;
  void loop() override;
  bool get_configured_();

#ifdef USE_TEXT_SENSOR
  void set_received_text_sensor(text_sensor::TextSensor *sensor);
  void receive_message(const uint8_t*, size_t);
  void concat_received_bytes(const uint8_t*, size_t);
  bool ends_with(const uint8_t*, size_t, const char *);
#endif

 protected:

#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *received_;
#endif
  void check_logger_conflict();
  bool check_read_timeout_(size_t len = 1);

  InternalGPIOPin *tx_pin_; // +
  InternalGPIOPin *rx_pin_; // -
  size_t rx_buffer_size_;
  size_t tx_buffer_size_;
  uint32_t baud_rate_;
  uint8_t stop_bits_;
  uint8_t data_bits_;
  VCPParityOptions parity_;
  uint8_t* rx_buffer_;
  size_t rx_buffer_data_len_;
  // uart_port_t uart_num_;
  // QueueHandle_t uart_event_queue_;
  // uart_config_t get_config_();
  SemaphoreHandle_t lock_;
  CdcAcmDevice *vcp;
};

}  // namespace usb_vcp
}  // namespace esphome
