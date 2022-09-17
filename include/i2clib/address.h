/**
 * @section license License
 *
 * This file is subject to the terms and conditions defined in
 * file 'license.txt', which is part of this source code package.
 */

#pragma once

#include <cstdint>

#include <driver/i2c.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>

namespace i2c {

/**
 * @brief An I2C address.
 */
struct Address {
  /**
   * Size of I2C address (7 or 10 bit).
   */
  enum class Size {
    bit7,   // 7-bit slave address.
    bit10,  // 10-bit slave address.
  };

  uint16_t address;  // The 7 or 10-bit I2C address.
  Size addr_size;    // The I2C address size.
};

/**
 * @brief Encapsulate writing an address to an I2C command buffer.
 *
 * Properly handles 7 and 10-bit address sizes.
 */
class AddressWriter {
 public:
  /**
   * Mode of operation.
   *
   * Read or write.
   */
  enum class Mode {
    WRITE,  // The slave is being written to.
    READ,   // The slave is being read from.
  };

  /**
   * @brief Write the I2C address into the command handle.
   *
   * @param cmd    The command buffer handle.
   * @param addr   The address to write.
   * @param mode   The write mode (i.e. read/write).
   *
   * @return esp_err_t ESP_OK if the write was successful.
   */
  static esp_err_t Write(i2c_cmd_handle_t cmd, Address addr, Mode mode);

 private:
  AddressWriter() = delete;
  ~AddressWriter() = delete;
};

}  // namespace i2c
