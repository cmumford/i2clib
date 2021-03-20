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
 * Direction of operation.
 *
 * Read or write.
 */
enum class Direction {
  WRITE,  // The slave is being written to.
  READ,   // The slave is being read from.
};

/**
 * I2C mode of address (7 or 10 bit).
 */
enum class AddressMode {
  bit7,   // 7-bit slave address.
  bit10,  // 10-bit slave address.
};

/**
 * @brief Write the I2C slave address into the command handle.
 *
 * @param cmd        The command buffer handle.
 * @param slave_addr The 7 or 10-bit slave address.
 * @param addr_mode  The address mode (7/10-bit).
 * @param direction  Is slave being written/read from/to?
 *
 * @return esp_err_t ESP_OK if successful.
 */
esp_err_t WriteAddress(i2c_cmd_handle_t cmd,
                       uint16_t slave_addr,
                       AddressMode addr_mode,
                       Direction direction);

}  // namespace i2c
