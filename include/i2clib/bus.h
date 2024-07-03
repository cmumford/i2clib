/**
 * @section license License
 *
 * This file is subject to the terms and conditions defined in
 * file 'license.txt', which is part of this source code package.
 */

#pragma once

#include <cstdint>

#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <i2clib/status.h>

namespace i2c {

class Operation;

/**
 * Control the I2C bus.
 */
class Bus {
 public:
  /**
   * Initialization parameters for the I2C bus.
   */
  struct InitParams {
    i2c_port_t i2c_bus;      // The I2C bus (AKA port).
    uint8_t sda_gpio;        // The SDA (AKA SDIO) gpio pin number.
    uint8_t scl_gpio;        // The SCL (AKA SDCL) gpio pin number.
    uint32_t clk_speed;      // The I2C clock speed (Hz).
    bool sda_pullup_enable;  // Enable build-in SDA/SDIO pin pullup resistor.
    bool scl_pullup_enable;  // Enable build-in SCK/SCLK pin pullup resistor.
  };

  /**
   * @brief Initialize an I2C bus.
   *
   * This only need be called once.
   *
   * @param params Bus initialization parameters.
   *
   * @return true when successful, false when not.
   */
  static Status Initialize(const InitParams& params);

  /**
   * Shutdown the initialized I2C bus.
   */
  static Status Shutdown(i2c_port_t i2c_bus);

  /**
   * Set the I2C bus timeout.
   *
   * @param i2c_bus The I2C bus/port number.
   * @param timeout Timeout (unit: APB 80Mhz clock cycle)
   *
   * @return true when successful, false when not.
   */
  static Status SetTimeout(i2c_port_t i2c_bus, int timeout);

  /**
   * Get the I2C bus timeout.
   *
   * @param i2c_bus The I2C bus/port number.
   * @param timeout Timeout (unit: APB 80Mhz clock cycle)
   *
   * @return true when successful, false when not.
   */
  static Status GetTimeout(i2c_port_t i2c_bus, int* timeout);

  Bus() = delete;
  ~Bus() = delete;
  Bus& operator=(const Bus&) = delete;
};

}  // namespace i2c
