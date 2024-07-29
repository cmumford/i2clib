/**
 * @section license License
 *
 * This file is subject to the terms and conditions defined in
 * file 'license.txt', which is part of this source code package.
 */

#pragma once

#include <cstdint>
#include <expected>

#include <driver/i2c.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

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
   * @return ESP_OK when successful, another value when not.
   */
  static esp_err_t Initialize(const InitParams& params);

  /**
   * Shutdown the initialized I2C bus.
   */
  static esp_err_t Shutdown(i2c_port_t i2c_bus);

  /**
   * Set the I2C bus timeout.
   *
   * @param i2c_bus The I2C bus/port number.
   * @param timeout Timeout (unit: APB 80Mhz clock cycle)
   *
   * @return ESP_OK when successful, another value when not.
   */
  static esp_err_t SetTimeout(i2c_port_t i2c_bus, int timeout);

  /**
   * Get the I2C bus timeout.
   *
   * @param i2c_bus The I2C bus/port number.
   * @param timeout Timeout (unit: APB 80Mhz clock cycle)
   *
   * @return ESP_OK when successful, another value when not.
   */
  static std::expected<int, esp_err_t> GetTimeout(i2c_port_t i2c_bus);

  Bus() = delete;
  ~Bus() = delete;
  Bus& operator=(const Bus&) = delete;
};

}  // namespace i2c
