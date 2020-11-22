/**
 * @section license License
 *
 * This file is subject to the terms and conditions defined in
 * file 'license.txt', which is part of this source code package.
 */
#include <cstdint>
#include <memory>

#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#pragma once

namespace i2c {

class Operation;

/**
 * Perform read and write operations on the specified I2C bus.
 */
class Master {
 public:
  /**
   * @brief Initialize the I2C bus.
   *
   * @param i2c_bus The I2C bus (or port).
   * @param sda_gpio The SDA (AKA SDIO) gpio pin number.
   * @param scl_gpio The SCL (AKA SDCL) gpio pin number.
   * @param clk_speed The I2C clock speed.
   *
   * @return true when successful, false when not.
   */
  static bool Initialize(uint8_t i2c_bus,
                         uint8_t sda_gpio,
                         uint8_t scl_gpio,
                         uint32_t clk_speed);

  Master(i2c_port_t i2c_num = I2C_NUM_0, SemaphoreHandle_t i2c_mutex = nullptr);
  ~Master();

  bool WriteRegister(uint8_t addr, uint8_t reg, uint8_t val);
  bool ReadRegister(uint8_t addr, uint8_t reg, uint8_t* val);
  bool Ping(uint8_t addr);

  /**
   * Start an I2C write operation to the I2C slave address.
   *
   * @return The operation pointer - null if error creating operation.
   */
  std::unique_ptr<Operation> CreateWriteOp(uint8_t slave_addr,
                                              uint8_t reg,
                                              const char* op_name);

  /**
   * Start an I2C read operation to the I2C slave address.
   *
   * @return The operation pointer - null if error creating operation.
   */
  std::unique_ptr<Operation> CreateReadOp(uint8_t slave_addr,
                                             uint8_t reg,
                                             const char* op_name);

 private:
  i2c_port_t i2c_num_;
  SemaphoreHandle_t i2c_mutex_;
};

}  // namespace i2c
