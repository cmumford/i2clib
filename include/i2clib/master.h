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

#include <i2clib/address.h>

namespace i2c {

class Operation;

/**
 * Perform read and write operations on the specified I2C bus.
 *
 * This implemens the basic functionality required for an I2C master
 * IAW the I2C specificaion (UM10204).
 */
class Master {
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
   * This only need be called once for each bus before creating instances of
   * this class.
   *
   * @param params Master initialization parameters.
   *
   * @return true when successful, false when not.
   */
  static bool Initialize(const InitParams& params);

  /**
   * Shutdown the initialized I2C bus.
   */
  static bool Shutdown(uint8_t i2c_bus);

  /**
   * Set the I2C bus timeout.
   *
   * @param i2c_bus The I2C bus/port number.
   * @param timeout Timeout (unit: APB 80Mhz clock cycle)
   *
   * @return true when successful, false when not.
   */
  static bool SetTimeout(uint8_t i2c_bus, int timeout);

  /**
   * Get the I2C bus timeout.
   *
   * @param i2c_bus The I2C bus/port number.
   * @param timeout Timeout (unit: APB 80Mhz clock cycle)
   *
   * @return true when successful, false when not.
   */
  static bool GetTimeout(uint8_t i2c_bus, int* timeout);

  /**
   * Construct a new Master object to interract with an I2C bus as a master.
   *
   * The name "Master" might be slightly misleading as there can be as many
   * of these objects instantiated for a given I2C bus as desired.
   *
   * @param i2c_num The I2C bus/port number.
   * @param i2c_mutex An optional mutex to synchronize all access to the bus.
   */
  Master(i2c_port_t i2c_num = I2C_NUM_0, SemaphoreHandle_t i2c_mutex = nullptr);
  ~Master();

  /**
   * Read from the specified I2C slave.
   */
  bool Read(uint16_t slave_addr, void* buff, size_t buff_size, bool send_start);

  /**
   * Detect if a slave device is listening at a specific address.
   *
   * @param addr The I2C address.
   *
   * @return true if successful, false if not.
   */
  bool Ping(uint16_t addr, Address::Size addr_size);

  /**
   * Start an I2C write operation to the I2C slave address.
   *
   * @return The operation pointer - null if error creating operation.
   */
  Operation CreateWriteOp(uint16_t slave_addr,
                          Address::Size addr_size,
                          uint8_t reg,
                          const char* op_name);

  /**
   * Start an I2C read operation to the I2C slave address.
   *
   * @return The operation pointer - null if error creating operation.
   */
  Operation CreateReadOp(uint16_t slave_addr,
                         Address::Size addr_size,
                         uint8_t reg,
                         const char* op_name);

  /**
   * Create a started I2C read operation from the I2C slave address.
   *
   * @return The operation pointer - null if error creating operation.
   */
  Operation CreateReadOp(uint16_t slave_addr,
                         Address::Size addr_size,
                         const char* op_name);

 protected:
  i2c_port_t i2c_num_;  // The I2C port on which this object communicates.
  SemaphoreHandle_t i2c_mutex_;  // Sync mutex. If null will not sync.
};

}  // namespace i2c
