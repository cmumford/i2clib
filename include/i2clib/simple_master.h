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
#include <i2clib/master.h>

namespace i2c {

class Operation;

/**
 * Perform read and write operations on the specified I2C bus.
 */
class SimpleMaster : public Master {
 public:
  /**
   * Construct a new SimpleMaster object to interract with an I2C bus as a
   * master.
   *
   * The name "SimpleMaster" might be slightly misleading as there can be as
   * many of these objects instantiated for a given I2C bus as desired.
   *
   * @param i2c_num The I2C bus/port number.
   * @param i2c_mutex An optional mutex to synchronize all access to the bus.
   */
  SimpleMaster(i2c_port_t i2c_num = I2C_NUM_0,
               SemaphoreHandle_t i2c_mutex = nullptr);
  ~SimpleMaster();

  /**
   * Write a single byte value to the specified register.
   *
   * @param addr The I2C slave address.
   * @param addr_size The address size.
   * @param reg  The I2C slave register.
   * @param val  The byte to write.
   *
   * @return true when successful, false when not.
   */
  bool WriteRegister(uint16_t addr,
                     Address::Size addr_size,
                     uint8_t reg,
                     uint8_t val);

  /**
   * Read a single byte value from the specified register.
   *
   * @param addr The I2C slave address.
   * @param addr_size The address size.
   * @param reg  The I2C slave register.
   * @param val  Location to store the read byte.
   *
   * @return true when successful, false when not.
   */
  bool ReadRegister(uint16_t addr,
                    Address::Size addr_size,
                    uint8_t reg,
                    uint8_t* val);
};

}  // namespace i2c
