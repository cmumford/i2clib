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

/**
 * A single I2C command operation.
 *
 * The command is started when this instance is created, and this instance
 * will automatically stop the operation when deleted.
 */
class Operation {
 public:

  enum class ExecuteEnd {
    SendStop,
    NoStop,
  };

  Operation(const Operation&) = delete;
  Operation(Operation&&) = default;
  ~Operation();

  Operation& operator=(Operation&&) = default;

  /**
   * Queue a read to be executed later.
   *
   * @param val The address to which to write the bytes read.
   *            this must be valid until this operation is either executed
   *            or deleted (which ever is first).
   * @param num_bytes The number of bytes to read.
   *
   * @return true if the read is successfully queued (but not executed), false
   *         if not.
   */
  bool Read(void* val, size_t num_bytes);

  /**
   * Queue the write of a byte in this operation.
   */
  bool WriteByte(uint8_t val);

  /**
   * @brief Queue a write operation.
   *
   * @param val The data to write
   * @param num_bytes The size (in bytes) of \p val.
   *
   * @return true if successfully enqueued, false upon error.
   */
  bool Write(const void* val, size_t num_bytes);

  /**
   * Restart the I2C operation.
   *
   * This will restart an I2C operation at the current register. Many I2C
   * devices will auto-increment the current register. This version of the
   * restart will not change the current register.
   *
   * @param dir        The operation type (i.e. read/write).
   */
  bool Restart(Direction dir);

  /**
   * Restart the I2C operation.
   *
   * When this instance is created the operation is already started. This will
   * enqueue another start into this operation.
   *
   * @param reg         The register where reading/writing will now commence.
   * @param type        The operation type (i.e. read/write).
   */
  bool RestartReg(uint8_t reg, Direction dir);

  /**
   * Execute all queued tasks.
   *
   * @param end Whether to end this operation. If ExecuteEnd::SendStop
   *            (default), then this operation is stopped, and no further
   *            actions will succeed. If not then further actions can be
   *            requested.
   */
  bool Execute(ExecuteEnd end = ExecuteEnd::SendStop);

  /**
   * Is this ready to be used?
   */
  bool ready() const { return !stopped_; }

 private:
  friend class Master;

  /**
   * Create an invalid (already stopped) operation.
   */
  Operation(const char* op_name);

  Operation(i2c_cmd_handle_t cmd,
            i2c_port_t i2c_num,
            uint16_t slave_addr,
            AddressMode addr_mode,
            SemaphoreHandle_t i2c_mutex,
            const char* op_name);

  bool stopped_;                 // Was I2C STOP ever written?
  i2c_cmd_handle_t cmd_;         // The started command.
  const i2c_port_t i2c_num_;     // I2C bus or port number.
  const uint16_t slave_addr_;    // I2C slave address.
  const AddressMode addr_mode_;  // 7 or 10 bit I2C address.
  SemaphoreHandle_t i2c_mutex_;  // Mutex used for synchronization.
  const char* name_;             // The operation name - used for debugging.
};

}  // namespace i2c
