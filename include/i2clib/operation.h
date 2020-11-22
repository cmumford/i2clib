/**
 * @section license License
 *
 * This file is subject to the terms and conditions defined in
 * file 'license.txt', which is part of this source code package.
 */
#include <cstdint>

#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#pragma once

namespace i2c {

enum class OperationType { READ, WRITE };

/**
 * A single I2C command operation.
 *
 * The command is started when this instance is created, and this instance
 * will automatically stop the operation when deleted.
 */
class Operation {
 public:
  ~Operation();

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
   * @param num_bytes The size of \p val.
   *
   * @return true if successfully enqueued, false upon error.
   */
  bool Write(const void* val, size_t num_bytes);

  /**
   * Restart the I2C operation.
   *
   * When this instance is created the operation is already started. This will
   * enqueue another start into this operation.
   */
  bool Restart(uint8_t i2c_address, uint8_t reg, OperationType type);

  /**
   * Execute all queued tasks.
   */
  bool Execute();

 private:
  friend class Master;

  Operation(i2c_cmd_handle_t cmd,
               i2c_port_t i2c_num,
               SemaphoreHandle_t i2c_mutex,
               const char* op_name);

  i2c_cmd_handle_t cmd_;         // The started command.
  const i2c_port_t i2c_num_;     // I2C bus or port number.
  SemaphoreHandle_t i2c_mutex_;  // Mutex used for synchronization.
  const char* name_;             // The operation name - used for debugging.
};

}  // namespace i2c
