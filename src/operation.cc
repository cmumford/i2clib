/**
 * @section license License
 *
 * This file is subject to the terms and conditions defined in
 * file 'license.txt', which is part of this source code package.
 */
#include <i2clib/operation.h>

#include <type_traits>

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <esp_log.h>

#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

static_assert(!std::is_copy_constructible<i2c::Operation>::value,
              "i2c::Operation cannot be copy constructed");
static_assert(std::is_move_constructible<i2c::Operation>::value,
              "i2c::Operation should be move constructed");
static_assert(!std::is_default_constructible<i2c::Operation>::value,
              "i2c::Operation cannot be default constructed");
static_assert(!std::is_copy_assignable<i2c::Operation>::value,
              "i2c::Operation cannot be copy assigned");

namespace i2c {

namespace {
constexpr char TAG[] = "I2C-op";
constexpr TickType_t kI2CCmdWaitTicks = 1000 / portTICK_RATE_MS;
constexpr bool ACK_CHECK_EN = true;  ///< I2C master will check ack from slave.
}  // namespace

Operation::Operation(i2c_cmd_handle_t cmd,
                     i2c_port_t i2c_num,
                     SemaphoreHandle_t i2c_mutex,
                     const char* op_name)
    : cmd_(cmd), i2c_num_(i2c_num), i2c_mutex_(i2c_mutex), name_(op_name) {}

Operation::~Operation() {
  if (cmd_) {
    ESP_LOGW(TAG, "Op \"%s\" created but never executed (doing so now).",
             name_);
    Execute();
  }
}

bool Operation::Read(void* dst, size_t num_bytes) {
  esp_err_t err;
  if (num_bytes > 1) {
    err = i2c_master_read(cmd_, static_cast<uint8_t*>(dst), num_bytes - 1,
                          I2C_MASTER_ACK);
    if (err != ESP_OK)
      goto READ_END;
  }
  err = i2c_master_read_byte(cmd_, static_cast<uint8_t*>(dst) + num_bytes - 1,
                             I2C_MASTER_NACK);
READ_END:
  return err == ESP_OK;
}

bool Operation::Write(const void* data, size_t num_bytes) {
  // In newer IDF's data is const.
  return i2c_master_write(cmd_, (uint8_t*)(data), num_bytes, ACK_CHECK_EN) ==
         ESP_OK;
}

bool Operation::WriteByte(uint8_t val) {
  return i2c_master_write_byte(cmd_, val, I2C_MASTER_ACK) == ESP_OK;
}

bool Operation::Execute() {
  if (!cmd_)
    return true;

  esp_err_t err = i2c_master_stop(cmd_);
  if (err != ESP_OK)
    ESP_LOGE(TAG, "i2c_master_stop failed: %s", esp_err_to_name(err));

  if (i2c_mutex_)
    xSemaphoreTake(i2c_mutex_, portMAX_DELAY);

  err = i2c_master_cmd_begin(i2c_num_, cmd_, kI2CCmdWaitTicks);

  if (i2c_mutex_)
    xSemaphoreGive(i2c_mutex_);

  i2c_cmd_link_delete(cmd_);
  cmd_ = nullptr;

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "i2c_master_cmd_begin for \"%s\" failed: %s", name_,
             esp_err_to_name(err));
    return false;
  }

  ESP_LOGV(TAG, "\"%s\" completed successfully.", name_);
  return true;
}

bool Operation::Restart(uint8_t slave_addr, uint8_t reg, OperationType type) {
  esp_err_t err = i2c_master_start(cmd_);
  if (err != ESP_OK)
    goto RESTART_DONE;
  err = i2c_master_write_byte(cmd_, (slave_addr << 1) | I2C_MASTER_WRITE,
                              ACK_CHECK_EN);
  if (err != ESP_OK)
    goto RESTART_DONE;
  err = i2c_master_write_byte(cmd_, reg, ACK_CHECK_EN);
  if (err != ESP_OK)
    goto RESTART_DONE;
  if (type == OperationType::WRITE)
    goto RESTART_DONE;
  err = i2c_master_start(cmd_);
  if (err == ESP_OK) {
    err = i2c_master_write_byte(cmd_, (slave_addr << 1) | I2C_MASTER_READ,
                                ACK_CHECK_EN);
  }

RESTART_DONE:
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "%s restart failed: %s", name_, esp_err_to_name(err));
    return false;
  }
  ESP_LOGV(TAG, "%s Restarted", name_);
  return true;
}

}  // namespace i2c
