/**
 * @section license License
 *
 * This file is subject to the terms and conditions defined in
 * file 'license.txt', which is part of this source code package.
 */
#include <i2clib/operation.h>

#include <type_traits>

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
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
constexpr TickType_t kI2CCmdWaitTicks = pdMS_TO_TICKS(1000);
constexpr bool ACK_CHECK_EN = true;  ///< I2C master will check ack from slave.
}  // namespace

Operation::Operation(const char* op_name)
    : stopped_(true),
      cmd_(nullptr),
      i2c_num_(I2C_NUM_0),
      slave_addr_{0, Address::Size::k7bit},
      i2c_mutex_(nullptr),
      name_(op_name) {}

Operation::Operation(i2c_cmd_handle_t cmd,
                     i2c_port_t i2c_num,
                     Address slave_addr,
                     SemaphoreHandle_t i2c_mutex,
                     const char* op_name)
    : stopped_(false),
      cmd_(cmd),
      i2c_num_(i2c_num),
      slave_addr_(slave_addr),
      i2c_mutex_(i2c_mutex),
      name_(op_name) {}

Operation::~Operation() {
  if (cmd_) {
    ESP_LOGW(TAG, "Op %s created but never executed (doing so now).", name_);
    Execute();
  }
}

esp_err_t Operation::Read(void* dst, size_t num_bytes) {
  if (stopped_)
    return ESP_ERR_INVALID_STATE;
  esp_err_t err;
  if (!cmd_) {
    err = Restart(AddressWriter::Mode::kRead);
    if (err != ESP_OK)
      return err;
  }
  if (num_bytes > 1) {
    err = i2c_master_read(cmd_, static_cast<uint8_t*>(dst), num_bytes - 1,
                          I2C_MASTER_ACK);
    if (err != ESP_OK)
      goto READ_END;
  }
  err = i2c_master_read_byte(cmd_, static_cast<uint8_t*>(dst) + num_bytes - 1,
                             I2C_MASTER_NACK);
READ_END:
  return err;
}

esp_err_t Operation::Write(const void* data, size_t num_bytes) {
  if (!stopped_ && !cmd_) {
    esp_err_t err = Restart(AddressWriter::Mode::kWrite);
    if (err != ESP_OK)
      return err;
  }
  // TODO: In newer IDF's data is const. Remove typecast eventually.
  return i2c_master_write(cmd_, (uint8_t*)(data), num_bytes, ACK_CHECK_EN);
}

esp_err_t Operation::WriteByte(uint8_t val) {
  if (stopped_)
    return ESP_ERR_INVALID_STATE;
  if (!cmd_) {
    esp_err_t err = Restart(AddressWriter::Mode::kWrite);
    if (err != ESP_OK)
      return err;
  }
  return i2c_master_write_byte(cmd_, val, I2C_MASTER_ACK);
}

esp_err_t Operation::Execute(ExecuteEnd end_method) {
  if (stopped_ || !cmd_)
    return ESP_ERR_INVALID_STATE;

  ESP_LOGV(TAG, "Execute STOP:%c",
           end_method == ExecuteEnd::SendStop ? 'Y' : 'N');

  esp_err_t err = ESP_OK;
  if (end_method == ExecuteEnd::SendStop) {
    err = i2c_master_stop(cmd_);
    if (err != ESP_OK)
      goto EXECUTE_END;
  }

  if (i2c_mutex_)
    xSemaphoreTake(i2c_mutex_, portMAX_DELAY);

  err = i2c_master_cmd_begin(i2c_num_, cmd_, kI2CCmdWaitTicks);

  if (i2c_mutex_)
    xSemaphoreGive(i2c_mutex_);

  if (end_method == ExecuteEnd::SendStop)
    stopped_ = true;

  i2c_cmd_link_delete(cmd_);
  cmd_ = nullptr;

EXECUTE_END:
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "i2c_master_cmd_begin for %s on port %u failed: %s", name_,
             i2c_num_, esp_err_to_name(err));
    return err;
  }

  ESP_LOGV(TAG, "%s completed successfully.", name_);
  return ESP_OK;
}

esp_err_t Operation::Restart(AddressWriter::Mode type) {
  if (stopped_)
    return ESP_ERR_INVALID_STATE;
  if (!cmd_) {
    cmd_ = i2c_cmd_link_create();
    if (!cmd_) {
      stopped_ = true;  // This operation is now ended.
      return ESP_ERR_NO_MEM;
    }
  }
  esp_err_t err = i2c_master_start(cmd_);
  if (err != ESP_OK)
    goto RESTART_DONE;
  err = AddressWriter::Write(cmd_, slave_addr_, AddressWriter::Mode::kWrite);
  if (err != ESP_OK)
    goto RESTART_DONE;
  if (type == AddressWriter::Mode::kWrite)
    goto RESTART_DONE;
  err = i2c_master_start(cmd_);
  if (err == ESP_OK) {
    err = AddressWriter::Write(cmd_, slave_addr_, AddressWriter::Mode::kRead);
  }

RESTART_DONE:
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "%s restart %s failed: %s (%p)", name_,
             type == AddressWriter::Mode::kWrite ? "write" : "read",
             esp_err_to_name(err), cmd_);
    return err;
  }
  ESP_LOGV(TAG, "%s restart %s success.", name_,
           type == AddressWriter::Mode::kWrite ? "write" : "read");
  return ESP_OK;
}

esp_err_t Operation::RestartReg(uint8_t reg, AddressWriter::Mode mode) {
  if (stopped_)
    return ESP_ERR_INVALID_STATE;
  if (!cmd_) {
    cmd_ = i2c_cmd_link_create();
    if (!cmd_) {
      stopped_ = true;  // This operation is now ended.
      return ESP_ERR_NO_MEM;
    }
  }
  esp_err_t err = i2c_master_start(cmd_);
  if (err != ESP_OK)
    goto RESTART_DONE;
  err = AddressWriter::Write(cmd_, slave_addr_, AddressWriter::Mode::kWrite);
  if (err != ESP_OK)
    goto RESTART_DONE;
  err = i2c_master_write_byte(cmd_, reg, ACK_CHECK_EN);
  if (err != ESP_OK)
    goto RESTART_DONE;
  if (mode == AddressWriter::Mode::kWrite)
    goto RESTART_DONE;
  err = i2c_master_start(cmd_);
  if (err == ESP_OK) {
    err = AddressWriter::Write(cmd_, slave_addr_, AddressWriter::Mode::kRead);
  }

RESTART_DONE:
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "%s restart failed: %s.", name_, esp_err_to_name(err));
    return err;
  }
  ESP_LOGV(TAG, "%s restarted for %s.", name_,
           mode == AddressWriter::Mode::kRead ? "read" : "write");
  return ESP_OK;
}

}  // namespace i2c
