/**
 * @section license License
 *
 * This file is subject to the terms and conditions defined in
 * file 'license.txt', which is part of this source code package.
 */

#include <i2clib/master.h>

#include <type_traits>

#include <i2clib/operation.h>

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include <esp_log.h>

static_assert(std::is_copy_constructible<i2c::Master>::value,
              "i2c::Master should be copy constructed");
static_assert(std::is_move_constructible<i2c::Master>::value,
              "i2c::Master should be move constructed");
static_assert(std::is_default_constructible<i2c::Master>::value,
              "i2c::Master should be default constructed");
static_assert(std::is_copy_assignable<i2c::Master>::value,
              "i2c::Master should be copy assignable");
static_assert(std::is_move_assignable<i2c::Master>::value,
              "i2c::Master should be move assignable");

namespace i2c {

namespace {

constexpr char TAG[] = "I2C-master";
constexpr bool ACK_CHECK_EN = true;
constexpr TickType_t kI2CCmdWaitTicks = 1000 / portTICK_RATE_MS;

i2c_cmd_handle_t StartCommand(Address slave_addr, AddressWriter::Mode mode) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  if (!cmd)
    return nullptr;
  esp_err_t err = i2c_master_start(cmd);
  if (err == ESP_OK)
    err = AddressWriter::Write(cmd, slave_addr, mode);
  if (err != ESP_OK) {
    i2c_cmd_link_delete(cmd);
    return nullptr;
  }
  return cmd;
}

}  // namespace

Master::Master(i2c_port_t i2c_num, SemaphoreHandle_t i2c_mutex)
    : i2c_num_(i2c_num), i2c_mutex_(i2c_mutex) {}

Master::~Master() = default;

bool Master::Read(Address slave_addr,
                  void* buff,
                  size_t buff_size,
                  bool send_start) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  if (!cmd)
    return false;
  esp_err_t err = ESP_OK;
  if (send_start) {
    err = i2c_master_start(cmd);
    if (err != ESP_OK)
      goto READ_DONE;
  }

  err = i2c_master_write_byte(cmd, (slave_addr.address << 1) | I2C_MASTER_READ,
                              ACK_CHECK_EN);
  if (err != ESP_OK)
    goto READ_DONE;
  if (buff_size > 1) {
    err = i2c_master_read(cmd, static_cast<uint8_t*>(buff), buff_size - 1,
                          I2C_MASTER_ACK);
    if (err != ESP_OK)
      goto READ_DONE;
  }
  err = i2c_master_read_byte(cmd, static_cast<uint8_t*>(buff) + buff_size - 1,
                             I2C_MASTER_NACK);
  if (err != ESP_OK)
    goto READ_DONE;
  err = i2c_master_stop(cmd);
  if (err != ESP_OK)
    goto READ_DONE;

  if (i2c_mutex_)
    xSemaphoreTake(i2c_mutex_, portMAX_DELAY);
  err = i2c_master_cmd_begin(i2c_num_, cmd, kI2CCmdWaitTicks);
  if (i2c_mutex_)
    xSemaphoreGive(i2c_mutex_);

READ_DONE:
  i2c_cmd_link_delete(cmd);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Read of %u bytes failed: %s.", buff_size,
             esp_err_to_name(err));
  } else {
    ESP_LOGV(TAG, "Read of %u bytes succeeded.", buff_size);
  }

  return err == ESP_OK;
}

bool Master::Ping(Address addr) {
  i2c_cmd_handle_t cmd = StartCommand(addr, AddressWriter::Mode::WRITE);
  if (!cmd)
    return false;
  esp_err_t err = i2c_master_stop(cmd);
  if (err != ESP_OK)
    goto PING_DONE;

  if (i2c_mutex_)
    xSemaphoreTake(i2c_mutex_, portMAX_DELAY);

  err = i2c_master_cmd_begin(i2c_num_, cmd, kI2CCmdWaitTicks);

  if (i2c_mutex_)
    xSemaphoreGive(i2c_mutex_);

PING_DONE:
  if (err != ESP_OK)
    ESP_LOGE(TAG, "Ping 0x%x failed: %s", addr.address, esp_err_to_name(err));
  i2c_cmd_link_delete(cmd);
  return err == ESP_OK;
}

Operation Master::CreateWriteOp(Address slave_addr,
                                uint8_t reg,
                                const char* op_name) {
  i2c_cmd_handle_t cmd = StartCommand(slave_addr, AddressWriter::Mode::WRITE);
  if (!cmd)
    return Operation(op_name);
  esp_err_t err = i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "%s CreateWriteOp failed: %s", op_name, esp_err_to_name(err));
    i2c_cmd_link_delete(cmd);
    return Operation(op_name);
  }
  return Operation(cmd, i2c_num_, slave_addr, i2c_mutex_, op_name);
}

Operation Master::CreateReadOp(Address slave_addr,
                               uint8_t reg,
                               const char* op_name) {
  i2c_cmd_handle_t cmd = StartCommand(slave_addr, AddressWriter::Mode::WRITE);
  if (!cmd)
    return Operation(op_name);
  esp_err_t err = i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  if (err != ESP_OK)
    goto READ_OP_DONE;
  err = i2c_master_start(cmd);
  if (err == ESP_OK)
    err = AddressWriter::Write(cmd, slave_addr, AddressWriter::Mode::READ);

READ_OP_DONE:
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "%s CreateReadOp failed: %s", op_name, esp_err_to_name(err));
    i2c_cmd_link_delete(cmd);
    return Operation(op_name);
  }
  return Operation(cmd, i2c_num_, slave_addr, i2c_mutex_, op_name);
}

Operation Master::CreateReadOp(Address slave_addr, const char* op_name) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  if (!cmd)
    return Operation(op_name);

  esp_err_t err = i2c_master_start(cmd);
  if (err != ESP_OK)
    goto DONE;

  err = AddressWriter::Write(cmd, slave_addr, AddressWriter::Mode::READ);

DONE:
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "%s CreateReadOp failed: %s", op_name, esp_err_to_name(err));
    i2c_cmd_link_delete(cmd);
    return Operation(op_name);
  }
  return Operation(cmd, i2c_num_, slave_addr, i2c_mutex_, op_name);
}

}  // namespace i2c
