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

i2c_cmd_handle_t StartWriteCommand(uint8_t slave_addr) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  if (!cmd)
    return nullptr;
  esp_err_t err = i2c_master_start(cmd);
  if (err == ESP_OK) {
    err = i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE,
                                ACK_CHECK_EN);
  }
  if (err != ESP_OK) {
    i2c_cmd_link_delete(cmd);
    return nullptr;
  }
  return cmd;
}

}  // namespace

// Static
bool Master::Initialize(uint8_t i2c_bus,
                        uint8_t sda_gpio,
                        uint8_t scl_gpio,
                        uint32_t clk_speed) {
  const i2c_config_t config = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = sda_gpio,
      .scl_io_num = scl_gpio,
      .sda_pullup_en = GPIO_PULLUP_DISABLE,
      .scl_pullup_en = GPIO_PULLUP_DISABLE,
      .master = {.clk_speed = clk_speed},
  };

  esp_err_t err = i2c_param_config(i2c_bus, &config);
  if (err == ESP_OK)
    err = i2c_driver_install(i2c_bus, I2C_MODE_MASTER, 0, 0, 0) == ESP_OK;

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error initializing I2C on port%u, SDA/SCL=%d/%d: %s",
             i2c_bus, sda_gpio, scl_gpio, esp_err_to_name(err));
    return false;
  }

  ESP_LOGD(TAG, "I2C initialized on port%u, SDA/SCL=%d/%d: %s", i2c_bus,
           sda_gpio, scl_gpio, esp_err_to_name(err));
  return true;
}


Master::Master(i2c_port_t i2c_num, SemaphoreHandle_t i2c_mutex)
    : i2c_num_(i2c_num), i2c_mutex_(i2c_mutex) {}

Master::~Master() = default;

bool Master::WriteRegister(uint8_t addr, uint8_t reg, uint8_t val) {
  std::unique_ptr<Operation> op = CreateWriteOp(addr, reg, "WriteRegister");
  if (!op)
    return false;
  if (!op->WriteByte(val))
    return false;
  return op->Execute();
}

bool Master::ReadRegister(uint8_t addr, uint8_t reg, uint8_t* val) {
  std::unique_ptr<Operation> op = CreateReadOp(addr, reg, "ReadRegister");
  if (!op)
    return false;
  if (!op->Read(val, sizeof(*val)))
    return false;
  return op->Execute();
}

bool Master::Ping(uint8_t addr) {
  i2c_cmd_handle_t cmd = StartWriteCommand(addr);
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
    ESP_LOGE(TAG, "Ping 0x%x failed: %s", addr, esp_err_to_name(err));
  i2c_cmd_link_delete(cmd);
  return err == ESP_OK;
}

std::unique_ptr<Operation> Master::CreateWriteOp(uint8_t slave_addr,
                                                 uint8_t reg,
                                                 const char* op_name) {
  i2c_cmd_handle_t cmd = StartWriteCommand(slave_addr);
  if (!cmd)
    return nullptr;
  esp_err_t err = i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "%s CreateWriteOp failed: %s", op_name, esp_err_to_name(err));
    i2c_cmd_link_delete(cmd);
    return nullptr;
  }
  return std::unique_ptr<Operation>(
      new Operation(cmd, i2c_num_, i2c_mutex_, op_name));
}

std::unique_ptr<Operation> Master::CreateReadOp(uint8_t slave_addr,
                                                uint8_t reg,
                                                const char* op_name) {
  i2c_cmd_handle_t cmd = StartWriteCommand(slave_addr);
  if (!cmd)
    return nullptr;
  esp_err_t err = i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  if (err != ESP_OK)
    goto READ_OP_DONE;
  err = i2c_master_start(cmd);
  if (err == ESP_OK) {
    err = i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_READ,
                                ACK_CHECK_EN);
  }

READ_OP_DONE:
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "%s CreateReadOp failed: %s", op_name, esp_err_to_name(err));
    i2c_cmd_link_delete(cmd);
    return nullptr;
  }
  return std::unique_ptr<Operation>(
      new Operation(cmd, i2c_num_, i2c_mutex_, op_name));
}

}  // namespace i2c
