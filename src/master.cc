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
constexpr size_t kSlaveReceiveBuffLen = 0;
constexpr size_t kSlaveTransmitBuffLen = 0;
constexpr int kInterruptAllocFlags = ESP_INTR_FLAG_IRAM;

i2c_cmd_handle_t StartCommand(uint8_t slave_addr, i2c_rw_t read_write) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  if (!cmd)
    return nullptr;
  esp_err_t err = i2c_master_start(cmd);
  if (err == ESP_OK) {
    err = i2c_master_write_byte(cmd, (slave_addr << 1) | read_write,
                                ACK_CHECK_EN);
  }
  if (err != ESP_OK) {
    i2c_cmd_link_delete(cmd);
    return nullptr;
  }
  return cmd;
}

}  // namespace

// static
bool Master::Initialize(const InitParams& params) {
  const i2c_config_t config = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = params.sda_gpio,
    .scl_io_num = params.scl_gpio,
    .sda_pullup_en =
        params.sda_pullup_enable ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
    .scl_pullup_en =
        params.scl_pullup_enable ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
    .master =
        {
            .clk_speed = params.clk_speed,
        },
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 3, 0)
    .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL,
#endif
  };
  esp_err_t err = i2c_param_config(params.i2c_bus, &config);
  if (err == ESP_OK) {
    err = i2c_driver_install(params.i2c_bus, I2C_MODE_MASTER,
                             kSlaveReceiveBuffLen, kSlaveTransmitBuffLen,
                             kInterruptAllocFlags);
  }
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error initializing I2C on port %u, SDA/SCL=%d/%d: %s",
             params.i2c_bus, params.sda_gpio, params.scl_gpio,
             esp_err_to_name(err));
    return false;
  }
  ESP_LOGD(TAG, "I2C initialized on port %u, SDA/SCL=%d/%d.", params.i2c_bus,
           params.sda_gpio, params.scl_gpio);
  return true;
}

// static
bool Master::Shutdown(uint8_t i2c_bus) {
  return i2c_driver_delete(i2c_bus) == ESP_OK;
}

// static
bool Master::SetTimeout(uint8_t i2c_bus, int timeout) {
  return i2c_set_timeout(i2c_bus, timeout) == ESP_OK;
}

// static
bool Master::GetTimeout(uint8_t i2c_bus, int* timeout) {
  return i2c_get_timeout(i2c_bus, timeout) == ESP_OK;
}

Master::Master(i2c_port_t i2c_num, SemaphoreHandle_t i2c_mutex)
    : i2c_num_(i2c_num), i2c_mutex_(i2c_mutex) {}

Master::~Master() = default;

bool Master::WriteRegister(uint8_t addr, uint8_t reg, uint8_t val) {
  Operation op = CreateWriteOp(addr, reg, "WriteRegister");
  if (!op.ready())
    return false;
  if (!op.WriteByte(val))
    return false;
  return op.Execute();
}

bool Master::ReadRegister(uint8_t addr, uint8_t reg, uint8_t* val) {
  Operation op = CreateReadOp(addr, reg, "ReadRegister");
  if (!op.ready())
    return false;
  if (!op.Read(val, sizeof(*val)))
    return false;
  return op.Execute();
}

bool Master::Read(uint8_t slave_addr,
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

  err = i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_READ,
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

bool Master::Ping(uint8_t addr) {
  i2c_cmd_handle_t cmd = StartCommand(addr, I2C_MASTER_WRITE);
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

Operation Master::CreateWriteOp(uint8_t slave_addr,
                                uint8_t reg,
                                const char* op_name) {
  i2c_cmd_handle_t cmd = StartCommand(slave_addr, I2C_MASTER_WRITE);
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

Operation Master::CreateReadOp(uint8_t slave_addr,
                               uint8_t reg,
                               const char* op_name) {
  i2c_cmd_handle_t cmd = StartCommand(slave_addr, I2C_MASTER_WRITE);
  if (!cmd)
    return Operation(op_name);
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
    return Operation(op_name);
  }
  return Operation(cmd, i2c_num_, slave_addr, i2c_mutex_, op_name);
}

Operation Master::CreateReadOp(uint8_t slave_addr, const char* op_name) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  if (!cmd)
    return Operation(op_name);

  esp_err_t err = i2c_master_start(cmd);
  if (err != ESP_OK)
    goto DONE;

  err = i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_READ,
                              ACK_CHECK_EN);

DONE:
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "%s CreateReadOp failed: %s", op_name, esp_err_to_name(err));
    i2c_cmd_link_delete(cmd);
    return Operation(op_name);
  }
  return Operation(cmd, i2c_num_, slave_addr, i2c_mutex_, op_name);
}

}  // namespace i2c
