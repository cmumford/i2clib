/**
 * @section license License
 *
 * This file is subject to the terms and conditions defined in
 * file 'license.txt', which is part of this source code package.
 */

#include <i2clib/bus.h>

#include <type_traits>

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include <esp_log.h>

static_assert(!std::is_copy_constructible<i2c::Bus>::value,
              "i2c::Bus should be copy constructed");
static_assert(!std::is_move_constructible<i2c::Bus>::value,
              "i2c::Bus should be move constructed");
static_assert(!std::is_default_constructible<i2c::Bus>::value,
              "i2c::Bus should be default constructed");
static_assert(!std::is_copy_assignable<i2c::Bus>::value,
              "i2c::Bus should be copy assignable");
static_assert(!std::is_move_assignable<i2c::Bus>::value,
              "i2c::Bus should be move assignable");

namespace i2c {

namespace {

constexpr char TAG[] = "I2C-bus";
constexpr size_t kSlaveReceiveBuffLen = 0;
constexpr size_t kSlaveTransmitBuffLen = 0;
constexpr int kInterruptAllocFlags = ESP_INTR_FLAG_IRAM;

}  // namespace

// static
bool Bus::Initialize(const InitParams& params) {
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
bool Bus::Shutdown(uint8_t i2c_bus) {
  return i2c_driver_delete(i2c_bus) == ESP_OK;
}

// static
bool Bus::SetTimeout(uint8_t i2c_bus, int timeout) {
  return i2c_set_timeout(i2c_bus, timeout) == ESP_OK;
}

// static
bool Bus::GetTimeout(uint8_t i2c_bus, int* timeout) {
  return i2c_get_timeout(i2c_bus, timeout) == ESP_OK;
}

}  // namespace i2c
