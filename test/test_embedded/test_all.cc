/**
 * @section license License
 *
 * This file is subject to the terms and conditions defined in
 * file 'license.txt', which is part of this source code package.
 */

#include <unity.h>

#include <i2clib/bus.h>
#include <i2clib/simple_master.h>
#include <i2clib/status.h>

using i2c::Bus;
using i2c::SimpleMaster;
using i2c::Status;

#define TEST_ASSERT_OK(s) TEST_ASSERT_TRUE(s.ok())

#define TEST_ASSERT_NOT_OK(s) TEST_ASSERT_TRUE(!s.ok())

/**
 * The I2C bus speed when running tests.
 *
 * Max of 1MHz recommended by:
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html#_CPPv4N12i2c_config_t9clk_speedE
 */
constexpr int kI2CClockHz = 100000;
constexpr uint8_t kTestSlaveAddress = 0x68;

SemaphoreHandle_t g_i2c_mutex;

namespace {

Status InitI2C(uint8_t i2c_bus) {
  const Bus::InitParams params = {
      .i2c_bus = i2c_bus,
      .sda_gpio = PORT_1_I2C_SDA_GPIO,
      .scl_gpio = PORT_1_I2C_CLK_GPIO,
      .clk_speed = kI2CClockHz,
      .sda_pullup_enable = false,
      .scl_pullup_enable = false,
  };
  return Bus::Initialize(params);
}

void test_invalid_init_port() {
  constexpr uint8_t kInvalidI2CPort = 200;
  TEST_ASSERT_NOT_OK(InitI2C(kInvalidI2CPort));
}

void test_double_init() {
  TEST_ASSERT_OK(InitI2C(TEST_I2C_PORT1));

  TEST_ASSERT_NOT_OK(InitI2C(TEST_I2C_PORT1));
}

void test_create_master() {
  InitI2C(TEST_I2C_PORT1);

  SimpleMaster master(TEST_I2C_PORT1, g_i2c_mutex);
  // Not much to test - no crash.
}

void test_shutdown_ok() {
  TEST_ASSERT_OK(InitI2C(TEST_I2C_PORT1));

  TEST_ASSERT_OK(Bus::Shutdown(TEST_I2C_PORT1));
}

void test_shutdown_not_running() {
  TEST_ASSERT_NOT_OK(Bus::Shutdown(TEST_I2C_PORT1));
}

void process() {
  g_i2c_mutex = xSemaphoreCreateMutex();

  UNITY_BEGIN();

  RUN_TEST(test_invalid_init_port);
  RUN_TEST(test_double_init);
  RUN_TEST(test_create_master);
  RUN_TEST(test_shutdown_ok);
  RUN_TEST(test_shutdown_not_running);

  UNITY_END();
}

void WaitForDebugMonitor() {
  // Poor man's way of waiting till the monitor has connected.
  const TickType_t kStartupDelay = 1000 / portTICK_PERIOD_MS;
  vTaskDelay(kStartupDelay);
}

}  // namespace

// Called before each test.
void setUp() {}

void tearDown() {
  Bus::Shutdown(TEST_I2C_PORT1);
}

extern "C" void app_main() {
  // If we don't wait, then sometimes the earlier logging will get dropped.
  // Also, seems to prevent test from hanging and requiring manual reset.
  WaitForDebugMonitor();

  process();
}
