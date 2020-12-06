/**
 * @section license License
 *
 * This file is subject to the terms and conditions defined in
 * file 'license.txt', which is part of this source code package.
 */

#include <unity.h>

#include <i2clib/master.h>

using i2c::Master;

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

bool InitI2C(uint8_t i2c_bus) {
  const Master::InitParams params = {
      .i2c_bus = i2c_bus,
      .sda_gpio = PORT_1_I2C_SDA_GPIO,
      .scl_gpio = PORT_1_I2C_CLK_GPIO,
      .clk_speed = kI2CClockHz,
      .sda_pullup_enable = false,
      .scl_pullup_enable = false,
  };
  return Master::Initialize(params);
}

void test_invalid_init_port() {
  constexpr uint8_t kInvalidI2CPort = 200;
  TEST_ASSERT_FALSE(InitI2C(kInvalidI2CPort));
}

void test_double_init() {
  TEST_ASSERT_TRUE(InitI2C(TEST_I2C_PORT1));

  TEST_ASSERT_FALSE(InitI2C(TEST_I2C_PORT1));
}

void test_create_master() {
  InitI2C(TEST_I2C_PORT1);

  Master master(TEST_I2C_PORT1, g_i2c_mutex);
  // Not much to test - no crash.
}

void test_shutdown_ok() {
  TEST_ASSERT_TRUE(InitI2C(TEST_I2C_PORT1));

  TEST_ASSERT_TRUE(Master::Shutdown(TEST_I2C_PORT1));
}

void test_shutdown_not_running() {
  TEST_ASSERT_FALSE(Master::Shutdown(TEST_I2C_PORT1));
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
  Master::Shutdown(TEST_I2C_PORT1);
}

extern "C" void app_main() {
  // If we don't wait, then sometimes the earlier logging will get dropped.
  // Also, seems to prevent test from hanging and requiring manual reset.
  WaitForDebugMonitor();

  process();
}
