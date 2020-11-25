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

void test_init_failed() {
  constexpr uint8_t kInvalidI2CPort = 200;
  TEST_ASSERT_FALSE(Master::Initialize(
      kInvalidI2CPort, PORT_1_I2C_SDA_GPIO, PORT_1_I2C_CLK_GPIO, kI2CClockHz));
}

void test_create_master() {
  Master master(TEST_I2C_PORT1, g_i2c_mutex);
  // Not much to test - no crash.
}

void process() {
  g_i2c_mutex = xSemaphoreCreateMutex();

  Master::Initialize(TEST_I2C_PORT1, PORT_1_I2C_SDA_GPIO,
                          PORT_1_I2C_CLK_GPIO, kI2CClockHz);

  UNITY_BEGIN();

  RUN_TEST(test_init_failed);
  RUN_TEST(test_create_master);

  UNITY_END();
}

void WaitForDebugMonitor() {
  // Poor man's way of waiting till the monitor has connected.
  const TickType_t kStartupDelay = 1000 / portTICK_PERIOD_MS;
  vTaskDelay(kStartupDelay);
}

}  // namespace

// Called before each test.
void setUp(void) {}

extern "C" void app_main() {
  // If we don't wait, then sometimes the earlier logging will get dropped.
  // Also, seems to prevent test from hanging and requiring manual reset.
  WaitForDebugMonitor();

  process();
}
