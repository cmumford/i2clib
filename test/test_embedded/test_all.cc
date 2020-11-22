/**
 * @section license License
 *
 * This file is subject to the terms and conditions defined in
 * file 'license.txt', which is part of this source code package.
 */

#include <unity.h>

#include <i2clib/i2c.h>

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

void test_create_master() {
  std::unique_ptr<i2c::I2CMaster> master(
      new i2c::I2CMaster(TEST_I2C_PORT1, g_i2c_mutex));
  TEST_ASSERT_NOT_NULL(master);
}

void process() {
  g_i2c_mutex = xSemaphoreCreateMutex();

  i2c::I2CMaster::Initialize(TEST_I2C_PORT1, PORT_1_I2C_SDA_GPIO,
                             PORT_1_I2C_CLK_GPIO, kI2CClockHz);

  UNITY_BEGIN();

  RUN_TEST(test_create_master);

  UNITY_END();
}

}  // namespace

// Called before each test.
void setUp(void) {}

extern "C" void app_main() {
  process();
}
