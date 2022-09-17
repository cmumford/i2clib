/**
 * @section license License
 *
 * This file is subject to the terms and conditions defined in
 * file 'license.txt', which is part of this source code package.
 */

#include <i2clib/simple_master.h>

#include <type_traits>

#include <i2clib/operation.h>

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include <esp_log.h>

static_assert(std::is_copy_constructible<i2c::SimpleMaster>::value,
              "i2c::SimpleMaster should be copy constructed");
static_assert(std::is_move_constructible<i2c::SimpleMaster>::value,
              "i2c::SimpleMaster should be move constructed");
static_assert(std::is_default_constructible<i2c::SimpleMaster>::value,
              "i2c::SimpleMaster should be default constructed");
static_assert(std::is_copy_assignable<i2c::SimpleMaster>::value,
              "i2c::SimpleMaster should be copy assignable");
static_assert(std::is_move_assignable<i2c::SimpleMaster>::value,
              "i2c::SimpleMaster should be move assignable");

namespace i2c {

namespace {
constexpr char TAG[] = "I2C-smaster";
}  // namespace

SimpleMaster::SimpleMaster(i2c_port_t i2c_num, SemaphoreHandle_t i2c_mutex)
    : Master(i2c_num, i2c_mutex) {}

SimpleMaster::~SimpleMaster() = default;

bool SimpleMaster::WriteRegister(Address addr, uint8_t reg, uint8_t val) {
  Operation op = CreateWriteOp(addr, reg, "WriteRegister");
  if (!op.ready())
    return false;
  if (!op.WriteByte(val))
    return false;
  return op.Execute();
}

bool SimpleMaster::ReadRegister(Address addr, uint8_t reg, uint8_t* val) {
  Operation op = CreateReadOp(addr, reg, "ReadRegister");
  if (!op.ready())
    return false;
  if (!op.Read(val, sizeof(*val)))
    return false;
  return op.Execute();
}

}  // namespace i2c
