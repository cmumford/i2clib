/**
 * @section license License
 *
 * This file is subject to the terms and conditions defined in
 * file 'license.txt', which is part of this source code package.
 */

#include <cstdint>

#include <esp_log.h>
#include <i2clib/address.h>

namespace i2c {

namespace {

constexpr bool ACK_CHECK_EN = true;  ///< I2C master will check ack from slave.
constexpr char TAG[] = "Addr";

uint8_t Create10BitHighAddressByte(uint16_t slave_addr, Direction dir) {
  // High byte starts with 0b11110.
  return 0b11110000 | ((slave_addr >> 7) & 0b00000110) |
         (dir == Direction::WRITE ? I2C_MASTER_WRITE : I2C_MASTER_READ);
}

uint8_t Create10BitLowAddressByte(uint16_t slave_addr) {
  return slave_addr & 0xff;
}

uint8_t Create7BitAddressByte(uint16_t slave_addr, Direction dir) {
  configASSERT(!(slave_addr >> 9));  // Verify not using beyond 7 bits.
  return (slave_addr << 1) |
         (dir == Direction::WRITE ? I2C_MASTER_WRITE : I2C_MASTER_READ);
}

}  // namespace

esp_err_t WriteAddress(i2c_cmd_handle_t cmd,
                       uint16_t slave_addr,
                       AddressMode addr_mode,
                       Direction dir) {
  esp_err_t err;

  if (addr_mode == AddressMode::bit7) {
    err = i2c_master_write_byte(cmd, Create7BitAddressByte(slave_addr, dir),
                                ACK_CHECK_EN);
  } else {
    err = i2c_master_write_byte(
        cmd, Create10BitHighAddressByte(slave_addr, dir), ACK_CHECK_EN);
    if (err == ESP_OK) {
      err = i2c_master_write_byte(cmd, Create10BitLowAddressByte(slave_addr),
                                  ACK_CHECK_EN);
    }
  }

  if (err != ESP_OK) {
    if (addr_mode == AddressMode::bit7) {
      ESP_LOGE(TAG, "Error writing 7-bit address 0x02%x", slave_addr);
    } else {
      ESP_LOGE(TAG, "Error writing 10-bit address 0x04%x", slave_addr);
    }
  }
  return err;
}

}  // namespace i2c
