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

uint8_t Create10BitHighAddressByte(uint16_t slave_addr,
                                   AddressWriter::Mode mode) {
  // High byte starts with 0b11110.
  return 0b11110000 | ((slave_addr >> 7) & 0b00000110) |
         (mode == AddressWriter::Mode::WRITE ? I2C_MASTER_WRITE
                                             : I2C_MASTER_READ);
}

uint8_t Create10BitLowAddressByte(uint16_t slave_addr) {
  return slave_addr & 0xff;
}

uint8_t Create7BitAddressByte(uint16_t slave_addr, AddressWriter::Mode mode) {
  configASSERT(!(slave_addr >> 9));  // Verify not using beyond 7 bits.
  return (slave_addr << 1) |
         (mode == AddressWriter::Mode::WRITE ? I2C_MASTER_WRITE
                                             : I2C_MASTER_READ);
}

}  // namespace

// static
esp_err_t AddressWriter::Write(i2c_cmd_handle_t cmd,
                               Address slave_addr,
                               Mode mode) {
  esp_err_t err;

  if (slave_addr.addr_size == Address::Size::bit7) {
    err = i2c_master_write_byte(
        cmd, Create7BitAddressByte(slave_addr.address, mode), ACK_CHECK_EN);
  } else {
    err = i2c_master_write_byte(
        cmd, Create10BitHighAddressByte(slave_addr.address, mode),
        ACK_CHECK_EN);
    if (err == ESP_OK) {
      err = i2c_master_write_byte(
          cmd, Create10BitLowAddressByte(slave_addr.address), ACK_CHECK_EN);
    }
  }
  return err;
}

}  // namespace i2c
