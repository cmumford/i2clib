/**
 * @section license License
 *
 * This file is subject to the terms and conditions defined in
 * file 'license.txt', which is part of this source code package.
 */

#include "status_esp.h"

namespace i2c {

Status ConvertEspStatus(esp_err_t error_code) {
  switch (error_code) {
    case ESP_OK:
      return Status::OK();
    case ESP_FAIL:
      return Status::Unknown();
    case ESP_ERR_NO_MEM:
      return Status::NoMemory();
    case ESP_ERR_INVALID_ARG:
      return Status::InvalidArgument();
    case ESP_ERR_INVALID_STATE:
      return Status::InvalidState();
    case ESP_ERR_INVALID_SIZE:
      return Status::InvalidSize();
    case ESP_ERR_NOT_FOUND:
      return Status::NotFound();
    case ESP_ERR_NOT_SUPPORTED:
      return Status::NotSupported();
    case ESP_ERR_TIMEOUT:
      return Status::Timeout();
    case ESP_ERR_INVALID_RESPONSE:
      return Status::InvalidResponse();
    case ESP_ERR_INVALID_CRC:
      return Status::InvalidCRC();
    case ESP_ERR_INVALID_VERSION:
      return Status::InvalidVersion();
    case ESP_ERR_INVALID_MAC:
      return Status::InvalidMAC();
    case ESP_ERR_NOT_FINISHED:
      return Status::NotFinished();
  }
  return Status::OK();
}

}  // namespace i2c