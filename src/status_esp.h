/**
 * @section license License
 *
 * This file is subject to the terms and conditions defined in
 * file 'license.txt', which is part of this source code package.
 */

#pragma once

#include "i2clib/status.h"

#include <esp_err.h>

namespace i2c {

Status ConvertEspStatus(esp_err_t error_code);

}  // namespace i2c