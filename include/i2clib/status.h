/**
 * @section license License
 *
 * This file is subject to the terms and conditions defined in
 * file 'license.txt', which is part of this source code package.
 */

#pragma once

namespace i2c {

class Status {
 public:
  enum class StatusCode {
    kOK,          // Successful.
    kUnknown,     // Unknown error.
    kOpNotReady,  // Operation not ready.
    kNoMemory,
    kInvalidArgument,
    kInvalidState,
    kInvalidSize,
    kNotFound,
    kNotSupported,
    kTimeout,
    kInvalidResponse,
    kInvalidCRC,
    kInvalidVersion,
    kInvalidMAC,
    kNotFinished,
  };

  [[nodiscard]] static Status OK() { return StatusCode::kOK; }
  [[nodiscard]] static Status Unknown() { return StatusCode::kUnknown; }
  [[nodiscard]] static Status OpNotReady() { return StatusCode::kOpNotReady; }
  [[nodiscard]] static Status NoMemory() { return StatusCode::kNoMemory; }
  [[nodiscard]] static Status InvalidArgument() {
    return StatusCode::kInvalidArgument;
  }
  [[nodiscard]] static Status InvalidState() {
    return StatusCode::kInvalidState;
  }
  [[nodiscard]] static Status InvalidSize() { return StatusCode::kInvalidSize; }
  [[nodiscard]] static Status NotFound() { return StatusCode::kNotFound; }
  [[nodiscard]] static Status NotSupported() {
    return StatusCode::kNotSupported;
  }
  [[nodiscard]] static Status Timeout() { return StatusCode::kTimeout; }
  [[nodiscard]] static Status InvalidResponse() {
    return StatusCode::kInvalidResponse;
  }
  [[nodiscard]] static Status InvalidCRC() { return StatusCode::kInvalidCRC; }
  [[nodiscard]] static Status InvalidVersion() {
    return StatusCode::kInvalidVersion;
  }
  [[nodiscard]] static Status InvalidMAC() { return StatusCode::kInvalidMAC; }
  [[nodiscard]] static Status NotFinished() { return StatusCode::kNotFinished; }

  Status(StatusCode code = StatusCode::kOK) : code_(code){};

  [[nodiscard]] constexpr bool ok() const { return code_ == StatusCode::kOK; }
  [[nodiscard]] constexpr bool IsUnknown() const {
    return code_ == StatusCode::kUnknown;
  }
  [[nodiscard]] constexpr bool IsOpNotReady() const {
    return code_ == StatusCode::kOpNotReady;
  }
  [[nodiscard]] constexpr bool IsNoMemory() const {
    return code_ == StatusCode::kNoMemory;
  }
  [[nodiscard]] constexpr bool IsInvalidArgument() const {
    return code_ == StatusCode::kInvalidArgument;
  }
  [[nodiscard]] constexpr bool IsInvalidState() const {
    return code_ == StatusCode::kInvalidState;
  }
  [[nodiscard]] constexpr bool IsInvalidSize() const {
    return code_ == StatusCode::kInvalidSize;
  }
  [[nodiscard]] constexpr bool IsNotFound() const {
    return code_ == StatusCode::kNotFound;
  }
  [[nodiscard]] constexpr bool IsNotSupported() const {
    return code_ == StatusCode::kNotSupported;
  }
  [[nodiscard]] constexpr bool IsTimeout() const {
    return code_ == StatusCode::kTimeout;
  }
  [[nodiscard]] constexpr bool IsInvalidResponse() const {
    return code_ == StatusCode::kInvalidResponse;
  }
  [[nodiscard]] constexpr bool IsInvalidCRC() const {
    return code_ == StatusCode::kInvalidCRC;
  }
  [[nodiscard]] constexpr bool IsInvalidVersion() const {
    return code_ == StatusCode::kInvalidVersion;
  }
  [[nodiscard]] constexpr bool IsInvalidMAC() const {
    return code_ == StatusCode::kInvalidMAC;
  }
  [[nodiscard]] constexpr bool IsNotFinished() const {
    return code_ == StatusCode::kNotFinished;
  }

 private:
  StatusCode code_;
};

}  // namespace i2c