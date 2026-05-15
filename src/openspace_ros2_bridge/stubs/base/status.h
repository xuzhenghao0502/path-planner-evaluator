#pragma once
#include <cstdint>

namespace base {

enum class Status : uint8_t {
  OK = 0,
  ERROR = 1,
  FAILED = 2,
};

}  // namespace base
