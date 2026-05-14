#pragma once
#include <cstdint>

namespace gpal::pnc::planning {

struct StopReason {
  enum class Type : uint8_t { NONE = 0, DESTINATION = 1, OBSTACLE = 2 };
  Type type = Type::NONE;
};

}  // namespace gpal::pnc::planning
