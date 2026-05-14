#pragma once
#include <map>
#include <string>
#include <memory>

namespace gpal::pnc::planning {

struct ParkingPathPlannerConfig {
  enum class ParkingType : uint8_t { VERTICAL = 0, PARALLEL = 1, ESCAPE = 2 };
};

struct ParkingPathPlannerProfile {
  enum class ParkingMode : uint8_t { PARK_IN = 0, PARK_OUT = 1 };
  struct ParkingFailureIndicators {
    double parking_max_time_consuming_ = 60.0;
    double parking_max_time_consuming() const { return parking_max_time_consuming_; }
  };
  ParkingFailureIndicators parking_failure_indicators_;
  const ParkingFailureIndicators& parking_failure_indicators() const { return parking_failure_indicators_; }
};

struct ParkingPathGeneratorProfile {};
struct PlannerProfile {};

}  // namespace gpal::pnc::planning
