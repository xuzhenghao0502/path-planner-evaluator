#pragma once

#include <vector>
#include <string>

namespace gpal::pnc::adas {

enum class PdcWarningState {
  kOff = 0,
  kStandby = 1,
  kActive = 2,
  kFault = 3,
};

enum class DistanceLevel {
  kSuperFar = 0,
  kFar = 1,
  kMedium = 2,
  kNear = 3,
};

struct PdcResult {
  bool is_active = false;
  PdcWarningState warning_state = PdcWarningState::kOff;
  std::vector<float> distances;
  std::vector<std::string> target_object_ids;
  std::vector<DistanceLevel> distance_levels;
  void clear() {
    is_active = false;
    warning_state = PdcWarningState::kOff;
    distances.clear();
    target_object_ids.clear();
    distance_levels.clear();
  }
};

}  // namespace gpal::pnc::adas