#pragma once
#include <string>

namespace gpal::pnc::adas {

enum class SafetyWarningFeature {
  kNone = 0,
  kRcwState = 1,
  kRctaState = 2,
  kFctaState = 3,
};

enum class RearWarningState {
  kOff = 0,
  kStandby = 1,
  kWarning = 2,
  kBraking = 3,
  kFault = 4,
};

struct RctaResult {
  bool is_active = false;
  RearWarningState rcta_state = RearWarningState::kOff;
  std::string rcta_target_object_id = "";
  void clear() {
    is_active = false;
    rcta_state = RearWarningState::kOff;
    rcta_target_object_id = "";
  }
};

struct FctaResult {
  bool is_active = false;
  RearWarningState fcta_state = RearWarningState::kOff;
  std::string fcta_target_object_id = "";
  void clear() {
    is_active = false;
    fcta_state = RearWarningState::kOff;
    fcta_target_object_id = "";
  }
};

struct RcwResult {
  RctaResult rcta_result = {};
  FctaResult fcta_result = {};
  bool is_active = false;
  RearWarningState rcw_state = RearWarningState::kOff;
  std::string rcw_target_object_id = "";
  void clear() {
    rcta_result.clear();
    fcta_result.clear();
    is_active = false;
    rcw_state = RearWarningState::kOff;
    rcw_target_object_id = "";
  }
};

}  // namespace gpal::pnc::adas