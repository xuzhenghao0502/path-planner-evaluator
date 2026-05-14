#pragma once
#include <string>

namespace gpal::pnc::adas {

enum class WarningFeatureState {
  kNone = 0,
  kBsdState = 1,
  kDowState = 2,
  kLcaState = 3,
};

enum class WarningState {
  kOff = 0,
  kStandby = 1,
  kLeftWarningLevel1 = 2,
  kLeftWarningLevel2 = 3,
  kRightWarningLevel1 = 4,
  kRightWarningLevel2 = 5,
  kBothWarning = 6,
  kFault = 7,
};

struct Spots {
  double max_x = 0.0;
  double min_x = 0.0;
  double max_y = 0.0;
  double min_y = 0.0;
};

struct DowResult {
  bool is_left_active = false;
  bool is_right_active = false;
  WarningState dow_state = WarningState::kOff;
  std::string dow_target_object_id = "";
  void clear() {
    is_left_active = false;
    is_right_active = false;
    dow_state = WarningState::kOff;
    dow_target_object_id = "";
  }
};

struct LcaResult {
  bool is_left_active = false;
  bool is_right_active = false;
  WarningState lca_state = WarningState::kOff;
  std::string lca_target_object_id = "";
  void clear() {
    is_left_active = false;
    is_right_active = false;
    lca_state = WarningState::kOff;
    lca_target_object_id = "";
  }
};

struct BsdResult {
  DowResult dow_result = {};
  LcaResult lca_result = {};
  bool is_left_active = false;
  bool is_right_active = false;
  WarningState bsd_state = WarningState::kOff;
  std::string bsd_target_object_id = "";
  void clear() {
    dow_result.clear();
    lca_result.clear();
    is_left_active = false;
    is_right_active = false;
    bsd_state = WarningState::kOff;
    bsd_target_object_id = "";
  }
};

}  // namespace gpal::pnc::adas