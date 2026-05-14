#pragma once

#include "adas_common_data.h"

namespace gpal::pnc::adas {

enum class FeatureState {
  kNone = 0,
  kLdwState = 1,
  kLkaState = 2,
};

enum class SensitivityLevel {
  kInvalidLevel = 0,
  kAdvancedLevel = 1,
  kStandardLevel = 2,
  kDelayedLevel = 3,
};

enum class LkaState {
  kOff = 0,
  kReady = 1,
  kOverride = 2,
  kPendingActive = 3,
  kLeftActive = 4,
  kRightActive = 5,
  kFault = 6,
};

struct LdwResult {
  bool is_left_active;
  bool is_right_active;
  uint32_t ldw_target_lane_id;
  LkaState ldw_state;
  void clear() {
    is_left_active = false;
    is_right_active = false;
    ldw_target_lane_id = 0;
    ldw_state = LkaState::kOff;
  }
};

struct LateralControlDebug {
  int32_t solve_status_ocp;
  float consumption_time;
  float lateral_error;
  float lateral_error_rate;
  float heading_error;
  float heading_error_rate;
  proto::TrajectoryPoint match_point;
  proto::TrajectoryPoint ref_point;
  void clear() {
    solve_status_ocp = 0;
    consumption_time = 0.0;
    lateral_error = 0.0;
    lateral_error_rate = 0.0;
    heading_error = 0.0;
    heading_error_rate = 0.0;
    match_point.Clear();
    ref_point.Clear();
  }
};

struct LkaResult {
  LdwResult ldw_result;
  LateralControlDebug lat_control_debug;
  bool is_left_active;
  bool is_right_active;
  uint32_t lka_target_lane_id;
  uint8_t steering_angle_status;
  double steering_angle;
  LkaState lka_state;
  proto::Trajectory trajectory;
  void clear() {
    ldw_result.clear();
    lat_control_debug.clear();
    is_left_active = false;
    is_right_active = false;
    lka_target_lane_id = 0;
    steering_angle_status = 0;
    steering_angle = 0.0;
    lka_state = LkaState::kOff;
    trajectory.Clear();
  }
};

}  // namespace gpal::pnc::adas