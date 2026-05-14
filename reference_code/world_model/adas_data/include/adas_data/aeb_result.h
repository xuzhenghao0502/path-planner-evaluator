/*
 * @Author: 王健 jianwang@geometricalpal.com
 * @Date: 2025-04-23 10:32:46
 * @LastEditors: 王健 jianwang@geometricalpal.com
 * @LastEditTime: 2025-05-08 15:31:27
 * @FilePath: /eka—x86/eka/app/pnc/world_model/adas_data/include/adas_data/aeb_result.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include "adas_common_data.h"
#include "obstacle/obstacle.h"

namespace gpal::pnc::adas {

enum class AebState { kOff = 0, kOn = 1, kStandby = 2, kInhibit = 3, kActivate = 4, kFault = 5 };

enum class FcwLevel : uint8_t { kNoWarning = 0, kLevel1 = 1, kLevel2 = 2 };

struct ObjectInfoRawInfomation {
  int32_t obj_id = -1;
  bool obj_vaild = false;
  bool obj_in_path = false;
  float obj_utility = 0.0;
  float obj_posnlat_abs = 0.0;
};
struct PropertyInfomation {
  int32_t obj_id = -1;
  gpal::proto::PerceptionObstacle_ObstacleType obj_type =
      gpal::proto::PerceptionObstacle_ObstacleType::PerceptionObstacle_ObstacleType_kTypeBicycle;
  float p_lgt = 0.0;
  float p_lat = 0.0;
  float spd = 0.0;
  float v_lgt = 0.0;
  float v_lat = 0.0;
  float a_lgt = 0.0;
  float a_lat = 0.0;
  float a = 0.0;
  float curvature = 0.0;
  float rear_p_lgt = 0.0;
  float rear_p_lat = 0.0;
  float ang_dir = 0.0;
};
struct MotionTypeInfomation {
  bool stationary = false;
  bool moving_oncoming = false;
  bool parallell_vehicle_long_predict = false;
  bool parallell_vehicle_short_predict = false;
  bool object_is_vehicle = false;
  bool object_is_motor_vehicle = false;
  bool curved_motion = false;
  bool side_is_closest = false;
  float abs_heading = 0.0;
};
struct BoundingBoxInfomation {
  float length_side_lat = 0.0;
  float length_side_lgt = 0.0;
  float sin_rotation = 0.0;
  float cos_rotation = 0.0;
};
struct ObjDefaultPredictInfomation {
  float p_lgt;
  float p_lat;
  float heading;
  bool host_left_side_close;
  float dist_from_left;
  float dist_from_right;
};
struct HostDefaultPredictInfomation {
  float pred_heading;
  float pred_pos_lgt;
  float pred_pos_lat;
};
struct CurrentInPathInfomation {
  int sum_of_sign;
  float closest_corners_distance;
};
struct ShortPredTimeInfomation {
  float ttr = 0.0;
  float ttm = 0.0;
  bool ttm_selected = false;
  bool inpath = false;
  bool inpath_ttr_predict_position = false;
  bool inpath_ttm_predict_position = false;
  bool inpath_current_position = false;
  bool use_prediction = false;
  ObjDefaultPredictInfomation obj_default_predict_info;
  HostDefaultPredictInfomation host_default_predict_info;
  CurrentInPathInfomation current_in_path_info;
  float modified_ttr = 0.0;
};
struct OffsetsInfomation {
  float offs_lgt_long_predict = 0.0;
  float offs_lat_long_predict = 0.0;
  float offs_lgt_short_predict = 0.0;
  float offs_lat_short_predict = 0.0;
  float offs_lat_manoeuvre = 0.0;
  float offs_lat_inpath_primary_target = 0.0;
  float offs_lat_inpath_close_edge = 0.0;
  float offs_lat_inpath_far_edge = 0.0;
  float offs_lat_intersection = 0.0;
  float offs_lat_multiple_target = 0.0;
};
struct ObjDataInfomation {
  PropertyInfomation porperty;
  MotionTypeInfomation motion_type;
  BoundingBoxInfomation bounding_box;
  ShortPredTimeInfomation short_pred_time;
  OffsetsInfomation offsets;
};
struct EgoPathPointInfomation {
  float x = 0.0;
  float y = 0.0;
  float curvature = 0.0;
  float heading = 0.0;
};
struct EgoInfomation {
  float v_lgt = 0.0;
  float v_lat = 0.0;
  float a_lgt = 0.0;
  float a_lat = 0.0;
  float yaw_rate = 0.0;
  float curvature = 0.0;
  float curve_radius = 0.0;
  float steer_angle = 0.0;
  float steer_angle_spd = 0.0;
  EgoPathPointInfomation ego_path_info[40];
};
struct PrimaryTargetInfomation {
  int32_t obj_id = 0;
  gpal::proto::PerceptionObstacle_ObstacleType obj_type =
      gpal::proto::PerceptionObstacle_ObstacleType::PerceptionObstacle_ObstacleType_kTypeBicycle;
  bool new_ta_found = false;
  float posn_lgt = 0.0;
  float posn_lat = 0.0;
  float v_lgt = 0.0;
  float v_lat = 0.0;
  float a_lgt = 0.0;
  float a_lat = 0.0;
  float ttc_val = 0.0;
  float ang_dir = 0.0;
};
struct MotionRequestInfomation {
  float a_lat_request = 0.0;
  uint8_t a_lat_quality = 0;
  float a_neg_lgt_request = 0.0;
  float jerk_neg_lgt_request = 0.0;
  float crvt_request_left = 0.0;
  float crvt_rate_request_left = 0.0;
  float crvt_request_right = 0.0;
  float crvt_rate_request_right = 0.0;
  float a_lat_request_warn = 0.0;
  float a_neg_lgt_request_warn = 0.0;
};
struct BrakeInfomation {
  float curvature_abs_threshold = 0.0;
  float curvature_rate_abs_threshold = 0.0;
  float a_lat_threthold = 0.0;
  float a_neg_lgt_threthold = 0.0;
  float jerk_longitudinal_negative_minimum = 0.0;
  bool steering_threat = false;
  bool braking_threat = false;
  bool scenario_threat = false;
  bool a_lgt_threat = false;
};
struct WarningInfomation {
  bool btn = false;
  bool stn = false;
  bool obj_valid = false;
  uint8_t warning_type = 0;
};
struct ScenarioInfomation {
  float object_crvt_lat_posn = 0.0;
  float object_crvt_lgt_posn = 0.0;
  float object_crvt_heading = 0.0;
  float object_crvt_speed = 0.0;
  float object_yaw_angle = 0.0;
  int primary_object_count = 0;
  float first_obj_posn_lgt = 0.0;
  uint32_t scenario_flag = 0;
  uint32_t scenario_check_result = 0;
  uint32_t scenario_host_state = 0;
  uint32_t scenario_inhibit_condition = 0;
};
struct RiskConditionInfomation {
  MotionRequestInfomation motion_request;
  BrakeInfomation fullbrake_threat;
  BrakeInfomation prebrake_threat;
  WarningInfomation warning_threat;
  ScenarioInfomation scenario_info;
};
struct InhibitInfomation {
  uint32_t aeb_collision_inhibit_flag = 0;
  uint32_t aeb_sm_inhibit_flag = 0;
  uint32_t fcw_collision_inhibit_flag = 0;
  uint32_t fcw_sm_inhibit_flag = 0;
  uint32_t aeb_collision_deactive_flag = 0;
  uint32_t aeb_sm_deactive_flag = 0;
  uint32_t fcw_collision_deactive_flag = 0;
  uint32_t fcw_sm_deactive_flag = 0;
};
struct AebOutputResult {
  bool is_aeb_active = false;
  bool is_fcw_active = false;
  uint32_t aeb_mode = 0;
  uint32_t fcw_mode = 0;
  uint32_t aeb_decel = 0;
};

struct DebugInfomation {
  std::vector<std::shared_ptr<ObjectInfoRawInfomation>> obj_info_raw;
  std::vector<std::shared_ptr<ObjDataInfomation>> obj_data_info;
  EgoInfomation ego_info;
  PrimaryTargetInfomation primary_target_intv_info;
  PrimaryTargetInfomation primary_target_warn_info;
  RiskConditionInfomation risk_condition_info;
  InhibitInfomation inhibit_info;
  AebOutputResult aeb_output_result;
};

struct AebResult {
  AebState aeb_state{};
  float deceleration_value = 0.0;
  FcwLevel fcw_level{};
  DebugInfomation debug_infomation;
};
}  // namespace gpal::pnc::adas