#include "bound_parser/longitudinal_bound_parser.h"

#include "spatiotemporal_abstract_module.h"

namespace gpal::pnc::planning {

bool LongitudinalBoundParser::init() {
  return true;
}
void LongitudinalBoundParser::reset() {
  // 重置逻辑
  speed_limit_process_.reset();
}

std::string LongitudinalBoundParser::id() const {
  return "LongitudinalBoundParser";
}

// 实现带有参数的run方法
bool LongitudinalBoundParser::run(DataManager& data_manager) {
  // 示例：从datamanager获取输入并处理
  // const auto& input_data = data_manager.inputData();
  // auto& output_boundary_info = data_manager.mutableBoundaryInfo();
  // profile_ = *data_manager.configInfo().longitudinal_bound_parser_profile;

  // if (!caculateVBoundary(data_manager)) {
  //   STLOG(E, "[LongitudinalBoundParser::run] caculateVBoundary failed");
  // }

  // if (!caculateSBoundary(data_manager)) {
  //   STLOG(E, "[LongitudinalBoundParser::run] caculateSBoundary failed");
  // }

  return true;
}

bool LongitudinalBoundParser::caculateVBoundary(DataManager& data_manager) {
  const auto& input_data = data_manager.inputData();
  auto& output_data = data_manager.mutableOutputData();
  auto& traj_bundary = data_manager.mutableBoundaryInfo().trajectory_boundary;
  if (!speed_limit_process_.init(profile_)) {
    return false;
  }
  auto speed_limits = speed_limit_process_.getSpeedLimit(
      input_data.target_ref_line_info, traj_bundary->referencePointsData(), input_data.console, input_data.chassis,
      input_data.vehicle_info, input_data.stage_state, input_data.env_road_cognition->getScenarioInfo(), *input_data.decision_result);
  output_data.max_speed_limit = speed_limit_process_.getMaxSpeedLimit();
  size_t num = traj_bundary->size();
  if (num != speed_limits.size()) {
    STLOG(E, "[LongitudinalBoundParser::caculateVBoundary] trajectory boundary size not equal to speed limit size");
    return false;
  }
  auto& v_soft_boundary = traj_bundary->mutableSoftSpeedBound().mutableOriginData();
  for (size_t i = 0; i < num; i++) {
    v_soft_boundary[i].clipUpper(speed_limits[i].speed_limit, speed_limits[i].id);
  }

  double overspeed_percent = 0.2;
  double speed_buffer = 5.0;
  double v_hard_upper_bound =
      fmax(output_data.max_speed_limit * (1 + overspeed_percent), input_data.chassis->Speed() + speed_buffer * KMH_MS);
  auto& v_hard_boundary = traj_bundary->mutableHardSpeedBound().mutableOriginData();
  for (size_t i = 0; i < num; i++) {
    v_hard_upper_bound = fmax(v_hard_upper_bound, speed_limits[i].speed_limit + 0.1);
    v_hard_boundary[i].clipUpper(v_hard_upper_bound);
  }
  return true;
}

bool LongitudinalBoundParser::caculateSBoundary(DataManager& data_manager) {
  const auto& input_data = data_manager.inputData();
  auto& time_related_boundary = data_manager.mutableBoundaryInfo().time_related_boundary;
  const double bound_buffer = 0.1;
  double s_soft_upper = 1000.0 + input_data.vehicle_info->start_point.path_pt().s();
  double s_hard_upper = 1000.0 + input_data.vehicle_info->start_point.path_pt().s();
  double s_soft_lower = input_data.vehicle_info->start_point.path_pt().s() - bound_buffer;
  double s_hard_lower = input_data.vehicle_info->start_point.path_pt().s() - bound_buffer;
  string s_upper_id = "";

  // add speed wall boundary
  if (!speed_wall_process_.init(profile_)) {
    return false;
  }
  std::vector<SpeedWall> speed_walls;
  speed_wall_process_.getSpeedWall(input_data.target_ref_line_info, input_data.chassis, input_data.vehicle_info,
                                   input_data.destination_remain_dis_info, *input_data.decision_result, &speed_walls);
  for (const auto& speed_wall : speed_walls) {
    double safe_distance = speed_wall.stop_distance;
    double speed_wall_s_soft_upper = fmax(kMathEpsilon, speed_wall.st_wall[0].y() - safe_distance)
                                     + input_data.vehicle_info->start_point.path_pt().s();
    if(speed_wall_s_soft_upper < s_soft_upper) {
      s_upper_id = speed_wall_id_map.at(speed_wall.type);
    }
    s_soft_upper = fmin(s_soft_upper, speed_wall_s_soft_upper);
    s_hard_upper = fmin(speed_wall.st_wall[0].y() + input_data.vehicle_info->start_point.path_pt().s(), s_hard_upper);
  }
  // block s boundary
  double block_s = data_manager.boundaryInfo().trajectory_boundary->boundaryBlockS();
  double block_s_safe_distance = profile_.block_s_safe_distance();
  bool is_gate_block_s = false;
  if (s_upper_id == "junction_stop" && abs(block_s - s_soft_upper) < profile_.block_s_ignore_range()) {
    is_gate_block_s = true;
  }
  if (profile_.enable_gate_fs_ignore() && is_gate_block_s) {
    STLOG(W, " block_s ignore because gate" );
  } else {
    if (block_s > 0.0) {
      if (block_s - block_s_safe_distance < s_soft_upper) {
        s_upper_id = "block";
      }
      s_soft_upper = fmin(s_soft_upper, block_s - block_s_safe_distance);
      s_hard_upper = fmin(s_hard_upper, block_s);
    }
  }
  if (profile_.enable_local_path_check()) {
    // local_path block s boundary
    if (!local_path_process_.init(profile_)) {
      return false;
    }
    auto local_path_collision_dis =
        local_path_process_.getLocalPathBlockDis(input_data.chassis, input_data.vehicle_state, input_data.localization,
                                                 input_data.freespace, *input_data.decision_result);
    data_manager.mutableOutputData().local_path = local_path_process_.getLocalPath();
    if (local_path_process_.getBlockFSInfo().s < local_path_process_.getBlockObsInfo().s) {
      data_manager.mutableOutputData().block_fs_info = local_path_process_.getBlockFSInfo();
    } else {
      data_manager.mutableOutputData().block_fs_info = local_path_process_.getBlockObsInfo();
    }
    // local_path bound 引入后会导致绕行功能异常
    // double local_path_block_safe_dis = 0.5;
    // double local_path_block_s = fmax(kMathEpsilon, local_path_collision_dis - local_path_block_safe_dis)
    //                                    + input_data.vehicle_info->start_point.path_pt().s();
    // if (local_path_collision_dis > 0.0) {
    //   if (local_path_block_s < s_soft_upper) {
    //     s_upper_id = "block_local";
    //   }
    //   s_soft_upper = fmin(s_soft_upper, local_path_block_s);
    //   s_hard_upper = fmin(s_hard_upper, local_path_block_s);
    // }
  }

  // bound check
  s_soft_upper = fmax(s_soft_upper, s_soft_lower + 2 * bound_buffer);
  s_hard_upper = fmax(s_hard_upper, s_hard_lower + 2 * bound_buffer);
  s_hard_upper = fmax(s_hard_upper, s_soft_upper);
  // update s_soft_boundary
  auto& s_soft_boundary = time_related_boundary->mutableSSoftBound().mutableOriginData();
  for (auto& s_soft_bound : s_soft_boundary) {
    s_soft_bound.clipUpper(s_soft_upper, s_upper_id);
    s_soft_bound.clipLower(s_soft_lower);
  }
  // update s_hard_boundary
  auto& s_hard_boundary = time_related_boundary->mutableSHardBound().mutableOriginData();
  for (auto& s_hard_bound : s_hard_boundary) {
    s_hard_bound.clipUpper(s_hard_upper, s_upper_id);
    s_hard_bound.clipLower(s_hard_lower);
  }

  return true;
}

REGIST_MODULE(LongitudinalBoundParser);
}  // namespace gpal::pnc::planning
