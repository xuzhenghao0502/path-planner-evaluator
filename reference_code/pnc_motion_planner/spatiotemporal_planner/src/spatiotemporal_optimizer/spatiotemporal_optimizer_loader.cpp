#include "spatiotemporal_optimizer/spatiotemporal_optimizer_loader.h"

namespace gpal::pnc::planning {

std::string SpatiotemporalOptimizerLoader::id() const {
  return "SpatiotemporalOptimizerLoader";
}

bool SpatiotemporalOptimizerLoader::init() {
  config_manager_ = Singleton<ConfigManager>::get_instance();
  CHECK_NOTNULL(config_manager_);
  vehicle_param_ = config_manager_->vehicle_config().vehicle_param();
  return true;
}

void SpatiotemporalOptimizerLoader::reset() {
  optimizer_parameters_.clear();
  interaction_infos_.clear();
  feature_s_points_.clear();
  feature_v_points_.clear();
  conflict_areas_.clear();
  need_stop_ = false;
  init_guess_.clear();
  target_obstacle_data_.clear();
  destination_remain_dis_info_ = {false, 10000.0};  ///< FIRST: 是否有终点信息, SECOND: 剩余距离信息
  destination_point_ = TrajectoryPt();              ///< 目的地参考点
  env_road_cognition_ = EnvRoadCognition();         ///< 环境道路认知信息
}

bool SpatiotemporalOptimizerLoader::run(SpatiotemporalPlannerDataManager& data_manager) {
  // ... 获取输入数据 ...
  const auto& input_data = data_manager.inputData();
  const auto& target_ref_line_info = input_data.target_ref_line_info;
  const auto& decision_lateral_boundary = input_data.decision_result->getLateralBoundaryDecision();
  const auto& boundary_info = data_manager.boundaryInfo();
  auto& objects_info = data_manager.mutableObjectsInfo();
  auto& optimizer_info = data_manager.mutableOptimizerInfo();
  auto& optimizer_config = data_manager.mutableConfigInfo().spatiotemporal_optimizer_profile;
  auto& output_data = data_manager.mutableOutputData();
  const auto& target_point_info = input_data.decision_result->getRefTrajInfo()->target_point;
  init_guess_.clear();
  debug_info_.clear();
  target_obstacle_data_.clear();
  localization_ = *input_data.localization;
  thw_ = getThw(input_data.console->followingDistanceLevel());
  destination_remain_dis_info_ = input_data.destination_remain_dis_info;
  destination_point_ = input_data.destination_point;
  env_road_cognition_ = *input_data.env_road_cognition;

  optimizer_config_ = *data_manager.configInfo().spatiotemporal_optimizer_profile;
  target_ref_line_.reset(new ReferenceLine(input_data.target_ref_line_info->ref_line()));
  current_ref_line_.reset(new ReferenceLine(input_data.current_ref_line_info->ref_line()));
  xy_planning_start_point_ = input_data.vehicle_info->start_point;
  sl_planning_start_point_ = input_data.vehicle_info->sl_info;
  driven_sl_info_ = input_data.vehicle_info->driven_sl_info;
  behavior_ = input_data.decision_result->getCurrFsmState();
  target_obstacle_data_ = data_manager.objectsInfo().target_objects_info;
  scenario_info_ = input_data.env_road_cognition->getScenarioInfo();
  auto& current_driving_scenario = optimizer_info.current_driving_scenario;
  // todo: 模型size,时间步长初始化
  const auto time_grid = data_manager.gridsInfo().time_grid_info;
  dt_ = time_grid.step;
  horizon_ = time_grid.horizon;
  N_ = size_t(horizon_ / dt_) + 1;
  optimizer_parameters_.assign(N_, optimizer_parameter_);
  interaction_infos_.assign(N_, InteractionInfo());
  feature_s_points_.assign(N_, std::make_pair(false, 0.0));
  feature_v_points_.assign(N_, std::make_pair(false, 0.0));
  need_stop_ = false;
  // todo: 初始化项
  optimizer_info.lane_keep_start_s = calcLaneKeepStartS(target_point_info);

  // todo: 初解解析
  if (!checkInitGuessValid(input_data.decision_result)) {
    STLOG(E, "[SpatiotemporalOcpOptimizer::run] checkInitGuessValid failed.");
    debug_info_ += "checkInitGuessValid faild\n";
    return false;
  }

  // todo: cost目标优化
  if (optimizer_config_.enable_conflict_area()) {
    constructConflictAreaTarget();
    for (auto conflict_area : conflict_areas_) {
      std::cout << " conflict_area  " << conflict_area.conflict_object_id << " start_index "
                << conflict_area.start_index << " end_index " << conflict_area.end_index << " start_s "
                << conflict_area.start_s << " end_s " << conflict_area.end_s << " is_conflict "
                << conflict_area.is_conflict << " conflict_area_type "
                << static_cast<int>(conflict_area.conflict_area_type) << std::endl;
    }
  }
  constructFollowTrajectoryTarget(boundary_info, target_point_info);
  currentDrivingScenario();
  updateObjectiveTargetParameters(feature_s_points_, need_stop_, optimizer_parameters_);
  updateLateralOffset(boundary_info, target_obstacle_data_, optimizer_parameters_);
  constructTerminalTarget();
  CalculateSteerFromGuess(init_guess_, vehicle_param_.wheel_base());
  CalculateControlInputs(init_guess_, dt_);

  // 输出
  outputProcess(data_manager);

  return true;
}

double SpatiotemporalOptimizerLoader::calcLaneKeepStartS(const Decision::RefTargetPoint& target_point_info) {
  double max_s = std::max<float>(0.0, boundary_length_);

  double res_s = 0.0;
  double l0 = sl_planning_start_point_.second[0];
  double dl0 = sl_planning_start_point_.second[1];
  double whole_time = optimizer_config_.lane_keep_start_s_param().lane_keep_start_time();
  double upper_time = optimizer_config_.lane_keep_start_s_param().lane_change_time_upper(),
         lower_time = optimizer_config_.lane_keep_start_s_param().lane_change_time_lower();
  double upper_v = optimizer_config_.lane_keep_start_s_param().lane_change_velocity_upper(),
         lower_v = optimizer_config_.lane_keep_start_s_param().lane_change_velocity_lower();
  double current_l = driven_sl_info_.second[0];
  // 换道中.
  if (behavior_ == FsmState::LEFT_CHANGE || behavior_ == FsmState::RIGHT_CHANGE) {
    if (optimizer_config_.enable_use_decision_lane_change_point()) {
      res_s = std::min(target_point_info.s - sl_planning_start_point_.first.at(0), max_s);
      debug_info_ += fmt::format(
          "\nuse decision lane change point  , lane_keep_start_s: {:.2f} , "
          "target_s: {:.2f}, start_s: {:.2f} \n",
          res_s, target_point_info.s, sl_planning_start_point_.first.at(0));
    } else {
      double init_l = 3.75;  // 使用标准车道宽度

      // 换道时间. 30km/h ~ 80km/h 对应 3.0s ~ 5.5s
      if (optimizer_config_.lane_keep_start_s_param().enable_adaptive_lane_change_time()) {
        whole_time = (std::clamp(xy_planning_start_point_.v(), lower_v, upper_v) - lower_v) / (upper_v - lower_v)
                         * (upper_time - lower_time)
                     + lower_time;
      }
      double l_dot = fabs(init_l) / whole_time;
      double res_l = std::fmax(fabs(l0), fabs(fabs(init_l) - fabs(current_l)));
      double time_left = res_l / l_dot;
      res_s = std::min(time_left * xy_planning_start_point_.v(), max_s);
      debug_info_ +=
          fmt::format("\nbehavior: {}, lane_keep_start_s: {:.2f}, time: {:.2f},  lat_acc: {:.2f}\n", int(behavior_),
                      res_s, whole_time, optimizer_config_.lane_keep_start_s_param().lane_keep_lat_acc());
    }

  } else {
    // 其他场景.
    double v0 = fabs(xy_planning_start_point_.v() * dl0);  // lateral speed
    double a = optimizer_config_.lane_keep_start_s_param().lane_keep_lat_acc();
    double t0 = 0.0, t1 = 0.0, t = 0.0;
    if ((l0 > 0 && dl0 < 0) || (l0 < 0 && dl0 > 0)) {
      t0 = 0.0;
      double v1 = v0 * v0 + 2 * a * fabs(l0);
      t1 = (sqrt(v1) - abs(v0)) / a;
    } else {
      t0 = 0.0;
      double l1 = fabs(l0) + 0.5 * a * t0 * t0;
      t1 = sqrt(2 * l1 / a);
    }
    t = t0 + t1;
    res_s = std::min(max_s, t * std::fabs(xy_planning_start_point_.v()));
    debug_info_ +=
        fmt::format("\nbehavior: {}, lane_keep_start_s: {:.2f}, time: {:.2f},  lat_acc: {:.2f}\n", int(behavior_),
                    res_s, whole_time, optimizer_config_.lane_keep_start_s_param().lane_keep_lat_acc());
  }

  return sl_planning_start_point_.first.at(0) + res_s;
}

void SpatiotemporalOptimizerLoader::currentDrivingScenario() {
  current_driving_scenario_[DrivingScenarios::END] = false;
  current_driving_scenario_[DrivingScenarios::JUNCTION] = false;
  current_driving_scenario_[DrivingScenarios::JUNCTION_FORWARD] = false;
  current_driving_scenario_[DrivingScenarios::MERGING] = false;
  current_driving_scenario_[DrivingScenarios::LANE_CHANGE] = false;
  current_driving_scenario_[DrivingScenarios::EMERGENCY_BRAKE] = false;
  current_driving_scenario_[DrivingScenarios::VEHICLE_START] = false;
  current_driving_scenario_[DrivingScenarios::GATE] = false;

  // for destination
  // 优化目标的s与终点s的偏差一定范围内进入终点模式
  bool is_destination_reachable =
      destination_remain_dis_info_.first
      && std::abs(destination_point_.path_pt().s() - optimizer_parameters_.back()["s_coarse"].second)
             < optimizer_config_.destination_reachable_thres();
  // 终点位姿
  auto destination_point = target_ref_line_->getDestinationData().destination_point;
  auto lateral_error = destination_point.offset();
  auto match_point = target_ref_line_->getReferencePoint(destination_point.local_s());
  float heading_error = gpal::pnc::planning::math::NormalizeAngle(match_point.heading() - destination_point.heading());
  if (std::abs(lateral_error) > optimizer_config_.end_lateral_error_threshold()
      || std::abs(heading_error) > optimizer_config_.end_heading_error_threshold()) {
    is_destination_reachable = false;
  }
  // 闸机
  auto gates = target_ref_line_->getGatesFromSRange(sl_planning_start_point_.first.at(0), 50.0, 0.0);
  if (!gates.empty() && gates.front().s < destination_point.local_s()) {
    is_destination_reachable = false;
  }

  // for junction
  bool is_in_junction = false;
  for (auto& junction_info : env_road_cognition_.getScenarioInfo().junctions_info()) {
    if (junction_info.position_relation() == proto::road_cognition::JunctionInfo::kInJunction) {
      is_in_junction = true;
      break;
    }
  }

  // for junction forward
  bool is_in_junction_forward = false;
  if (is_in_junction) {
    std::cout << " is_in_junction_forward "
              << int(target_ref_line_->getDirectionFromS(sl_planning_start_point_.first.at(0)).direction) << std::endl;
    if (target_ref_line_->getDirectionFromS(sl_planning_start_point_.first.at(0)).direction
        == DrivingDirection::kDirectionForwardOnly) {
      is_in_junction_forward = true;
    }
  }

  // for merge
  bool is_in_merge = false;
  for (auto& ego_scenario_type : env_road_cognition_.getScenarioInfo().ego_scenario_types()) {
    if (ego_scenario_type == proto::road_cognition::ScenarioInfo::kScenarioNearMerge) {
      is_in_merge = true;
      break;
    }
  }
  // for emergency break
  bool is_emergency_break = false;
  double acc_average = 0.0;
  double s_start = optimizer_parameters_.front()["s_coarse"].second;
  double v_start = optimizer_parameters_.front()["v_coarse"].second;
  for (int i = i; i < N_; ++i) {
    double acc_temp =
        2 * (optimizer_parameters_.at(i)["s_coarse"].second - s_start - v_start * (i * dt_)) / ((i * dt_) * (i * dt_));
    acc_average = std::max(acc_average, std::fabs(acc_temp));
    if (acc_average > optimizer_config_.emergency_break_acc_thres()) {
      is_emergency_break = true;
      break;
    }
  }

  // for vehicle start
  bool is_vehicle_start = false;
  double current_speed = optimizer_parameters_.front()["v_coarse"].second;
  double start_acc_threshold = optimizer_config_.vehicle_start_acc_thres();
  if (current_speed < 3.0) {  // 速度低于3m/s认为是起步状态
    double start_acc = 0.0;
    if (N_ > 1) {
      double v_next = optimizer_parameters_.at(1)["v_coarse"].second;
      start_acc = (v_next - current_speed) / dt_;
    }
    if (start_acc > start_acc_threshold || (current_speed < 0.1 && start_acc > 0.1)) {
      is_vehicle_start = true;
    }
  }

  // for gate
  bool is_in_gate = false;
  if (nearest_obs_id_ == "junction_stop" && nearest_obs_s_ < optimizer_config_.gate_stop_thres()) {
    is_in_gate = true;
  }

  if (is_destination_reachable) {
    current_driving_scenario_[DrivingScenarios::END] = true;
  }
  if (is_in_junction) {
    current_driving_scenario_[DrivingScenarios::JUNCTION] = true;
  }
  if (is_in_junction_forward) {
    current_driving_scenario_[DrivingScenarios::JUNCTION_FORWARD] = true;
  }
  if (is_in_merge) {
    current_driving_scenario_[DrivingScenarios::MERGING] = true;
  }
  if (behavior_ == FsmState::LEFT_CHANGE || behavior_ == FsmState::RIGHT_CHANGE || behavior_ == FsmState::LEFT_RETURN
      || behavior_ == FsmState::RIGHT_RETURN) {
    current_driving_scenario_[DrivingScenarios::LANE_CHANGE] = true;
  }

  if (is_emergency_break) {
    current_driving_scenario_[DrivingScenarios::EMERGENCY_BRAKE] = true;
  }
  if (is_vehicle_start) {
    current_driving_scenario_[DrivingScenarios::VEHICLE_START] = true;
  }
  if (is_in_gate) {
    current_driving_scenario_[DrivingScenarios::GATE] = true;
  }
  return;
}

void SpatiotemporalOptimizerLoader::constructTerminalTarget() {
  double mean_acc =
      (optimizer_parameters_.back()["v_coarse"].second - optimizer_parameters_.front()["v_coarse"].second) / horizon_;
  double terminal_v = optimizer_parameters_.back()["v_coarse"].second;
  if (mean_acc < -0.5 || terminal_v < 1e-2) {
    optimizer_parameters_.back()["s_terminal"] = {true, optimizer_parameters_.back()["s_coarse"].second};
    optimizer_parameters_.back()["v_terminal"] = {true, optimizer_parameters_.back()["v_coarse"].second};
  } else {
    optimizer_parameters_.back()["s_terminal"] = {false, 0.0};
    optimizer_parameters_.back()["v_terminal"] = {false, 0.0};
  }
  if (current_driving_scenario_[DrivingScenarios::END]) {
    optimizer_parameters_.back()["s_terminal"] = {true, optimizer_parameters_.back()["s_coarse"].second};
    optimizer_parameters_.back()["v_terminal"] = {true, optimizer_parameters_.back()["v_coarse"].second};
    double cofficient = 1.0;
    if (optimizer_parameters_.back()["s_coarse"].second
        > destination_point_.path_pt().s() - optimizer_config_.destination_reached_forhead_buffer()) {
      cofficient = 1.0;
    } else {
      double numerator = std::max(
          optimizer_parameters_.back()["s_coarse"].second - optimizer_parameters_.front()["s_coarse"].second, 0.0);
      double denominator =
          std::max(destination_point_.path_pt().s() - optimizer_config_.destination_reached_forhead_buffer()
                       - optimizer_parameters_.front()["s_coarse"].second,
                   1e-3);
      cofficient = std::clamp(numerator / denominator, 0.0, 1.0);
    }
    double terminal_l = cofficient * destination_point_.path_pt().l();
    optimizer_parameters_.back()["l_terminal"] = {true, terminal_l};
    double theta_terminal =
        spatiotemporal_functions::getUnifySpaceHeading(init_guess_.back().theta, destination_point_.path_pt().theta());
    optimizer_parameters_.back()["theta_terminal"] = {true, theta_terminal};

  } else if (current_driving_scenario_[DrivingScenarios::GATE]) {
    optimizer_parameters_.back()["s_terminal"] = {true, sl_planning_start_point_.first.at(0) + nearest_obs_s_};
    optimizer_parameters_.back()["v_terminal"] = {true, 0.0};
    optimizer_parameters_.back()["l_terminal"] = {true, 0.0};
    auto gate_point = target_ref_line_->getReferencePoint(sl_planning_start_point_.first.at(0) + nearest_obs_s_);
    double theta_terminal =
        spatiotemporal_functions::getUnifySpaceHeading(init_guess_.back().theta, gate_point.heading());
    optimizer_parameters_.back()["theta_terminal"] = {true, theta_terminal};
  } else if (current_driving_scenario_[DrivingScenarios::JUNCTION]) {
    optimizer_parameters_.back()["l_terminal"] = {true, 0.0};
    optimizer_parameters_.back()["theta_terminal"] = {false, 0.0};
  } else {
    optimizer_parameters_.back()["l_terminal"] = {false, 0.0};
    optimizer_parameters_.back()["theta_terminal"] = {false, 0.0};
  }

  return;
}
void SpatiotemporalOptimizerLoader::constructConflictAreaTarget() {
  conflict_areas_.clear();

  for (const auto& obstacle_traj : target_obstacle_data_) {
    if (obstacle_traj.empty())
      continue;  // 增加空轨迹保护
    // 暂时只考虑overtake障碍物
    bool is_valid = std::any_of(obstacle_traj.begin(), obstacle_traj.end(), [](const ObjectInfo& obj) {
      return obj.longitudinal_od_tag == LongitudinalOdTag::OVERTAKE
             || obj.longitudinal_od_tag == LongitudinalOdTag::YIELD;
    });

    // todo: 非博弈对象不参与冲突区域构建
    if (!is_valid) {
      continue;
    }
    ConflictArea temp_conflict_area;
    for (size_t i = 0; i < N_; i++) {
      if (obstacle_traj[i].object_game_type == ObjectGameType::CROSS_GAME
          || obstacle_traj[i].object_game_type == ObjectGameType::MERGE_GAME) {
        // 交叉博弈对象参与冲突区域构建
        bool is_cross_lane = obstacle_traj[i].end_l * obstacle_traj[i].start_l > 0.0 ? false : true;

        if (temp_conflict_area.is_conflict == true
            && (((std::abs(obstacle_traj[i].end_l) >= 1.6 && std::abs(obstacle_traj[i].start_l) >= 1.6)
                 && !is_cross_lane)
                || i > temp_conflict_area.start_index
                           + optimizer_config_.conflict_end_time() / dt_)) {  // 超过3s则认为冲突结束
          temp_conflict_area.end_index = i;
          temp_conflict_area.end_s = std::max(obstacle_traj[i].start_s, obstacle_traj[i].end_s);
          break;
        }

        // 当前考虑NOA场景常规车辆的正常速度行驶
        if ((((std::abs(obstacle_traj[i].start_l) < 1.5 || std::abs(obstacle_traj[i].end_l) < 1.5) && !is_cross_lane)
             || is_cross_lane)
            && !temp_conflict_area.is_conflict) {
          temp_conflict_area.is_conflict = true;
          temp_conflict_area.conflict_object_id = obstacle_traj[i].id;
          temp_conflict_area.start_index = i;
          temp_conflict_area.start_s = std::min(obstacle_traj[i].start_s, obstacle_traj[i].end_s);
          if (obstacle_traj[i].longitudinal_od_tag == LongitudinalOdTag::OVERTAKE) {
            temp_conflict_area.conflict_area_type = ConflictAreaType::OVERTAKE;
          } else if (obstacle_traj[i].longitudinal_od_tag == LongitudinalOdTag::YIELD) {
            temp_conflict_area.conflict_area_type = ConflictAreaType::YIELD;
          }
        }
      }
    }
    if (temp_conflict_area.is_conflict == true && temp_conflict_area.end_index == 0) {
      temp_conflict_area.end_index = N_ - 1;
      temp_conflict_area.end_s = std::max(obstacle_traj[N_ - 1].start_s, obstacle_traj[N_ - 1].end_s);
    }
    if (temp_conflict_area.is_conflict == true) {
      conflict_areas_.emplace_back(temp_conflict_area);
    }
  }
}
void SpatiotemporalOptimizerLoader::constructFollowTrajectoryTarget(
    const SpatiotemporalPlannerDataManager::BoundaryInfo& boundary_info,
    const Decision::RefTargetPoint& target_point_info) {
  auto follow_object = extractFollowObjects();
  auto opposite_obs_feature_map = processOppositeObstacles();
  auto opposite_object = extractOppositeObjects(opposite_obs_feature_map);
  auto interaction_object = extractInteractionObjects();
  constructInteractionInfo(interaction_object, interaction_infos_);

  nearest_obs_id_ = "";
  nearest_obs_s_ = std::numeric_limits<double>::max();
  nearest_obs_speed_ = 0.0;
  need_stop_ = false;
  extractNearestObstacle(follow_object, opposite_object, boundary_info, nearest_obs_id_, nearest_obs_s_,
                         nearest_obs_speed_);

  std::vector<SpatiotemporalState> objective_target;
  initializeObjectiveTarget(objective_target);

  double prev_s = sl_planning_start_point_.first.at(0);
  double prev_v = xy_planning_start_point_.v();
  double prev_a = xy_planning_start_point_.a();
  double prev_heading = xy_planning_start_point_.path_pt().theta();
  STLOG(D, "[SpatiotemporalOcpOptimizer::constructFollowTrajectoryTarget] prev_s = ", prev_s, " prev_v = ", prev_v,
        " prev_a = ", prev_a, " prev_heading = ", prev_heading);

  double k_4 = 10.0;
  double empirical_a = 0.0;
  double empirical_u = 0.0;
  std::string id = "";
  double ttc = 999.0;
  double max_soft_acc = getMaxAccLimitBasedScenario(scenario_info_);

  auto& v_soft_bound = boundary_info.trajectory_boundary->softSpeedBound();
  auto& s_soft_bound = boundary_info.time_related_boundary->sSoftBound();
  int last_yield_feature_point_index = 0;
  feature_s_points_[0] = make_pair(false, prev_s);
  feature_v_points_[0] = make_pair(false, prev_v);
  float safe_ttc = optimizer_config_.collision_check_param().ttc_for_interaction();
  float safe_distance = optimizer_config_.collision_check_param().min_distance_for_interaction();

  for (int i = 1; i < N_; i++) {
    // 计算加速度
    std::tie(empirical_a, id, ttc) = calculateEmpiricalAcceleration(
        i, prev_s, prev_v, v_soft_bound, s_soft_bound, follow_object[i], opposite_object[i], opposite_obs_feature_map);
    empirical_a = std::clamp(empirical_a, -6.0, max_soft_acc);
    // 计算控制输入
    double empirical_u = calculateControlInput(prev_a, empirical_a, k_4, id, ttc);

    // 更新状态
    std::tie(prev_s, prev_v, prev_a) = updateState(prev_s, prev_v, prev_a, empirical_u);

    // 获取参考路径点
    auto pt = getInitPoint(prev_s);
    double x_ref = pt.x();
    double y_ref = pt.y();
    prev_heading = spatiotemporal_functions::getUnifySpaceHeading(prev_heading, pt.heading());
    // 添加到目标轨迹
    objective_target.emplace_back(
        SpatiotemporalState{x_ref, y_ref, prev_heading, 0.0, 0.0, 0.0, prev_s, prev_v, prev_a, empirical_u});

    // check with interaction info, get feature point and update objective target parameters
    auto interaction_info = interaction_infos_[i];
    bool is_feature_point = false;
    if (behavior_ == FsmState::LEFT_ATTEMPT || behavior_ == FsmState::RIGHT_ATTEMPT) {
      if (static_cast<int>(std::round(target_point_info.t / dt_)) == i) {
        prev_s = target_point_info.s;
        is_feature_point = true;
      }
    }
    // cout<<" interaction_info.has_upper_bound_object = "<<(int)interaction_info.has_upper_bound_object<<"
    // interaction_info.upper_bound_s = "<<interaction_info.upper_bound_s
    //     <<" interaction_info.has_lower_bound_object = "<<(int)interaction_info.has_lower_bound_object<<"
    //     interaction_info.lower_bound_s = "<<interaction_info.lower_bound_s<<endl;
    if (interaction_info.has_upper_bound_object && prev_s > interaction_info.upper_bound_s - safe_distance) {
      double ego_interaction_v =
          max(2 * (interaction_info.upper_bound_s - safe_distance - sl_planning_start_point_.first.at(0)) / (i * dt_)
                  - xy_planning_start_point_.v(),
              0.0);
      float bound_buffer =
          std::fmax(safe_ttc * (ego_interaction_v - interaction_info.upper_bound_object.v), safe_distance);
      prev_s = fmax(interaction_info.upper_bound_s - bound_buffer, sl_planning_start_point_.first.at(0));
      prev_v = fmax(2 * (prev_s - sl_planning_start_point_.first.at(0)) / (i * dt_) - xy_planning_start_point_.v(), 0);
      prev_a = (prev_v - xy_planning_start_point_.v()) / (i * dt_);
      if (prev_a < -6.0) {
        prev_a = -6.0;
        prev_v = xy_planning_start_point_.v() + prev_a * (i * dt_);
        prev_s = sl_planning_start_point_.first.at(0) + xy_planning_start_point_.v() * (i * dt_)
                 + 0.5 * prev_a * (i * dt_) * (i * dt_);
      }
      is_feature_point = true;
      for (int j = 1; j < i; j++) {
        float s_j = sl_planning_start_point_.first.at(0) + xy_planning_start_point_.v() * (j * dt_)
                    + 0.5 * prev_a * (j * dt_) * (j * dt_);
        if (s_j < feature_s_points_[j].second) {
          feature_s_points_[j].first = false;
          feature_v_points_[j].first = false;
        }
      }
      last_yield_feature_point_index = i;
    } else if (interaction_info.has_lower_bound_object && prev_s < interaction_info.lower_bound_s + safe_distance) {
      double last_feature_point_s = feature_s_points_[last_yield_feature_point_index].second;
      double last_feature_point_v = feature_v_points_[last_yield_feature_point_index].second;
      double delta_t = fmax((i - last_yield_feature_point_index) * dt_, 0.0);
      float bound_buffer = std::fmax(safe_ttc * (interaction_info.lower_bound_object.v - prev_v), safe_distance);
      double desire_v =
          2 * (interaction_info.lower_bound_s + bound_buffer - last_feature_point_s) / delta_t - last_feature_point_v;
      prev_a = (desire_v - last_feature_point_v) / delta_t;
      prev_a = min(1.5, max(0.0, prev_a));
      prev_v = last_feature_point_v + prev_a * delta_t;
      prev_s = last_feature_point_s + last_feature_point_v * delta_t + 0.5 * prev_a * delta_t * delta_t;
      is_feature_point = true;
    } else if (interaction_info.has_lower_bound_object) {
      is_feature_point = true;
    }

    if (objective_target[i - 1].a * objective_target[i].a < 0
        || (abs(objective_target[i - 1].a) > 1E-2 && abs(objective_target[i].a) < 1E-2)
        || (objective_target[i - 1].jerk * objective_target[i].jerk < -0.1) || objective_target[i - 1].a < -2.0) {
      is_feature_point = true;
    }

    if (i == N_ - 1) {
      is_feature_point = false;
    }

    if (is_feature_point) {
      feature_s_points_[i] = std::make_pair(true, prev_s);
      feature_v_points_[i] = std::make_pair(true, prev_v);
    }

    if (i == N_ - 2) {
      feature_s_points_[i] = std::make_pair(true, prev_s);
      feature_v_points_[i] = std::make_pair(true, prev_v);
    }
    optimizer_parameters_[i]["s_coarse"] = {false, prev_s};
    optimizer_parameters_[i]["v_coarse"] = {false, prev_v};

    // cout<<" N = "<<i<<"  prev_s = "<<prev_s<<"  prev_v = "<<prev_v<<"  prev_a = "<<prev_a<<"  empirical_u =
    // "<<empirical_u<<"  id = "<<id<<endl;
  }
  need_stop_ =
      needStopBasedOnNearestObstacle(nearest_obs_id_, nearest_obs_s_, nearest_obs_speed_, xy_planning_start_point_.v());
}

std::vector<std::vector<ObjectInfo>> SpatiotemporalOptimizerLoader::extractFollowObjects() {
  std::vector<std::vector<ObjectInfo>> follow_object;
  follow_object.reserve(N_);
  for (size_t i = 0; i < N_; i++) {
    std::vector<ObjectInfo> temp_follow_object;
    for (const auto& obstacle_traj : target_obstacle_data_) {
      const auto& obj = obstacle_traj[i];
      if (obj.longitudinal_od_tag == LongitudinalOdTag::FOLLOW) {
        temp_follow_object.push_back(obj);
      }
    }
    follow_object.emplace_back(std::move(temp_follow_object));
  }
  return follow_object;
}

std::vector<std::vector<ObjectInfo>> SpatiotemporalOptimizerLoader::extractInteractionObjects() {
  std::vector<std::vector<ObjectInfo>> key_object;
  key_object.reserve(N_);
  for (size_t i = 0; i < N_; i++) {
    std::vector<ObjectInfo> temp_key_object;
    for (const auto& obstacle_traj : target_obstacle_data_) {
      const auto& obj = obstacle_traj[i];

      if (obj.longitudinal_od_tag == LongitudinalOdTag::OVERTAKE) {
        temp_key_object.push_back(obj);
      } else if (obj.longitudinal_od_tag == LongitudinalOdTag::YIELD) {  // 只考虑cross博弈的 yeild 场景
        temp_key_object.push_back(obj);
      }
    }
    key_object.emplace_back(std::move(temp_key_object));
  }

  return key_object;
}

void SpatiotemporalOptimizerLoader::constructInteractionInfo(std::vector<std::vector<ObjectInfo>> key_object,
                                                             std::vector<InteractionInfo>& interaction_infos) {
  float ego_s = sl_planning_start_point_.first.at(0);
  float ego_v = xy_planning_start_point_.v();
  for (size_t i = 0; i < N_; ++i) {
    auto& interaction_info = interaction_infos[i];
    interaction_info.t = i * dt_;
    for (auto& obj : key_object[i]) {
      if (obj.longitudinal_od_tag == LongitudinalOdTag::YIELD) {
        auto& current_object = interaction_info.upper_bound_object;
        auto& current_s = interaction_info.upper_bound_s;
        auto& has_upper_bound_object = interaction_info.has_upper_bound_object;
        const float current_min_s = std::min(obj.start_s, obj.end_s) - vehicle_param_.front_edge_to_ego();
        if (current_min_s < current_s) {
          current_s = std::max(ego_s, current_min_s);
          current_object = obj;
          has_upper_bound_object = true;
        }
        // cout << "[yield] i = " << i << "  current_s = " << current_s << "  obj.start_s = " << obj.start_s
        //      << " current_min_s = " << current_min_s << " default_buffer = " << default_buffer << "  obj.v = " <<
        //      obj.v
        //      << "  ego_v= " << ego_v << "  obj_id = " << obj.id
        //      << " has_upper_bound_object =   " << (int)has_upper_bound_object << endl;
      } else if (obj.longitudinal_od_tag == LongitudinalOdTag::OVERTAKE) {
        auto& current_object = interaction_info.lower_bound_object;
        auto& current_s = interaction_info.lower_bound_s;
        auto& current_upper_bound_s = interaction_info.upper_bound_s;
        auto& has_lower_bound_object = interaction_info.has_lower_bound_object;
        const float current_max_s = std::max(obj.start_s, obj.end_s) + vehicle_param_.rear_edge_to_ego();
        if (current_max_s > current_s) {
          current_s = std::min(current_max_s, current_upper_bound_s);
          current_object = obj;
          has_lower_bound_object = true;
        }
        // cout << "[overtake] i = " << i << "  current_s = " << current_s << "  obj.end_s = " << obj.end_s
        //      << " current_max_s = " << current_max_s << " default_buffer = " << default_buffer << "  obj.v = " <<
        //      obj.v
        //      << "  ego_v= " << ego_v << "  obj_id = " << obj.id
        //      << " has_upper_bound_object =   " << (int)has_lower_bound_object << endl;
      }
    }
  }
  return;
}

unordered_map<string, std::pair<double, double>> SpatiotemporalOptimizerLoader::processOppositeObstacles() {
  unordered_map<string, std::pair<double, double>> opposite_obs_feature_map;
  for (const auto& obstacle_traj : target_obstacle_data_) {
    if (obstacle_traj.empty()) {
      continue;
    }
    const auto& first_point = obstacle_traj[0];
    if (first_point.object_game_type == ObjectGameType::OPPOSITE_GAME) {
      double time_confident = calculateConfidentTime(obstacle_traj, xy_planning_start_point_.v());
      double min_s_bound =
          cacualteMinDesireBound(obstacle_traj, xy_planning_start_point_.path_pt().s(), xy_planning_start_point_.v());
      opposite_obs_feature_map[first_point.id] = std::make_pair(time_confident, min_s_bound);
    }
  }
  return opposite_obs_feature_map;
}

std::vector<std::vector<ObjectInfo>> SpatiotemporalOptimizerLoader::extractOppositeObjects(
    const unordered_map<string, std::pair<double, double>>& opposite_obs_feature_map) {
  std::vector<std::vector<ObjectInfo>> opposite_object;
  opposite_object.reserve(N_);
  for (size_t i = 0; i < N_; i++) {
    std::vector<ObjectInfo> temp_opposite_object;
    for (const auto& obstacle_traj : target_obstacle_data_) {
      if (obstacle_traj.size() <= i) {
        continue;
      }
      const auto& obj = obstacle_traj[i];
      if (obj.longitudinal_od_tag == LongitudinalOdTag::YIELD
          && obj.object_game_type == ObjectGameType::OPPOSITE_GAME) {
        auto it = opposite_obs_feature_map.find(obstacle_traj[0].id);
        if (it != opposite_obs_feature_map.end() && it->second.first > i * dt_) {
          temp_opposite_object.push_back(obj);
        }
      }
    }
    opposite_object.emplace_back(std::move(temp_opposite_object));
  }
  return opposite_object;
}

void SpatiotemporalOptimizerLoader::extractNearestObstacle(
    const std::vector<std::vector<ObjectInfo>>& follow_object, std::vector<std::vector<ObjectInfo>>& opposite_object,
    const SpatiotemporalPlannerDataManager::BoundaryInfo& boundary_info, string& nearest_obs_id, double& nearest_obs_s,
    double& nearest_obs_speed) {
  double ego_s = xy_planning_start_point_.path_pt().s();
  // 处理boundary_info  --- 决策已处理减去 front_edge_to_ego
  double s_soft_upper_bound = boundary_info.time_related_boundary->sSoftBound().evaluate(0.0).upper();
  if (nearest_obs_s > s_soft_upper_bound - ego_s) {
    nearest_obs_id = boundary_info.time_related_boundary->sSoftBound().evaluate(0.0).upperType();
    nearest_obs_s = s_soft_upper_bound - ego_s;
    nearest_obs_speed = 0.0;
  }
  // 处理follow_object
  if (!follow_object.empty()) {
    for (const auto& obj : follow_object.front()) {
      if (nearest_obs_s > obj.start_s - ego_s - vehicle_param_.front_edge_to_ego()) {
        nearest_obs_id = obj.id;
        nearest_obs_s = obj.start_s - ego_s - vehicle_param_.front_edge_to_ego();
        nearest_obs_speed = obj.v;
      }
    }
  }
  // 处理opposite_object
  if (!opposite_object.empty()) {
    for (const auto& obj : opposite_object.front()) {
      if (nearest_obs_s > obj.start_s - ego_s - vehicle_param_.front_edge_to_ego()) {
        nearest_obs_id = obj.id;
        nearest_obs_s = obj.start_s - ego_s - vehicle_param_.front_edge_to_ego();
        nearest_obs_speed = obj.v;
      }
    }
  }
  nearest_obs_s = fmax(nearest_obs_s, 0.0);
}

void SpatiotemporalOptimizerLoader::initializeObjectiveTarget(std::vector<SpatiotemporalState>& objective_target) {
  objective_target.clear();
  objective_target.reserve(N_);

  // 添加起始点状态
  objective_target.emplace_back(SpatiotemporalState{
      xy_planning_start_point_.path_pt().x(), xy_planning_start_point_.path_pt().y(),
      xy_planning_start_point_.path_pt().theta(), 0.0, xy_planning_start_point_.path_pt().front_steer(), 0.0,
      sl_planning_start_point_.first.at(0), xy_planning_start_point_.v(), xy_planning_start_point_.a(), 0.0});

  optimizer_parameters_[0]["s_coarse"] = {false, sl_planning_start_point_.first.at(0)};
  optimizer_parameters_[0]["v_coarse"] = {false, xy_planning_start_point_.v()};
}

std::tuple<double, std::string, double> SpatiotemporalOptimizerLoader::calculateEmpiricalAcceleration(
    int step_index, double prev_s, double prev_v, const math::IntervalData<Boundary>& v_soft_bound,
    const math::IntervalData<Boundary>& s_soft_bound, const std::vector<ObjectInfo>& follow_obstacles,
    const std::vector<ObjectInfo>& opposite_obstacles,
    const unordered_map<string, std::pair<double, double>>& opposite_obs_feature_map) {
  double k_2 = 0.3, k_3 = 10.0, k_4 = 4.0;
  double min_speed_limit = 0.0 * KMH_MS;
  double K_0_ = thw_;
  double gamma_ = 0.3;

  double s_soft_upper_bound = s_soft_bound.evaluate(step_index * dt_).upper();
  double distance = s_soft_upper_bound - prev_s;
  double delta_v = prev_v;
  double gamma = caculateGamma(prev_v, distance, 0.0);
  double ideal_distance = (K_0_ + gamma * std::max(0.0, delta_v)) * prev_v;
  double delta_dist = distance - ideal_distance;
  double k_1 = (delta_v >= 0.5) ? gamma : 0.0;

  double bound_ttc = (distance) / max(delta_v, 1.0);
  std::vector<double> bound_ttc_table = {1.0, 2.0, 3.0, 4.0, 5.0};
  std::vector<double> v_gain_table = {0.9, 0.8, 0.7, 0.6, 0.5};
  double v_gain = 1.0;
  if (bound_ttc > 0) {
    v_gain = math::TableLookUp1D(bound_ttc_table, v_gain_table, bound_ttc);
  }
  double empirical_a = calcEmpiricalAcc(prev_v, delta_dist, delta_v * v_gain, 0.0, K_0_, k_1, k_2, 0.0);
  std::string influencing_id = "bound";
  double influencing_ttc = delta_dist / max(delta_v, 1.0);

  // 处理跟随障碍物
  for (const auto& obs : follow_obstacles) {
    double safe_distance_ = obs.safe_distance;
    SLPoint sl_pt;
    target_ref_line_->xy2sl(math::Vec3d(obs.x, obs.y, 0), &sl_pt);
    double local_s = sl_pt.s();

    double obs_distance = local_s - prev_s - obs.length / 2 - vehicle_param_.front_edge_to_ego();
    double obs_delta_v = prev_v - obs.v;
    if (obs_distance < safe_distance_) {
      obs_delta_v = obs_delta_v < 0 ? 0.0 : obs_delta_v;
    }
    double obs_acc = obs.v > 1.0 ? obs.acc : 0.0;
    double obs_gamma = caculateGamma(prev_v, obs_distance, obs.v);

    if (behavior_ == FsmState::LEFT_CHANGE || behavior_ == FsmState::RIGHT_CHANGE || behavior_ == FsmState::LEFT_HOLD
        || behavior_ == FsmState::RIGHT_HOLD || behavior_ == FsmState::LEFT_ATTEMPT
        || behavior_ == FsmState::RIGHT_ATTEMPT) {
      K_0_ = optimizer_config_.collision_check_param().safe_thw_for_interaction();
      float lane_change_gamma_reduction_factor = 0.5;
      obs_gamma = obs_gamma * lane_change_gamma_reduction_factor;
    }

    double target_thw =
        fmax(K_0_ + obs_gamma * obs_delta_v, optimizer_config_.collision_check_param().min_time_headway());
    double current_thw = fmax(obs_distance / prev_v, optimizer_config_.collision_check_param().min_time_headway());
    double current_ttc = (obs_distance - safe_distance_) / max(obs_delta_v, 1.0);
    std::vector<double> ttc_table = {1.0, 2.0, 3.0, 4.0, 5.0, 10, 30, 50};
    std::vector<double> alpha_table = {0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2};
    double alpha = 1.0;
    if (current_ttc > 0) {
      alpha = math::TableLookUp1D(ttc_table, alpha_table, current_ttc);
    }
    if (target_thw > current_thw) {
      target_thw = alpha * target_thw + (1 - alpha) * current_thw;
    }
    double obs_ideal_distance = safe_distance_ + target_thw * prev_v;
    double obs_delta_dist = obs_distance - obs_ideal_distance;
    double obs_k_1 = (obs_delta_v >= 0.5) ? obs_gamma : 0.0;
    double obs_acc_coefficient = k_4 * obs_gamma;
    // 近距离状态下增大加速度计算模型内的距离权重0.3～1.0
    k_2 = min(max((2.0 - obs_distance / safe_distance_), k_2), 1.0);

    // cout<<" obs id = "<<obs.id<<" obs_distance = "<<obs_distance<<" obs_ideal_distance = "<<obs_ideal_distance<<"
    // obs_delta_dist = "<<obs_delta_dist
    //   <<"  obs_delta_v = "<<obs_delta_v<<" obs_gamma = "<<obs_gamma<<" obs_k_1 = "<<obs_k_1<<" obs_acc =
    //   "<<obs_acc<<"  target_thw = "<<target_thw <<"  current_thw = "<<current_thw<<" current_ttc = "<<current_ttc<<"
    //   k_2 = "<<k_2<<endl;
    double obs_empirical_a =
        calcEmpiricalAcc(prev_v, obs_delta_dist, obs_delta_v, obs_acc, K_0_, obs_k_1, k_2, obs_acc_coefficient);
    // 基于静态障碍物停车避撞的最小减速度计算
    if (obs.v < optimizer_config_.collision_check_param().static_obs_speed_thres() && obs_empirical_a < 0.0) {
      obs_empirical_a = min(-prev_v * prev_v / (2 * (max(obs_distance - safe_distance_, 0.1))), obs_empirical_a);
    }

    if (empirical_a > obs_empirical_a) {
      empirical_a = obs_empirical_a;
      influencing_id = obs.id;
      influencing_ttc = current_ttc;
    }
  }

  // 处理对向障碍物
  for (const auto& obs : opposite_obstacles) {
    double safe_distance_ = obs.safe_distance;
    auto it = opposite_obs_feature_map.find(obs.id);
    if (it == opposite_obs_feature_map.end())
      continue;
    SLPoint sl_pt;
    target_ref_line_->xy2sl(math::Vec3d(obs.x, obs.y, 0), &sl_pt);
    double local_s = sl_pt.s();

    double obs_distance = local_s - prev_s - obs.length / 2 - vehicle_param_.front_edge_to_ego();
    double obs_delta_v = prev_v;
    double obs_gamma = caculateGamma(prev_v, obs_distance, 0.0);
    double obs_ideal_distance = safe_distance_ + (K_0_ + obs_gamma * std::max(0.0, obs_delta_v)) * prev_v;
    double obs_delta_dist = obs_distance - obs_ideal_distance;
    double obs_k_1 = (obs_delta_v >= 0.5) ? obs_gamma : 0.0;
    double obs_acc_coefficient = k_4 * obs_gamma;
    double obs_empirical_a =
        calcEmpiricalAcc(prev_v, obs_delta_dist, obs_delta_v, obs.acc, K_0_, obs_k_1, k_2, obs_acc_coefficient);
    double current_ttc = (obs_distance - safe_distance_) / max(obs_delta_v, 1.0);
    if (empirical_a > obs_empirical_a) {
      empirical_a = obs_empirical_a;
      influencing_id = obs.id;
      influencing_ttc = current_ttc;
    }
  }

  // 应用速度限制
  double current_speed_limit = v_soft_bound.evaluate(prev_s).upper();
  auto match_bound = v_soft_bound.match(prev_s);
  if (-k_3 * (prev_v - current_speed_limit) < empirical_a) {
    empirical_a = -k_3 * (prev_v - current_speed_limit);
    influencing_id = "speed_limit_" + match_bound.upperType();
  }

  return {empirical_a, influencing_id, influencing_ttc};
}

// 计算控制输入
double SpatiotemporalOptimizerLoader::calculateControlInput(double prev_a, double empirical_a, double k_4, string id,
                                                            double ttc) {
  double empirical_u = -k_4 * (prev_a - empirical_a);
  double empirical_u_upper_bound = 8.0;
  double empirical_u_lower_bound = -8.0;

  if (optimizer_config_.enable_jerk_adjust_base_ttc()) {
    // base jerk limit
    vector<double> ttc_table = {1.0, 2.0, 3.0, 5.0, 10.0};
    vector<double> empirical_u_lower_bound_table = {-8.0, -3.0, -2.0, -1.0, -0.5};
    empirical_u_lower_bound = math::TableLookUp1D(ttc_table, empirical_u_lower_bound_table, ttc);
    // speed limit jerk limit
    std::map<string, double> speed_limit_jerk = {
        {"speed_limit_default", -1.0}, {"speed_limit_curve", -3.0}, {"speed_limit_turn", -4.0}};
    if (id.find("speed_limit") == 0) {
      if (id == "speed_limit_turn") {
        empirical_u_lower_bound = speed_limit_jerk[id];
      } else if (id == "speed_limit_curve") {
        empirical_u_lower_bound = speed_limit_jerk[id];
      } else {
        empirical_u_lower_bound = speed_limit_jerk["speed_limit_default"];
      }
    }
  }
  return std::clamp(empirical_u, empirical_u_lower_bound, empirical_u_upper_bound);
}

// 更新状态
std::tuple<double, double, double> SpatiotemporalOptimizerLoader::updateState(double prev_s, double prev_v,
                                                                              double prev_a, double empirical_u) {
  double new_s =
      prev_s + std::max(0.0, prev_v * dt_ + 0.5 * prev_a * dt_ * dt_ + (1.0 / 6.0) * empirical_u * dt_ * dt_ * dt_);
  double new_v = std::max(0.0, prev_v + prev_a * dt_ + 0.5 * empirical_u * dt_ * dt_);
  double new_a = prev_a + empirical_u * dt_;
  return {new_s, new_v, new_a};
}

void SpatiotemporalOptimizerLoader::updateObjectiveTargetParameters(
    std::vector<std::pair<bool, double>> feature_s_points, bool need_stop,
    std::vector<std::unordered_map<std::string, std::pair<bool, double>>>& optimizer_parameters) {
  for (size_t i = 0; i < N_; i++) {
    auto [is_s_feature_point, value_s] = feature_s_points_[i];
    auto [is_v_feature_point, value_v] = feature_v_points_[i];
    optimizer_parameters[i]["a_offset"] = {false, 0.0};
    if (need_stop) {
      optimizer_parameters[i]["s_coarse"] = {false, 0.0};
      optimizer_parameters[i]["v_coarse"] = {false, 0.0};
    } else {
      if (!optimizer_parameters[i]["s_coarse"].first && is_s_feature_point) {
        optimizer_parameters[i]["s_coarse"] = {is_s_feature_point, value_s};
      }
      if (!optimizer_parameters[i]["v_coarse"].first && is_v_feature_point) {
        optimizer_parameters[i]["v_coarse"] = {is_v_feature_point, value_v};
      }
    }
    if (behavior_ == FsmState::LEFT_HOLD || behavior_ == FsmState::RIGHT_HOLD) {
      optimizer_parameters[i]["l_offset"] = {true, init_guess_.at(i).l};
    } else {
      optimizer_parameters[i]["l_offset"] = {false, 0.0};
    }
    // cout<<" i = "<<i<<" is_s_feature_point = "<<(int)is_s_feature_point<<" s_coarse =
    // "<<optimizer_parameters_[i]["s_coarse"].second<<" v_coarse =
    // "<<optimizer_parameters_[i]["v_coarse"].second<<endl;
  }
}

void SpatiotemporalOptimizerLoader::outputProcess(SpatiotemporalPlannerDataManager& data_manager) {
  data_manager.mutableOptimizerInfo().optimizer_parameters = optimizer_parameters_;
  data_manager.mutableOptimizerInfo().init_guess = init_guess_;
  data_manager.mutableOptimizerInfo().current_driving_scenario = current_driving_scenario_;
  data_manager.mutableOutputData().debug_info = debug_info_;
  data_manager.mutableOutputData().nearest_obs_info =
      std::make_tuple(nearest_obs_id_, nearest_obs_s_, nearest_obs_speed_);
  data_manager.mutableOutputData().need_stop = need_stop_;

  // coarse_trajectory - 仅输出s-v-t信息，用于可视化显示
  auto& coarse_trajectory = data_manager.mutableOutputData().coarse_trajectory;
  coarse_trajectory.clear();
  for (int i = 0; i < optimizer_parameters_.size(); i++) {
    TrajectoryPt pt;
    pt.mutable_path_pt()->set_s(optimizer_parameters_[i]["s_coarse"].second);
    pt.set_v(optimizer_parameters_[i]["v_coarse"].second);
    pt.set_relative_time(i * dt_);
    coarse_trajectory.emplace_back(pt);
  }

  // object bound 输出
  auto& interaction_infos = data_manager.mutableOutputData().interaction_infos;
  interaction_infos.clear();
  interaction_infos = interaction_infos_;
}

bool SpatiotemporalOptimizerLoader::checkInitGuessValid(const DecisionResult* decision_result) {
  init_guess_.clear();
  bool has_follow_object = false;
  std::vector<ObjectInfo> follow_object;
  // todo: sort follow object by distance to ego
  for (auto objs : target_obstacle_data_) {
    if (objs.front().longitudinal_od_tag == LongitudinalOdTag::FOLLOW) {
      has_follow_object = true;
      follow_object = objs;
      break;
    }
  }

  double prev_s = sl_planning_start_point_.first.at(0);
  double prev_v = xy_planning_start_point_.v();
  double prev_a = xy_planning_start_point_.a();
  double prev_heading = xy_planning_start_point_.path_pt().theta();
  SLPoint sl_pt;

  if (!decision_result->getRefTrajInfo()->traj_points.empty()
      && decision_result->getRefTrajInfo()->traj_points.size() > 50) {
    auto start_state = decision_result->getRefTrajInfo()->traj_points.front();
    target_ref_line_->xy2sl(math::Vec3d(start_state.x, start_state.y, 0), start_state.yaw, &sl_pt);
    init_guess_.emplace_back(SpatiotemporalState{start_state.x, start_state.y, start_state.yaw, sl_pt.l(), 0.0, 0.0,
                                                 start_state.s, start_state.v, start_state.a, 0.0});
    for (int i = 1; i < decision_result->getRefTrajInfo()->traj_points.size(); i++) {
      auto traj_pt = decision_result->getRefTrajInfo()->traj_points[i];
      target_ref_line_->xy2sl(math::Vec3d(traj_pt.x, traj_pt.y, 0), traj_pt.yaw, &sl_pt);
      SpatiotemporalState state(
          {traj_pt.x, traj_pt.y, traj_pt.yaw, sl_pt.l(), 0.0, 0.0, traj_pt.s, traj_pt.v, traj_pt.a, 0.0});
      state.theta = spatiotemporal_functions::getUnifySpaceHeading(init_guess_[i - 1].theta, state.theta);
      init_guess_.emplace_back(state);
      // ERT_LOG_I(" decision init_guess_: s = ", init_guess_.back().s, " x = ", init_guess_.back().x, " y = ", init_guess_.back().y, " theta = ", init_guess_.back().theta, " l = ", init_guess_.back().l, " v = ", init_guess_.back().v, " a = ", init_guess_.back().a);
    }
    STLOG(D, "[SpatiotemporalOcpOptimizer::checkInitGuessValid] Using decision init guess.");
  } else {
    STLOG(E, "[SpatiotemporalOcpOptimizer::checkInitGuessValid] Decision init guess is empty.");
    return false;
  }
  return true;
}

double SpatiotemporalOptimizerLoader::calculateTargetAcc(double s_gap, double v_ego, double acc_ego, double v_obs,
                                                         double acc_obs) {
  // 常数定义
  constexpr double dt = 0.1;  // 新增：时间分辨率
  constexpr double ttc_threshold = 3.0;
  constexpr double min_gap = 5.0;
  constexpr double normal_acc = 0.0;
  constexpr double brake_acc = -6.0;
  constexpr double epsilon = 1e-6;

  // 计算允许的最小加速度（保证v_ego + acc >= 0）
  const double min_acc = -v_ego / dt;

  // 立即制动条件判断
  const bool emergency_condition =
      s_gap <= min_gap || (s_gap <= min_gap + v_ego * ttc_threshold && v_ego > v_obs + epsilon);

  // 精确TTC计算
  double ttc = INFINITY;
  const double a_diff = acc_ego - acc_obs;

  // 二次方程求解器
  auto solve_quadratic = [epsilon](double a, double b, double c) -> double {
    const double discriminant = b * b - 4 * a * c;
    if (discriminant < 0)
      return INFINITY;

    const double sqrt_d = std::sqrt(discriminant);
    const double t1 = (-b + sqrt_d) / (2 * a);
    const double t2 = (-b - sqrt_d) / (2 * a);

    double valid_ttc = INFINITY;
    if (t1 > epsilon)
      valid_ttc = t1;
    if (t2 > epsilon && t2 < valid_ttc)
      valid_ttc = t2;
    return valid_ttc;
  };

  // 分情况计算TTC
  if (std::abs(a_diff) > epsilon) {
    ttc = solve_quadratic(0.5 * a_diff, v_ego - v_obs, -s_gap);
  } else {
    const double rel_speed = v_ego - v_obs;
    if (rel_speed > epsilon) {
      ttc = s_gap / rel_speed;
      if (ttc < epsilon)
        ttc = INFINITY;
    }
  }

  // 分级控制策略
  double target_acc = normal_acc;
  constexpr double mid_ttc = 5.0;    // 中等反应阈值
  constexpr double mild_acc = -1.5;  // 温和减速

  if (emergency_condition) {
    target_acc = brake_acc;
  } else if (ttc <= ttc_threshold) {
    target_acc = brake_acc;
  } else if (ttc <= mid_ttc) {
    const double speed_factor = std::clamp((v_ego - v_obs) / 10.0, 0.0, 1.0);
    target_acc = mild_acc * (1.0 - speed_factor) + brake_acc * speed_factor;
  }

  // 应用速度非负约束
  target_acc = std::max(target_acc, min_acc);

  // 添加物理限制（示例值，根据实际车辆参数调整）
  const double max_brake = std::max(-10.0, -v_ego / dt * 1.1);
  constexpr double max_accel = 2.0;
  return std::clamp(target_acc, max_brake, max_accel);
};

double SpatiotemporalOptimizerLoader::calcEmpiricalAcc(const double& ego_v, const double& delta_dist,
                                                       const double& delta_v, const double& front_a, const double& k_0,
                                                       const double& k_1, const double& k_2,
                                                       const double& acc_coefficient) {
  return (k_2 * delta_dist - delta_v + acc_coefficient * front_a * fmax(1.0, delta_v))
         / (k_0 + k_1 * fmax(0.0, delta_v));
}

double SpatiotemporalOptimizerLoader::calculateConfidentTime(const std::vector<ObjectInfo>& obj_info,
                                                             double ego_speed) {
  double confident_time = horizon_;
  const double ego_half_width = 0.5 * vehicle_param_.width();
  double max_l_overlap_from_prediction = 0.0;
  for (const auto& obj : obj_info) {
    double min_dis = 0.0;
    if (obj.start_l * obj.end_l <= 0) {
      min_dis = 0.0;
    } else {
      min_dis = min(abs(obj.start_l), abs(obj.end_l));
    }
    if (std::abs(min_dis) <= ego_half_width + 0.1) {
      max_l_overlap_from_prediction = std::max(max_l_overlap_from_prediction, ego_half_width - std::abs(min_dis));
    }
  }

  const std::vector<double> follow_distance_table = {15.0, 25.0, 30.0, 50.0, 60.0};
  std::vector<double> coeff_table_wrt_distance = {2.0, 1.6, 1.2, 1.0, 1.0};
  double coeff_wrt_distance = math::TableLookUp1D(follow_distance_table, coeff_table_wrt_distance, obj_info[0].local_s);

  std::vector<double> delta_speed_table = {0.0, 3.0, 6.0, 10.0, 15.0};
  std::vector<double> coeff_table_wrt_delta_speed = {1.0, 1.1, 1.3, 1.6, 2.0};

  double obstacle_speed = 0;  //  TODO
  double delta_speed = ego_speed - obstacle_speed;
  double coeff_wrt_delta_speed = math::TableLookUp1D(delta_speed_table, coeff_table_wrt_delta_speed, delta_speed);

  const std::vector<double> coeff_table = {1.0, 1.5, 2.0, 3.0, 3.5, 4.0};
  std::vector<double> confident_time_table = {2.0, 2.1, 2.3, 2.5, 2.8, 3.0};
  confident_time = math::TableLookUp1D(coeff_table, confident_time_table, coeff_wrt_distance * coeff_wrt_delta_speed);
  // Consider max-overlap:
  double max_overlap_ratio = max_l_overlap_from_prediction / std::max(ego_half_width, 1.0);
  const std::vector<double> max_overlap_ratio_table = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.75, 1.0};
  std::vector<double> confident_time_table_wrt_max_overlap_ratio = {0.5, 0.5, 1.0, 1.3, 1.6, 2.0, 3.0, 4.0};
  double max_confident_time_wrt_overlap_ratio =
      math::TableLookUp1D(max_overlap_ratio_table, confident_time_table_wrt_max_overlap_ratio, max_overlap_ratio);
  confident_time = std::min(confident_time, max_confident_time_wrt_overlap_ratio);
  return confident_time;
}

double SpatiotemporalOptimizerLoader::cacualteMinDesireBound(const std::vector<ObjectInfo>& obj_info, double ego_s,
                                                             double ego_speed) {
  double min_s = obj_info[0].local_s;
  double od_safe_distance = 5.0;
  double comfort_decc_for_brake = -1.2;
  double desire_bound = ego_s + od_safe_distance + ego_speed * ego_speed / std::abs(comfort_decc_for_brake);
  double t_buffer = 1.0;
  double stop_buffer = fabs(obj_info[0].v) * t_buffer;
  desire_bound = std::fmin(
      fmax(min_s - (vehicle_param_.length() - vehicle_param_.rear_edge_to_ego()) - stop_buffer, 0.0), desire_bound);
  return desire_bound;
}

bool SpatiotemporalOptimizerLoader::needStopBasedOnNearestObstacle(string nearest_obs_id, double nearest_obs_distance,
                                                                   double nearest_obs_v, double ego_speed) {
  ERT_PLOG_D << " >>>>>>>>>>>>>>>>>>>> nearest_front_obj_id_ =  " << nearest_obs_id
             << " nearest_front_obj_invasion_s_ =  " << nearest_obs_distance << "  nearest_obs_v = " << nearest_obs_v
             << " ego_speed = " << ego_speed;
  //------------------------------------------------
  static int static_counter = 0;
  static bool obstacle_static = false;
  if (nearest_obs_id == "None") {
    static_counter = 0;
    obstacle_static = false;
  }

  bool nearest_obstacle_static = false;
  double front_distance = nearest_obs_distance;
  bool is_nearest_bound =
      nearest_obs_id == "destination" || nearest_obs_id == "junction_stop" || nearest_obs_id == "tsr";

  if (!is_nearest_bound) {
    if (!obstacle_static && abs(nearest_obs_v) <= 1.0 * KMH_MS) {
      ++static_counter;
    }
    if (abs(nearest_obs_v) >= 2.0 * KMH_MS) {
      static_counter--;
    }
    if (static_counter >= 3) {
      static_counter = 3;
      obstacle_static = true;
    } else if (static_counter <= 0) {
      static_counter = 0;
      obstacle_static = false;
    }
    nearest_obstacle_static = obstacle_static;
    front_distance = nearest_obs_distance;
  } else {
    nearest_obstacle_static = true;
    front_distance = nearest_obs_distance;
  }

  double stop_distance = (is_nearest_bound) ? 1.0 : 5.0;
  double stay_static_distance_buffer = (is_nearest_bound) ? 1.0 : 3.0;

  bool should_stop =
      nearest_obstacle_static && (ego_speed <= 0.1 && front_distance <= stop_distance + stay_static_distance_buffer);
  ERT_PLOG_D << "  >>>>>>>>>>>>>>>>>>>  nearest_obstacle_static = " << (int)nearest_obstacle_static
             << "  need_stop = " << (int)should_stop;

  return should_stop;
}

double SpatiotemporalOptimizerLoader::caculateGamma(double ego_speed, double obj_distance, double obj_speed) {
  double init_k = (obj_distance - 3.0) / fmax(0.1, ego_speed);

  double dv_risk = (ego_speed - obj_speed) * ego_speed / fmax(obj_distance, 0.1);

  // Calculate the delta_v_slope based on distance and risk
  std::vector<double> init_K_table_for_dv_slope = {0.0, 1.0, 2.0, 3.0, 4.0};
  std::vector<double> dv_slope_table_wrt_init_K = {0.4, 0.3, 0.2, 0.1, 0.0};

  std::vector<double> risk_table_for_dv_slope = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
  std::vector<double> dv_slope_table_wrt_risk = {0.0, 0.1, 0.15, 0.20, 0.25, 0.25};

  std::vector<double> object_dis_table = {40.0, 60.0, 80.0, 100.0, 120.0};
  std::vector<double> dv_attenuation_coeff_table = {1.0, 0.8, 0.7, 0.6, 0.5};
  double dv_attenuation_coeff = math::TableLookUp1D(object_dis_table, dv_attenuation_coeff_table, obj_distance);

  double delta_v_slope_for_k = math::TableLookUp1D(init_K_table_for_dv_slope, dv_slope_table_wrt_init_K, init_k);
  delta_v_slope_for_k =
      fmax(delta_v_slope_for_k,
           dv_attenuation_coeff * math::TableLookUp1D(risk_table_for_dv_slope, dv_slope_table_wrt_risk, dv_risk));
  return delta_v_slope_for_k;
}

void SpatiotemporalOptimizerLoader::CalculateSteerFromGuess(std::vector<SpatiotemporalState>& init_guess,
                                                            double wheelbase) {
  size_t n = init_guess.size();
  if (n < 2)
    return;  // 至少需要2个点计算差分

  // 安全计算角度差（处理 2π 跳变）
  auto safeDeltaTheta = [](double theta_next, double theta_prev) -> double {
    double delta = theta_next - theta_prev;
    // 角度归一化到 [-π, π]
    if (delta > M_PI)
      delta -= 2 * M_PI;
    else if (delta < -M_PI)
      delta += 2 * M_PI;
    return delta;
  };

  // 处理首尾点（使用单侧差分）
  for (size_t i = 0; i < n; ++i) {
    double delta_theta, delta_s;
    if (i == 0) {  // 首点：用后一节点差分
      delta_theta = safeDeltaTheta(init_guess[i + 1].theta, init_guess[i].theta);
      delta_s = init_guess[i + 1].s - init_guess[i].s;
    } else if (i == n - 1) {  // 尾点：用前一节点差分
      delta_theta = safeDeltaTheta(init_guess[i].theta, init_guess[i - 1].theta);
      delta_s = init_guess[i].s - init_guess[i - 1].s;
    } else {  // 中间点：中心差分（精度更高）
      delta_theta = safeDeltaTheta(init_guess[i + 1].theta, init_guess[i - 1].theta);
      delta_s = init_guess[i + 1].s - init_guess[i - 1].s;
    }

    // 计算曲率 κ = dθ/ds (rad/s)
    double kappa = (std::fabs(delta_s) > 1e-5) ? (delta_theta / delta_s) : 0.0;

    // 运动学模型: κ = tan(steer) / wheelbase
    init_guess[i].steer = std::atan(kappa * wheelbase);
  }
}

void SpatiotemporalOptimizerLoader::CalculateControlInputs(std::vector<SpatiotemporalState>& init_guess, double dt) {
  size_t n = init_guess.size();
  if (n < 2)
    return;

  // 遍历所有点计算差分
  for (size_t i = 0; i < n; ++i) {
    // ======================= dsteer = d(steer)/dt 计算 =======================
    if (i == 0 || i == 1 || i == 2) {
      // 首点：前向差分
      // double delta_steer = 0.0;
      init_guess[i].dsteer = 0.0;
    } else if (i == n - 1) {
      // 尾点：后向差分
      double delta_steer = init_guess[i].steer - init_guess[i - 1].steer;
      init_guess[i].dsteer = delta_steer / dt;
    } else {
      // 中间点：中心差分（精度更高）
      double delta_steer = init_guess[i + 1].steer - init_guess[i - 1].steer;
      init_guess[i].dsteer = delta_steer / (2 * dt);
    }

    // ======================= jerk = da/dt 计算 =======================
    if (i == 0 || i == 1 || i == 2) {
      // 首点：前向差分
      // double delta_a = init_guess[i+1].a - init_guess[i].a;
      init_guess[i].jerk = 0.0;
    } else if (i == n - 1) {
      // 尾点：后向差分
      double delta_a = init_guess[i].a - init_guess[i - 1].a;
      init_guess[i].jerk = delta_a / dt;
    } else {
      // 中间点：中心差分
      double delta_a = init_guess[i + 1].a - init_guess[i - 1].a;
      init_guess[i].jerk = delta_a / (2 * dt);
    }
  }
}

double SpatiotemporalOptimizerLoader::getMaxAccLimitBasedScenario(
    const proto::road_cognition::ScenarioInfo& scenario_info) {
  double max_soft_acc = optimizer_config_.default_max_soft_acc();
  if (behavior_ == FsmState::LEFT_CHANGE || behavior_ == FsmState::RIGHT_CHANGE) {
    return optimizer_config_.lane_change_max_soft_acc();
  }
  if (scenario_info.ego_scenario_types().empty()) {
    return max_soft_acc;
  }
  for (auto ego_scenario_type : scenario_info.ego_scenario_types()) {
    if (ego_scenario_type == proto::road_cognition::ScenarioInfo_ScenarioType_kScenarioNearMerge) {
      return optimizer_config_.merge_max_soft_acc();
    }
  }
  return max_soft_acc;
}

double SpatiotemporalOptimizerLoader::getThw(FollowingDistanceLevel following_distance_level) {
  // Min: 1.1 Low: 1.3 Mid: 1.5 High: 1.7  Max: 1.9
  switch (following_distance_level) {
    case FollowingDistanceLevel::Invalid:
      return 1.5;
    case FollowingDistanceLevel::Min:
      return 1.1;
    case FollowingDistanceLevel::Low:
      return 1.3;
    case FollowingDistanceLevel::Mid:
      return 1.5;
    case FollowingDistanceLevel::High:
      return 1.7;
    case FollowingDistanceLevel::Max:
      return 1.9;
    default:
      return 1.5;
  }
}

void SpatiotemporalOptimizerLoader::updateLateralOffset(
    const SpatiotemporalPlannerDataManager::BoundaryInfo& boundary_info,
    const std::vector<std::vector<ObjectInfo>> target_obstacle_data,
    std::vector<std::unordered_map<std::string, std::pair<bool, double>>>& optimizer_parameters) {
  if (!optimizer_config_.enable_get_l_offset()) {
    return;
  }
  for (size_t i = 0; i < N_; i++) {
    optimizer_parameters[i]["l_offset"] = {true, init_guess_.at(i).l};
  }

  return;
}

std::vector<std::vector<ObjectInfo>> SpatiotemporalOptimizerLoader::extractNudgeObjects(
    const std::vector<std::vector<ObjectInfo>> target_obstacle_data) {
  std::vector<std::vector<ObjectInfo>> nudge_object;
  nudge_object.reserve(N_);
  for (size_t i = 0; i < N_; i++) {
    std::vector<ObjectInfo> temp_nudge_object;
    for (const auto& obstacle_traj : target_obstacle_data) {
      const auto& obj = obstacle_traj[i];
      if (obj.lateral_od_tag == LateralOdTag::DYNAMIC_LEFT_BYPASS
          || obj.lateral_od_tag == LateralOdTag::DYNAMIC_RIGHT_BYPASS || obj.lateral_od_tag == LateralOdTag::LEFT_BYPASS
          || obj.lateral_od_tag == LateralOdTag::RIGHT_BYPASS || obj.lateral_od_tag == LateralOdTag::RISKY_LEFT_BYPASS
          || obj.lateral_od_tag == LateralOdTag::RISKY_RIGHT_BYPASS) {
        temp_nudge_object.push_back(obj);
      }
    }
    if (!temp_nudge_object.empty()) {
      nudge_object.emplace_back(std::move(temp_nudge_object));
    }
  }
  return nudge_object;
}

REGIST_MODULE(SpatiotemporalOptimizerLoader);

}  // namespace gpal::pnc::planning
