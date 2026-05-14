#include "preprocess/base_data_preprocess.h"

namespace gpal::pnc::planning {

bool BaseDataPreprocess::init() {
  // 确保配置已注册
  registerAllSpatiotemporalConfigs();

  // 初始化场景管理器
  scenario_manager_ = std::make_unique<SpatiotemporalPlannerScenarioManager>();
  if (!scenario_manager_) {
    STLOG(E, "[BaseDataPreprocess::init] Failed to create SpatiotemporalPlannerScenarioManager");
    return false;
  }

  // 获取配置管理器
  config_manager_ = Singleton<ConfigManager>::get_instance();
  if (config_manager_ == nullptr) {
    STLOG(E, "[BaseDataPreprocess::init] ConfigManager is not initialized.");
    return false;
  }

  // 重置状态
  last_manager_key_ = SpatiotemporalPlannerScenarioManager::ManagerKey(MacroScenario::UNKNOWN_MACRO_SCENARIO,
                                                                       DrivingStyle::UNKNOWN_DRIVING_STYLE);

  STLOG(I, "[BaseDataPreprocess::init] Initialization completed successfully");
  return true;
}

void BaseDataPreprocess::reset() {
  // 重置状态
  last_manager_key_ = SpatiotemporalPlannerScenarioManager::ManagerKey(MacroScenario::UNKNOWN_MACRO_SCENARIO,
                                                                       DrivingStyle::UNKNOWN_DRIVING_STYLE);
}

std::string BaseDataPreprocess::id() const {
  return "BaseDataPreprocess";
}

bool BaseDataPreprocess::run(SpatiotemporalPlannerDataManager& data_manager) {
  auto& input_data = data_manager.mutableInputData();
  // 以下调用顺序不可调整
  if (!scenarioConfigPreprocess(data_manager)) {
    STLOG(E, "[BaseDataPreprocess::run] ScenarioConfigPreprocess failed");
    return false;
  }

  if (!updateTrajectoryBoundary(data_manager)) {
    STLOG(E, "[BaseDataPreprocess::run] UpdateTrajectoryBoundary failed");
    return false;
  }
  if (!vehicleInfoPreprocess(data_manager)) {
    STLOG(E, "[BaseDataPreprocess::run] VehicleInfoPreprocess failed");
    return false;
  }
  if (!updateTimeRelatedBoundary(data_manager)) {
    STLOG(E, "[BaseDataPreprocess::run] UpdateTimeRelatedBoundary failed");
    return false;
  }
  if (!updateVelocityRelatedBoundary(data_manager)) {
    STLOG(E, "[BaseDataPreprocess::run] UpdateVelocityRelatedBoundary failed");
    return false;
  }

  return true;
}

TrajectoryPt BaseDataPreprocess::getPlanningStartPoint(SpatiotemporalPlannerDataManager& data_manager) {
  auto& input_data = data_manager.mutableInputData();
  auto& output_data = data_manager.outputData();
  auto& preprocess_config = data_manager.configInfo().spatiotemporal_preprocess_profile;
  const auto& solver_info = data_manager.optimizerInfo().solve_info;
  TrajectoryPt planning_start_point;

  bool is_auto_mode = input_data.chassis->drivingMode()
                      == proto::PowertrainInfo::DrivingMode::PowertrainInfo_DrivingMode_kCompleteAutoDrive;

  std::vector<double> speed_vec;
  std::vector<double> kappa_vec;
  for (const auto& ele : preprocess_config->speed_kappa_bound_map().elements()) {
    speed_vec.emplace_back(ele.speed());
    kappa_vec.emplace_back(ele.kappa_bound());
  }
  double kapp_bound_ctrl = math::TableLookUp1D(speed_vec, kappa_vec,  static_cast<double>(input_data.chassis->Speed()));
  double steer_ratio = config_manager_->vehicle_config().dynamic_param().steer_ratio();
  double current_steer_angle = input_data.chassis->SteeringAngle() * ANG2RAD / steer_ratio;
  double current_vehicle_kappa =
      std::tan(current_steer_angle) / config_manager_->vehicle_config().vehicle_param().wheel_base();
  current_vehicle_kappa = std::clamp(current_vehicle_kappa, -kapp_bound_ctrl, kapp_bound_ctrl);  // left:+  right:-

  if (!output_data.planning_trajectory.empty()
      && input_data.localization->isLastOdometryLocalization() == input_data.localization->isOdometryLocalization()
      && is_auto_mode && solver_info.status == SolveStatus::SOLVED) {
    double min_dist = std::numeric_limits<double>::max();
    TrajectoryPt nearest_point = output_data.planning_trajectory.getNearestPoint(
        math::Vec3d(input_data.localization->vehicleAlignPosePoint().x(),
                    input_data.localization->vehicleAlignPosePoint().y(),
                    input_data.localization->vehicleAlignPosePoint().z()),
        min_dist);

    double current_local_s = input_data.target_ref_line_info->ref_line().getFrenetPoint(nearest_point.path_pt()).s();
    nearest_point.mutable_path_pt()->set_s(current_local_s);

    planning_start_point = nearest_point;

    const double dx = input_data.localization->vehicleAlignPosePoint().x() - nearest_point.path_pt().x();
    const double dy = input_data.localization->vehicleAlignPosePoint().y() - nearest_point.path_pt().y();
    const double cos_theta = std::cos(input_data.localization->vehicleAlignPosePoint().yaw());
    const double sin_theta = std::sin(input_data.localization->vehicleAlignPosePoint().yaw());
    const double longi_error = cos_theta * dx + sin_theta * dy;
    const double lat_error = -sin_theta * dx + cos_theta * dy;
    double heading_err = input_data.localization->vehicleAlignPosePoint().yaw() - nearest_point.path_pt().theta();
    heading_err = math::NormalizeAngle(heading_err);

    const bool enable_replan = preprocess_config->enable_replan();
    const float replan_longitudinal_error_thrd = input_data.localization->isOdometryLocalization()
                                                     ? preprocess_config->replan_longitudinal_error_thresold_lane()
                                                     : preprocess_config->replan_longitudinal_error_thresold();
    const float replan_heading_thrd = input_data.localization->isOdometryLocalization()
                                          ? preprocess_config->replan_heading_thresold_lane() * ANG2RAD
                                          : preprocess_config->replan_heading_thresold() * ANG2RAD;
    const float replan_lateral_error_thrd = input_data.localization->isOdometryLocalization()
                                                ? preprocess_config->replan_lateral_error_thresold_lane()
                                                : preprocess_config->replan_lateral_error_thresold();
    const bool enable_replan_lateral_exceed_heading = preprocess_config->enable_replan_lateral_exceed_heading();
    // PLOGI << "longi_error = " << longi_error
    //          << "  lat_error = " << lat_error
    //          <<"   heading_err = " << heading_err
    //          ;
    if (std::abs(longi_error) > replan_longitudinal_error_thrd ) {
      // 有历史轨迹时: 纵向偏差较大.
      planning_start_point.mutable_path_pt()->set_x(input_data.localization->vehicleAlignPosePoint().x());
      planning_start_point.mutable_path_pt()->set_y(input_data.localization->vehicleAlignPosePoint().y());
      planning_start_point.mutable_path_pt()->set_z(input_data.localization->vehicleAlignPosePoint().z());
      planning_start_point.mutable_path_pt()->set_theta(input_data.localization->vehicleAlignPosePoint().yaw());
      planning_start_point.mutable_path_pt()->set_kappa(current_vehicle_kappa);
      planning_start_point.mutable_path_pt()->set_front_steer(current_steer_angle);
      planning_start_point.mutable_path_pt()->set_s(
          input_data.target_ref_line_info->ref_line().getFrenetPoint(planning_start_point.path_pt()).s());
      STLOG(D, "[Planning::parsePlanningStartPoint]: longi_error = ", longi_error,
            " > thrd = ", replan_longitudinal_error_thrd);
    } else if (enable_replan && std::fabs(heading_err) > replan_heading_thrd) {
      // 有历史轨迹时: 角度偏差较大.
      double compensate_heading_err = heading_err;
      compensate_heading_err += heading_err > 0 ? -replan_heading_thrd : replan_heading_thrd;
      double compensate_heading = nearest_point.path_pt().theta() + compensate_heading_err;
      compensate_heading = math::NormalizeAngle(compensate_heading);
      if (enable_replan_lateral_exceed_heading && std::abs(lat_error) > replan_lateral_error_thrd) {
        double offset = lat_error;
        offset += lat_error > 0 ? -replan_lateral_error_thrd : replan_lateral_error_thrd;
        *planning_start_point.mutable_path_pt() = nearest_point.path_pt().lateralShift(offset);
      }
      planning_start_point.mutable_path_pt()->set_z(input_data.localization->vehicleAlignPosePoint().z());
      planning_start_point.mutable_path_pt()->set_theta(compensate_heading);
      planning_start_point.mutable_path_pt()->set_front_steer(current_steer_angle);
      STLOG(D, "[Planning::parsePlanningStartPoint]: heading_err = ", heading_err, " > thrd = ", replan_heading_thrd);
    } else if (enable_replan && std::abs(lat_error) > replan_lateral_error_thrd) {
      // 有历史轨迹时: 横向偏差较大.
      double offset = lat_error;
      offset += lat_error > 0 ? -replan_lateral_error_thrd : replan_lateral_error_thrd;
      *planning_start_point.mutable_path_pt() = nearest_point.path_pt().lateralShift(offset);
      planning_start_point.mutable_path_pt()->set_z(input_data.localization->vehicleAlignPosePoint().z());
      planning_start_point.mutable_path_pt()->set_front_steer(current_steer_angle);
      STLOG(D, "[Planning::parsePlanningStartPoint]: lat_error = ", lat_error, " > thrd = ", replan_lateral_error_thrd,
            "  offset = ", offset);
    } else {
      // 有历史轨迹时: 偏差满足要求
      double offset = 0.0;
      *planning_start_point.mutable_path_pt() = nearest_point.path_pt().lateralShift(offset);
      planning_start_point.mutable_path_pt()->set_z(input_data.localization->vehicleAlignPosePoint().z());
      // PLOGI << "[Planning::parsePlanningStartPoint]: reused "<< std::endl;
    }

    // 纵向起点规划
    double v = nearest_point.v();
    double a = nearest_point.a();
    double ego_speed = input_data.chassis->Speed();
    std::vector<double> speed_m_s_allow_error_table = {0.0, 0.139, 0.139, 0.555};
    std::vector<double> speed_m_s_allow_lower_error_table = {0.139, 0.139, 0.139, 0.555};
    std::vector<double> ego_speed_km_h_table_for_allow_error = {3.0, 5.0, 10.0, 40.0};
    double allow_error =
        math::TableLookUp1D(ego_speed_km_h_table_for_allow_error, speed_m_s_allow_error_table, ego_speed * MS_KMH);
    v = fmin(fmax(v, ego_speed - allow_error), ego_speed + allow_error);
    if (v < kMathEpsilon) {  // 减速时缩小允许的误差
      double tolerance_for_decelerate = 0.0;
      std::vector<double> decelerate_table = {-0.3, -0.2, -0.1, 0.0};
      std::vector<double> tolerance_table = {0.0, 0.2 * allow_error, 0.8 * allow_error, allow_error};
      tolerance_for_decelerate = math::TableLookUp1D(decelerate_table, tolerance_table, a);
      v = fmin(v, ego_speed + tolerance_for_decelerate);
    }
    double allow_lower_error = math::TableLookUp1D(ego_speed_km_h_table_for_allow_error,
                                                   speed_m_s_allow_lower_error_table, ego_speed * MS_KMH);
    v = fmin(fmax(v, ego_speed - allow_lower_error), ego_speed + allow_error);
    // x_0_.a = fmin(fmax(x_0_.a, speed_planner_config_.a_hard_lower_bound()),
    // speed_planner_config_.a_hard_upper_bound());
    planning_start_point.set_v(fmax(0.0, v));
    planning_start_point.set_a(a);

  } else {
    // 无历史轨迹，或定位坐标系发生改变, 或非智驾模式时:

    planning_start_point.mutable_path_pt()->set_x(input_data.localization->vehicleAlignPosePoint().x());
    planning_start_point.mutable_path_pt()->set_y(input_data.localization->vehicleAlignPosePoint().y());
    planning_start_point.mutable_path_pt()->set_z(input_data.localization->vehicleAlignPosePoint().z());
    planning_start_point.mutable_path_pt()->set_theta(input_data.localization->vehicleAlignPosePoint().yaw());
    planning_start_point.mutable_path_pt()->set_kappa(current_vehicle_kappa);
    planning_start_point.mutable_path_pt()->set_front_steer(current_steer_angle);

    double current_local_s =
        input_data.target_ref_line_info->ref_line().getFrenetPoint(planning_start_point.path_pt()).s();
    planning_start_point.mutable_path_pt()->set_s(current_local_s);
    planning_start_point.set_v(input_data.chassis->Speed());
    planning_start_point.set_a(input_data.chassis->LogituAcc());
  }

  return planning_start_point;
}

bool BaseDataPreprocess::referenceLinePreprocess(SpatiotemporalPlannerDataManager& data_manager) {
  // 输入检查
  if (data_manager.inputData().target_ref_line_info == nullptr
      || !data_manager.inputData().target_ref_line_info->isValid())
    return false;

  // 初始化trajectory_boundary
  auto& boundary_info = data_manager.mutableBoundaryInfo();
  if (!boundary_info.trajectory_boundary) {
    boundary_info.trajectory_boundary = std::make_unique<TrajectoryBoundary>();
  }
  auto& traj_boundary = *boundary_info.trajectory_boundary;

  // 参考线等间距处理
  double ref_line_interval =
      data_manager.configInfo().spatiotemporal_preprocess_profile->ref_line_interval();  // 参考线等间距采样间隔
  const auto& ref_line = data_manager.inputData().target_ref_line_info->ref_line();
  double ego_s = ref_line
                     .getFrenetPoint(PathPt(data_manager.inputData().localization->vehicleAlignPosePoint().x(),
                                            data_manager.inputData().localization->vehicleAlignPosePoint().y(),
                                            data_manager.inputData().localization->vehicleAlignPosePoint().z()))
                     .s();

  // for(int i = 0; i < ref_line.reference_points().size(); i++){
  //   cout<<"org ref_line.reference_points().local_s()["<<i<<"]: "<<ref_line.reference_points().at(i).local_s()<<
  //   "  x = "<<ref_line.reference_points().at(i).x()
  //   <<"  y = "<<ref_line.reference_points().at(i).y()<<endl;
  // }

  if (isnan(ego_s)) {
    STLOG(E, "[BaseDataPreprocess::referenceLinePreprocess] Invalid nan number ego_s: ", ego_s);
    return false;
  }

  double back_length = config_manager_->vehicle_config().vehicle_param().length();
  back_length = min(back_length, ego_s - ref_line.reference_points().front().local_s());
  double front_length = data_manager.configInfo().spatiotemporal_preprocess_profile->min_reference_length();
  double max_acc = data_manager.configInfo().spatiotemporal_preprocess_profile->max_acc();
  double length_buffer = data_manager.configInfo().spatiotemporal_preprocess_profile->reference_length_buffer();
  double ego_speed = data_manager.inputData().chassis->Speed();
  double time_horizon = data_manager.gridsInfo().time_grid_info.horizon;
  double dt = max(0.0, (kMaxSpeedMS - ego_speed) / max_acc);
  if (dt > time_horizon) {
    front_length =
        max(front_length, ego_speed * time_horizon + 0.5 * max_acc * time_horizon * time_horizon + length_buffer);
  } else {
    front_length =
        max(front_length, ego_speed * dt + 0.5 * max_acc * dt * dt + kMaxSpeedMS * (time_horizon - dt) + length_buffer);
  }
  front_length = min(front_length, ref_line.reference_points().back().local_s() - ego_s);
  const int ref_back_num = static_cast<int>(back_length / ref_line_interval);
  const int ref_front_num = static_cast<int>(front_length / ref_line_interval);

  std::vector<ReferencePoint> reference_points;
  reference_points.reserve(ref_back_num + ref_front_num + 1);
  for (int i = ref_back_num; i > 0; i--) {
    reference_points.emplace_back(ref_line.getReferencePoint(ego_s - i * ref_line_interval));
  }
  for (int i = 0; i <= ref_front_num; i++) {
    reference_points.emplace_back(ref_line.getReferencePoint(ego_s + i * ref_line_interval));
  }

  if (reference_points.empty()) {
    STLOG(E, "[BaseDataPreprocess::referenceLinePreprocess] No valid reference points found for ego_s: ", ego_s);
    return false;
  }

  // for(auto& referencept: reference_points){
  //   cout<<"referencept local_s: "<<referencept.local_s()<<endl;
  // }

  traj_boundary.update(std::move(reference_points));
  return traj_boundary.isValid();
}

bool BaseDataPreprocess::vehicleInfoPreprocess(SpatiotemporalPlannerDataManager& data_manager) {
  // 输入检查
  const auto target_ref_line_info = data_manager.inputData().target_ref_line_info;
  const auto current_ref_line_info = data_manager.inputData().current_ref_line_info;
  if (target_ref_line_info == nullptr || current_ref_line_info == nullptr || !target_ref_line_info->isValid()
      || !current_ref_line_info->isValid()) {
    STLOG(E, "[BaseDataPreprocess::vehicleInfoPreprocess] Invalid reference line info");
    return false;
  }

  // 初始化vehicle_info
  auto& input_info = data_manager.mutableInputData();
  auto& output_info = data_manager.mutableOutputData();
  if (!input_info.vehicle_info) {
    input_info.vehicle_info = std::make_unique<SpatiotemporalPlannerDataManager::VehicleInfo>();
  }
  auto& vehicle_info = *input_info.vehicle_info;

  // 更新start_point
  vehicle_info.start_point = getPlanningStartPoint(data_manager);
  output_info.debug_info += fmt::format("\n front_steer = {:.5f} , theta = {:.5f}",
                                        vehicle_info.start_point.path_pt().front_steer() * RAD2ANG,
                                        vehicle_info.start_point.path_pt().theta() * RAD2ANG);

  // 更新sl_info
  vehicle_info.sl_info = target_ref_line_info->ref_line().toFrenetFrame(vehicle_info.start_point);
  vehicle_info.sl_info.first[0] = std::fmin(vehicle_info.sl_info.first[0], target_ref_line_info->ref_line().length());
  output_info.debug_info +=
      fmt::format("\nadc_s: <{:.2f}, {:.2f}, {:.2f}> adc_l: <{:.2f}, {:.2f}, {:.2f}>", vehicle_info.sl_info.first[0],
                  vehicle_info.sl_info.first[1], vehicle_info.sl_info.first[2], vehicle_info.sl_info.second[0],
                  vehicle_info.sl_info.second[1], vehicle_info.sl_info.second[2]);

  // 更新driven_sl_info
  vehicle_info.driven_sl_info = current_ref_line_info->ref_line().toFrenetFrame(vehicle_info.start_point);
  vehicle_info.driven_sl_info.first[0] =
      std::fmin(vehicle_info.driven_sl_info.first[0], current_ref_line_info->ref_line().length());
  // 更新规划起点s、l信息
  auto frenet_l = vehicle_info.sl_info.second[0];

  // 更新frenet_end_s
  const auto& traj_boundary = *data_manager.boundaryInfo().trajectory_boundary;
  vehicle_info.frenet_end_s = std::fmin(vehicle_info.sl_info.first[0] + traj_boundary.delta_s() * traj_boundary.size(),
                                        target_ref_line_info->ref_line().length());

  // 更新规划起点左、右边界
  constexpr double max_ldd = 0.2;
  double adc_lat_decel_buffer = (vehicle_info.sl_info.second[1] > 0 ? 0.5 : -0.5) * vehicle_info.sl_info.second[1]
                                * vehicle_info.sl_info.second[1] / max_ldd;  // buffer = v_lateral^v_lateral/2/a_lateral

  vehicle_info.curr_right_bound = std::fmin(frenet_l, frenet_l + adc_lat_decel_buffer);
  vehicle_info.curr_left_bound = std::fmax(frenet_l, frenet_l + adc_lat_decel_buffer);

    // 更新目标点信息
    auto destination_point = target_ref_line_info->ref_line().getDestinationData().destination_point;
    auto path_point = PathPt(destination_point.x(), destination_point.y(), destination_point.z(), 0.0, destination_point.heading());
    path_point.set_s(destination_point.local_s());
    path_point.set_l(destination_point.offset());
    input_info.destination_point = TrajectoryPt(path_point);

  // todo::参考线变更检查???
  return true;
}

bool BaseDataPreprocess::scenarioConfigPreprocess(SpatiotemporalPlannerDataManager& data_manager) {
  STLOG(I, "[BaseDataPreprocess::scenarioConfigPreprocess] Starting scenario and config preprocessing");

  const auto& input_data = data_manager.inputData();
  auto& scenario_info = data_manager.mutableScenarioInfo();
  auto& config_info = data_manager.mutableConfigInfo();

  // 1. 获取当前场景的管理键
  auto current_manager_key = scenario_manager_->managerKey(input_data.stage_state);
  STLOG(D, "[BaseDataPreprocess::scenarioConfigPreprocess] Current scenario key: ", "macro=",
        static_cast<int>(current_manager_key.macro), ", style=", static_cast<int>(current_manager_key.style),
        ", stage=", static_cast<int>(input_data.stage_state));

  // 2. 检测场景是否发生变化
  bool scenario_changed = false;
  if (current_manager_key != last_manager_key_) {
    // 场景键发生变化
    scenario_changed = true;
    STLOG(I, "[BaseDataPreprocess::scenarioConfigPreprocess] Scenario changed from last key: ", "macro=",
          static_cast<int>(last_manager_key_.macro), ", style=", static_cast<int>(last_manager_key_.style),
          " to current key: ", "macro=", static_cast<int>(current_manager_key.macro),
          ", style=", static_cast<int>(current_manager_key.style));
  } else {
    STLOG(D, "[BaseDataPreprocess::scenarioConfigPreprocess] Scenario unchanged");
  }

  // 3. 更新场景信息
  scenario_info.manager_key = current_manager_key;
  scenario_info.has_scenario_changed = scenario_changed;

  // 4. 如果场景发生变化，重新加载配置
  if (scenario_changed) {
    STLOG(I, "[BaseDataPreprocess::scenarioConfigPreprocess] Scenario changed, reloading configs");

    // 使用配置注册表加载所有配置
    auto& registry = ConfigRegistry::getInstance();

    if (!registry.loadAllConfigs(current_manager_key, config_info)) {
      STLOG(E, "[BaseDataPreprocess::scenarioConfigPreprocess] Failed to load all configs for scenario: ", "macro=",
            static_cast<int>(current_manager_key.macro), ", style=", static_cast<int>(current_manager_key.style));
      return false;
    }

    // 5. 验证关键配置是否加载成功
    if (!config_info.decision_object_parser_profile) {
      STLOG(E, "[BaseDataPreprocess::scenarioConfigPreprocess] Missing DecisionObjectParserProfile");
      return false;
    }

    if (!config_info.lateral_path_bound_parser_profile) {
      STLOG(E, "[BaseDataPreprocess::scenarioConfigPreprocess] Missing LateralPathBoundParserProfile");
      return false;
    }

    if (!config_info.longitudinal_bound_parser_profile) {
      STLOG(E, "[BaseDataPreprocess::scenarioConfigPreprocess] Missing LongitudinalBoundParserProfile");
      return false;
    }

    if (!config_info.spatiotemporal_optimizer_profile) {
      STLOG(E, "[BaseDataPreprocess::scenarioConfigPreprocess] Missing SpatiotemporalOptimizerProfile");
      return false;
    }

    if (!config_info.spatiotemporal_preprocess_profile) {
      STLOG(E, "[BaseDataPreprocess::scenarioConfigPreprocess] Missing SpatiotemporalPreprocessProfile");
      return false;
    }
  } else {
    STLOG(D, "[BaseDataPreprocess::scenarioConfigPreprocess] Scenario unchanged, keeping existing configs");
  }

  // 6. 记录当前场景键用于下次比较
  last_manager_key_ = current_manager_key;
  STLOG(I, "[BaseDataPreprocess::scenarioConfigPreprocess] Scenario config preprocessing completed");
  return true;
}

bool BaseDataPreprocess::timeRelatedBoundaryPreprocess(SpatiotemporalPlannerDataManager& data_manager) {
  STLOG(I, "[BaseDataPreprocess::timeRelatedBoundaryPreprocess] Starting time-related boundary preprocessing");

  auto& time_related_boundary = data_manager.mutableBoundaryInfo().time_related_boundary;
  const auto& time_grid_data = data_manager.gridsInfo().time_grid_info.grid;
  if (time_grid_data.empty()) {
    STLOG(E, "[BaseDataPreprocess::timeRelatedBoundaryPreprocess] Time grid is empty");
    return false;
  }
  if (!time_related_boundary) {
    time_related_boundary = std::make_unique<TimeRelatedBoundary>();
  }
  time_related_boundary->update(time_grid_data);
  return true;
}

bool BaseDataPreprocess::velocityRelatedBoundaryPreprocess(SpatiotemporalPlannerDataManager& data_manager) {
  STLOG(I, "[BaseDataPreprocess::velocityRelatedBoundaryPreprocess] Starting velocity-related boundary preprocessing");

  auto& velocity_related_boundary = data_manager.mutableBoundaryInfo().velocity_related_boundary;
  const auto& velocity_grid_data = data_manager.gridsInfo().velocity_grid_info.grid;
  if (velocity_grid_data.empty()) {
    STLOG(E, "[BaseDataPreprocess::velocityRelatedBoundaryPreprocess] Velocity grid is empty");
    return false;
  }
  if (!velocity_related_boundary) {
    velocity_related_boundary = std::make_unique<VelocityRelatedBoundary>();
  }
  velocity_related_boundary->update(velocity_grid_data);

  // update steer angle bound & dsteer angle bound
  auto& preprocess_config = data_manager.configInfo().spatiotemporal_preprocess_profile;
  std::vector<double> speed_vec;
  std::vector<double> steer_bound_vec;
  std::vector<double> dsteer_bound_vec;
  for (const auto& ele : preprocess_config->speed_steer_angle_bound_map().elements()) {
    speed_vec.emplace_back(ele.speed());
    steer_bound_vec.emplace_back(ele.steer_angle_bound());
    dsteer_bound_vec.emplace_back(ele.dsteer_angle_bound());
  }

  auto& steer_soft_bound = velocity_related_boundary->mutableSoftSteeringBound().mutableOriginData();
  auto& steer_hard_bound = velocity_related_boundary->mutableHardSteeringBound().mutableOriginData();
  auto& dsteer_soft_bound = velocity_related_boundary->mutableSoftDsteeringBound().mutableOriginData();
  auto& dsteer_hard_bound = velocity_related_boundary->mutableHardDsteeringBound().mutableOriginData();
  for (int i = 0; i < velocity_grid_data.size(); i++) {
    double abs_steer_bound = math::TableLookUp1D(speed_vec, steer_bound_vec, velocity_grid_data.at(i));
    double abs_dsteer_bound = math::TableLookUp1D(speed_vec, dsteer_bound_vec, velocity_grid_data.at(i));
    steer_soft_bound[i].clipLower(-abs_steer_bound);
    steer_soft_bound[i].clipUpper(abs_steer_bound);
    steer_hard_bound[i].clipLower(-abs_steer_bound);
    steer_hard_bound[i].clipUpper(abs_steer_bound);
    dsteer_soft_bound[i].clipLower(-abs_dsteer_bound);
    dsteer_soft_bound[i].clipUpper(abs_dsteer_bound);
    dsteer_hard_bound[i].clipLower(-abs_dsteer_bound);
    dsteer_hard_bound[i].clipUpper(abs_dsteer_bound);
  }
  return true;
}

bool BaseDataPreprocess::updateTrajectoryBoundary(SpatiotemporalPlannerDataManager& data_manager) {
  const auto* decision_result = data_manager.inputData().decision_result;
  if (decision_result == nullptr) {
    STLOG(E, "[BaseDataPreprocess::updateTrajectoryBoundary] decision_result is null");
    return false;
  }
  const auto& final_info = decision_result->getFinalDecisionBoundaryInfo();
  const auto& src = final_info.trajectory_boundary;
  if (src == nullptr || !src->isValid()) {
    STLOG(E, "[BaseDataPreprocess::updateTrajectoryBoundary] trajectory_boundary is null or invalid");
    return false;
  }
  auto& boundary_info = data_manager.mutableBoundaryInfo();
  if (!boundary_info.trajectory_boundary) {
    boundary_info.trajectory_boundary = std::make_unique<TrajectoryBoundary>();
  }
  auto& dest = *boundary_info.trajectory_boundary;
  const auto& ref_pts = src->referencePointsData().getOriginData();
  if (ref_pts.empty()) {
    STLOG(E, "[BaseDataPreprocess::updateTrajectoryBoundary] source reference points empty");
    return false;
  }
  dest.update(ref_pts);
  dest.setSoftLateralBound(src->softLateralBound());
  dest.setHardLateralBound(src->hardLateralBound());
  dest.setSoftSpeedBound(src->softSpeedBound());
  dest.setHardSpeedBound(src->hardSpeedBound());
  dest.setBoundaryBlockS(src->boundaryBlockS());
  return dest.isValid();
}

bool BaseDataPreprocess::updateTimeRelatedBoundary(SpatiotemporalPlannerDataManager& data_manager) {
  const auto* decision_result = data_manager.inputData().decision_result;
  if (decision_result == nullptr) {
    STLOG(E, "[BaseDataPreprocess::updateTimeRelatedBoundary] decision_result is null");
    return false;
  }
  const auto& final_info = decision_result->getFinalDecisionBoundaryInfo();
  const auto& src = final_info.time_related_boundary;
  if (src == nullptr || !src->isValid()) {
    STLOG(E, "[BaseDataPreprocess::updateTimeRelatedBoundary] time_related_boundary is null or invalid");
    return false;
  }
  auto& boundary_info = data_manager.mutableBoundaryInfo();
  if (!boundary_info.time_related_boundary) {
    boundary_info.time_related_boundary = std::make_unique<TimeRelatedBoundary>();
  }
  auto& dest = *boundary_info.time_related_boundary;
  std::vector<double> time_indices;
  time_indices.reserve(src->size());
  for (size_t i = 0; i < src->size(); ++i) {
    time_indices.push_back(src->start_s() + (static_cast<double>(i) * src->delta_s()));
  }
  if (time_indices.size() < 2) {
    STLOG(E, "[BaseDataPreprocess::updateTimeRelatedBoundary] source time grid too small");
    return false;
  }
  dest.update(time_indices);
  dest.setWallConstraintsBound(src->getWallConstraintsBound());
  dest.setStaticObjBound(src->getStaticObjBound());
  dest.setDynamicObjBound(src->getDynamicObjBound());
  dest.setSStaticSoftBound(src->sStaticSoftBound());
  dest.setSStaticHardBound(src->sStaticHardBound());
  //TODO: 需在求解中进行边界适配
  dest.setSSoftBound(src->getWallConstraintsBound());
  dest.setSHardBound(src->getWallConstraintsBound());
  for (const auto& id : src->getStaticObjIds()) {
    if (!dest.hasStaticObj(id)) {
      dest.insertStaticObjId(id);
    }
  }
  for (const auto& id : src->getDynamicObjIds()) {
    if (!dest.hasDynamicObj(id)) {
      dest.insertDynamicObjId(id);
    }
  }
  return dest.isValid();
}

bool BaseDataPreprocess::updateVelocityRelatedBoundary(SpatiotemporalPlannerDataManager& data_manager) {
  const auto* decision_result = data_manager.inputData().decision_result;
  if (decision_result == nullptr) {
    STLOG(E, "[BaseDataPreprocess::updateVelocityRelatedBoundary] decision_result is null");
    return false;
  }
  const auto& final_info = decision_result->getFinalDecisionBoundaryInfo();
  const auto& src = final_info.velocity_related_boundary;
  if (src == nullptr || !src->isValid()) {
    STLOG(E, "[BaseDataPreprocess::updateVelocityRelatedBoundary] velocity_related_boundary is null or invalid");
    return false;
  }
  auto& boundary_info = data_manager.mutableBoundaryInfo();
  if (!boundary_info.velocity_related_boundary) {
    boundary_info.velocity_related_boundary = std::make_unique<VelocityRelatedBoundary>();
  }
  auto& dest = *boundary_info.velocity_related_boundary;
  std::vector<double> velocity_indices;
  velocity_indices.reserve(src->size());
  for (size_t i = 0; i < src->size(); ++i) {
    velocity_indices.push_back(src->start_s() + (static_cast<double>(i) * src->delta_s()));
  }
  if (velocity_indices.size() < 2) {
    STLOG(E, "[BaseDataPreprocess::updateVelocityRelatedBoundary] source velocity grid too small");
    return false;
  }
  dest.update(velocity_indices);
  dest.setSoftSteeringBound(src->softSteeringBound());
  dest.setHardSteeringBound(src->hardSteeringBound());
  dest.setSoftDsteeringBound(src->softDsteeringBound());
  dest.setHardDsteeringBound(src->hardDsteeringBound());
  return dest.isValid();
}

REGIST_MODULE(BaseDataPreprocess);

}  // namespace gpal::pnc::planning
