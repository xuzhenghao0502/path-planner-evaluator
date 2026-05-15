#include "redundant_planner.h"

namespace gpal::pnc::planning {

bool RedundantPlanner::init() {
  config_manager_ = Singleton<ConfigManager>::get_instance();
  CHECK_NOTNULL(config_manager_);
  planning_config_ = config_manager_->getConfig<RedundantPlannerConfig>("RedundantPlannerConfig");
  lateral_planner_.init();
  local_path_planner_.init();
  speed_planner_.init();
  return true;
}

bool RedundantPlanner::reset() {
  // 重置横向和纵向规划器
  lateral_planner_.reset();
  speed_planner_.reset();
  task_handler_.reset(200);
  return true;
}

bool RedundantPlanner::startRunningAsync(const std::shared_ptr<ReferenceLineInfo>& target_ref_line_info,
                                         const std::shared_ptr<ReferenceLineInfo>& drive_ref_line_info,
                                         const LocalView* const local_view, const DecisionResult* const decision_result,
                                         const StageState& stage_state, const int64_t& time_stamp,
                                         const std::pair<bool, double>& remain_dis_info) {
  if (!planning_config_.enable()) {
    RLOG(W, "[RedundantPlanner] Planner is disabled in configuration.");
    return false;
  }
  RLOG(I, "[RedundantPlanner] \U0001F43B start redundant planner...");
  // 检查是否已有任务在运行
  if (future_.valid() && future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    if (planning_config_.enbale_overrunning_task_reset()) {  // 如果开启了“容忍后重置”模式
      if (task_is_overrunning_) {
        // 已经容忍过一帧，任务仍然没有结束，判定为卡死，强制重置
        RLOG(E, "[RedundantPlanner] Task has been overrunning for two frames. Forcing reset.");
        task_handler_.reset(200);  // 重置任务队列，丢弃旧任务
      } else {
        // 第一次发现任务超时，进入“容忍”状态，本帧不启动新任务
        RLOG(W, "[RedundantPlanner] Task is overrunning. Allowing it to continue for one more frame.");
        task_is_overrunning_ = true;
        return false;
      }
    } else {  // 如果关闭了“重置”开关，则永远不重置
      RLOG(W, "[RedundantPlanner] Task is overrunning and reset is disabled. Waiting for it to complete.");
      return false;  // 直接返回 false，阻止启动新任务，让旧任务继续运行下去
    }
  }

  // 成功启动新任务前，重置超时容忍标志
  task_is_overrunning_ = false;

  // 为了线程安全，必须拷贝所有需要的数据，防止原始 frame 中的数据被修改或释放
  auto start_time = std::chrono::steady_clock::now();
  if (target_ref_line_info != nullptr) {
    internal_data_.target_ref_line_info_ptr = std::make_shared<ReferenceLineInfo>(*target_ref_line_info);
  }
  if (drive_ref_line_info != nullptr) {
    internal_data_.drive_ref_line_info_ptr = std::make_shared<ReferenceLineInfo>(*drive_ref_line_info);
  }

  if (decision_result != nullptr) {
    internal_data_.decision_result_ptr = deepCopyDecisionResult(decision_result);
  }

  if (local_view != nullptr) {
    internal_data_.localization_ptr = std::make_shared<Localization>(*local_view->getLocalizationPtr());
    internal_data_.console_ptr = std::make_shared<Console>(*local_view->getConsolePtr());
    internal_data_.freespace_ptr = std::make_shared<Freespace>(*local_view->getFreespacePtr());
    internal_data_.chassis_ptr = std::make_shared<Chassis>(*local_view->getChassisPtr());
    internal_data_.vehicle_state_ptr = std::make_shared<VehicleState>(*local_view->getVehicleStatePtr());
    internal_data_.local_view_ptr = deepCopyLocalView(local_view);
  }
  internal_data_.stage_state = stage_state;
  internal_data_.time_stamp = time_stamp;
  internal_data_.remain_dis_info = remain_dis_info;
  auto end_time = std::chrono::steady_clock::now();
  auto data_copy_duration = std::chrono::duration<float, std::milli>(end_time - start_time);
  RLOG(D, "[RedundantPlanner] [RedundantDataPreparation] ", data_copy_duration.count(), " ms");
  // --- 将数据拷贝时间添加到 debug_info 的最前面 ---
  internal_data_.debug_info = fmt::format("\n[R] [Copy: {:.2f}] ", data_copy_duration.count());
  // 提交异步任务
  future_ = task_handler_.execute([this] { return this->runInternal(); });
  return true;
}

std::optional<RedundantPlannerResult> RedundantPlanner::getResultIfReady() {
  return getResultWithWait(std::chrono::milliseconds(0));
}

std::optional<RedundantPlannerResult> RedundantPlanner::getResultWithWait(const std::chrono::milliseconds& timeout) {
  if (!future_.valid()) {
    return std::nullopt;  // 任务从未启动，也无历史结果可依赖
  }

  auto status = future_.wait_for(timeout);
  if (status == std::future_status::ready) {
    // 任务在规定时间内完成了
    try {
      bool success = future_.get();
      if (success) {
        std::lock_guard<std::mutex> lock(current_result_mutex_);
        return current_result_;
      }
    } catch (const std::exception& e) {
      RLOG(E, "RedundantPlanner task threw an exception: %s", e.what());
      // 即使有异常，也应该尝试 fallback
    }
  } else {
    // 任务超时了
    RLOG(W, "[RedundantPlanner] Task timed out, trying to fallback to the last successful result.");
  }

  // 无论是任务完成但失败、抛出异常，还是等待超时，都执行以下 fallback 逻辑
  std::lock_guard<std::mutex> lock(last_successful_result_mutex_);
  if (last_successful_result_.trajectory.empty()) {
    RLOG(W, "[RedundantPlanner] No last successful trajectory available for fallback.");
    return std::nullopt;
  }

  if (planning_config_.only_use_last_successful_result()) {
    if (!last_result_was_used_) {
      last_result_was_used_ = true;
      RLOG(I, "[RedundantPlanner] Providing last successful trajectory as a ONE-TIME fallback.");
      return last_successful_result_;
    } else {
      RLOG(W, "[RedundantPlanner] Last successful trajectory has already been used.");
      return std::nullopt;
    }
  } else {
    RLOG(I, "[RedundantPlanner] Providing last successful trajectory as a CONTINUOUS fallback.");
    return last_successful_result_;
  }
}

std::string RedundantPlanner::getDebugInfo() const {
  std::lock_guard<std::mutex> lock(current_result_mutex_);
  return current_debug_info_;
}

bool RedundantPlanner::runInternal() {
  if (internal_data_.speed_result_ptr == nullptr) {
    RLOG(D, "speed_result_ptr is null, creating a new SpeedResult");
    internal_data_.speed_result_ptr = std::make_shared<SpeedResult>();
  }
  internal_data_.prev_speed_data = internal_data_.speed_result_ptr->speed_data();
  if (internal_data_.path_data == nullptr) {
    RLOG(D, "path_data is null, creating a new PathData");
    internal_data_.path_data = std::make_shared<PathData>();
  }
  internal_data_.pre_path_data = internal_data_.path_data;
  internal_data_.path_data->clear();
  if (internal_data_.path_boundary_ptr == nullptr) {
    RLOG(D, "path_boundary_ptr is null, creating a new PathBoundary");
    internal_data_.path_boundary_ptr = std::make_shared<PathBoundary>();
  }
  internal_data_.planning_start_point = parsePlanningStartPoint(internal_data_);

  auto lat_start = std::chrono::steady_clock::now();
  bool lat_success = lateralProcess();
  auto lat_end = std::chrono::steady_clock::now();
  auto lat_duration = std::chrono::duration<float, std::milli>(lat_end - lat_start).count();

  bool local_path_success = local_path_planner_.processing(internal_data_);

  auto speed_start = std::chrono::steady_clock::now();
  bool speed_success = false;
  if (lat_success) {
    speed_success = speedProcess();
  }
  auto speed_end = std::chrono::steady_clock::now();
  auto speed_duration = std::chrono::duration<float, std::milli>(speed_end - speed_start).count();
  RLOG(D, "[RedundantPlanner] [LateralPlanning] ", lat_duration, " ms, success: ", lat_success);
  RLOG(D, "[RedundantPlanner] [SpeedPlanning] ", speed_duration, " ms, success: ", speed_success);
  // 格式化调试信息
  const bool unconstrained_mode = planning_config_.enable_unconstrained_lateral_mode();
  internal_data_.debug_info += fmt::format("[Lat({}): {:.2f} {}] ",
                                           unconstrained_mode ? "UC" : "C",
                                           lat_duration,
                                           lat_success ? "" : "[Failed]");
  if (lat_success) {
    internal_data_.debug_info += fmt::format("[Spd: {:.2f} {}] ", speed_duration, speed_success ? "" : "[Failed]");
  }

  bool overall_success = lat_success && speed_success;
  RedundantPlannerResult result;

  if (overall_success) {
    toResult(*this->internal_data_.speed_result_ptr, *this->internal_data_.path_data,
             this->internal_data_.localization_ptr.get(), result);
    if (result.trajectory.empty()) {
      internal_data_.debug_info += "[Failed: Trajectory is empty]";
      overall_success = false;
    }
  }

  // 更新状态
  {
    std::lock_guard<std::mutex> lock(current_result_mutex_);
    current_debug_info_ = internal_data_.debug_info;
    if (overall_success) {
      current_result_ = result;
    } else {
      current_result_.trajectory.clear();
    }
  }

  if (overall_success) {
    std::lock_guard<std::mutex> lock(last_successful_result_mutex_);
    last_successful_result_ = result;
    last_result_was_used_ = false;
  }

  return overall_success;
}

bool RedundantPlanner::lateralProcess() {
  if (!lateral_planner_.processing(this->internal_data_)) {
    RLOG(E, "[RedundantPlanner] Lateral planning failed.");
    return false;  // 横向规划失败
  }
  RLOG(D, "[RedundantPlanner] Lateral planning completed successfully.");
  return true;  // 横向规划成功
}

bool RedundantPlanner::speedProcess() {
  if (!speed_planner_.processing(this->internal_data_)) {
    RLOG(E, "[RedundantPlanner] Speed planning failed.");
    return false;  // 纵向规划失败
  }
  RLOG(D, "[RedundantPlanner] Speed planning completed successfully.");
  return true;  // 返回是否成功
}

bool RedundantPlanner::toResult(const SpeedResult& speed_result, const PathData& path_data,
                                const Localization* localization, RedundantPlannerResult& result) {
  // 清空现有轨迹数据
  result.trajectory.clear();
  result.path_data = path_data;
  result.speed_result = speed_result;
  RLOG(D, "[RedundantPlanner] Converting speed result to trajectory...");

  TrajectoryPt point;
  const auto& discretized_path = path_data.discretized_path();
  const auto& speed_data = speed_result.speed_data();
  constexpr double time_resolution = 0.1;  // 时间分辨率，单位为秒
  for (size_t i = 0; i < speed_data.size(); ++i) {
    TrajectoryPt point;
    point.set_v(speed_data[i].v());
    point.set_a(speed_data[i].a());
    point.set_relative_time(static_cast<double>(i) * time_resolution);
    if (discretized_path.empty()) {
      point.mutable_path_pt()->set_x(localization->vehicleAlignPosePoint().x());
      point.mutable_path_pt()->set_y(localization->vehicleAlignPosePoint().y());
      point.mutable_path_pt()->set_z(localization->vehicleAlignPosePoint().z());
      point.mutable_path_pt()->set_theta(localization->vehicleAlignPosePoint().yaw());
      point.mutable_path_pt()->set_s(0.0);
    } else {
      point.set_path_pt(discretized_path.evaluate(speed_data[i].s()));
    }
    result.trajectory.push_back(point);
  }
  RLOG(D, "[RedundantPlanner] Successfully transformed speed result to trajectory with ", result.trajectory.size(),
       " points.");
  return true;  // 转换成功
}

TrajectoryPt RedundantPlanner::parsePlanningStartPoint(const SnapShotData& snap_shot_data) {
  const auto& vehicle_state = snap_shot_data.vehicle_state_ptr;
  const auto& localization = snap_shot_data.localization_ptr;
  const auto& chassis = snap_shot_data.chassis_ptr;
  const auto& pre_path_data = snap_shot_data.pre_path_data;
  TrajectoryPt planning_start_point;
  std::unique_ptr<DiscretizedPath> start_path = std::make_unique<DiscretizedPath>();

  bool is_lateral_auto_mode =
      chassis->drivingMode() == proto::PowertrainInfo::DrivingMode::PowertrainInfo_DrivingMode_kCompleteAutoDrive
      || chassis->drivingMode() == proto::PowertrainInfo::DrivingMode::PowertrainInfo_DrivingMode_kAutoSteerOnly;

  if (pre_path_data != nullptr && !pre_path_data->discretized_path().empty()
      && localization->isLastOdometryLocalization() == localization->isOdometryLocalization() && is_lateral_auto_mode) {
    double min_dist = std::numeric_limits<double>::max();
    TrajectoryPt nearest_point;
    nearest_point = pre_path_data->discretized_path().getNearestPoint(
        math::Vec3d(localization->vehicleAlignPosePoint().x(), localization->vehicleAlignPosePoint().y(),
                    localization->vehicleAlignPosePoint().z()),
        min_dist);
    min_dist = std::numeric_limits<double>::max();
    TrajectoryPt nearest_point_tmp = pre_path_data->discretized_path().getNearestPoint(
        math::Vec3d(nearest_point.path_pt().x(), nearest_point.path_pt().y(), nearest_point.path_pt().z()), min_dist);
    nearest_point.mutable_path_pt()->set_s(nearest_point_tmp.path_pt().s());
    planning_start_point = nearest_point;

    const double dx = localization->vehicleAlignPosePoint().x() - nearest_point.path_pt().x();
    const double dy = localization->vehicleAlignPosePoint().y() - nearest_point.path_pt().y();
    const double cos_theta = std::cos(localization->vehicleAlignPosePoint().yaw());
    const double sin_theta = std::sin(localization->vehicleAlignPosePoint().yaw());
    const double longi_error = cos_theta * dx + sin_theta * dy;
    const double lat_error = -sin_theta * dx + cos_theta * dy;
    double heading_err = localization->vehicleAlignPosePoint().yaw() - nearest_point.path_pt().theta();
    heading_err = math::NormalizeAngle(heading_err);

    std::vector<double> speed_vec;
    std::vector<double> kappa_vec;
    auto ocp_path_optimizer_config = config_manager_->getConfig<OcpPathOptimizerConfig>("OcpPathOptimizerConfig");
    for (const auto& ele : ocp_path_optimizer_config.speed_kappa_bound_map().elements()) {
      speed_vec.emplace_back(ele.speed());
      kappa_vec.emplace_back(ele.kappa_bound());
    }
    double kapp_bound_ctrl = math::TableLookUp1D(speed_vec, kappa_vec, vehicle_state->linear_velocity() * MS_KMH);
    double is_prev_kappa_valid =
        planning_config_.enable_replan_kappa() ? (std::fabs(nearest_point.path_pt().kappa()) <= kapp_bound_ctrl) : true;

    const bool enable_replan = planning_config_.enable_replan_planning_start_point();
    const float replan_longitudinal_error_thrd = localization->isOdometryLocalization()
                                                     ? planning_config_.replan_longitudinal_error_thresold_lane()
                                                     : planning_config_.replan_longitudinal_error_thresold();
    const float replan_heading_thrd = localization->isOdometryLocalization()
                                          ? planning_config_.replan_heading_thresold_lane() * ANG2RAD
                                          : planning_config_.replan_heading_thresold() * ANG2RAD;
    const float replan_lateral_error_thrd = localization->isOdometryLocalization()
                                                ? planning_config_.replan_lateral_error_thresold_lane()
                                                : planning_config_.replan_lateral_error_thresold();
    const bool enable_replan_lateral_exceed_heading = planning_config_.enable_replan_lateral_exceed_heading();

    if (std::abs(longi_error) > replan_longitudinal_error_thrd || !is_prev_kappa_valid) {
      // 有历史轨迹时: 纵向偏差较大.
      planning_start_point.mutable_path_pt()->set_x(localization->vehicleAlignPosePoint().x());
      planning_start_point.mutable_path_pt()->set_y(localization->vehicleAlignPosePoint().y());
      planning_start_point.mutable_path_pt()->set_z(localization->vehicleAlignPosePoint().z());
      planning_start_point.mutable_path_pt()->set_theta(localization->vehicleAlignPosePoint().yaw());
      planning_start_point.mutable_path_pt()->set_kappa(vehicle_state->kappa());
      planning_start_point.mutable_path_pt()->set_front_steer(vehicle_state->front_steer());
      ERT_PLOG_I << "[Planning::parsePlanningStartPoint]: longi_error = " << longi_error
                 << " and thrd = " << replan_longitudinal_error_thrd
                 << ",  prev kappa = " << nearest_point.path_pt().kappa() << " and thrd = " << kapp_bound_ctrl;
    } else if (enable_replan && std::fabs(heading_err) > replan_heading_thrd) {
      // 有历史轨迹时: 角度偏差较大.
      double compensate_heading_err = heading_err;
      compensate_heading_err += heading_err > 0 ? -replan_heading_thrd : replan_heading_thrd;
      double compensate_heading = nearest_point.path_pt().theta() + compensate_heading_err;
      compensate_heading = math::NormalizeAngle(compensate_heading);
      ERT_PLOG_I << "[Planning::parsePlanningStartPoint]: current heading = "
                 << localization->vehicleAlignPosePoint().yaw() * RAD2ANG
                 << " path heading = " << nearest_point.path_pt().theta() * RAD2ANG
                 << " heading_err = " << heading_err * RAD2ANG
                 << " compensate_heading = " << compensate_heading * RAD2ANG;
      planning_start_point = nearest_point;
      if (enable_replan_lateral_exceed_heading && std::abs(lat_error) > replan_lateral_error_thrd) {
        double offset = lat_error;
        offset += lat_error > 0 ? -replan_lateral_error_thrd : replan_lateral_error_thrd;
        planning_start_point = nearest_point.path_pt().lateralShift(offset);
      }
      planning_start_point.mutable_path_pt()->set_z(localization->vehicleAlignPosePoint().z());
      planning_start_point.mutable_path_pt()->set_theta(compensate_heading);
      planning_start_point.mutable_path_pt()->set_front_steer(vehicle_state->front_steer());

    } else if (enable_replan && std::abs(lat_error) > replan_lateral_error_thrd) {
      // 有历史轨迹时: 横向偏差较大.
      double offset = lat_error;
      offset += lat_error > 0 ? -replan_lateral_error_thrd : replan_lateral_error_thrd;
      geneStartPath(nearest_point, pre_path_data->discretized_path(), chassis->Speed(), offset, start_path.get(),
                    &planning_start_point);
      planning_start_point.mutable_path_pt()->set_z(localization->vehicleAlignPosePoint().z());
      planning_start_point.mutable_path_pt()->set_front_steer(vehicle_state->front_steer());
      ERT_PLOG_I << "[Planning::parsePlanningStartPoint]: lat_error = " << lat_error
                 << " >  thrd = " << replan_lateral_error_thrd << "  offset = " << offset;
    } else {
      // 有历史轨迹时: 偏差满足要求
      double offset = 0.0;
      geneStartPath(nearest_point, pre_path_data->discretized_path(), chassis->Speed(), offset, start_path.get(),
                    &planning_start_point);
      planning_start_point.mutable_path_pt()->set_z(localization->vehicleAlignPosePoint().z());
    }
  } else {
    // 无历史轨迹，或定位坐标系发生改变, 或横向非智驾模式时:
    planning_start_point.mutable_path_pt()->set_x(localization->vehicleAlignPosePoint().x());
    planning_start_point.mutable_path_pt()->set_y(localization->vehicleAlignPosePoint().y());
    planning_start_point.mutable_path_pt()->set_z(localization->vehicleAlignPosePoint().z());
    planning_start_point.mutable_path_pt()->set_theta(localization->vehicleAlignPosePoint().yaw());
    planning_start_point.mutable_path_pt()->set_kappa(vehicle_state->kappa());
    planning_start_point.mutable_path_pt()->set_front_steer(vehicle_state->front_steer());
  }

  planning_start_point.set_v(chassis->Speed());
  planning_start_point.set_a(chassis->Acc());

  return planning_start_point;
}

void RedundantPlanner::geneStartPath(const TrajectoryPt& nearest_pt, const DiscretizedPath& prev_path,
                                     const double& curr_v, const double& lateral_diff, DiscretizedPath* starting_path,
                                     TrajectoryPt* starting_pt) {
  const float seconds_to_start_pointauto = 0.0;
  double s_postpone = curr_v * seconds_to_start_pointauto;
  s_postpone = std::max(0.1, s_postpone);
  starting_path->clear();
  auto s1 = nearest_pt.path_pt().s();
  auto s2 = s1 + s_postpone;
  prev_path.getPathPts(s1, s2, starting_path);
  if (starting_path->empty()) {
    *starting_pt = nearest_pt.path_pt().lateralShift(lateral_diff);
    return;
  }
  for (auto& pt : *starting_path) {
    if (lateral_diff != 0)
      pt = pt.lateralShift(lateral_diff);
    pt.set_s(pt.s() - s1);
  }
  *starting_pt = starting_path->front();
  if (starting_path->size() < 2) {
    starting_path->clear();
    return;
  }
}

std::shared_ptr<DecisionResult> RedundantPlanner::deepCopyDecisionResult(const DecisionResult* decision_result) {
  auto result = std::make_shared<DecisionResult>(*decision_result);
  result->getMutableOdDecisions() = std::make_shared<Decision::DecisionObjectMap>(*decision_result->getOdDecisions());

  return result;
}

std::shared_ptr<LocalView> RedundantPlanner::deepCopyLocalView(const LocalView* local_view) {
  auto view_copy = std::make_shared<LocalView>(*local_view);

  // 深拷贝各个组件
  view_copy->getMutableLocalizationPtr() = std::make_shared<Localization>(*local_view->getLocalizationPtr());

  view_copy->getMutableConsolePtr() = std::make_shared<Console>(*local_view->getConsolePtr());
  view_copy->getMutableFreespacePtr() = std::make_shared<Freespace>(*local_view->getFreespacePtr());
  view_copy->getMutableChassisPtr() = std::make_shared<Chassis>(*local_view->getChassisPtr());
  view_copy->getMutableVehicleStatePtr() = std::make_shared<VehicleState>(*local_view->getVehicleStatePtr());
  auto indexed_obstacles_copy = std::make_shared<IndexedObstacles>();
  // 获取原始障碍物列表
  const auto& original_obstacles = local_view->getIndexedObstaclesPtr()->items();

  // 深拷贝每个障碍物对象
  for (const auto& obs_ptr : original_obstacles) {
    if (obs_ptr) {
      // 创建障碍物的深拷贝
      auto obstacle_copy = std::make_shared<Obstacle>(*obs_ptr);
      indexed_obstacles_copy->add(obstacle_copy);
    }
  }

  view_copy->getMutableIndexedObstaclesPtr() = indexed_obstacles_copy;
  return view_copy;
}

}  // namespace gpal::pnc::planning