#include "speed_planner/park_speed_planner.h"

namespace gpal::pnc::planning {

gpal::pnc::planning::Status ParkSpeedPlanner::runOnce(const LocalView& local_view, const StageState& stage_state,
                                                      const shared_ptr<ReferenceLineInfo> target_reference_line_info,
                                                      const shared_ptr<DecisionResult> decision_result,
                                                      const shared_ptr<PathData> path_data, int64_t time_stamp,
                                                      shared_ptr<SpeedResult> speed_result) {
  speed_result->clear();
  obstacle_remain_distance_ = 10000.0;
  ERT_PLOG_I << "[ParkSpeedPlanner] process with context";

  if (path_data->discretized_path().empty() || path_data->getPlannerStatus() == PathData::StatusType::FAILED) {
    ERT_PLOG_I << "[ParkSpeedPlanner] Error !!!!!!  lateral path is invalid  ";
    resetTrajectory(local_view.getLocalizationPtr(), speed_result->mutableTrajectoryResult());
    speed_result->setDestinationStopFlag(getDestinationStopFlag(local_view, *path_data));
    return Status(ErrorCode::PLANNING_ERROR, " lateral path is empty");
  }
  auto time_start = GetSystemUsTime();
  ERT_PLOG_I << "[ParkSpeedPlanner] calculate init state ";

  calcBehaviorState(stage_state, path_data->discretized_path(), speed_result->mutableBehaviorState());
  speed_result->mutableBehaviorState()->park_in_state_ = true;
  speed_result->mutableBehaviorState()->search_parklot_state_ = false;
  // 0. 纵向初始u状态计算
  calcInitState(local_view);

  // 1.速度规划预处理
  // a.横向轨迹校验及预处理
  // b.障碍物预处理（？？）
  // c.限速计算（曲率限速，地图限速等）
  ERT_PLOG_I << "[ParkSpeedPlanner] speed preprocessor start ";

  speed_preprocessor_ptr_->ParkProcess(local_view, *path_data, *decision_result, x_0_, speed_result);
  auto lateral_path_group = speed_preprocessor_ptr_->getLateralPathResult();
  auto obstacle_map = speed_result->obstacleSet();

  if (lateral_path_group.discretized_path_group_.origin_path_.empty()
      || lateral_path_group.discretized_path_group_.extand_interval_path_.empty()) {
    ERT_PLOG_I << "[ParkSpeedPlanner] Error !!!!!!  lateral path is invalid  ";
    resetTrajectory(local_view.getLocalizationPtr(), speed_result->mutableTrajectoryResult());
    speed_result->setDestinationStopFlag(getDestinationStopFlag(local_view, *path_data));
    return gpal::pnc::planning::Status::OK();
  }

  // 2.障碍物st图计算及driveboundary生成
  // a.st图投影计算
  // b.
  // c.driveboundary生成
  // d.st图显示信息
  ERT_PLOG_I << "[ParkSpeedPlanner] st graph start ";

  st_graph_processor_ptr_->process(local_view, *decision_result, lateral_path_group, speed_result->getBehaviorState(),
                                   time_stamp, obstacle_map, speed_result);

  // 3.求解器参数计算
  ERT_PLOG_I << "[ParkSpeedPlanner] speed constraint calculate ";
  auto speed_param = speed_model_param_ptr_->calculateSpeedModelParam(
      local_view, *decision_result, obstacle_map, lateral_path_group.discretized_path_group_, x_0_, speed_result);

  // 4.优化求解：
  ERT_PLOG_I << "[ParkSpeedPlanner] speed ocp qp optimizer start ";
  speed_ocp_qp_optimizer_->runOptimizer(*speed_model_param_ptr_, x_0_);
  auto x_u = speed_ocp_qp_optimizer_->getOcpSolvedResult();
  auto risk_x_u = speed_ocp_qp_optimizer_->getRiskOcpSolvedResult();
  // 5.结果输出
  ERT_PLOG_I << "[ParkSpeedPlanner] speed data post ";
  speedDataPostProcess(local_view, obstacle_map, *path_data, *decision_result, speed_result->localPathLowerBoundary(),
                       speed_result->nearestInvasionObstacle(), x_u, speed_result->mutable_speed_data(),
                       speed_result->mutable_stop_reason());

  // for (auto& sp : frame->getSpeedResult()->speed_data()) {
  //   ERT_PLOG_I << "speed_result:  s = " << sp.s() << "  v = " << sp.v() << "  a = " << sp.a() << "   t = " << sp.t()
  //   ;
  // }

  getTrajectory(local_view.getLocalizationPtr(), path_data->discretized_path(), speed_result->speed_data(),
                speed_result->mutableTrajectoryResult());  
  resetTrajectoryResultBasedAbortTask(local_view, path_data->discretized_path(), stage_state, speed_result);
  speed_result->setDestinationStopFlag(getDestinationStopFlag(local_view, *path_data));

  obstacle_remain_distance_ = max(0.0, obstacle_remain_distance_ - speed_planner_config_.obs_stop_buffer());
  speed_result->setObstacleRemainDistance(obstacle_remain_distance_);

  return gpal::pnc::planning::Status::OK();
}

void ParkSpeedPlanner::speedDataPostProcess(const LocalView& local_view, const ObstacleSet& obstacle_map,
                                            const PathData& path_data, const DecisionResult& decision_result,
                                            const std::vector<std::pair<STPoint, std::string>>& local_path_lower_bound,
                                            const InvasionObstacle& nearest_invasion_obstacle,
                                            const std::pair<std::vector<OcpVariable>, std::vector<OcpVariable>>& x_u,
                                            SpeedData* speed_data, StopReason* stop_reason) {
  setOcpSpeedData(x_u, speed_data);
  if (speed_planner_config_.use_const_park_speed()) {
    setConstParkSpeedData(path_data, speed_data);
  } else if (speed_planner_config_.use_empirical_park_speed()) {
    setEmpiricalParkSpeedData(path_data, nearest_invasion_obstacle, speed_data);
  }
  decideStopBasedOnNearestObstacle(nearest_invasion_obstacle, speed_data);
  emergencyStopBasedLocalPath(path_data, local_path_lower_bound, decision_result, speed_data, stop_reason);
  if (local_view.getConsolePtr()->taskStage()
      == proto::TaskCommand::TaskStage::TaskCommand_TaskStage_kParkingSuspendTask) {
    resetSpeedData(0.0, 0.0, speed_data);
    stop_reason->updateStopReason("suspend_task", StopReason::StopReasonType::SUSPEND_TASK);
  }
  stopReasonCheck(local_view, nearest_invasion_obstacle, speed_data, stop_reason);
}

void ParkSpeedPlanner::setConstParkSpeedData(const PathData& path_data, SpeedData* speed_data) {
  if (!path_data.getRemainDisInfo().first) {
    return;
  }
  float remain_dis = max(path_data.getRemainDisInfo().second, 0.0);
  double desire_speed = std::sqrt(remain_dis * 0.15 * 2);
  desire_speed = fmin(desire_speed, 2.0 * KMH_MS);

  size_t speed_data_size_ = std::round(time_horizon_ / time_resolution_) + 1UL;
  if (speed_data->size() != speed_data_size_) {
    speed_data->resize(speed_data_size_);
  }
  if (last_speed_data_.size() != speed_data_size_) {
    last_speed_data_.resize(speed_data_size_);
  }
  for (size_t i = 0UL; i < speed_data_size_; i++) {
    double t = static_cast<double>(i) * time_resolution_;
    speed_data->at(i).set_t(t);
    speed_data->at(i).set_a(0.0);
    if (desire_speed * t < remain_dis) {
      speed_data->at(i).set_s(desire_speed * t);
      speed_data->at(i).set_v(desire_speed);
    } else {
      speed_data->at(i).set_s(remain_dis);
      speed_data->at(i).set_v(0.0);
    }
  }
}

void ParkSpeedPlanner::setEmpiricalParkSpeedData(const PathData& path_data, const InvasionObstacle& nearest_invasion_obstacle, SpeedData* speed_data) {
  if (!path_data.getRemainDisInfo().first) {
    return;
  }
  float remain_dis = max(path_data.getRemainDisInfo().second, 0.0);
  if (nearest_invasion_obstacle.obj_id_ != "None" && nearest_invasion_obstacle.obj_id_ != "destination") {
    remain_dis = fmin(remain_dis, max(nearest_invasion_obstacle.invasion_s_ - speed_planner_config_.obs_stop_buffer(), 0.0));
  }
  double init_v = std::sqrt(remain_dis * 0.15 * 2);
  init_v = fmin(init_v, 2.0 * KMH_MS);
  float desire_a = fmax(-init_v * init_v/( 2* remain_dis), -4.0);
  float delta_t = init_v/(abs(desire_a) + kEpsilon);
  size_t speed_data_size_ = std::round(time_horizon_ / time_resolution_) + 1UL;
  if (speed_data->size() != speed_data_size_) {
    speed_data->resize(speed_data_size_);
  }
  if (last_speed_data_.size() != speed_data_size_) {
    last_speed_data_.resize(speed_data_size_);
  }
  for (size_t i = 0UL; i < speed_data_size_; i++) {
    double t = static_cast<double>(i) * time_resolution_;
    speed_data->at(i).set_t(t);
    if (t < delta_t) {
      speed_data->at(i).set_a(desire_a);
      speed_data->at(i).set_v(init_v + t * desire_a);
      speed_data->at(i).set_s(init_v * t + 0.5 * desire_a * t * t);
    } else {
      speed_data->at(i).set_a(0.0);
      speed_data->at(i).set_v(0.0);
      speed_data->at(i).set_s(remain_dis);
    }
    // cout<<" t = "<<t<<"  s = "<<speed_data->at(i).s()<<" v = "<<speed_data->at(i).v()<<" a = "<<speed_data->at(i).a()<<endl;
  }
}

void ParkSpeedPlanner::decideStopBasedOnNearestObstacle(const InvasionObstacle& nearest_invasion_obstacle,
                                                        SpeedData* speed_data) {
  ERT_PLOG_I << " >>>>>>>>>>>>>>>>>>>> nearest_front_obj_id_ =  " << nearest_invasion_obstacle.obj_id_
             << " nearest_front_obj_invasion_s_ =  " << nearest_invasion_obstacle.invasion_s_;

  if (nearest_invasion_obstacle.obj_id_ != "None") {
    double front_distance = nearest_invasion_obstacle.invasion_s_;
    double stop_distance = 0.0;
    double stay_static_distance_buffer = speed_planner_config_.obs_stop_buffer();
    if (nearest_invasion_obstacle.obj_id_ == "destination") {
      stay_static_distance_buffer = speed_planner_config_.destination_stop_buffer();
      stop_distance = 0.0;
    }
    if (front_distance <= stop_distance + stay_static_distance_buffer) {
      resetSpeedData(0.0, 0.0, speed_data);
    }
    if (nearest_invasion_obstacle.obj_id_ != "destination") {
      obstacle_remain_distance_ = min(obstacle_remain_distance_, nearest_invasion_obstacle.invasion_s_);
    }
  }
}

void ParkSpeedPlanner::emergencyStopBasedLocalPath(
    const PathData& path_data, const std::vector<std::pair<STPoint, std::string>>& local_path_lower_bound,
    const DecisionResult& decision_result,
    SpeedData* speed_data, StopReason* stop_reason) {
  double stop_distance = kPostiveInfinity;
  string stop_id = "";
  double valid_time_horizon = 3.0;
  for (auto& lower_bound : local_path_lower_bound) {
    if (lower_bound.first.t() < valid_time_horizon && lower_bound.first.s() < stop_distance) {
      stop_distance = lower_bound.first.s();
      stop_id = lower_bound.second;
    }
  }
  for (auto& block_fs_info : path_data.blockFSInfo()) {
    if (block_fs_info.is_valid && block_fs_info.s < stop_distance) {
      stop_distance = block_fs_info.s;
      stop_id = "block_fs";
    }
  }
  //add od check
  PathData::BlockFSInfo block_fs_info = calculateBlockObsInfo(path_data.local_path(), decision_result);
  if (block_fs_info.is_valid && block_fs_info.s < stop_distance) {
    stop_distance = block_fs_info.s;
    stop_id = "block_od";
  }

  obstacle_remain_distance_ = min(obstacle_remain_distance_, stop_distance);
  double local_path_stop_dis = 0.7;
  if(stop_id == "block_od"){
    local_path_stop_dis = 1.0;
  }
  ERT_PLOG_I << "  stop_distance = " << stop_distance << "   stop_id = " << stop_id;
  if (stop_distance < local_path_stop_dis) {
    resetSpeedData(0.0, 0.0, speed_data);
    if (stop_id == "block_fs") {
      stop_reason->updateStopReason(stop_id, StopReason::StopReasonType::BLOCK_FS, stop_distance);
    } else {
      stop_reason->updateStopReason(stop_id, StopReason::StopReasonType::BLOCK_OD, stop_distance);
    }
  }
}


bool ParkSpeedPlanner::getDestinationStopFlag(const LocalView& local_view, const PathData& path_data) {
  bool stop_at_destination = false;
  ERT_PLOG_I << ">>>>>>>>>>>>>>>> bool   " << (int)path_data.getRemainDisInfo().first
             << "  s = " << path_data.getRemainDisInfo().second;
  if (!path_data.getRemainDisInfo().first) {
    return stop_at_destination;
  }
  double stop_distance_threshold = 0.5;
  if (path_data.getRemainDisInfo().second < stop_distance_threshold
      && local_view.getChassisPtr()->Speed() < 0.1 * KMH_MS) {
    stop_at_destination = true;
  }
  return stop_at_destination;
}

PathData::BlockFSInfo ParkSpeedPlanner::calculateBlockObsInfo(const DiscretizedPath& local_path, const DecisionResult& decision_result) {
  const auto& decision_od = decision_result.getOdDecisions();
  double valid_length = 5.0;
  double step = 0.2;
  DiscretizedPath interval_local_path;
  PathData::BlockFSInfo  block_obs_info;
  if (local_path.empty()) {
    ERT_LOG_E(" local path is empty !!!!!!!");
    return block_obs_info;
  }
  for (double s = 0.0; s <= min(local_path.length(), valid_length); s += step) {
    interval_local_path.emplace_back(local_path.evaluate(s));
  }
  double half_s_buffer = 0.2;  
  double half_l_buffer = 0.2;  
  for (const auto& pt : interval_local_path) {
    math::Box2d ego_box = getEgoBox(pt, half_s_buffer, half_l_buffer);
    for (const auto& od : *decision_od) {
      if(!od.second.is_static){
        continue;
      }
      if (ego_box.HasOverlap(od.second.cur_box)) {
        block_obs_info.s = pt.s();
        block_obs_info.is_valid = true;
        block_obs_info.path_type = PathData::PathType::LOCAL;
        block_obs_info.check_point.set_x(pt.x());
        block_obs_info.check_point.set_y(pt.y());
        block_obs_info.check_point.set_z(pt.z());
        block_obs_info.vis_pts = ego_box.GetAllCorners();
        return block_obs_info;   
      }
    }
  }
  return block_obs_info;
}

math::Box2d ParkSpeedPlanner::getEgoBox(const PathPt& check_pt, double half_s_buffer, double half_l_buffer) const {
  PathPt center_point;
  center_point.set_x(
      check_pt.x()
      + std::cos(check_pt.theta())
            * (vehicle_param_.length() / 2.0 - vehicle_param_.rear_edge_to_ego()));
  center_point.set_y(
      check_pt.y()
      + std::sin(check_pt.theta())
            * (vehicle_param_.length() / 2.0 - vehicle_param_.rear_edge_to_ego()));
  center_point.set_theta(check_pt.theta());
  math::Box2d box(math::Vec2d(center_point.x(), center_point.y()), center_point.theta(),
                  vehicle_param_.length() + half_s_buffer * 2.0,
                  vehicle_param_.width() + half_l_buffer * 2.0);
  return box;
}

}  // namespace gpal::pnc::planning