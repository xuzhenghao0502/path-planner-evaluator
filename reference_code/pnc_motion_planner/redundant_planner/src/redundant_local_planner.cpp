#include "redundant_local_planner.h"
#include <fmt/chrono.h>
#include <limits>
#include <algorithm> // for std::for_each

namespace gpal::pnc::planning {

bool RedundantLocalPlanner::init() {
  config_manager_ = Singleton<ConfigManager>::get_instance();
  CHECK_NOTNULL(config_manager_);
  optimizer_config_ = config_manager_->getConfig<LocalPathOptimizerConfig>("LocalPathOptimizerConfig");
  
  // 初始化内部组件
  if (!optimizer_.init()) {
      RLOG(E, "[RedundantLocalPlanner] optimizer init failed.");
      return false;
  }
  if (!path_planner_.init()) {
      RLOG(E, "[RedundantLocalPlanner] path_planner init failed.");
      return false;
  }
  return true;
}

bool RedundantLocalPlanner::reset() {
  // LocalPathPlanner 并没有实质的 reset 逻辑，保持一致
  return true;
}

bool RedundantLocalPlanner::processing(SnapShotData& snap_shot_data) {
  auto t1 = std::chrono::steady_clock::now();

  // 1. 基础校验 (对应 frame == nullptr)
  if (snap_shot_data.path_data == nullptr || snap_shot_data.local_view_ptr == nullptr) {
    RLOG(E, "[RedundantLocalPlanner] input data is nullptr");
    return false; 
  }

  auto* path_data = snap_shot_data.path_data.get();
  auto* local_view = snap_shot_data.local_view_ptr.get();
  
  path_data->mutableLocalPathDebugInfo()->clear();

  // 1）clear data before begin
  debug_status_.clear();
  debug_info_ = "\n[L]: ";
  path_data->mutableLocalPath()->clear();

  // 2）local path ocp plan
  bool opt_success = processPathOptimizer(snap_shot_data);

  // 3）local path fallback plan
  // 检查是否为横向自动模式
  bool is_lateral_auto_mode = 
      local_view->getChassisPtr()->drivingMode() == proto::PowertrainInfo::DrivingMode::PowertrainInfo_DrivingMode_kCompleteAutoDrive || 
      local_view->getChassisPtr()->drivingMode() == proto::PowertrainInfo::DrivingMode::PowertrainInfo_DrivingMode_kAutoSteerOnly;

  if (path_data->local_path().empty() || path_data->discretized_path().empty()
      || path_data->discretized_path().back().s() < optimizer_config_.min_ref_path_length()
      || !is_lateral_auto_mode) {
    
    int direction = local_view->getChassisPtr()->Gear() == 7 ? -1 : 1;
    debug_info_ += fmt::format("fallback, diretion: {}; ", direction);
    path_data->mutableLocalPath()->clear();
    
    // 生成 fallback 路径
    *path_data->mutableLocalPath() = optimizer_.GenerateFallBackLocalPath(
        direction, *local_view->getVehicleStatePtr(), local_view->getChassisPtr()->Speed(), 0.0);
    
    if (direction == -1) {
      std::for_each(path_data->mutableLocalPath()->begin(),
                    path_data->mutableLocalPath()->end(),
                    [&](PathPt& point) { point.set_direction(PathPt::Direction::BACKWARD); });
    }
  }

  // 4）cal block fs info (计算碰撞检测)
  if (!path_data->local_path().empty()
      && optimizer_config_.local_path_block_fs_config().enable_fs_collision_check()) {
    
    // path: map->ego
    std::vector<PathPt> ego_local_path;
    Eigen::Matrix4d tf_map_2_ego = local_view->getLocalizationPtr()->getTfMap2Ego();
    
    for (const auto& path_pt : path_data->local_path()) {
      math::Vec3d flu_position_vec3d(path_pt.x(), path_pt.y(), path_pt.z());
      transfer::transformPoint(tf_map_2_ego, &flu_position_vec3d);
      math::Vec3d flu_rpy_vec3d(0.0, 0.0, path_pt.theta());
      transfer::transformRPY(tf_map_2_ego, &flu_rpy_vec3d);

      ego_local_path.emplace_back(path_pt);
      ego_local_path.back().set_x(flu_position_vec3d.x());
      ego_local_path.back().set_y(flu_position_vec3d.y());
      ego_local_path.back().set_z(flu_position_vec3d.z());
      ego_local_path.back().set_theta(flu_rpy_vec3d.z());
    }

    // 调用基类 path_planner_ 的 collisionCheck
    auto block_fs_info =
        path_planner_.collisionCheck(*local_view->getFreespacePtr(), ego_local_path,
                                     optimizer_config_.local_path_block_fs_config().collision_check_buffer(),
                                     optimizer_config_.local_path_block_fs_config().corner_width(),
                                     optimizer_config_.local_path_block_fs_config().enable_curve_decide_process(),
                                     optimizer_config_.local_path_block_fs_config().curve_look_ahead_distance(),
                                     optimizer_config_.local_path_block_fs_config().curve_kappa_thresold(),
                                     optimizer_config_.local_path_block_fs_config().side_box_length(),
                                     optimizer_config_.local_path_block_fs_config().side_box_width());
    
    if (block_fs_info.is_valid) {
      generateBlockFSInfo(block_fs_info, snap_shot_data);
      path_data->mutableBlockFSInfo()->emplace_back(block_fs_info);
      path_data->mutablePlannerDebugStatus()->push_back(
          PathData::DebugStatusType::BLOCK_FS_LOCAL_PATH);
    }

    // 地图边界block fs计算
    if (optimizer_config_.local_path_block_fs_config().enable_map_bound_collision_check()) {
      std::vector<math::LineSegment2d> boundary_segs = mapBoundary(snap_shot_data);
      block_fs_info = path_planner_.collisionCheck(
          boundary_segs, ego_local_path, optimizer_config_.local_path_block_fs_config().collision_check_buffer(),
          optimizer_config_.local_path_block_fs_config().enable_curve_decide_process(),
          optimizer_config_.local_path_block_fs_config().curve_look_ahead_distance(),
          optimizer_config_.local_path_block_fs_config().curve_kappa_thresold(),
          optimizer_config_.local_path_block_fs_config().side_box_length(),
          optimizer_config_.local_path_block_fs_config().side_box_width());

      if (block_fs_info.is_valid) {
        generateBlockFSInfo(block_fs_info, snap_shot_data);
        path_data->mutableBlockFSInfo()->emplace_back(block_fs_info);
        path_data->mutablePlannerDebugStatus()->push_back(
            PathData::DebugStatusType::BLOCK_FS_LOCAL_PATH);
      }
    }
  }

  // Debug Info
  for (const auto& bs : path_data->blockFSInfo()) {
    debug_info_ += fmt::format("\n");
    debug_info_ +=
        fmt::format("block_fs_info: valid: {};  type: {};  s: {:.2f}; ", bs.is_valid, (int)bs.path_type, bs.s);
  }

  debug_info_ += fmt::format("\n");
  *(path_data->mutableLocalPathDebugInfo()) = debug_info_;
  
  // 将 debug info 追加到 snap_shot_data 的总 debug info 中以便外层查看
  snap_shot_data.debug_info += debug_info_;

  auto t2 = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  RLOG(D, fmt::format("[RedundantLocalPlanner] cost: {} ms", duration));

  return true;
}

bool RedundantLocalPlanner::processPathOptimizer(SnapShotData& snap_shot_data) {
  auto* path_data = snap_shot_data.path_data.get();
  auto* local_view = snap_shot_data.local_view_ptr.get();

  if (path_data->discretized_path().empty()) {
    debug_info_ += fmt::format("path empty; ");
    return false;
  }

  // 1）preProcess
  optimizer_.setProfile("regular");
  debug_info_ += fmt::format("regular; ");
  if (!optimizer_.preProcess(snap_shot_data.planning_start_point, path_data->discretized_path(), "regular")) {
    RLOG(I, "[RedundantLocalPlanner::processPathOptimizer]: preProcess FAILD");
    debug_info_ += fmt::format("preprocess failed; ");
    return false;
  }

  // 2）local path plan
  const SpeedData speed_data; 
  auto status = optimizer_.generateLocalPathProc(*local_view->getVehicleStatePtr(), speed_data,
                                                 path_data->discretized_path(), snap_shot_data.time_stamp);
  
  *path_data->mutableLocalPath() = optimizer_.getLocalPathResult();
  
  if (status == LocalPathOptimizer::LocalPathStatus::SOLVE_FAILD) {
    *path_data->mutableLocalPath() = optimizer_.getLocalPathInitGuess();
    debug_info_ += fmt::format("ocp failed, use init guess; ");
  }

  // 3）extend local path when too short
  if (!path_data->local_path().empty()
      && path_data->local_path().back().s() < optimizer_config_.force_tail_length()) {
    
    auto start_pose = path_data->mutableLocalPath()->back();
    VehicleState start_state;
    start_state.set_x(start_pose.x());
    start_state.set_y(start_pose.y());
    start_state.set_z(start_pose.z());
    start_state.set_yaw(start_pose.theta());
    
    const float cur_speed = 1.0f; 
    const float cur_s = path_data->mutableLocalPath()->back().s();
    int direction = optimizer_.getReverseMode() ? -1 : 1;
    
    auto res_local_path = optimizer_.GenerateFallBackLocalPath(direction, start_state, cur_speed, cur_s);
    path_data->mutableLocalPath()->insert(
        path_data->mutableLocalPath()->end(), res_local_path.begin(), res_local_path.end());
    debug_info_ += fmt::format("{:.2f}m, so extend; ", cur_s);
  }

  // 4）add the path direction when reverse plan
  if (optimizer_.getReverseMode()) {
    std::for_each(path_data->mutableLocalPath()->begin(),
                  path_data->mutableLocalPath()->end(),
                  [&](PathPt& point) { point.set_direction(PathPt::Direction::BACKWARD); });
  }
  debug_info_ += fmt::format("reverse:{}; ", static_cast<int>(optimizer_.getReverseMode()));

  return true;
}

std::vector<math::LineSegment2d> RedundantLocalPlanner::mapBoundary(SnapShotData& snap_shot_data) {
  // 适配 targetReferenceLineInfo
  if (snap_shot_data.target_ref_line_info_ptr == nullptr || !snap_shot_data.target_ref_line_info_ptr->isValid()) {
    return std::vector<math::LineSegment2d>();
  }

  auto* local_view = snap_shot_data.local_view_ptr.get();
  Eigen::Matrix4d tf_map_2_ego = local_view->getLocalizationPtr()->getTfMap2Ego();
  
  std::vector<math::LineSegment2d> boundary_seg;
  const double min_length = 2.0;
  const double max_heading_err = 0.17;
  const double map_boundary_range = optimizer_config_.local_path_block_fs_config().map_bound_range();
  
  auto ref = snap_shot_data.target_ref_line_info_ptr->ref_line();
  math::Vec3d adc_pose = math::Vec3d(ref.adcLocalization().x(), ref.adcLocalization().y(), ref.adcLocalization().z());

  for (const auto& lane_marking : local_view->getEnvRoadCognitionPtr()->getLaneMarkings()) {
    std::vector<math::Vec2d> current_segment_pts;

    for (const auto& pt : lane_marking.points) {
      double distance = pt.DistanceTo(adc_pose);

      if (distance <= map_boundary_range) {
        math::Vec3d vec3d = pt;
        transfer::transformPoint(tf_map_2_ego, &vec3d);
        current_segment_pts.emplace_back(vec3d.x(), vec3d.y());
      } else if (!current_segment_pts.empty()) {
        std::vector<math::LineSegment2d> temp_seg;
        appendLineSegments(current_segment_pts, temp_seg, min_length, max_heading_err);
        boundary_seg.insert(boundary_seg.end(), temp_seg.begin(), temp_seg.end());
        current_segment_pts.clear();
      }
    }

    if (!current_segment_pts.empty()) {
      std::vector<math::LineSegment2d> temp_seg;
      appendLineSegments(current_segment_pts, temp_seg, min_length, max_heading_err);
      boundary_seg.insert(boundary_seg.end(), temp_seg.begin(), temp_seg.end());
    }
  }

  return boundary_seg;
}

void RedundantLocalPlanner::appendLineSegments(const std::vector<math::Vec2d>& points,
                                               std::vector<math::LineSegment2d>& segments, double min_length,
                                               double max_heading_err) {
  if (points.empty()) {
    return;
  }
  for (auto iter = points.begin(); iter != points.end(); iter++) {
    if (segments.empty()) {
      if (iter != points.begin()) {
        segments.emplace_back(*(iter - 1), *iter);
      }
      continue;
    }
    auto pos_err = (*iter) - segments.back().end();
    if (pos_err.Length() < min_length) {
      continue;
    }
    const double heading_err = math::NormalizeAngle(pos_err.Angle() - segments.back().heading());
    if (std::abs(heading_err) < max_heading_err) {
      segments.back() = math::LineSegment2d(segments.back().start(), *iter);
      continue;
    }
    segments.emplace_back(segments.back().end(), *iter);
  }
}

void RedundantLocalPlanner::generateBlockFSInfo(PathData::BlockFSInfo& block_fs_info,
                                                const SnapShotData& snap_shot_data) {
  auto* local_view = snap_shot_data.local_view_ptr.get();
  block_fs_info.path_type = PathData::PathType::LOCAL;
  block_fs_info.block_point_direction =
      path_planner_.getBlockFsPointDirection(block_fs_info.check_point, block_fs_info.block_point);

  Eigen::Matrix4d tf_ego_2_map = local_view->getLocalizationPtr()->getTfEgo2Map();
  math::Vec3d map_check_point(block_fs_info.check_point.x(), block_fs_info.check_point.y(),
                              block_fs_info.check_point.z());
  transfer::transformPoint(tf_ego_2_map, &map_check_point);
  block_fs_info.check_point = PathPt(map_check_point.x(), map_check_point.y(), map_check_point.z());

  math::Vec3d map_block_point(block_fs_info.block_point.x(), block_fs_info.block_point.y(), 0.0);
  transfer::transformPoint(tf_ego_2_map, &map_block_point);
  block_fs_info.block_point = math::Vec2d(map_block_point.x(), map_block_point.y());

  std::vector<math::Vec2d> map_vis_pts;
  for (const auto& vis_pt : block_fs_info.vis_pts) {
    math::Vec3d map_vis_pt(vis_pt.x(), vis_pt.y(), 0.0);
    transfer::transformPoint(tf_ego_2_map, &map_vis_pt);
    map_vis_pts.emplace_back(map_vis_pt.x(), map_vis_pt.y());
  }
  block_fs_info.vis_pts = std::move(map_vis_pts);
}

}  // namespace gpal::pnc::planning