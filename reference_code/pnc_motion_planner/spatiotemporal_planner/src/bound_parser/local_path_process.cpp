#include "bound_parser/local_path_process.h"

namespace gpal::pnc::planning {
bool LocalPathProcess::init(LongitudinalBoundParserProfile profile) {
  profile_ = profile;
  config_manager_ = Singleton<ConfigManager>::get_instance();
  CHECK_NOTNULL(config_manager_);
  optimizer_config_ = config_manager_->getConfig<LocalPathOptimizerConfig>("LocalPathOptimizerConfig");
  vehicle_config_ = config_manager_->vehicle_config();
  optimizer_.init();
  path_planner_.init();
  return true;
}

void LocalPathProcess::reset() {
  local_path_.clear();
  block_fs_info_ = PathData::BlockFSInfo();
  block_obs_info_ = PathData::BlockFSInfo();
}

double LocalPathProcess::getLocalPathBlockDis(const Chassis* chassis, const VehicleState* vehicle_state,
                                               const Localization* loc, const Freespace* freespace,
                                               const DecisionResult& decision_result) {
  local_path_.clear();
  double min_collision_dis = 10000.0;
  calculateLocalPath(chassis, vehicle_state);
  block_fs_info_ = calculateBlockFsInfo(loc, freespace);
  if(block_fs_info_.is_valid){
    min_collision_dis = min(min_collision_dis, block_fs_info_.s);
  }
  block_obs_info_ = calculateBlockObsInfo(decision_result);
  if(block_obs_info_.is_valid){
    min_collision_dis = min(min_collision_dis, block_obs_info_.s);
  }
  return min_collision_dis;
}

void LocalPathProcess::calculateLocalPath(const Chassis* chassis, const VehicleState* vehicle_state) {
  // generate local path
  int direction = chassis->Gear() == 7 ? -1 : 1;
  local_path_ = optimizer_.GenerateFallBackLocalPath(direction, *vehicle_state, chassis->Speed(), 0.0);
  if (direction == -1) {
    std::for_each(local_path_.begin(), local_path_.end(),
                  [&](PathPt& point) { point.set_direction(PathPt::Direction::BACKWARD); });
  }
}

PathData::BlockFSInfo LocalPathProcess::calculateBlockFsInfo(const Localization* loc, const Freespace* freespace) {
  double fs_block_s = 10000.0;
  if (!local_path_.empty()) {
    // path: map->ego
    std::vector<PathPt> ego_local_path;
    Eigen::Matrix4d tf_map_2_ego = loc->getTfMap2Ego();
    for (const auto& path_pt : local_path_) {
      math::Vec3d flu_position_vec3d(path_pt.x(), path_pt.y(), path_pt.z());
      transfer::transformPoint(tf_map_2_ego, &flu_position_vec3d);
      math::Vec3d flu_rpy_vec3d(0.0, 0.0, path_pt.theta());
      transfer::transformRPY(tf_map_2_ego, &flu_rpy_vec3d);

      ego_local_path.emplace_back(path_pt);
      ego_local_path.back().set_x(flu_position_vec3d.x());
      ego_local_path.back().set_y(flu_position_vec3d.y());
      ego_local_path.back().set_z(flu_position_vec3d.z());
      ego_local_path.back().set_theta(flu_rpy_vec3d.z());
      if (path_pt.s() > 5.0) {
        break;
      }
    }
    auto block_fs_info = path_planner_.collisionCheck(
        *freespace, ego_local_path, optimizer_config_.local_path_block_fs_config().collision_check_buffer(),
        optimizer_config_.local_path_block_fs_config().corner_width(),
        optimizer_config_.local_path_block_fs_config().enable_curve_decide_process(),
        optimizer_config_.local_path_block_fs_config().curve_look_ahead_distance(),
        optimizer_config_.local_path_block_fs_config().curve_kappa_thresold(),
        optimizer_config_.local_path_block_fs_config().side_box_length(),
        optimizer_config_.local_path_block_fs_config().side_box_width());
    if (block_fs_info.is_valid) {
      block_fs_info.path_type = PathData::PathType::LOCAL;
      block_fs_info.block_point_direction =
          path_planner_.getBlockFsPointDirection(block_fs_info.check_point, block_fs_info.block_point);

      // ego->map
      Eigen::Matrix4d tf_ego_2_map = loc->getTfEgo2Map();
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
    return block_fs_info;
  }
  return PathData::BlockFSInfo();
}

PathData::BlockFSInfo LocalPathProcess::calculateBlockObsInfo(const DecisionResult& decision_result) {
  const auto& decision_od = decision_result.getOdDecisions();
  double valid_length = 5.0;
  double step = 0.2;
  DiscretizedPath interval_local_path;
  PathData::BlockFSInfo  block_obs_info;
  if (local_path_.empty()) {
    ERT_LOG_E(" local path is empty !!!!!!!");
    return block_obs_info;
  }
  for (double s = 0.0; s <= min(local_path_.length(), valid_length); s += step) {
    interval_local_path.emplace_back(local_path_.evaluate(s));
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

math::Box2d LocalPathProcess::getEgoBox(const PathPt& check_pt, double half_s_buffer, double half_l_buffer) const {
  PathPt center_point;
  center_point.set_x(
      check_pt.x()
      + std::cos(check_pt.theta())
            * (vehicle_config_.vehicle_param().length() / 2.0 - vehicle_config_.vehicle_param().rear_edge_to_ego()));
  center_point.set_y(
      check_pt.y()
      + std::sin(check_pt.theta())
            * (vehicle_config_.vehicle_param().length() / 2.0 - vehicle_config_.vehicle_param().rear_edge_to_ego()));
  center_point.set_theta(check_pt.theta());
  math::Box2d box(math::Vec2d(center_point.x(), center_point.y()), center_point.theta(),
                  vehicle_config_.vehicle_param().length() + half_s_buffer * 2.0,
                  vehicle_config_.vehicle_param().width() + half_l_buffer * 2.0);
  return box;
}

}  // namespace gpal::pnc::planning