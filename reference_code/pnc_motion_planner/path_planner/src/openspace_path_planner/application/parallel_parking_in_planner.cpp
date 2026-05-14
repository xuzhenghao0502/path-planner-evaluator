#include "openspace_path_planner/application/parallel_parking_in_planner.h"

namespace gpal::pnc::planning {
void ParallelParkingInPlanner::convertSlotCorners(std::vector<PathPt>& slot_corners,
                                                  const std::pair<bool, float>& slot_direction) {
  // 水平库位角点约定
  // -------------------------------------------------
  // 输入: 顺时针角点, 0-3为入口
  // 内部: 转换后, 0-3为顶部(车头朝向), 0-1为入口
  // -------------------------------------------------

  if (park_env_type_ == ParkEnvType::DEAD_END) {
    if (slot_corners[1].y() > 0) {
      std::swap(slot_corners[1], slot_corners[3]);
    } else {
      slot_corners.insert(slot_corners.begin(), slot_corners.back());
      slot_corners.pop_back();
    }
    return;
  }

  if (slot_direction.first) {
    std::swap(slot_corners[1], slot_corners[3]);  // 使角点0和1为库位入口方向
    double curr_slot_heading =
        atan2(slot_corners[0].y() - slot_corners[1].y(), slot_corners[0].x() - slot_corners[1].x());
    // 确保角点0和3为库位top方向
    if (abs(math::NormalizeAngle(curr_slot_heading - slot_direction.second)) > M_PI_2) {
      // 角点0和3为库位bottom方向，需要对调角点
      std::swap(slot_corners[1], slot_corners[0]);
      std::swap(slot_corners[2], slot_corners[3]);
    }
  } else {
    // 判断库位在自车的左侧还是右侧
    if (slot_corners[1].y() > 0) {
      slot_corners.insert(slot_corners.begin(), slot_corners.back());
      slot_corners.pop_back();
    } else {
      std::swap(slot_corners[1], slot_corners[3]);
    }
  }
}

void ParallelParkingInPlanner::parkEnvRecognizer(const LocalView& local_view,
                                                 const std::vector<Decision::DecisionObject>& select_ods,
                                                 const std::vector<math::LineSegment2d>& boundary_seg) {
  park_env_type_ = ParkEnvType::GENERAL;

  // 1. 断头路环境识别
  if (planner_profile_->park_env_recog_config().enable_park_env_recognizer()) {
    bool is_dead_end = true;

    // 1.1 基于库位角点，确定采样box中心点和box方向
    double sample_heading = 0.0;
    double sample_box_heading = 0.0;
    std::vector<PathPt> slot_corners;
    PathPt slot_top_inner_pt, slot_top_outer_pt;
    for (int i = 0; i < 4; i++) {
      PathPt corner;
      corner.set_x(local_view.getParkLotPtr()->data_.gstSpace[0].gfPos[i][0]);
      corner.set_y(local_view.getParkLotPtr()->data_.gstSpace[0].gfPos[i][1]);
      corner.set_z(local_view.getParkLotPtr()->data_.gstSpace[0].gfPos[i][2]);
      slot_corners.push_back(corner);
    }
    if (slot_corners[1].y() > 0) {
      slot_top_inner_pt = slot_corners[2];
      slot_top_outer_pt = slot_corners[3];
      sample_heading =
          atan2(slot_top_outer_pt.y() - slot_top_inner_pt.y(), slot_top_outer_pt.x() - slot_top_inner_pt.x());
      sample_box_heading = atan2(slot_corners[3].y() - slot_corners[0].y(), slot_corners[3].x() - slot_corners[0].x());
    } else {
      slot_top_inner_pt = slot_corners[1];
      slot_top_outer_pt = slot_corners[0];
      sample_heading =
          atan2(slot_top_outer_pt.y() - slot_top_inner_pt.y(), slot_top_outer_pt.x() - slot_top_inner_pt.x());
      sample_box_heading = atan2(slot_corners[0].y() - slot_corners[3].y(), slot_corners[0].x() - slot_corners[3].x());
    }
    PARKING_LOG(D, "slot_top_inner_pt x = ", slot_top_inner_pt.x(), " y = ", slot_top_inner_pt.y());
    PARKING_LOG(D, "slot_top_outer_pt x = ", slot_top_outer_pt.x(), " y = ", slot_top_outer_pt.y());
    PARKING_LOG(D, "sample_heading = ", sample_heading * RAD2ANG,
                " sample_box_heading = ", sample_box_heading * RAD2ANG);

    // 1.2 采样检测库位通道前方道路是否通畅
    const double sample_step = planner_profile_->park_env_recog_config().sample_step();
    const int sample_num = planner_profile_->park_env_recog_config().sample_num();
    const double sample_box_width = planner_profile_->park_env_recog_config().sample_box_width();
    const double sample_box_length = planner_profile_->park_env_recog_config().sample_box_length();
    math::Vec2d sample_init_box_center(slot_top_outer_pt.x() + sample_box_width / 2.0 * cos(sample_heading),
                                       slot_top_outer_pt.y() + sample_box_width / 2.0 * sin(sample_heading));
    sample_init_box_center.set_x(sample_init_box_center.x() + sample_box_length / 2.0 * cos(sample_box_heading));
    sample_init_box_center.set_y(sample_init_box_center.y() + sample_box_length / 2.0 * sin(sample_box_heading));
    for (int i = 0; i < sample_num; i++) {
      bool is_collision = false;
      // 产生碰撞检测box
      math::Vec2d sample_box_center(sample_init_box_center.x() + i * sample_step * cos(sample_heading),
                                    sample_init_box_center.y() + i * sample_step * sin(sample_heading));
      math::Box2d sample_box(sample_box_center, sample_box_heading, sample_box_length, sample_box_width);
      PARKING_LOG(D, "sample_box: ", sample_box.DebugString());

      if (boxIsCollided(select_ods, sample_box) || boxIsCollided(boundary_seg, sample_box)
          || boxIsCollided(local_view, sample_box)) {
        is_collision = true;
        continue;
      }
      is_dead_end = false;
      break;
    }
    if (is_dead_end)
      park_env_type_ = ParkEnvType::DEAD_END;
  }

  // 2. 其他环境识别
  // nothing
  return;
}

void ParallelParkingInPlanner::slotProcess(const LocalView& local_view,
                                           const std::vector<Decision::DecisionObject>& select_ods,
                                           const std::vector<math::LineSegment2d>& boundary_seg) {
  // 获取车辆坐标系下，融合库位信息，车头前进方向为x，左为y
  std::vector<PathPt> slot_corners;
  for (int i = 0; i < 4; i++) {
    PathPt corner;
    corner.set_x(local_view.getParkLotPtr()->data_.gstSpace[0].gfPos[i][0]);
    corner.set_y(local_view.getParkLotPtr()->data_.gstSpace[0].gfPos[i][1]);
    corner.set_z(local_view.getParkLotPtr()->data_.gstSpace[0].gfPos[i][2]);
    slot_corners.push_back(corner);
  }
  auto slot_direction = local_view.getParkLotPtr()->data_.gstSpace[0].direction;
  generateSlot(slot_corners, slot_direction);

  // 基于环境对当前库位位置进行微调
  slotCorrection(local_view, select_ods, boundary_seg);

  PARKING_LOG(D, "slot_length: ", slot_.slot.length());
  PARKING_LOG(D, "destination_point x = ", slot_.destination_point.x(),
              " destination_point.y = ", slot_.destination_point.y());
}

bool ParallelParkingInPlanner::generateSearchElements(const LocalView& local_view,
                                                      const std::vector<Decision::DecisionObject>& select_ods,
                                                      const std::vector<math::LineSegment2d>& boundary_seg,
                                                      SearchElements& search_elements) {
  PathPt start_pose;
  PathPt end_pose;

  // 不停车泊入需要使用未来时刻轨迹点作为搜索起点
  if (!driving_discretized_path_.empty()) {
    auto driving_path = driving_discretized_path_;
    auto tf_map_2_ego = local_view.getLocalizationPtr()->getTfMap2Ego();
    auto tf_ego_2_map = local_view.getLocalizationPtr()->getTfEgo2Map();
    for (auto& pt : driving_path) {
      transfer::transformPoint(tf_map_2_ego, &pt);
      math::Vec3d pt_3d_rpy(0.0, 0.0, pt.theta());
      transfer::transformRPY(tf_map_2_ego, &pt_3d_rpy);
      pt.set_theta(pt_3d_rpy.z());
    }
    int nearest_idx = 0;
    double nearest_dist = std::numeric_limits<double>::max();
    findNearestPointInTraj(driving_path, PathPt(0.0, 0.0, 0.0), nearest_idx, nearest_dist);

    DiscretizedPath splice_path;
    for (int i = nearest_idx; i < driving_path.size(); i++) {
      splice_path.emplace_back(driving_path[i]);
      if (driving_path[i].DistanceTo(driving_path[nearest_idx])
          > planner_profile_->dynamic_search_config().preview_dis()) {
        start_pose = driving_path[i];
        break;
      }
      start_pose = driving_path[i];
    }
    if (!splice_path.empty()) {
      splice_path.pop_back();
    }
    if (driving_phase_splice_path_.empty()) {
      driving_phase_splice_path_ = std::move(splice_path);
    }

    if (!start_pose_map_bak_.first) {
      start_pose_map_bak_ = std::make_pair(true, start_pose);
      transfer::transformPoint(tf_ego_2_map, &start_pose_map_bak_.second);
      math::Vec3d pt_3d_rpy(0.0, 0.0, start_pose_map_bak_.second.theta());
      transfer::transformRPY(tf_ego_2_map, &pt_3d_rpy);
      start_pose_map_bak_.second.set_theta(pt_3d_rpy.z());
    }

    auto veh_pose_mappt = local_view.getLocalizationPtr()->vehicleAlignPosePoint();
    PathPt veh_pose(veh_pose_mappt.x(), veh_pose_mappt.y(), veh_pose_mappt.z(), 0.0, veh_pose_mappt.yaw());
    math::Vec2d unit_vec2d = start_pose_map_bak_.second.CreateUnitVec2d(start_pose_map_bak_.second.theta());
    math::Vec2d relative_vec2d = veh_pose - start_pose_map_bak_.second;
    non_stop_planning_dist_ = -unit_vec2d.InnerProd(relative_vec2d);
  }

  double end_pose_theta = atan2((slot_.slot.topEdgeCenter().y() - slot_.slot.bottomEdgeCenter().y()),
                                (slot_.slot.topEdgeCenter().x() - slot_.slot.bottomEdgeCenter().x()));
  end_pose = slot_.destination_point;
  end_pose.set_theta(end_pose_theta);
  search_elements = SearchElements(start_pose, end_pose);

  return true;
}

void ParallelParkingInPlanner::customBoundary(const LocalView& local_view,
                                              std::vector<Decision::DecisionObject>& select_ods) {
  if (!planner_profile_->custom_boundary_config().enable_custom_boundary()) {
    return;
  }
  /*
   * 自定义边界约束
   *
   *                       boundary_roadside
   * ------------------------------------------------------------------
   *                |  2 +———————————————————+ 3   |
   *                |    |                   |     |
   * boundary_rear  |    |bottom          top|     | boundary_front           horizon_heading
   *                |    |                   |     |                         ^
   *                |  1 +———————slot————————+ 0   |                         |
   *                                                                          ——>  longit_heading
   *                    >>>>>>>>>>>>>>>>>>>> 寻库方向
   *
   * ------------------------------------------------------------------
   *                        boundary_aisle
   */
  if (planner_profile_->custom_boundary_config().enable_custom_virtual_boundary()) {
    double longit_heading = atan2((slot_.slot.topEdgeCenter().y() - slot_.slot.bottomEdgeCenter().y()),
                                  (slot_.slot.topEdgeCenter().x() - slot_.slot.bottomEdgeCenter().x()));
    double horizon_heading = atan2((slot_.slot.corner(3).y() - slot_.slot.corner(0).y()),
                                   (slot_.slot.corner(3).x() - slot_.slot.corner(0).x()));

    std::vector<math::Vec2d> boundary_roadside;
    std::vector<math::Vec2d> boundary_front;
    std::vector<math::Vec2d> boundary_rear;
    std::vector<math::Vec2d> boundary_aisle;

    // 路沿边界
    const double roadside_gap = planner_profile_->custom_boundary_config().roadside_gap();
    math::Vec2d roadside_gap_point(slot_.slot.corner(3).x() + roadside_gap * cos(horizon_heading),
                                   slot_.slot.corner(3).y() + roadside_gap * sin(horizon_heading));

    math::Vec2d roadside_point1(roadside_gap_point.x() + 20.0 * cos(longit_heading),
                                roadside_gap_point.y() + 20.0 * sin(longit_heading));

    math::Vec2d roadside_point2(roadside_gap_point.x() - 30.0 * cos(longit_heading),
                                roadside_gap_point.y() - 30.0 * sin(longit_heading));
    boundary_roadside.push_back(roadside_point1);
    boundary_roadside.push_back(roadside_point2);
    park_roi_decider_.addBoundary(std::make_tuple("roadside", OpenspaceObjectType::ROAD_SIDE, boundary_roadside));

    // 库位前后边界
    const double before_after_gap = planner_profile_->custom_boundary_config().both_sides_gap();
    math::Vec2d front_point1(slot_.slot.corner(3).x() + before_after_gap * cos(longit_heading),
                             slot_.slot.corner(3).y() + before_after_gap * sin(longit_heading));

    math::Vec2d front_point2(slot_.slot.corner(0).x() + before_after_gap * cos(longit_heading),
                             slot_.slot.corner(0).y() + before_after_gap * sin(longit_heading));
    boundary_front.push_back(front_point1);
    boundary_front.push_back(front_point2);
    park_roi_decider_.addBoundary(std::make_tuple("slot_front", OpenspaceObjectType::SLOT, boundary_front));

    math::Vec2d rear_point1(slot_.slot.corner(2).x() - before_after_gap * cos(longit_heading),
                            slot_.slot.corner(2).y() - before_after_gap * sin(longit_heading));

    math::Vec2d rear_point2(slot_.slot.corner(1).x() - before_after_gap * cos(longit_heading),
                            slot_.slot.corner(1).y() - before_after_gap * sin(longit_heading));
    boundary_rear.push_back(rear_point1);
    boundary_rear.push_back(rear_point2);
    park_roi_decider_.addBoundary(std::make_tuple("slot_rear", OpenspaceObjectType::SLOT, boundary_rear));

    // 通道边界
    const double aisle_gap = planner_profile_->custom_boundary_config().aisle_gap();
    math::Vec2d aisle_gap_point(slot_.slot.corner(0).x() - aisle_gap * cos(horizon_heading),
                                slot_.slot.corner(0).y() - aisle_gap * sin(horizon_heading));

    math::Vec2d aisle_point1(aisle_gap_point.x() + 30.0 * cos(longit_heading),
                             aisle_gap_point.y() + 30.0 * sin(longit_heading));

    math::Vec2d aisle_point2(aisle_gap_point.x() - 20.0 * cos(longit_heading),
                             aisle_gap_point.y() - 20.0 * sin(longit_heading));

    boundary_aisle.push_back(aisle_point1);
    boundary_aisle.push_back(aisle_point2);
    park_roi_decider_.addBoundary(std::make_tuple("aisle", OpenspaceObjectType::AISLE, boundary_aisle));
  }
}

void ParallelParkingInPlanner::geometricConnect(const LocalView& local_view, const RoiDecideResult& roi) {
  if (orin_combined_flu_path_.empty()) {
    return;
  }

  // 若不停车泊入，则拼接起始行车轨迹段
  if (!driving_phase_splice_path_.empty()) {
    // 轨迹时间补偿
    trajectoryTimeCompensate(tf_ego_2_map_search_, local_view.getLocalizationPtr()->getTfMap2Ego(),
                             driving_phase_splice_path_);
    orin_combined_flu_path_.insert(orin_combined_flu_path_.begin(), driving_phase_splice_path_.begin(),
                                   driving_phase_splice_path_.end());
  }

  if (!planner_profile_->geometric_connect_config().enable_parallel_park_traj_end_extend()) {
    return;
  }

  /** 水平泊车末端轨迹增加前后调整直线 */
  PathPt back_point = orin_combined_flu_path_.back();
  double cos_theta = cos(back_point.theta());
  double sin_theta = sin(back_point.theta());

  // 确定安全延伸距离
  double max_extend_dis = planner_profile_->geometric_connect_config().parallel_park_traj_end_extend_max_dis();
  double min_extend_dis = planner_profile_->geometric_connect_config().parallel_park_traj_end_extend_min_dis();
  double extend_dis = 0.0;
  double step = back_point.direction() == PathPt::Direction::BACKWARD ? -0.01 : 0.01;
  int index = static_cast<int>(std::round(max_extend_dis / abs(step)));
  for (int i = 1; i <= index; ++i) {
    PathPt extend_point(back_point.x() + i * step * cos_theta, back_point.y() + i * step * sin_theta, back_point.z());
    extend_point.set_theta(back_point.theta());
    if (!pointCollisonCheck(
            *local_view.getFreespacePtr(), extend_point, roi,
            planner_profile_->geometric_connect_config().parallel_park_traj_end_extend_width_safe_buff(),
            planner_profile_->geometric_connect_config().parallel_park_traj_end_extend_length_safe_buff())) {
      break;
    }
    extend_dis = i * abs(step);
  }
  PARKING_LOG(D, "parallel_park_traj_end_extend_extend_dis: ", extend_dis);

  std::vector<PathPt> extend_path;
  step = 0.2;
  index = static_cast<int>(std::round(extend_dis / abs(step)));
  if (index > 0 && extend_dis > min_extend_dis) {
    // 扩展延伸轨迹
    step = back_point.direction() == PathPt::Direction::BACKWARD ? -extend_dis / index : extend_dis / index;
    for (int i = 1; i <= index; ++i) {
      PathPt extend_point(back_point.x() + i * step * cos_theta, back_point.y() + i * step * sin_theta, back_point.z());
      extend_point.set_theta(back_point.theta());
      extend_point.set_direction(back_point.direction());
      extend_path.emplace_back(extend_point);
    }
    orin_combined_flu_path_.insert(orin_combined_flu_path_.end(), extend_path.begin(), extend_path.end());

    // 扩展返回轨迹
    std::reverse(extend_path.begin(), extend_path.end());
    auto return_direc = back_point.direction() == PathPt::Direction::BACKWARD ? PathPt::Direction::FORWARD
                                                                              : PathPt::Direction::BACKWARD;
    back_point.set_direction(return_direc);
    back_point.set_kappa(0.0);
    extend_path.emplace_back(back_point);
    std::for_each(extend_path.begin(), extend_path.end(),
                  [return_direc](PathPt& pt) { pt.set_direction(return_direc); });
    orin_combined_flu_path_.insert(orin_combined_flu_path_.end(), extend_path.begin(), extend_path.end());
  }
}

void ParallelParkingInPlanner::trajPostProcess(const LocalView& local_view, const RoiDecideResult& roi) {
  // nothing
  return;
}

bool ParallelParkingInPlanner::isParkingCompleted(const LocalView& local_view, const bool destination_stop_flag) {
  if (planner_status_ == PlannerStatus::FINISHED)
    return true;
  is_last_traj_ = false;
  if (realtime_optimizer_segmented_flu_paths_.size() <= 1) {
    is_last_traj_ = true;
    if ((fabs(local_view.getChassisPtr()->Speed()) < (planner_profile_->parkin_stop_speed() * KMH_MS))
        && remain_dis_ <= planner_profile_->shift_min_dis_require() && destination_stop_flag) {
      // 反馈纵向泊车完成
      planner_status_ = PlannerStatus::FINISHED;
      return true;
    }
  }
  return false;
}

void ParallelParkingInPlanner::calVehicleSlotClearance() {
  std::vector<PathPt> slot_left_line, slot_right_line, slot_rear_line, slot_front_line;
  // 确定库位四个边线
  if (slot_.slot.direction() == Slot::SlotDirec::LEFT) {
    slot_left_line = {slot_.slot.corner(2), slot_.slot.corner(3)};
    slot_right_line = {slot_.slot.corner(1), slot_.slot.corner(0)};
    slot_front_line = {slot_.slot.corner(0), slot_.slot.corner(3)};
    slot_rear_line = {slot_.slot.corner(2), slot_.slot.corner(1)};
  } else {
    slot_left_line = {slot_.slot.corner(1), slot_.slot.corner(0)};
    slot_right_line = {slot_.slot.corner(2), slot_.slot.corner(3)};
    slot_front_line = {slot_.slot.corner(3), slot_.slot.corner(0)};
    slot_rear_line = {slot_.slot.corner(1), slot_.slot.corner(2)};
  }
  ReCalculateLineHeading(slot_left_line);
  ReCalculateLineHeading(slot_right_line);
  ReCalculateLineHeading(slot_rear_line);
  ReCalculateLineHeading(slot_front_line);

  // 计算自车六个计算点
  PathPt LF_wheel(vehicle_config_->vehicle_param().wheel_base(),
                  vehicle_config_->vehicle_param().width_without_rearview_mirror() / 2.0, slot_.slot.center().z());
  PathPt RF_wheel(vehicle_config_->vehicle_param().wheel_base(),
                  -vehicle_config_->vehicle_param().width_without_rearview_mirror() / 2.0, slot_.slot.center().z());
  PathPt LR_wheel(0.0, vehicle_config_->vehicle_param().width_without_rearview_mirror() / 2.0, slot_.slot.center().z());
  PathPt RR_wheel(0.0, -vehicle_config_->vehicle_param().width_without_rearview_mirror() / 2.0,
                  slot_.slot.center().z());
  PathPt veh_front_edge(vehicle_config_->vehicle_param().front_edge_to_ego(), 0.0, slot_.slot.center().z());
  PathPt veh_rear_edge(-vehicle_config_->vehicle_param().rear_edge_to_ego(), 0.0, slot_.slot.center().z());

  // 计算误差距离
  auto L_F_error = CalculateRoadError(LF_wheel, slot_left_line);
  auto R_F_error = CalculateRoadError(RF_wheel, slot_right_line);
  auto L_R_error = CalculateRoadError(LR_wheel, slot_left_line);
  auto R_R_error = CalculateRoadError(RR_wheel, slot_right_line);
  auto front_error = CalculateRoadError(veh_front_edge, slot_front_line);
  auto rear_error = CalculateRoadError(veh_rear_edge, slot_rear_line);

  debug_info_ += fmt::format(
      "Vehicle_Slot_Clearance: Left: [{:.2f}  {:.2f}], Right: [{:.2f}  {:.2f}], Front_Rear: [{:.2f}  {:.2f}]\n",
      -L_F_error.f_rear_road_error, -L_R_error.f_rear_road_error, R_F_error.f_rear_road_error,
      R_R_error.f_rear_road_error, front_error.f_rear_road_error, rear_error.f_rear_road_error);
}

bool ParallelParkingInPlanner::isNeedGeometricReplan() {
  if (!planner_profile_->parking_replan_config().enable_geometric_replan()) {
    return false;
  }
  // 当自车处于库位内部时，判定成功
  PathPt veh_left_rear_pt = calOffsetPoint(PathPt(0, 0, 0), -vehicle_config_->vehicle_param().rear_edge_to_ego(),
                                           vehicle_config_->vehicle_param().width_without_rearview_mirror() / 2);
  PathPt veh_right_rear_pt = calOffsetPoint(PathPt(0, 0, 0), -vehicle_config_->vehicle_param().rear_edge_to_ego(),
                                            -vehicle_config_->vehicle_param().width_without_rearview_mirror() / 2);
  math::LineSegment2d slot_inner_side_line(slot_.slot.corner(2), slot_.slot.corner(3));
  math::Vec2d foot_point;
  double dis_lr_pt_2_slot_inner_side = slot_inner_side_line.GetPerpendicularFoot(veh_left_rear_pt, &foot_point);
  double dis_rr_pt_2_slot_inner_side = slot_inner_side_line.GetPerpendicularFoot(veh_right_rear_pt, &foot_point);
  double angle_veh_2_slot = slot_.slot.heading() * RAD2ANG;
  PARKING_LOG(I, "dis_lr_pt_2_slot_inner_side = ", dis_lr_pt_2_slot_inner_side);
  PARKING_LOG(I, "dis_rr_pt_2_slot_inner_side = ", dis_rr_pt_2_slot_inner_side);
  PARKING_LOG(I, "angle_veh_2_slot = ", angle_veh_2_slot);
  if (std::max(dis_lr_pt_2_slot_inner_side, dis_rr_pt_2_slot_inner_side)
          < planner_profile_->parking_replan_config().geometric_replan_dis_threshold()
      && (fabs(angle_veh_2_slot) < planner_profile_->parking_replan_config().geometric_replan_angle_threshold()
          || fabs(angle_veh_2_slot)
                 > (180 - planner_profile_->parking_replan_config().geometric_replan_angle_threshold()))) {
    return true;
  }
  return false;
}

bool ParallelParkingInPlanner::generateGeometricFineTunePath(const LocalView& local_view, const RoiDecideResult& roi,
                                                             const double& radius, const int max_shift_num,
                                                             const double short_path_length,
                                                             const double short_path_extend_max_length) {
  /**1. 计算库位无碰撞空间*/
  auto free_space_slot = calFreeSpaceSlot(local_view, roi, slot_.slot);

  /**2. 计算揉库轨迹*/
  PathPt start_pt;
  std::vector<std::vector<PathPt>> geometric_replan_paths;

  if (!geometry_path_generator_.calFineTuningPathInsideParallelSlot(free_space_slot, start_pt, radius, max_shift_num,
                                                                    geometric_replan_paths)) {
    PARKING_LOG(W, "calFineTuningPathInsideParallelSlot failed");
    return false;
  } else {
    for (auto segs : roi.obstacles_linesegments) {
      for (auto seg : std::get<2>(segs)) {
        PARKING_LOG(I, "roi_boundary: start.x = ", seg.start().x(), " start.y = ", seg.start().y(),
                    " end.x = ", seg.end().x(), " end.y = ", seg.end().y());
      }
    }
    for (auto corner : slot_.slot.corners()) {
      PARKING_LOG(I, "slot_corner: corner.x = ", corner.x(), " corner.y = ", corner.y());
    }
  }
  std::vector<DiscretizedPath> geometric_replan_discretized_paths(geometric_replan_paths.begin(),
                                                                  geometric_replan_paths.end());

  /**3. 轨迹接回库位中间*/
  PathPt back_pt = geometric_replan_discretized_paths.back().back();
  PathPt back_extend_pt =
      PathPt(back_pt.x() + 1.0 * std::cos(back_pt.theta()), back_pt.y() + 1.0 * std::sin(back_pt.theta()), back_pt.z());
  float factor = 0;
  PathPt target_pt = CalculatePedalPoint(slot_.destination_point, back_pt, back_extend_pt, factor);
  target_pt.set_z(back_pt.z());

  double step = 0.1;
  double extend_dis = target_pt.DistanceTo(back_pt);
  int index = static_cast<int>(std::round(extend_dis / abs(step)));
  if (index > 0) {
    step = extend_dis / index;
  }
  if (factor < 0)
    step = -step;

  DiscretizedPath extend_path;
  for (int i = 0; i <= index; ++i) {
    PathPt extend_point(back_pt.x() + i * step * std::cos(back_pt.theta()),
                        back_pt.y() + i * step * std::sin(back_pt.theta()), back_pt.z());
    extend_point.set_theta(back_pt.theta());
    extend_point.set_direction(factor > 0 ? PathPt::Direction::FORWARD : PathPt::Direction::BACKWARD);
    extend_path.emplace_back(extend_point);
  }
  if (!extend_path.empty() && back_pt.direction() == extend_path.front().direction()) {
    geometric_replan_discretized_paths.back().insert(geometric_replan_discretized_paths.back().end(),
                                                     extend_path.begin() + 1, extend_path.end());
  } else {
    geometric_replan_discretized_paths.emplace_back(extend_path);
  }

  /**4. 延长短路径*/
  int size = geometric_replan_discretized_paths.size();
  PARKING_LOG(D, "factor: ", factor, " extend_dis: ", extend_dis);
  if (extend_dis < short_path_length) {
    if (factor < 0 && size > 1) {
      PathPt shift_pt = geometric_replan_discretized_paths.at(size - 2).back();

      auto straight_line =
          safe_straight_line_generator(*local_view.getFreespacePtr(), roi, shift_pt, short_path_extend_max_length,
                                       planner_profile_->trajectory_post_process_config().width_safe_buff(),
                                       planner_profile_->trajectory_post_process_config().length_safe_buff());
      geometric_replan_discretized_paths.at(size - 2).insert(geometric_replan_discretized_paths.at(size - 2).end(),
                                                             straight_line.begin(), straight_line.end());
      reverse_straight_line(straight_line);
      geometric_replan_discretized_paths.back().insert(geometric_replan_discretized_paths.back().begin(),
                                                       straight_line.begin(), straight_line.end());
    } else if (factor >= 0) {
      PathPt shift_pt = geometric_replan_discretized_paths.back().back();

      auto straight_line =
          safe_straight_line_generator(*local_view.getFreespacePtr(), roi, shift_pt, short_path_extend_max_length,
                                       planner_profile_->trajectory_post_process_config().width_safe_buff(),
                                       planner_profile_->trajectory_post_process_config().length_safe_buff());
      geometric_replan_discretized_paths.back().insert(geometric_replan_discretized_paths.back().end(),
                                                       straight_line.begin(), straight_line.end());
      reverse_straight_line(straight_line);
      geometric_replan_discretized_paths.emplace_back(straight_line);
    }
  }

  debugPrintTraj(geometric_replan_discretized_paths, "tractor_flu_path");

  /**5. 轨迹转换*/
  optimizer_segmented_map_paths_ = std::move(local2GlobalPaths(local_view, geometric_replan_discretized_paths));
  cal_info_ = debug_info_ + "finished: 1 traj_segs: " + std::to_string(optimizer_segmented_map_paths_.size()) + "\n";
  return true;
}

void ParallelParkingInPlanner::slotCorrection(const LocalView& local_view,
                                              const std::vector<Decision::DecisionObject>& select_ods,
                                              const std::vector<math::LineSegment2d>& boundary_seg) {
  corrected_slot_corner_ = corrected_slot_.second;
  if (!planner_profile_->slot_correction_config().enable_slot_correction()) {
    return;
  }

  /****** 水平库位泊入位置调整策略 ******
   *
   * 当检测库位被障碍物占据时，寻找最近可用泊入位置：
   *
   * 阶段一：邻近区域搜索（针对轻微/局部障碍）
   * 1. 在当前库位前后搜索可用位置
   * 2. 逐步向外侧偏移，重复前后搜索
   * 3. 选择前后移动距离最小的可用位置
   *
   * 适用：轻微路沿突出、库位前后有小段空闲等情况
   *
   * 阶段二：外侧大范围搜索（针对完全占用）
   * 1. 大幅增加外侧搜索范围
   * 2. 只检查向外侧平移后的位置是否可用
   * 3. 找到第一个可用位置或超出范围
   *
   * 适用：库位及前后区域均被车辆占据
   *
   * 结果：返回最优位置或搜索失败
   */

  // 0. 当前库位中,获取停靠位置的自车box(考虑后视镜)
  const double slot_heading = slot_.slot.heading();
  const double slot_door_heading = slot_.slot.doorHeading();
  math::Box2d vehicle_box =
      math::Box2d(slot_.destination_point, slot_heading, vehicle_config_->vehicle_param().front_edge_to_ego(),
                  vehicle_config_->vehicle_param().rear_edge_to_ego(), vehicle_config_->vehicle_param().width());

  // 1. 先前后，再向外侧，循环遍历出一个不碰撞的box，不能超过地图边界（若有）
  const int explore_max_longit_num = planner_profile_->slot_correction_config().explore_max_longit_num();
  const double explore_longit_step = planner_profile_->slot_correction_config().explore_longit_step();
  const double explore_longit_heading = slot_.slot.heading();
  const int explore_max_lateral_num = planner_profile_->slot_correction_config().explore_max_lateral_num();
  const double explore_lateral_step = planner_profile_->slot_correction_config().explore_lateral_step();
  const double explore_lateral_heading = slot_door_heading;

  bool is_found = false;
  double min_longit_shift = std::numeric_limits<double>::max();
  auto no_collision_box = vehicle_box;
  auto current_box = vehicle_box;
  double no_collision_lateral_shift = 0.0;
  double no_collision_longit_shift = 0.0;

  for (int i = 0; i <= explore_max_lateral_num; ++i) {
    // 横向移动
    current_box = vehicle_box;
    double lateral_shift = explore_lateral_step * i;
    current_box.Shift(math::Vec2d(lateral_shift * std::cos(explore_lateral_heading),
                                  lateral_shift * std::sin(explore_lateral_heading)));
    // PARKING_LOG(D, "current_box: ", current_box.DebugString());

    // 正向纵向搜索
    auto forward_box = current_box;
    for (int j = 0; j <= explore_max_longit_num; ++j) {
      if (j > 0) {
        forward_box.Shift(math::Vec2d(explore_longit_step * std::cos(explore_longit_heading),
                                      explore_longit_step * std::sin(explore_longit_heading)));
      }
      // PARKING_LOG(D, "forward_search_box: ", forward_box.DebugString());

      if (boxIsCollided(boundary_seg, forward_box)) {
        break;
      }
      if (!boxIsCollided(local_view, select_ods, forward_box)) {
        double longit_shift = std::abs(j * explore_longit_step);
        if (longit_shift < min_longit_shift) {
          min_longit_shift = longit_shift;
          no_collision_box = forward_box;
          no_collision_lateral_shift = lateral_shift;
          no_collision_longit_shift = j * explore_longit_step;
          is_found = true;
        }
        break;  // 找到该横向位置的最小纵向移动
      }
    }

    // 反向纵向搜索
    auto backward_box = current_box;
    for (int j = -1; j >= -explore_max_longit_num; --j) {
      backward_box.Shift(math::Vec2d(-explore_longit_step * std::cos(explore_longit_heading),
                                     -explore_longit_step * std::sin(explore_longit_heading)));

      // PARKING_LOG(D, "backward_search_box: ", backward_box.DebugString());

      if (boxIsCollided(boundary_seg, backward_box)) {
        break;
      }
      if (!boxIsCollided(local_view, select_ods, backward_box)) {
        double longit_shift = std::abs(j * explore_longit_step);
        if (longit_shift < min_longit_shift) {
          min_longit_shift = longit_shift;
          no_collision_box = backward_box;
          no_collision_lateral_shift = lateral_shift;
          no_collision_longit_shift = j * explore_longit_step;
          is_found = true;
        }
        break;  // 找到该横向位置的最小纵向移动
      }
    }
  }

  // 2. 若在遍历范围无法找到一个无碰撞位置，则直接向外侧扩大范围再寻找（不再前后搜索）
  if (!is_found) {
    for (int i = explore_max_lateral_num + 1;
         i <= explore_max_lateral_num + planner_profile_->slot_correction_config().explore_extra_lateral_num(); ++i) {
      // 横向移动
      current_box = vehicle_box;
      double lateral_shift = explore_lateral_step * i;
      current_box.Shift(math::Vec2d(lateral_shift * std::cos(explore_lateral_heading),
                                    lateral_shift * std::sin(explore_lateral_heading)));
      // PARKING_LOG(D, "current_box2: ", current_box.DebugString());

      if (boxIsCollided(boundary_seg, current_box)) {
        break;
      }
      if (!boxIsCollided(local_view, select_ods, current_box)) {
        no_collision_box = current_box;
        no_collision_lateral_shift = lateral_shift;
        no_collision_longit_shift = 0.0;
        is_found = true;
        break;
      }
    }
  }

  if (!is_found) {
    debug_info_ += "WARN: no collision box not found\n";
    PARKING_LOG(D, "没有找到不碰撞的库位");
    return;
  }
  PARKING_LOG(D, "no_collision_box: ", no_collision_box.DebugString());
  PARKING_LOG(D, "no_collision_lateral_shift: ", no_collision_lateral_shift);
  PARKING_LOG(D, "no_collision_longit_shift: ", no_collision_longit_shift);

  // 3. 再以这个不碰撞去精细判断前、后空间
  double forward_safe_dist =
      calcBoxSafeDist(local_view, select_ods, boundary_seg, no_collision_box,
                      math::Vec2d(std::cos(explore_longit_heading), std::sin(explore_longit_heading)), 3.0);
  double backward_safe_dist =
      calcBoxSafeDist(local_view, select_ods, boundary_seg, no_collision_box,
                      math::Vec2d(-std::cos(explore_longit_heading), -std::sin(explore_longit_heading)), 3.0);
  PARKING_LOG(D, "forward_safe_dist: ", forward_safe_dist);
  PARKING_LOG(D, "backward_safe_dist: ", backward_safe_dist);

  // 4. 对库位进行修正
  math::Box2d slot_correction = no_collision_box;
  const double forward_safe_dist_threshold = planner_profile_->slot_correction_config().forward_safe_dist_threshold();
  const double backward_safe_dist_threshold = planner_profile_->slot_correction_config().backward_safe_dist_threshold();
  const double lateral_safe_dist_threshold = planner_profile_->slot_correction_config().lateral_safe_dist_threshold();
  double longitudinal_corrected_dist = 0.0;
  double lateral_corrected_dist = 0.0;

  // 纵向修正
  if (forward_safe_dist + backward_safe_dist < forward_safe_dist_threshold + backward_safe_dist_threshold) {
    // 前后总体空间均不足，居中调整
    longitudinal_corrected_dist = (forward_safe_dist - backward_safe_dist) / 2.0;
  } else if (forward_safe_dist < forward_safe_dist_threshold) {
    // 总体空间充足，前空间不足，调整前向
    longitudinal_corrected_dist = std::min(forward_safe_dist - forward_safe_dist_threshold, 0.0);
  } else if (backward_safe_dist < backward_safe_dist_threshold) {
    // 总体空间充足，后空间不足，调整后向
    longitudinal_corrected_dist = std::max(backward_safe_dist_threshold - backward_safe_dist, 0.0);
  }
  slot_correction.Shift(math::Vec2d(longitudinal_corrected_dist * std::cos(explore_longit_heading),
                                    longitudinal_corrected_dist * std::sin(explore_longit_heading)));

  // 横向修正
  auto slot_correction_swept = no_collision_box;
  slot_correction_swept.Shift(math::Vec2d(longitudinal_corrected_dist / 2.0 * std::cos(explore_longit_heading),
                                          longitudinal_corrected_dist / 2.0 * std::sin(explore_longit_heading)));
  slot_correction_swept.LongitudinalExtend(longitudinal_corrected_dist);
  double lateral_safe_dist =
      calcBoxSafeDist(local_view, select_ods, boundary_seg, slot_correction_swept,
                      math::Vec2d(-std::cos(explore_lateral_heading), -std::sin(explore_lateral_heading)), 2.0);
  PARKING_LOG(D, "lateral_safe_dist: ", lateral_safe_dist);
  if (lateral_safe_dist < lateral_safe_dist_threshold) {
    // 内侧空间不足，调整内侧
    lateral_corrected_dist = lateral_safe_dist_threshold - lateral_safe_dist;
  } else if (no_collision_lateral_shift > 0.01) {
    // 内侧空间充足，考虑是否向内侧调整
    lateral_corrected_dist = std::max(-no_collision_lateral_shift, lateral_safe_dist_threshold - lateral_safe_dist);
  }
  slot_correction.Shift(math::Vec2d(lateral_corrected_dist * std::cos(explore_lateral_heading),
                                    lateral_corrected_dist * std::sin(explore_lateral_heading)));

  PARKING_LOG(D, "longitudinal_corrected_dist: ", longitudinal_corrected_dist);
  PARKING_LOG(D, "lateral_corrected_dist: ", lateral_corrected_dist);

  if (std::abs(longitudinal_corrected_dist + no_collision_longit_shift) < 0.01
      && std::abs(lateral_corrected_dist + no_collision_lateral_shift) < 0.01) {
    PARKING_LOG(D, "不需要调整库位");
    return;
  }

  debug_info_ += "slot modify: ";
  debug_info_ += fmt::format("longit: {}, lateral: {}\n", longitudinal_corrected_dist + no_collision_longit_shift,
                             lateral_corrected_dist + no_collision_lateral_shift);
  PARKING_LOG(D, "slot_correction: ", slot_correction.DebugString());

  std::vector<PathPt> slot_corners;
  for (auto pt : slot_correction.GetAllCorners()) {
    slot_corners.emplace_back(pt.x(), pt.y(), slot_.slot.corner(0).z());
  }

  // 5. 将库位角点顺序构造成原始输入形式，即顺时针角点, 0-3为入口
  double slot_left_door_heading =
      atan2(slot_corners[1].y() - slot_corners[0].y(), slot_corners[1].x() - slot_corners[0].x());
  if (abs(math::NormalizeAngle(slot_left_door_heading - slot_door_heading)) > M_PI_2) {
    slot_corners.insert(slot_corners.begin(), slot_corners.back());
    slot_corners.pop_back();
  } else {
    slot_corners.emplace_back(slot_corners.front());
    slot_corners.erase(slot_corners.begin());
  }
  std::swap(slot_corners[1], slot_corners[3]);

  std::vector<PathPt> slot_corners_map;
  auto tf_ego_2_map = local_view.getLocalizationPtr()->getTfEgo2Map();
  for (auto pt : slot_corners) {
    transfer::transformPoint(tf_ego_2_map, &pt);
    slot_corners_map.emplace_back(pt);
  }
  corrected_slot_corner_ = slot_corners_map;
  generateSlot(slot_corners, make_pair(true, slot_heading));
}

void ParallelParkingInPlanner::generateSlot(std::vector<PathPt>& slot_corners,
                                            const std::pair<bool, float>& slot_direction) {
  // 转换成泊车内部库位角点顺序
  convertSlotCorners(slot_corners, slot_direction);
  slot_ = SlotParam(slot_corners, Slot::SlotType::PARALLEL,
                    slot_corners[2].y() > 0 ? Slot::SlotDirec::LEFT : Slot::SlotDirec::RIGHT);

  // 计算泊车最终停靠位置
  double destination_coeff = ((slot_.slot.length() - vehicle_config_->vehicle_param().length()) / 2.0
                              + vehicle_config_->vehicle_param().rear_edge_to_ego())
                             / slot_.slot.length();

  slot_.destination_point.set_x(destination_coeff * slot_.slot.topEdgeCenter().x()
                                + (1.0 - destination_coeff) * slot_.slot.bottomEdgeCenter().x());
  slot_.destination_point.set_y(destination_coeff * slot_.slot.topEdgeCenter().y()
                                + (1.0 - destination_coeff) * slot_.slot.bottomEdgeCenter().y());
  slot_.destination_point.set_z(destination_coeff * slot_.slot.topEdgeCenter().z()
                                + (1.0 - destination_coeff) * slot_.slot.bottomEdgeCenter().z());
}

}  // namespace gpal::pnc::planning