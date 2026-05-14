#include "openspace_path_planner/application/vertical_parking_in_planner.h"

namespace gpal::pnc::planning {
void VerticalParkingInPlanner::convertSlotCorners(std::vector<PathPt>& slot_corners) {
  // 判断库位在自车的左侧还是右侧
  if (slot_corners[1].y() < 0) {
    std::swap(slot_corners[0], slot_corners[3]);
    std::swap(slot_corners[1], slot_corners[2]);
  }
}

void VerticalParkingInPlanner::parkEnvRecognizer(const LocalView& local_view,
                                                 const std::vector<Decision::DecisionObject>& select_ods,
                                                 const std::vector<math::LineSegment2d>& boundary_seg) {
  if (park_env_type_ != ParkEnvType::INVALID) {
    return;
  }
  park_env_type_ = ParkEnvType::GENERAL;

  // 1. 断头路环境识别
  if (planner_profile_->park_env_recog_config().enable_park_env_recognizer()) {
    bool is_dead_end = true;

    // 1.1 基于库位角点，确定采样box中心点和box方向
    double sample_heading = 0.0;
    double sample_box_heading = 0.0;
    std::vector<PathPt> slot_corners;
    PathPt slot_front_inner_pt, slot_front_outer_pt;
    for (int i = 0; i < 4; i++) {
      PathPt corner;
      corner.set_x(local_view.getParkLotPtr()->data_.gstSpace[0].gfPos[i][0]);
      corner.set_y(local_view.getParkLotPtr()->data_.gstSpace[0].gfPos[i][1]);
      corner.set_z(local_view.getParkLotPtr()->data_.gstSpace[0].gfPos[i][2]);
      slot_corners.push_back(corner);
    }
    if (slot_corners[1].y() > 0) {
      slot_front_inner_pt = slot_corners[2];
      slot_front_outer_pt = slot_corners[3];
      sample_heading =
          atan2(slot_front_outer_pt.y() - slot_front_inner_pt.y(), slot_front_outer_pt.x() - slot_front_inner_pt.x());
      sample_box_heading = atan2(slot_corners[3].y() - slot_corners[0].y(), slot_corners[3].x() - slot_corners[0].x());
    } else {
      slot_front_inner_pt = slot_corners[1];
      slot_front_outer_pt = slot_corners[0];
      sample_heading =
          atan2(slot_front_outer_pt.y() - slot_front_inner_pt.y(), slot_front_outer_pt.x() - slot_front_inner_pt.x());
      sample_box_heading = atan2(slot_corners[0].y() - slot_corners[3].y(), slot_corners[0].x() - slot_corners[3].x());
    }
    PARKING_LOG(D, "slot_front_inner_pt x = ", slot_front_inner_pt.x(), " y = ", slot_front_inner_pt.y());
    PARKING_LOG(D, "slot_front_outer_pt x = ", slot_front_outer_pt.x(), " y = ", slot_front_outer_pt.y());
    PARKING_LOG(D, "sample_heading = ", sample_heading * RAD2ANG,
                " sample_box_heading = ", sample_box_heading * RAD2ANG);

    // 1.2 采样检测库位通道前方道路是否通畅
    const double sample_step = planner_profile_->park_env_recog_config().sample_step();
    const int sample_num = planner_profile_->park_env_recog_config().sample_num();
    const double sample_box_width = planner_profile_->park_env_recog_config().sample_box_width();
    const double sample_box_length = planner_profile_->park_env_recog_config().sample_box_length();
    math::Vec2d sample_init_box_center(slot_front_outer_pt.x() + sample_box_width / 2.0 * cos(sample_heading),
                                       slot_front_outer_pt.y() + sample_box_width / 2.0 * sin(sample_heading));
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

void VerticalParkingInPlanner::slotProcess(const LocalView& local_view,
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
  generateSlot(slot_corners);

  // 基于环境对当前库位位置进行微调
  slotCorrection(local_view, select_ods);

  PARKING_LOG(D, "slot_length: ", slot_.slot.length());
  PARKING_LOG(D, "destination_point x = ", slot_.destination_point.x(),
              " destination_point.y = ", slot_.destination_point.y());
}

bool VerticalParkingInPlanner::generateSearchElements(const LocalView& local_view,
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

  // 泊车终点向前提前
  end_pose.set_x(end_pose.x() + planner_profile_->search_end_pose_offset() * cos(end_pose.theta()));
  end_pose.set_y(end_pose.y() + planner_profile_->search_end_pose_offset() * sin(end_pose.theta()));

  search_elements = SearchElements(start_pose, end_pose);

  return true;
}

void VerticalParkingInPlanner::customBoundary(const LocalView& local_view,
                                              std::vector<Decision::DecisionObject>& select_ods) {
  if (!planner_profile_->custom_boundary_config().enable_custom_boundary()) {
    return;
  }

  // 1. 自定义修正障碍物位置
  if (planner_profile_->custom_boundary_config().enable_custom_obstacle_boundary()) {
    double longit_heading = atan2((slot_.slot.corner(3).y() - slot_.slot.corner(0).y()),
                                  (slot_.slot.corner(3).x() - slot_.slot.corner(0).x()));
    double horizon_heading = atan2((slot_.slot.bottomEdgeCenter().y() - slot_.slot.topEdgeCenter().y()),
                                   (slot_.slot.bottomEdgeCenter().x() - slot_.slot.topEdgeCenter().x()));

    math::LineSegment2d slot_top_line(math::Vec2d(slot_.slot.corner(3).x() + 8.0 * cos(longit_heading),
                                                  slot_.slot.corner(3).y() + 8.0 * sin(longit_heading)),
                                      math::Vec2d(slot_.slot.corner(0).x() - 8.0 * cos(longit_heading),
                                                  slot_.slot.corner(0).y() - 8.0 * sin(longit_heading)));
    for (auto& od : select_ods) {
      // 找到在库位对面一定范围的障碍物
      if (!slot_top_line.HasIntersect(math::LineSegment2d(od.cur_box.center(), slot_.slot.bottomEdgeCenter()))) {
        continue;
      }
      double dis = od.cur_box.DistanceTo(slot_top_line);
      if (dis > planner_profile_->custom_boundary_config().aisle_width_threshold()) {
        // 将障碍物向库位方向拉近
        od.cur_box.Shift(math::Vec2d(planner_profile_->custom_boundary_config().push_gap() * cos(horizon_heading),
                                     planner_profile_->custom_boundary_config().push_gap() * sin(horizon_heading)));
      }
    }
  }

  /*
   * 2. 自定义虚拟边界约束
   *
   *                   boundary_roadside
   * ----------------------------------------------------------
   *                  |  1 +—————————+ 2   |
   *                  |    |  bottom |     |
   *                  |    |         |     |
   *                  |    |         |     |
   *                  |    |  slot   |     |
   *  boundary_rear   |    |         |     |  boundary_front
   *                  |    |         |     |
   *                  |    |         |     |                           horizon_heading
   *                  |    |   top   |     |                         ^
   *                  |  0 +—————————+ 3   |                         |
   *  top_rear————————                      ————————top_front         ——>  longit_heading
   *
   *
   *                >>>>>>>>>>>>>>>>>>>> 寻库方向
   *
   * -----------------------------------------------------------
   *                   boundary_aisle
   */
  if (planner_profile_->custom_boundary_config().enable_custom_virtual_boundary()) {
    double horizon_heading = atan2((slot_.slot.bottomEdgeCenter().y() - slot_.slot.topEdgeCenter().y()),
                                   (slot_.slot.bottomEdgeCenter().x() - slot_.slot.topEdgeCenter().x()));
    double longit_heading = atan2((slot_.slot.corner(3).y() - slot_.slot.corner(0).y()),
                                  (slot_.slot.corner(3).x() - slot_.slot.corner(0).x()));

    std::vector<math::Vec2d> boundary_roadside;
    std::vector<math::Vec2d> boundary_front;
    std::vector<math::Vec2d> boundary_rear;
    std::vector<math::Vec2d> boundary_aisle;

    std::vector<math::Vec2d> boundary_top_rear;
    std::vector<math::Vec2d> boundary_top_front;

    // 路沿边界
    const double roadside_gap = planner_profile_->custom_boundary_config().roadside_gap();
    math::Vec2d roadside_gap_point(slot_.slot.corner(2).x() + roadside_gap * cos(horizon_heading),
                                   slot_.slot.corner(2).y() + roadside_gap * sin(horizon_heading));

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

    math::Vec2d front_point2(slot_.slot.corner(2).x() + before_after_gap * cos(longit_heading),
                             slot_.slot.corner(2).y() + before_after_gap * sin(longit_heading));
    boundary_front.push_back(front_point1);
    boundary_front.push_back(front_point2);
    park_roi_decider_.addBoundary(std::make_tuple("slot_front", OpenspaceObjectType::SLOT, boundary_front));

    math::Vec2d rear_point1(slot_.slot.corner(1).x() - before_after_gap * cos(longit_heading),
                            slot_.slot.corner(1).y() - before_after_gap * sin(longit_heading));

    math::Vec2d rear_point2(slot_.slot.corner(0).x() - before_after_gap * cos(longit_heading),
                            slot_.slot.corner(0).y() - before_after_gap * sin(longit_heading));
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

    // 库位口突出边界
    const double top_gap = planner_profile_->custom_boundary_config().top_gap();
    math::Vec2d top_front_point1(front_point1.x() - top_gap * cos(horizon_heading),
                                 front_point1.y() - top_gap * sin(horizon_heading));
    math::Vec2d top_front_point2(top_front_point1.x() + 1.0 * cos(longit_heading),
                                 top_front_point1.y() + 1.0 * sin(longit_heading));

    math::Vec2d top_rear_point1(rear_point2.x() - top_gap * cos(horizon_heading),
                                rear_point2.y() - top_gap * sin(horizon_heading));
    math::Vec2d top_rear_point2(top_rear_point1.x() - 1.0 * cos(longit_heading),
                                top_rear_point1.y() - 1.0 * sin(longit_heading));

    boundary_top_front.push_back(top_front_point1);
    boundary_top_front.push_back(top_front_point2);
    park_roi_decider_.addBoundary(std::make_tuple("top_front", OpenspaceObjectType::SLOT, boundary_top_front));

    boundary_top_rear.push_back(top_rear_point1);
    boundary_top_rear.push_back(top_rear_point2);
    park_roi_decider_.addBoundary(std::make_tuple("top_rear", OpenspaceObjectType::SLOT, boundary_top_rear));
  }
}

void VerticalParkingInPlanner::geometricConnect(const LocalView& local_view, const RoiDecideResult& roi) {
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

  /** 搜索结束后拼接入库直线段 */
  // 判断直线轨迹方向
  math::Vec2d unit(std::cos(orin_combined_flu_path_.back().theta()), std::sin(orin_combined_flu_path_.back().theta()));
  math::Vec2d direc = slot_.destination_point - orin_combined_flu_path_.back();
  PathPt::Direction line_direc = unit.InnerProd(direc) > 0 ? PathPt::Direction::FORWARD : PathPt::Direction::BACKWARD;

  // 保留一段直线段在优化后拼接,旨在引导控制方向盘回正
  PathPt straight_line_point = slot_.destination_point;
  double straight_line_splice_length =
      planner_profile_->trajectory_post_process_config().last_segment_splice_straight_line_length();
  if (line_direc == PathPt::Direction::BACKWARD) {
    straight_line_point.set_x(slot_.destination_point.x() + cos(slot_.slot.heading()) * straight_line_splice_length);
    straight_line_point.set_y(slot_.destination_point.y() + sin(slot_.slot.heading()) * straight_line_splice_length);
  } else {
    straight_line_point.set_x(slot_.destination_point.x() - cos(slot_.slot.heading()) * straight_line_splice_length);
    straight_line_point.set_y(slot_.destination_point.y() - sin(slot_.slot.heading()) * straight_line_splice_length);
  }

  // 计算直线段离散点
  double step = 0.2;
  double straight_line_length = orin_combined_flu_path_.back().DistanceTo(straight_line_point);
  int index = static_cast<int>(std::round(straight_line_length / step));
  step = straight_line_length / index;
  straight_line_point.set_s(orin_combined_flu_path_.back().s() + straight_line_length);
  for (int i = 1; i <= index; ++i) {
    PathPt temp = interpolateUsingLinearApproximation(orin_combined_flu_path_.back(), straight_line_point, step * i);
    temp.set_direction(line_direc);
    temp.set_theta(slot_.slot.heading());
    temp.set_kappa(0.0);
    orin_combined_flu_path_.push_back(temp);
  }
}

void VerticalParkingInPlanner::trajPostProcess(const LocalView& local_view, const RoiDecideResult& roi) {
  if (optimizer_segmented_flu_path_.empty())
    return;

  // feat1:在轨迹的换挡处延长拼接安全的直线，旨在便于控制缩小误差
  if (planner_profile_->trajectory_post_process_config().enable_straight_line_at_gear_shift()) {
    int path_num = optimizer_segmented_flu_path_.size();
    for (int i = 0; i + 1 < path_num; ++i) {
      if (optimizer_segmented_flu_path_.at(i).empty() || optimizer_segmented_flu_path_.at(i + 1).empty()) {
        return;
      }
      PathPt back_point = optimizer_segmented_flu_path_.at(i).back();
      double max_length = planner_profile_->trajectory_post_process_config().straight_line_max_length_at_gear_shift();

      auto straight_line =
          safe_straight_line_generator(*local_view.getFreespacePtr(), roi, back_point, max_length,
                                       planner_profile_->trajectory_post_process_config().width_safe_buff(),
                                       planner_profile_->trajectory_post_process_config().length_safe_buff());
      optimizer_segmented_flu_path_.at(i).insert(optimizer_segmented_flu_path_.at(i).end(), straight_line.begin(),
                                                 straight_line.end());
      reverse_straight_line(straight_line);
      optimizer_segmented_flu_path_.at(i + 1).insert(optimizer_segmented_flu_path_.at(i + 1).begin(),
                                                     straight_line.begin(), straight_line.end());
    }
  }

  // feat2:在最后一段入库轨迹的末尾拼接一段直线，旨在便于控制提前回正车辆且最后能完全回正方向盘
  if (planner_profile_->trajectory_post_process_config().last_segment_splice_straight_line_length() > 0.01
      && !optimizer_segmented_flu_path_.back().empty()) {
    auto& final_path = optimizer_segmented_flu_path_.back();
    PathPt back_point = final_path.back();
    double max_length = planner_profile_->trajectory_post_process_config().last_segment_splice_straight_line_length();
    auto straight_line = straight_line_generator(back_point, max_length);
    final_path.insert(final_path.end(), straight_line.begin(), straight_line.end());
  }

  // feat3:在最后一段入库轨迹的末尾修正一定范围轨迹点的kappa和theta，旨在引导控制回正方向盘 ( 被feat2替代 )
  if (planner_profile_->trajectory_post_process_config().enable_modify_last_segment_traj()
      && !optimizer_segmented_flu_path_.back().empty()) {
    auto& final_path = optimizer_segmented_flu_path_.back();
    for (auto it = final_path.rbegin(); it != final_path.rend(); ++it) {
      it->set_kappa(0.0);
      it->set_theta(final_path.back().theta());
      if (it->DistanceTo(final_path.back()) > planner_profile_->trajectory_post_process_config().modify_length())
        break;
    }
  }
}

bool VerticalParkingInPlanner::isParkingCompleted(const LocalView& local_view, const bool destination_stop_flag) {
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

void VerticalParkingInPlanner::calVehicleSlotClearance() {
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
bool VerticalParkingInPlanner::needsFineTune(const PathPt& curr_traj_point) {
  PARKING_LOG(D, "current_fine_tune_count_ = ", current_fine_tune_count_);
  PARKING_LOG(D, "realtime_optimizer_segmented_flu_paths_.size() = ", realtime_optimizer_segmented_flu_paths_.size());
  if (realtime_optimizer_segmented_flu_paths_.size() > 1
      || current_fine_tune_count_ >= planner_profile_->fine_tune_config().max_fine_tune_count()) {
    return false;
  }

  // 自车在库位中线上的垂足点
  float factor = 0;
  PathPt ego_projection_on_slot_centerline =
      CalculatePedalPoint(PathPt(0, 0, 0, 0, 0), slot_.slot.topEdgeCenter(), slot_.slot.bottomEdgeCenter(), factor);
  // 自车垂足点和库位中线的横向距离
  double lateral_dis = ego_projection_on_slot_centerline.DistanceToOrigin();

  double longit_dis = remain_dis_;
  double angle_diff = slot_.slot.heading() * RAD2ANG;

  PARKING_LOG(D, "longit_dis = ", longit_dis);
  PARKING_LOG(D, "lateral_dis = ", lateral_dis);
  PARKING_LOG(D, "angle_diff = ", angle_diff);
  if (longit_dis < planner_profile_->fine_tune_config().rear_first_parking_longit_threshold() && longit_dis > 0.5) {
    if (std::abs(lateral_dis) > planner_profile_->fine_tune_config().rear_first_parking_lateral_threshold()
        || std::abs(angle_diff) > planner_profile_->fine_tune_config().rear_first_parking_angle_diff_threshold()) {
      return true;
    }
  }
  return false;
}

void VerticalParkingInPlanner::generateFineTunePath(const LocalView& local_view) {
  current_fine_tune_count_++;
  std::vector<DiscretizedPath> adjust_traj;
  int step_num = 3;

  float factor = 0;
  PathPt first_pt =
      CalculatePedalPoint(PathPt(0, 0, 0), slot_.slot.topEdgeCenter(), slot_.slot.bottomEdgeCenter(), factor);
  first_pt.set_theta(slot_.slot.heading());
  first_pt.set_direction(current_flu_path_.front().direction());
  for (int step = 0; step < step_num; ++step) {
    double extend_dis = 0.0;
    if (step == 0) {
      extend_dis = planner_profile_->fine_tune_config().stop_distance_buffer();
    } else if (step == 1) {
      extend_dis = planner_profile_->fine_tune_config().fine_tune_distance()
                   + planner_profile_->fine_tune_config().stop_distance_buffer();
    } else if (step == 2) {
      extend_dis = adjust_traj.back().back().DistanceTo(slot_.destination_point);
    }

    if (step == 0) {
      adjust_traj.emplace_back(straight_line_generator(first_pt, extend_dis));
    } else {
      first_pt = adjust_traj.back().back();
      first_pt.set_direction(first_pt.direction() == PathPt::Direction::BACKWARD ? PathPt::Direction::FORWARD
                                                                                 : PathPt::Direction::BACKWARD);
      adjust_traj.emplace_back(straight_line_generator(first_pt, extend_dis));
    }
  }

  debugPrintTraj(adjust_traj, "fine_tune");
  optimizer_segmented_map_paths_ = std::move(local2GlobalPaths(local_view, adjust_traj));
}

void VerticalParkingInPlanner::slotCorrection(const LocalView& local_view,
                                              const std::vector<Decision::DecisionObject>& select_ods) {
  // todo: 垂直泊车库位修正
  corrected_slot_corner_.clear();
  if (!planner_profile_->slot_correction_config().enable_slot_correction()) {
    return;
  }
}

void VerticalParkingInPlanner::generateSlot(std::vector<PathPt>& slot_corners) {
  // 转换成泊车内部库位角点顺序
  convertSlotCorners(slot_corners);
  slot_ = SlotParam(slot_corners, Slot::SlotType::VERTICAL,
                    slot_corners[2].y() > 0 ? Slot::SlotDirec::LEFT : Slot::SlotDirec::RIGHT);

  // 计算泊车最终停靠位置
  double real_destination_coeff =
      (vehicle_config_->vehicle_param().rear_edge_to_ego() + planner_profile_->park_completion_longit_pose_offset())
      / slot_.slot.length();
  slot_.destination_point.set_x(real_destination_coeff * slot_.slot.topEdgeCenter().x()
                                + (1.0 - real_destination_coeff) * slot_.slot.bottomEdgeCenter().x());
  slot_.destination_point.set_y(real_destination_coeff * slot_.slot.topEdgeCenter().y()
                                + (1.0 - real_destination_coeff) * slot_.slot.bottomEdgeCenter().y());
  slot_.destination_point.set_z(real_destination_coeff * slot_.slot.topEdgeCenter().z()
                                + (1.0 - real_destination_coeff) * slot_.slot.bottomEdgeCenter().z());
}

}  // namespace gpal::pnc::planning