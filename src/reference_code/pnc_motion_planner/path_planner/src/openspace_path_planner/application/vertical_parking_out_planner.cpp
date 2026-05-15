#include "openspace_path_planner/application/vertical_parking_out_planner.h"

namespace gpal::pnc::planning {
void VerticalParkingOutPlanner::parkEnvRecognizer(const LocalView& local_view,
                                                  const std::vector<Decision::DecisionObject>& select_ods,
                                                  const std::vector<math::LineSegment2d>& boundary_seg) {
  park_env_type_ = ParkEnvType::GENERAL;
  return;
}

bool VerticalParkingOutPlanner::generateSearchElements(const LocalView& local_view,
                                                       const std::vector<Decision::DecisionObject>& select_ods,
                                                       const std::vector<math::LineSegment2d>& boundary_seg,
                                                       SearchElements& search_elements) {
  PathPt start_pose;
  PathPt end_pose;

  // start_pose
  // 不停车泊出需要使用未来时刻轨迹点作为搜索起点
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

  // end_pose
  PARKING_LOG(D, "end_pose_map_bak_: ", end_pose_map_bak_.first, " pt: ", end_pose_map_bak_.second.x(), " ",
              end_pose_map_bak_.second.y(), " ", end_pose_map_bak_.second.theta());

  auto park_out_direction = local_view.getConsolePtr()->parkOutDirection();
  PARKING_LOG(D, "park_out_direction: ", park_out_direction);

  Eigen::Matrix4d tf_map_2_ego = local_view.getLocalizationPtr()->getTfMap2Ego();
  Eigen::Matrix4d tf_ego_2_map = local_view.getLocalizationPtr()->getTfEgo2Map();
  if (end_pose_map_bak_.first) {
    // 消除定位误差
    if (target_reference_line_info_.isValid()) {
      auto ref = target_reference_line_info_.ref_line();
      auto nearest_ref_pt = ref.getNearestReferencePoint(end_pose_map_bak_.second);
      end_pose = PathPt(nearest_ref_pt, 0.0, nearest_ref_pt.heading());

      transfer::transformPoint(tf_map_2_ego, &end_pose);
      math::Vec3d ego_rpy_vec3d(0.0, 0.0, end_pose.theta());
      transfer::transformRPY(tf_map_2_ego, &ego_rpy_vec3d);
      end_pose.set_theta(ego_rpy_vec3d.z());

      // 判断终点位置合理性
      is_ignore_od_and_fs_ = !endPointMoveToSafePosition(end_pose, local_view, select_ods, boundary_seg);

      end_pose_map_bak_ = make_pair(true, end_pose);
      transfer::transformPoint(tf_ego_2_map, &end_pose_map_bak_.second);
      ego_rpy_vec3d = math::Vec3d(0.0, 0.0, end_pose_map_bak_.second.theta());
      transfer::transformRPY(tf_ego_2_map, &ego_rpy_vec3d);
      end_pose_map_bak_.second.set_theta(ego_rpy_vec3d.z());
    } else {
      end_pose = end_pose_map_bak_.second;
      transfer::transformPoint(tf_map_2_ego, &end_pose);
      math::Vec3d ego_rpy_vec3d(0.0, 0.0, end_pose.theta());
      transfer::transformRPY(tf_map_2_ego, &ego_rpy_vec3d);
      end_pose.set_theta(ego_rpy_vec3d.z());
    }

  } else {
    if (target_reference_line_info_.isValid()) {
      auto ref = target_reference_line_info_.ref_line();
      auto sl_veh_point =
          ref.getFrenetPoint(PathPt(ref.adcLocalization().x(), ref.adcLocalization().y(), ref.adcLocalization().z()));

      // 停车场特殊场景，需要延长终点位置
      double end_pose_dis = planner_profile_->parkout_endpt_ref_dis();
      for (const auto& road_range : ref.getRoadRanges()) {
        if (road_range.start_s > sl_veh_point.s() || sl_veh_point.s() > road_range.end_s) {
          continue;
        }
        auto roads = local_view.getEnvRoadCognitionPtr()->getRoads();
        for (const auto& r : roads) {
          if (r.id == road_range.id) {
            for (const auto& type : r.road_types) {
              PARKING_LOG(D, "road_type: ", type);
              if (type == proto::MapCommon_RoadType_kParkingRoad) {
                end_pose_dis = planner_profile_->parkout_special_endpt_ref_dis();
              }
            }
          }
        }
      }

      // 如果是Uturn场景需要找到转弯后的终点位置
      double u_turn_kappa = 0.0;
      if (uTurnJudge(ref, sl_veh_point.s(), 30, 5, u_turn_kappa)) {
        auto ref_veh_point = ref.getNearestReferencePoint(sl_veh_point.s());
        int index = ref.getNearestReferenceIndex(sl_veh_point.s());
        for (int i = index; i + 1 < ref.reference_points().size(); i++) {
          const auto& pt1 = ref.reference_points()[i];
          const auto& pt2 = ref.reference_points()[i + 1];
          if (abs(math::NormalizeAngle(pt1.heading() - ref_veh_point.heading())) > M_PI_4 * 3.0
              && abs(math::NormalizeAngle(pt1.heading() - pt2.heading())) < 0.03) {
            end_pose_dis = pt1.local_s() - sl_veh_point.s();
            break;
          }
        }
      }

      // 如果前方有闸机，不能超过闸机范围
      auto gates = ref.getGatesFromSRange(sl_veh_point.s(), 30, 0);
      if (!gates.empty()) {
        float gate_min_s = 1e6;
        for (const auto& gate : gates) {
          gate_min_s = std::min(gate_min_s, gate.s);
        }
        end_pose_dis =
            std::min(end_pose_dis, gate_min_s - sl_veh_point.s() - planner_profile_->parkout_gate_endpt_ref_dis());

        //  在闸机处生成约束墙
        auto gate_wall_pt = ref.getNearestReferencePoint(gate_min_s);
        double gate_wall_heading = math::NormalizeAngle(gate_wall_pt.heading() + M_PI_2);
        math::Vec2d gate_wall_start(gate_wall_pt.x() + 2.0 * cos(gate_wall_heading),
                                    gate_wall_pt.y() + 2.0 * sin(gate_wall_heading));
        math::Vec2d gate_wall_end(gate_wall_pt.x() - 2.0 * cos(gate_wall_heading),
                                  gate_wall_pt.y() - 2.0 * sin(gate_wall_heading));
        gate_wall_ = std::make_pair(true, math::LineSegment2d(gate_wall_start, gate_wall_end));
      }

      auto end_pose_ref_pt = ref.getNearestReferencePoint(sl_veh_point.s() + end_pose_dis);
      end_pose = PathPt(end_pose_ref_pt);
      end_pose.set_theta(end_pose_ref_pt.heading());

      transfer::transformPoint(tf_map_2_ego, &end_pose);
      math::Vec3d ego_rpy_vec3d(0.0, 0.0, end_pose.theta());
      transfer::transformRPY(tf_map_2_ego, &ego_rpy_vec3d);
      end_pose.set_theta(ego_rpy_vec3d.z());
    } else {
      const double lateral_threshold =
          planner_profile_->end_point_select_config().vertical_end_point_lateral_threshold();
      const double longit_threshold = planner_profile_->end_point_select_config().vertical_end_point_longit_threshold();
      if (park_out_direction == proto::ParkingTaskInfo_ParkingOutDirection_kVerticalLeft) {
        end_pose = PathPt(vehicle_config_->vehicle_param().front_edge_to_ego()
                              + vehicle_config_->vehicle_param().width() / 2.0 + longit_threshold,
                          vehicle_config_->vehicle_param().rear_edge_to_ego() + lateral_threshold, 0.0, 0.0, M_PI_2);

      } else if (park_out_direction == proto::ParkingTaskInfo_ParkingOutDirection_kVerticalRight) {
        end_pose = PathPt(vehicle_config_->vehicle_param().front_edge_to_ego()
                              + vehicle_config_->vehicle_param().width() / 2.0 + longit_threshold,
                          -vehicle_config_->vehicle_param().rear_edge_to_ego() - lateral_threshold, 0.0, 0.0, -M_PI_2);
      } else {
        end_pose = PathPt(vehicle_config_->vehicle_param().front_edge_to_ego(), 0.0, 0.0, 0.0, 0.0);
      }
    }

    // 判断终点位置合理性
    is_ignore_od_and_fs_ = !endPointMoveToSafePosition(end_pose, local_view, select_ods, boundary_seg);

    end_pose_map_bak_ = make_pair(true, end_pose);
    transfer::transformPoint(tf_ego_2_map, &end_pose_map_bak_.second);
    math::Vec3d ego_rpy_vec3d(0.0, 0.0, end_pose_map_bak_.second.theta());
    transfer::transformRPY(tf_ego_2_map, &ego_rpy_vec3d);
    end_pose_map_bak_.second.set_theta(ego_rpy_vec3d.z());
  }

  search_elements = SearchElements(start_pose, end_pose);

  return true;
}

void VerticalParkingOutPlanner::geometricConnect(const LocalView& local_view, const RoiDecideResult& roi) {
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
}

void VerticalParkingOutPlanner::customBoundary(const LocalView& local_view,
                                               std::vector<Decision::DecisionObject>& select_ods) {
  if (!planner_profile_->custom_boundary_config().enable_custom_boundary()) {
    return;
  }

  /*
   * 自定义边界约束
   *
   * —————————————————————————————————————————————————————
   *                    boundary_aisle
   *
   *
   *
   *                         out                     ^ x
   *                          ^                      |
   *                          ^                 y <——
   *                          ^
   *                |    +—————————+     |
   *                |    |         |     |
   *                |    |         |     |
   *                |    |         |     |
   *                |    |   veh   |     |
   * boundary_left  |    |         |     |  boundary_right
   *                |    |         |     |
   *                |    |         |     |
   *                |    |         |     |
   *                |    +—————————+     |
   *  —————————————————————————————————————————————————————
   *                   boundary_roadside
   */
  if (planner_profile_->custom_boundary_config().enable_custom_virtual_boundary()) {
    std::vector<math::Vec2d> boundary_left;
    std::vector<math::Vec2d> boundary_right;
    std::vector<math::Vec2d> boundary_roadside;
    std::vector<math::Vec2d> boundary_aisle;

    // 后侧边界
    const double roadside_gap = planner_profile_->custom_boundary_config().roadside_gap();
    math::Vec2d rear_point1(-vehicle_config_->vehicle_param().rear_edge_to_ego() - roadside_gap, 30);

    math::Vec2d rear_point2(-vehicle_config_->vehicle_param().rear_edge_to_ego() - roadside_gap, -30);
    boundary_roadside.push_back(rear_point1);
    boundary_roadside.push_back(rear_point2);
    park_roi_decider_.addBoundary(std::make_tuple("roadside", OpenspaceObjectType::ROAD_SIDE, boundary_roadside));

    // 库位左右边界
    const double left_right_gap = planner_profile_->custom_boundary_config().both_sides_gap();
    math::Vec2d left_point1(vehicle_config_->vehicle_param().front_edge_to_ego(),
                            vehicle_config_->vehicle_param().width() / 2.0 + left_right_gap);

    math::Vec2d left_point2(-vehicle_config_->vehicle_param().rear_edge_to_ego(),
                            vehicle_config_->vehicle_param().width() / 2.0 + left_right_gap);
    boundary_left.push_back(left_point1);
    boundary_left.push_back(left_point2);
    park_roi_decider_.addBoundary(std::make_tuple("slot_left", OpenspaceObjectType::SLOT, boundary_left));

    math::Vec2d right_point1(vehicle_config_->vehicle_param().front_edge_to_ego(),
                             -vehicle_config_->vehicle_param().width() / 2.0 - left_right_gap);

    math::Vec2d right_point2(-vehicle_config_->vehicle_param().rear_edge_to_ego(),
                             -vehicle_config_->vehicle_param().width() / 2.0 - left_right_gap);
    boundary_right.push_back(right_point1);
    boundary_right.push_back(right_point2);
    park_roi_decider_.addBoundary(std::make_tuple("slot_right", OpenspaceObjectType::SLOT, boundary_right));

    // 通道边界
    const double aisle_gap = planner_profile_->custom_boundary_config().aisle_gap();
    math::Vec2d aisle_point1(vehicle_config_->vehicle_param().front_edge_to_ego() + aisle_gap, 30);

    math::Vec2d aisle_point2(vehicle_config_->vehicle_param().front_edge_to_ego() + aisle_gap, -30);

    boundary_aisle.push_back(aisle_point1);
    boundary_aisle.push_back(aisle_point2);
    park_roi_decider_.addBoundary(std::make_tuple("aisle", OpenspaceObjectType::AISLE, boundary_aisle));
  }

  // 增加闸机约束墙
  if (gate_wall_.first) {
    Eigen::Matrix4d tf_map_2_ego = local_view.getLocalizationPtr()->getTfMap2Ego();
    math::Vec3d start_pose(gate_wall_.second.start().x(), gate_wall_.second.start().y(),
                           local_view.getLocalizationPtr()->vehicleAlignPosePoint().z());
    math::Vec3d end_pose(gate_wall_.second.end().x(), gate_wall_.second.end().y(),
                         local_view.getLocalizationPtr()->vehicleAlignPosePoint().z());
    transfer::transformPoint(tf_map_2_ego, &start_pose);
    transfer::transformPoint(tf_map_2_ego, &end_pose);
    std::vector<math::Vec2d> gate_wall_v2d;
    gate_wall_v2d.push_back(start_pose);
    gate_wall_v2d.push_back(end_pose);
    park_roi_decider_.addBoundary(std::make_tuple("gate_wall", OpenspaceObjectType::GATE_WALL, gate_wall_v2d));
  }
}

void VerticalParkingOutPlanner::trajPostProcess(const LocalView& local_view, const RoiDecideResult& roi) {
  if (optimizer_segmented_flu_path_.empty() || optimizer_segmented_flu_path_.back().empty())
    return;

  // 拼接参考线，如果搜索终点与参考线偏离过大，则截取接参考线并平移，再拼接
  auto back_pt = optimizer_segmented_flu_path_.back().back();
  Eigen::Matrix4d tf_ego_2_map = local_view.getLocalizationPtr()->getTfEgo2Map();
  auto back_pt_map = back_pt;
  transfer::transformPoint(tf_ego_2_map, &back_pt_map);
  math::Vec3d map_rpy_vec3d(0.0, 0.0, back_pt_map.theta());
  transfer::transformRPY(tf_ego_2_map, &map_rpy_vec3d);
  back_pt_map.set_theta(map_rpy_vec3d.z());

  if (target_reference_line_info_.isValid()) {
    auto ref = target_reference_line_info_.ref_line();
    const double joint_length = 15.0;
    double s = 0.0;
    double l = 0.0;
    int min_idx = -1;
    double min_distance = 0.0;
    const double heading_threshold = 30.0;  // deg
    ref.getProjection(back_pt_map, back_pt_map.theta(), heading_threshold, s, l, min_distance, min_idx);
    auto ref_pts = ref.getReferencePoints(s, std::min(s + joint_length, ref.length()));
    if (ref_pts.size() < 2) {
      return;
    }
    ref_pts.erase(ref_pts.begin());

    if (back_pt.direction() == PathPt::Direction::BACKWARD) {
      optimizer_segmented_flu_path_.emplace_back();
    }
    Eigen::Matrix4d tf_map_2_ego = local_view.getLocalizationPtr()->getTfMap2Ego();
    for (const auto& ref_pt : ref_pts) {
      PathPt joint_pt(ref_pt, ref_pt.slope(), ref_pt.heading(), ref_pt.kappa(), ref_pt.dkappa());
      transfer::transformPoint(tf_map_2_ego, &joint_pt);
      math::Vec3d ego_rpy_vec3d(0.0, 0.0, joint_pt.theta());
      transfer::transformRPY(tf_map_2_ego, &ego_rpy_vec3d);
      joint_pt.set_theta(ego_rpy_vec3d.z());

      // 偏移距离，确保与参考线连贯
      double offset_heading = math::NormalizeAngle(back_pt.theta() + (l < 0.0 ? -M_PI_2 : M_PI_2));
      joint_pt.set_x(joint_pt.x() + l * cos(offset_heading));
      joint_pt.set_y(joint_pt.y() + l * sin(offset_heading));

      optimizer_segmented_flu_path_.back().emplace_back(joint_pt);
    }
  }

  return;
}

bool VerticalParkingOutPlanner::isParkingCompleted(const LocalView& local_view, const bool destination_stop_flag) {
  if (planner_status_ == PlannerStatus::FINISHED || planner_status_ == PlannerStatus::HOLD_ON)
    return true;

  if (target_reference_line_info_.isValid()) {
    // 判断自车是否在参考线附近
    auto veh_map_pt = local_view.getLocalizationPtr()->vehicleAlignPosePoint();
    auto ref = target_reference_line_info_.ref_line();
    auto ref_pt = ref.getReferencePoint(veh_map_pt.x(), veh_map_pt.y());
    double delta_heading = math::NormalizeAngle(ref_pt.heading() - veh_map_pt.yaw());

    SLPoint end_pose_sl;
    ref.xy2sl(end_pose_map_bak_.second, end_pose_map_bak_.second.theta(), &end_pose_sl);
    auto veh_pose_sl = ref.getFrenetPoint(PathPt(veh_map_pt.x(), veh_map_pt.y(), veh_map_pt.z()));
    double delta_s = end_pose_sl.s() - veh_pose_sl.s();
    double delta_l = veh_pose_sl.l();

    PARKING_LOG(I, "delta_s: ", delta_s, " delta_heading: ", delta_heading * RAD2ANG, " delta_l: ", delta_l);
    debug_info_ +=
        fmt::format("Ref Error: s: {:.2f}, h: {:.2f}, l: {:.2f}\n", delta_s, delta_heading * RAD2ANG, delta_l);

    // 如果是Uturn场景需要判断是否成功转弯
    double u_turn_kappa = 0.0;
    if (uTurnJudge(ref, veh_pose_sl.s(), 30, 30, u_turn_kappa)) {
      // delta_heading
      delta_heading = math::NormalizeAngle(end_pose_map_bak_.second.theta() - veh_map_pt.yaw());
      // delta_l
      double s = 0.0;
      int min_idx = -1;
      double min_distance = 0.0;
      ref.getProjection(math::Vec3d(veh_map_pt.x(), veh_map_pt.y(), veh_map_pt.z()), veh_map_pt.yaw(), s, delta_l,
                        min_distance, min_idx);
    }

    if ((realtime_optimizer_segmented_flu_paths_.size() <= 1
         && delta_s < planner_profile_->park_completion_config().dis_2_end_pose())
        || (fabs(delta_heading) < planner_profile_->park_completion_config().heading_error_2_reference_line() * ANG2RAD
            && fabs(delta_l) < planner_profile_->park_completion_config().l_2_reference_line())) {
      planner_status_ = PlannerStatus::HOLD_ON;
      return true;
    }

  } else {
    if ((fabs(local_view.getChassisPtr()->Speed()) < (planner_profile_->parkin_stop_speed() * KMH_MS))
        && remain_dis_ <= planner_profile_->shift_min_dis_require() && destination_stop_flag) {
      // 反馈纵向泊车完成
      planner_status_ = PlannerStatus::FINISHED;
      return true;
    }
  }

  return false;
}

bool VerticalParkingOutPlanner::uTurnJudge(const ReferenceLine& ref, const double s, const double forward_distance_thrd,
                                           const double backward_dist_thrd, double& u_turn_kappa) {
  u_turn_kappa = 0.0;
  for (const auto& direc : ref.getDirectionsFromSRange(s, forward_distance_thrd, backward_dist_thrd)) {
    if (direc.direction == DrivingDirection::kDirectionUTurnOnly) {
      auto uturn_point = ref.getReferencePoint((direc.start_s + direc.end_s) / 2.0);
      u_turn_kappa = uturn_point.kappa();
      return true;
    }
  }
  return false;
}

}  // namespace gpal::pnc::planning