#include "openspace_path_planner/application/parallel_parking_out_planner.h"

namespace gpal::pnc::planning {
void ParallelParkingOutPlanner::parkEnvRecognizer(const LocalView& local_view,
                                                  const std::vector<Decision::DecisionObject>& select_ods,
                                                  const std::vector<math::LineSegment2d>& boundary_seg) {
  park_env_type_ = ParkEnvType::GENERAL;
  return;
}

bool ParallelParkingOutPlanner::generateSearchElements(const LocalView& local_view,
                                                       const std::vector<Decision::DecisionObject>& select_ods,
                                                       const std::vector<math::LineSegment2d>& boundary_seg,
                                                       SearchElements& search_elements) {
  PathPt start_pose;
  PathPt end_pose;
  PARKING_LOG(D, "end_pose_map_bak_: ", end_pose_map_bak_.first, " pt: ", end_pose_map_bak_.second.x(), " ",
              end_pose_map_bak_.second.y(), " ", end_pose_map_bak_.second.theta());

  auto park_out_direction = local_view.getConsolePtr()->parkOutDirection();
  PARKING_LOG(D, "park_out_direction: ", park_out_direction);

  Eigen::Matrix4d tf_map_2_ego = local_view.getLocalizationPtr()->getTfMap2Ego();
  Eigen::Matrix4d tf_ego_2_map = local_view.getLocalizationPtr()->getTfEgo2Map();
  if (end_pose_map_bak_.first) {
    end_pose = end_pose_map_bak_.second;
    transfer::transformPoint(tf_map_2_ego, &end_pose);
    math::Vec3d ego_rpy_vec3d(0.0, 0.0, end_pose.theta());
    transfer::transformRPY(tf_map_2_ego, &ego_rpy_vec3d);
    end_pose.set_theta(ego_rpy_vec3d.z());
  } else {
    if (target_reference_line_info_.isValid()) {
      auto ref = target_reference_line_info_.ref_line();
      auto sl_veh_point =
          ref.getFrenetPoint(PathPt(ref.adcLocalization().x(), ref.adcLocalization().y(), ref.adcLocalization().z()));
      auto end_pose_ref_pt = ref.getNearestReferencePoint(sl_veh_point.s() + planner_profile_->parkout_endpt_ref_dis());
      end_pose = PathPt(end_pose_ref_pt);
      end_pose.set_theta(end_pose_ref_pt.heading());

      transfer::transformPoint(tf_map_2_ego, &end_pose);
      math::Vec3d ego_rpy_vec3d(0.0, 0.0, end_pose.theta());
      transfer::transformRPY(tf_map_2_ego, &ego_rpy_vec3d);
      end_pose.set_theta(ego_rpy_vec3d.z());
    } else {
      const double longit_threshold = planner_profile_->end_point_select_config().horizon_end_point_longit_threshold();
      const double lateral_threshold =
          planner_profile_->end_point_select_config().horizon_end_point_lateral_threshold();
      if (park_out_direction == proto::ParkingTaskInfo_ParkingOutDirection_kHorizontalRight) {
        end_pose = PathPt(vehicle_config_->vehicle_param().length() + longit_threshold,
                          -vehicle_config_->vehicle_param().width() / 2.0 - lateral_threshold, 0.0, 0.0, 0.0);
      } else {
        end_pose = PathPt(vehicle_config_->vehicle_param().length() + longit_threshold,
                          vehicle_config_->vehicle_param().width() / 2.0 + lateral_threshold, 0.0, 0.0, 0.0);
      }
    }

    // 判断终点位置合理性
    endPointMoveToSafePosition(end_pose, local_view, select_ods, boundary_seg);

    end_pose_map_bak_ = make_pair(true, end_pose);
    transfer::transformPoint(tf_ego_2_map, &end_pose_map_bak_.second);
    math::Vec3d ego_rpy_vec3d(0.0, 0.0, end_pose_map_bak_.second.theta());
    transfer::transformRPY(tf_ego_2_map, &ego_rpy_vec3d);
    end_pose_map_bak_.second.set_theta(ego_rpy_vec3d.z());
  }

  search_elements = SearchElements(start_pose, end_pose);

  return true;
}

void ParallelParkingOutPlanner::customBoundary(const LocalView& local_view,
                                               std::vector<Decision::DecisionObject>& select_ods) {
  if (!planner_profile_->custom_boundary_config().enable_custom_boundary()) {
    return;
  }
  /*
   * 自定义边界约束
   *
   *                       boundary_roadside
   * ———————————————————————————————————————————————————————————————————
   *                |   +———————————————————+    |
   *                |   |                   |    |
   * boundary_rear  |   |        veh        |    | boundary_front           y
   *                |   |                   |    |                         ^
   *                |   +———————————————————+    |                         |
   *                                                                          ——>  x
   *                              |
   *                              |
   *                              |
   *                              v  泊出方向
   *
   * ———————————————————————————————————————————————————————————————————
   *                        boundary_aisle
   */
  if (planner_profile_->custom_boundary_config().enable_custom_virtual_boundary()) {
    std::vector<math::Vec2d> boundary_roadside;
    std::vector<math::Vec2d> boundary_front;
    std::vector<math::Vec2d> boundary_rear;
    std::vector<math::Vec2d> boundary_aisle;

    // 路沿边界
    const double roadside_gap = planner_profile_->custom_boundary_config().roadside_gap();
    math::Vec2d roadside_point1(-30, vehicle_config_->vehicle_param().width() / 2.0 + roadside_gap);

    math::Vec2d roadside_point2(30, vehicle_config_->vehicle_param().width() / 2.0 + roadside_gap);
    boundary_roadside.push_back(roadside_point1);
    boundary_roadside.push_back(roadside_point2);
    park_roi_decider_.addBoundary(std::make_tuple("roadside", OpenspaceObjectType::ROAD_SIDE, boundary_roadside));

    // 库位前后边界
    const double before_gap = planner_profile_->custom_boundary_config().both_sides_gap();
    const double after_gap = planner_profile_->custom_boundary_config().both_sides_gap();
    math::Vec2d front_point1(vehicle_config_->vehicle_param().front_edge_to_ego() + before_gap,
                             vehicle_config_->vehicle_param().width() / 2.0);

    math::Vec2d front_point2(vehicle_config_->vehicle_param().front_edge_to_ego() + before_gap,
                             -vehicle_config_->vehicle_param().width() / 2.0);
    boundary_front.push_back(front_point1);
    boundary_front.push_back(front_point2);
    park_roi_decider_.addBoundary(std::make_tuple("slot_front", OpenspaceObjectType::SLOT, boundary_front));

    math::Vec2d rear_point1(-vehicle_config_->vehicle_param().rear_edge_to_ego() - after_gap,
                            vehicle_config_->vehicle_param().width() / 2.0);

    math::Vec2d rear_point2(-vehicle_config_->vehicle_param().rear_edge_to_ego() - after_gap,
                            -vehicle_config_->vehicle_param().width() / 2.0);
    boundary_rear.push_back(rear_point1);
    boundary_rear.push_back(rear_point2);
    park_roi_decider_.addBoundary(std::make_tuple("slot_rear", OpenspaceObjectType::SLOT, boundary_rear));

    // 通道边界
    const double aisle_gap = planner_profile_->custom_boundary_config().aisle_gap();
    math::Vec2d aisle_point1(30, -vehicle_config_->vehicle_param().width() / 2.0 - aisle_gap);

    math::Vec2d aisle_point2(-30, -vehicle_config_->vehicle_param().width() / 2.0 - aisle_gap);

    boundary_aisle.push_back(aisle_point1);
    boundary_aisle.push_back(aisle_point2);
    park_roi_decider_.addBoundary(std::make_tuple("aisle", OpenspaceObjectType::AISLE, boundary_aisle));
  }
}


void ParallelParkingOutPlanner::trajPostProcess(const LocalView& local_view, const RoiDecideResult& roi) {
  if (optimizer_segmented_flu_path_.empty() || optimizer_segmented_flu_path_.back().empty())
    return;

  // 拼接参考线
  auto back_pt = optimizer_segmented_flu_path_.back().back();
  Eigen::Matrix4d tf_ego_2_map = local_view.getLocalizationPtr()->getTfEgo2Map();
  transfer::transformPoint(tf_ego_2_map, &back_pt);
  math::Vec3d ego_rpy_vec3d(0.0, 0.0, back_pt.theta());
  transfer::transformRPY(tf_ego_2_map, &ego_rpy_vec3d);
  back_pt.set_theta(ego_rpy_vec3d.z());

  if (target_reference_line_info_.isValid()) {
    auto ref = target_reference_line_info_.ref_line();
    SLPoint back_sl;
    ref.xy2sl(back_pt, back_pt.theta(), &back_sl);
    auto ref_pts = ref.getReferencePoints(back_sl.s(), ref.length());
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

      optimizer_segmented_flu_path_.back().emplace_back(joint_pt);
    }
  }

  return;
}

bool ParallelParkingOutPlanner::isParkingCompleted(const LocalView& local_view, const bool destination_stop_flag) {
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


}  // namespace gpal::pnc::planning