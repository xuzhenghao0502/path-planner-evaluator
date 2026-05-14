#include "openspace_path_planner/application/escape_planner.h"

namespace gpal::pnc::planning {
void EscapePlanner::parkEnvRecognizer(const LocalView& local_view,
                                      const std::vector<Decision::DecisionObject>& select_ods,
                                      const std::vector<math::LineSegment2d>& boundary_seg) {
  park_env_type_ = ParkEnvType::GENERAL;
  return;
}

bool EscapePlanner::generateSearchElements(const LocalView& local_view,
                                           const std::vector<Decision::DecisionObject>& select_ods,
                                           const std::vector<math::LineSegment2d>& boundary_seg,
                                           SearchElements& search_elements) {
  PathPt start_pose;
  PathPt end_pose;
  PARKING_LOG(D, "end_pose_map_bak_: ", end_pose_map_bak_.first, " pt: ", end_pose_map_bak_.second.x(), " ",
              end_pose_map_bak_.second.y(), " ", end_pose_map_bak_.second.theta());

  Eigen::Matrix4d tf_map_2_ego = local_view.getLocalizationPtr()->getTfMap2Ego();
  Eigen::Matrix4d tf_ego_2_map = local_view.getLocalizationPtr()->getTfEgo2Map();
  if (end_pose_map_bak_.first) {
    end_pose = end_pose_map_bak_.second;
    transfer::transformPoint(tf_map_2_ego, &end_pose);
    math::Vec3d ego_rpy_vec3d(0.0, 0.0, end_pose.theta());
    transfer::transformRPY(tf_map_2_ego, &ego_rpy_vec3d);
    end_pose.set_theta(ego_rpy_vec3d.z());
  } else {
    if (!target_reference_line_info_.isValid()) {
      return false;
    }
    auto ref = target_reference_line_info_.ref_line();

    // 当前距离任务终点的距离
    auto destination = ref.getDestinationData();
    auto remain_distance = destination.destination_point.local_s() - ref.adcS();

    double end_pose_dis = planner_profile_->parkout_endpt_ref_dis();
    auto end_pose_ref_pt = ref.getNearestReferencePoint(ref.adcS() + std::min(end_pose_dis, remain_distance));
    end_pose = PathPt(end_pose_ref_pt);
    end_pose.set_theta(end_pose_ref_pt.heading());

    transfer::transformPoint(tf_map_2_ego, &end_pose);
    math::Vec3d ego_rpy_vec3d(0.0, 0.0, end_pose.theta());
    transfer::transformRPY(tf_map_2_ego, &ego_rpy_vec3d);
    end_pose.set_theta(ego_rpy_vec3d.z());

    // 判断终点位置合理性
    endPointMoveToSafePosition(end_pose, local_view, select_ods, boundary_seg);

    // 若搜索终点距离任务终点过近，调整为任务终点
    auto end_pose_modify = end_pose;
    transfer::transformPoint(tf_ego_2_map, &end_pose_modify);
    ego_rpy_vec3d = math::Vec3d(0.0, 0.0, end_pose_modify.theta());
    transfer::transformRPY(tf_ego_2_map, &ego_rpy_vec3d);
    end_pose_modify.set_theta(ego_rpy_vec3d.z());
    SLPoint end_pose_modify_sl;
    ref.xy2sl(end_pose_modify, end_pose_modify.theta(), &end_pose_modify_sl);
    PARKING_LOG(D, "end_pose_modify_sl.s(): ", end_pose_modify_sl.s(),
                " destination.destination_point.local_s(): ", destination.destination_point.local_s());
    if (destination.destination_point.local_s() - end_pose_modify_sl.s()
        < planner_profile_->escape_task_destination_dis_threshold()) {
      end_pose_map_bak_ =
          make_pair(true, PathPt(destination.destination_point, 0.0, destination.destination_point.heading()));
      end_pose = end_pose_map_bak_.second;
      transfer::transformPoint(tf_map_2_ego, &end_pose);
      math::Vec3d ego_rpy_vec3d(0.0, 0.0, end_pose.theta());
      transfer::transformRPY(tf_map_2_ego, &ego_rpy_vec3d);
      end_pose.set_theta(ego_rpy_vec3d.z());
    } else {
      end_pose_map_bak_ = make_pair(true, end_pose);
      transfer::transformPoint(tf_ego_2_map, &end_pose_map_bak_.second);
      ego_rpy_vec3d = math::Vec3d(0.0, 0.0, end_pose_map_bak_.second.theta());
      transfer::transformRPY(tf_ego_2_map, &ego_rpy_vec3d);
      end_pose_map_bak_.second.set_theta(ego_rpy_vec3d.z());
    }
  }

  search_elements = SearchElements(start_pose, end_pose);

  // 搜索目标区域设置
  search_elements.is_region_search = planner_profile_->end_region_config().enable_end_region();
  search_elements.goal_region =
      math::Box2d(end_pose, end_pose.theta(), planner_profile_->end_region_config().end_region_front_range(),
                  planner_profile_->end_region_config().end_region_back_range(),
                  planner_profile_->end_region_config().end_region_width_range());
  search_elements.goal_region_heading = end_pose.theta();
  search_elements.goal_region_heading_tolerance = planner_profile_->end_region_config().end_region_heading_tolerance();

  return true;
}

void EscapePlanner::trajPostProcess(const LocalView& local_view, const RoiDecideResult& roi) {
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
    SLPoint back_sl;
    const double joint_length = 10.0;
    ref.xy2sl(back_pt_map, back_pt_map.theta(), &back_sl);
    auto ref_pts = ref.getReferencePoints(back_sl.s(), std::min(back_sl.s() + joint_length, ref.length()));
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
      double offset_dis = abs(back_sl.l());
      double offset_heading = math::NormalizeAngle(back_pt.theta() + (back_sl.l() < 0.0 ? -M_PI_2 : M_PI_2));
      joint_pt.set_x(joint_pt.x() + offset_dis * cos(offset_heading));
      joint_pt.set_y(joint_pt.y() + offset_dis * sin(offset_heading));

      optimizer_segmented_flu_path_.back().emplace_back(joint_pt);
    }
  }

  return;
}

bool EscapePlanner::isParkingCompleted(const LocalView& local_view, const bool destination_stop_flag) {
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

    // 结束脱困条件：（1 || 2）
    // 1. 脱困轨迹完全走完；
    // 2. 超过设定的脱困终点时，自车位姿已回归参考线；
    // （说明：超过设定脱困终点不等于脱困轨迹走完，搜索可能在终点附近存在折返轨迹）
    if ((realtime_optimizer_segmented_flu_paths_.size() <= 1
         && delta_s < planner_profile_->park_completion_config().dis_2_end_pose())
        || (delta_s < planner_profile_->park_completion_config().dis_2_end_pose()
            && fabs(delta_heading)
                   < planner_profile_->park_completion_config().heading_error_2_reference_line() * ANG2RAD
            && fabs(delta_l) < planner_profile_->park_completion_config().l_2_reference_line())) {
      planner_status_ = PlannerStatus::HOLD_ON;
      return true;
    }
  }

  return false;
}

}  // namespace gpal::pnc::planning