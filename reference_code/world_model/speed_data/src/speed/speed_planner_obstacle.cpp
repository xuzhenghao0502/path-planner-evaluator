/**
 * @file speed_planner_obstacle.cpp
 * @brief 速度规划障碍物封装类
 * @details 本类整合感知与决策系统障碍物信息，为速度规划提供统一接口
 */
#include "speed/speed_planner_obstacle.h"
namespace gpal::pnc::planning {

SpeedPlannerObstacle::SpeedPlannerObstacle(const std::shared_ptr<Decision::DecisionObject>& od_decision) {
  if (od_decision == nullptr) {
    setObstacleDefaultPrediction();
    longitudinal_od_tag_ = LongitudinalOdTag::FOLLOW;
    object_game_type_ = ObjectGameType::NON_GAME;
    ERT_PLOG_E << "SpeedPlannerObstacle: od_decision is nullptr";
    return;
  }
  deepCopyFromDecision(od_decision);
}

void SpeedPlannerObstacle::deepCopyFromDecision(const std::shared_ptr<Decision::DecisionObject>& od_decision) {
  try {
    // ===== 1. 拷贝基础属性 =====
    id_ = od_decision->id;
    type_ = od_decision->type;
    speed_ = od_decision->spd;
    acceleration_ = od_decision->acc;
    is_static_ = od_decision->is_static;

    // ===== 2. 拷贝几何信息 =====
    x_ = od_decision->x;
    y_ = od_decision->y;
    z_ = 0.0;
    heading_ = od_decision->heading;
    length_ = od_decision->length;
    width_ = od_decision->width;
    bounding_box_ = math::Box2d({x_, y_}, heading_, length_, width_);

    // ===== 3. 拷贝决策标签 =====
    longitudinal_od_tag_ = od_decision->long_od_tag;
    object_game_type_ = od_decision->game_type;
    obstacle_decision_hwt_ = od_decision->safe_hwt_threshold_lane_change;

    // ===== 4. 拷贝预测轨迹 =====
    if (!od_decision->raw_predictions.empty() && !od_decision->raw_predictions.front().traj.empty()) {
      const auto& traj = od_decision->raw_predictions.front().traj;
      setObstaclePrediction(traj);
    } else {
      setObstacleDefaultPrediction();
      ERT_PLOG_W << "No prediction available, using default trajectory";
    }
  } catch (const std::exception& e) {
    ERT_PLOG_E << "Exception in deepCopyFromDecision for obstacle " << (od_decision ? od_decision->id : "unknown")
               << ": " << e.what();

    setObstacleDefaultPrediction();
    longitudinal_od_tag_ = LongitudinalOdTag::FOLLOW;
    object_game_type_ = ObjectGameType::NON_GAME;
  }
}

void SpeedPlannerObstacle::setObstacleDefaultPrediction() {
  predicted_trajectory_.clear();
  proto::TrajectoryPoint traj_point;
  traj_point.mutable_path_point()->set_x(this->perceptionBoundingBox().center_x());
  traj_point.mutable_path_point()->set_y(this->perceptionBoundingBox().center_y());
  traj_point.mutable_path_point()->set_theta(this->perceptionBoundingBox().heading());
  traj_point.mutable_path_point()->set_s(0.0);
  for (int i = 0; i <= 50; i++) {
    traj_point.set_relative_time(0.1 * i);
    predicted_trajectory_.push_back(traj_point);
  }
}

void SpeedPlannerObstacle::setObstaclePrediction(
    const std::vector<gpal::proto::TrajectoryPoint>& predicted_trajectory) {
  predicted_trajectory_.clear();
  for (const auto& traj_point : predicted_trajectory) {
    if (traj_point.relative_time() > -kEpsilon) {
      predicted_trajectory_.push_back(traj_point);
    }
  }
  if (predicted_trajectory_.empty()) {
    setObstacleDefaultPrediction();
  }
}

proto::TrajectoryPoint SpeedPlannerObstacle::getPointAtTime(const double relative_time) const {
  const auto& points = predicted_trajectory_;
  if (points.size() < 2) {
    proto::TrajectoryPoint point;
    point.mutable_path_point()->set_x(x_);
    point.mutable_path_point()->set_y(y_);
    point.mutable_path_point()->set_z(z_);
    point.mutable_path_point()->set_theta(heading_);
    point.mutable_path_point()->set_s(0.0);
    point.mutable_path_point()->set_kappa(0.0);
    point.mutable_path_point()->set_dkappa(0.0);
    point.set_v(0.0);
    point.set_a(0.0);
    point.set_relative_time(0.0);
    return point;
  } else {
    auto comp = [](const proto::TrajectoryPoint p, const double time) { return p.relative_time() < time; };
    auto it_lower = std::lower_bound(points.begin(), points.end(), relative_time, comp);

    if (it_lower == points.begin()) {
      return *points.begin();
    } else if (it_lower == points.end()) {
      return *points.rbegin();
    }
    return math::interpolateUsingLinearApproximation(*(it_lower - 1), *it_lower, relative_time);
  }
}

math::Box2d SpeedPlannerObstacle::getBoundingBox(const proto::TrajectoryPoint& point) const {
  return math::Box2d({point.path_point().x(), point.path_point().y()}, point.path_point().theta(), length_, width_);
}

math::Box2d SpeedPlannerObstacle::getBoundingBoxAtTime(const double relative_time) const {
  return getBoundingBox(getPointAtTime(relative_time));
}

bool SpeedPlannerObstacle::RiskFieldInfo::s_invade_in(double buffer, double* lower_bound) {
  if (min_box_distance > buffer) {
    *lower_bound = kPostiveInfinity;
    return false;
  }
  auto data = box_distance.getOriginData();
  if (data.front().second <= buffer) {
    *lower_bound = data.front().first;
    return true;
  }
  for (int i = 1; i < data.size(); ++i) {
    if (data[i].second <= buffer) {
      *lower_bound = math::lerp(data[i - 1].first, data[i - 1].second, data[i].first, data[i].second, buffer);
      return true;
    }
  }
  *lower_bound = kPostiveInfinity;
  return false;
}

bool SpeedPlannerObstacle::RiskFieldInfo::s_invade_out(double buffer, double* upper_bound) {
  if (min_box_distance > buffer) {
    *upper_bound = -kPostiveInfinity;
    return false;
  }
  auto data = box_distance.getOriginData();
  if (data.back().second <= buffer) {
    *upper_bound = data.back().first;
    return true;
  }
  for (int i = data.size() - 2; i >= 0; --i) {
    if (data[i].second <= buffer) {
      *upper_bound = math::lerp(data[i].first, data[i].second, data[i + 1].first, data[i + 1].second, buffer);
      return true;
    }
  }
  *upper_bound = -kPostiveInfinity;
  return false;
}

}  // namespace gpal::pnc::planning