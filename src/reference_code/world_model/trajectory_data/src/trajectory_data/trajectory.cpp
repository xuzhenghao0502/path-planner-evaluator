
#include "Eigen/Dense"
#include <algorithm>

#include "base/log.h"
#include "math/linear_interpolation.h"

#include "trajectory_data/trajectory.h"

namespace gpal::pnc::planning {

namespace {

double sign(double value, double epsilon = 1e-10) {
  if (value > epsilon) {
    return 1.0;
  } else if (value < -epsilon) {
    return -1.0;
  }
  return 0.0;
}

double clamp(double value, double lower, double upper) {
  value = std::min(value, upper);
  value = std::max(value, lower);
  return value;
}

double getNewtonStep(const double dJ, const double ddJ, const double max_step, double epsilon = 1e-10) {
  if (std::abs(ddJ) < epsilon) {
    return -sign(dJ) * max_step;
  }
  return clamp(-dJ / ddJ, -max_step, max_step);
}

}  // namespace

Trajectory::Trajectory(std::vector<TrajectoryPt> traj_points) : std::vector<TrajectoryPt>(std::move(traj_points)) {}


double Trajectory::length() const {
  if (empty()) {
    return 0.0;
  }
  return back().path_pt().s() - front().path_pt().s();
}

TrajectoryPt Trajectory::evaluate(const double traj_s) const {
  ACHECK(!empty());
  auto it_lower = queryLowerBound(traj_s);
  if (it_lower == begin()) {
    return front();
  }
  if (it_lower == end()) {
    return back();
  }
  return gpal::pnc::planning::math::interpolateUsingLinearApproximation(*(it_lower - 1), *it_lower, traj_s);
}


TrajectoryPt Trajectory::getNearestPoint(const gpal::pnc::planning::math::Vec3d& pt, double& min_dist) const {
  return getNearestPoint(pt, front().path_pt().s(), back().path_pt().s(), min_dist);
}

TrajectoryPt Trajectory::getNearestPoint(const gpal::pnc::planning::math::Vec3d& pt, double u0, double u1,
                                        double& min_dist) const {
  TrajectoryPt min_pt;
  if (u0 > u1) {
    std::swap(u0, u1);
  }
  std::pair<double, double> s_range(u0, u1);

  constexpr size_t iter_max = 10;
  constexpr double tol = 1e-3;
  double max_step = 0.5 * (s_range.second - s_range.first);
  min_dist = std::numeric_limits<double>::max();
  std::pair<double, double> newton_range(u0, u1);
  double min_traj_s = 0;
  bool min_found = false;
  double ui = s_range.first;
  bool has_extend_front = false;
  bool has_extend_rear = false;
  for (size_t i = 0; i < iter_max; i++) {
    auto traj_pt = evaluate(ui);
    Eigen::Vector2d pe(traj_pt.path_pt().x() - pt.x(), traj_pt.path_pt().y() - pt.y());
    double dist = pe.norm();
    if (dist < min_dist) {
      min_dist = dist;
      min_traj_s = ui;
    }
    const double cos_theta = cos(traj_pt.path_pt().theta());
    const double sin_theta = sin(traj_pt.path_pt().theta());
    Eigen::Vector2d dp(cos_theta, sin_theta);
    Eigen::Vector2d ddp(-sin_theta * traj_pt.path_pt().kappa(), cos_theta * traj_pt.path_pt().kappa());
    const double v = 1.0;
    const double dJ = v * (dp.transpose() * pe)(0);
    const double ddJ = (v * ddp.transpose() * pe + dp.transpose() * dp)(0);
    const double du = getNewtonStep(dJ, ddJ, max_step);
    if (std::abs(du) < tol) {
      min_found = true;
      break;
    }
    ui = traj_pt.path_pt().s() + du;
    if (du > 0) {
      newton_range.first = std::max(std::max(traj_pt.path_pt().s(), newton_range.first), s_range.first);
      newton_range.second = std::min(std::max(traj_pt.path_pt().s(), newton_range.second), s_range.second);
    } else if (du < 0) {
      newton_range.first = std::max(std::min(traj_pt.path_pt().s(), newton_range.first), s_range.first);
      newton_range.second = std::min(std::min(traj_pt.path_pt().s(), newton_range.second), s_range.second);
    }
    if (ui < s_range.first) {
      if (!has_extend_front) {
        has_extend_front = true;
        ui = min_traj_s;
        max_step *= 0.5;
        continue;
      }
      break;
    } else if (ui > s_range.second) {
      if (!has_extend_rear) {
        has_extend_rear = true;
        ui = min_traj_s;
        max_step *= 0.5;
        continue;
      }
      break;
    }
  }
  min_pt = evaluate(min_traj_s);
  if (!min_found) {
    if (newton_range.first > newton_range.second) {
      std::swap(newton_range.first, newton_range.second);
    }
    auto it_lower = queryLowerBound(newton_range.first);
    auto it_upper = queryLowerBound(newton_range.second);
    for (auto iter = it_lower; iter != it_upper; iter++) {
      double dist = iter->path_pt().DistanceTo(pt);
      if (dist < min_dist) {
        min_dist = dist;
        min_pt = *iter;
      }
    }
  }
  return min_pt;
}


std::vector<TrajectoryPt>::const_iterator Trajectory::queryLowerBound(const double traj_s) const {
  auto func = [](const TrajectoryPt& tp, const double traj_s) { return tp.path_pt().s() < traj_s; };
  return std::lower_bound(begin(), end(), traj_s, func);
}


TrajectoryPt Trajectory::evaluateReverse(const double traj_s) const {
  ACHECK(!empty());
  auto it_upper = queryUpperBound(traj_s);
  if (it_upper == begin()) {
    return front();
  }
  if (it_upper == end()) {
    return back();
  }
  return gpal::pnc::planning::math::interpolateUsingLinearApproximation(*(it_upper - 1), *it_upper, traj_s);
}


std::vector<TrajectoryPt>::const_iterator Trajectory::queryUpperBound(const double traj_s) const {
  auto func = [](const double traj_s, const TrajectoryPt& tp) { return traj_s < tp.path_pt().s(); };
  return std::upper_bound(begin(), end(), traj_s, func);
}

void Trajectory::getTrajectoryPts(const double lower_s, const double upper_s, Trajectory* traj_pts) const {
  traj_pts->clear();
  constexpr double min_legnth = 1.0;
  if (upper_s < lower_s + 1e-6) {
    return;
  }
  auto iter_lower = queryLowerBound(lower_s);
  if (iter_lower != end()) {
    auto iter_upper = queryUpperBound(upper_s);
    traj_pts->assign(iter_lower, iter_upper);
  }
}

}  // namespace gpal::pnc::planning
