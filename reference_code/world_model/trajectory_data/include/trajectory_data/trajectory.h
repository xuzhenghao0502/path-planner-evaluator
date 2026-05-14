#pragma once

#include <utility>
#include <vector>

#include "point/trajectory_pt.h"

namespace gpal::pnc::planning {

class Trajectory : public std::vector<TrajectoryPt> {
 public:
  /**
   * @brief 默认构造函数
   * @details 创建一个空的离散轨迹对象
   */
  Trajectory() = default;

  explicit Trajectory(std::vector<TrajectoryPt> trajectory_points);

  double length() const;

  TrajectoryPt evaluate(const double traj_s) const;

  TrajectoryPt evaluateReverse(const double traj_s) const;

  TrajectoryPt getNearestPoint(const gpal::pnc::planning::math::Vec3d& pt, double& min_dist) const;

  TrajectoryPt getNearestPoint(const gpal::pnc::planning::math::Vec3d& pt, double start_s, double end_s,
                         double& min_dist) const;

  void getTrajectoryPts(const double lower_s, const double upper_s, Trajectory* traj_pts) const;

 protected:
  std::vector<TrajectoryPt>::const_iterator queryLowerBound(const double traj_s) const;
  std::vector<TrajectoryPt>::const_iterator queryUpperBound(const double traj_s) const;
};


}