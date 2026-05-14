// LaneMarkerCoordinates.h
#ifndef _LANE_MARKING_PROCESS_H_
#define _LANE_MARKING_PROCESS_H_
#include <algorithm>
#include <array>
#include <iostream>
#include <vector>

#include "cubic_spline_interpolate.h"
#include "math/vec2d.h"

namespace gpal::pnc::planning {

class LaneMarkingProcess {
 private:
  std::vector<math::Vec3d> optimal_values_;  // 卡尔曼滤波中的最优估计值
  std::vector<double> variances_;            // 最优估计值方差
 public:
  LaneMarkingProcess();
  void Clear();
  void TrimStart();
  void Update(const std::vector<math::Vec3d> &lane_marking_buffer, const double speed, const bool is_curve,
              const double max_x_updatable);
  void Transform(const double veh_spd, const double yaw_rate);
  std::vector<math::Vec3d> Get();

 private:
  void FillPriorBasedOnMarker(const std::array<double, 2> &x_table, const std::array<double, 2> &y_table,
                              const std::vector<math::Vec3d> &lane_marking_buffer, const std::vector<double> slopes,
                              const double max_x_updatable);
  void FillPriorBasedOnOldData(const std::array<double, 2> &x_table, const std::array<double, 2> &y_table,
                               const std::vector<math::Vec3d> &lane_marking_buffer, const bool is_curve,
                               const double max_x_updatable);
  double SearchIndex(const std::array<double, 2> &x_table, const std::array<double, 2> &y_table, const double x);
};
}  // namespace gpal::pnc::planning

#endif  // LANEMARKERCOORDINATES_H
