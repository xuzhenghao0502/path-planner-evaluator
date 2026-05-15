#pragma once

#include <limits>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "obstacle/obstacle.h"
#include "path/path_boundary.h"
#include "path/path_data.h"
#include "point/trajectory_pt.h"
#include "proto/vehicle_state/vehicle_state.pb.h"
#include "reference_line/reference_line.h"
// #include "speed/speed_data.h"

namespace gpal::pnc::planning {

/**
 * @class ReferenceLineInfo
 * @brief ReferenceLineInfo holds all data for one reference line.
 */
class ReferenceLineInfo {
 public:
  ReferenceLineInfo() = default;

  ReferenceLineInfo(const VehicleState& vehicle_state, const TrajectoryPt& adc_planning_point,
                    const ReferenceLine& ref_line)
      : vehicle_state_(vehicle_state), adc_planning_point_(adc_planning_point), ref_line_(ref_line) {
    // 输入的adc_planning_point的s是基于历史轨迹计算的，这里需要用ref_line_更新s以保证匹配
    double matched_s =
        ref_line_.getReferencePoint(adc_planning_point_.path_pt().x(), adc_planning_point_.path_pt().y()).local_s();
    adc_planning_point_.mutable_path_pt()->set_s(matched_s);
  }

  const VehicleState& vehicle_state() const { return vehicle_state_; }

  const TrajectoryPt& adc_planning_point() const { return adc_planning_point_; }

  PathBoundary* mutable_path_boundary() { return &path_boundary_; }
  const PathBoundary& path_boundary() const { return path_boundary_; }

  const ReferenceLine& ref_line() const { return ref_line_; }
  ReferenceLine* mutable_ref_line() { return &ref_line_; }

  const bool isValid() const { return !ref_line_.reference_points().empty(); }

 private:
  VehicleState vehicle_state_;
  TrajectoryPt adc_planning_point_;
  ReferenceLine ref_line_;

  PathBoundary path_boundary_;
};

}  // namespace gpal::pnc::planning
