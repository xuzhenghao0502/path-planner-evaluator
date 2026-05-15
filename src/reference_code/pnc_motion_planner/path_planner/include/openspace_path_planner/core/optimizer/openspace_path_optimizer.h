#pragma once

#include <any>

#include "b_spline/b_spline_2d.h"
#include "b_spline/b_spline_2d_normal_line_smoother.h"
#include "base/log.h"
#include "config_manager/config_manager.h"
#include "math/discrete_points_math.h"
#include "math/line_segment2d.h"
#include "math/math_utils.h"
#include "math/vec2d.h"
#include "openspace_path_planner/core/manager/openspace_optimizer_data.h"
#include "openspace_path_planner/core/openspace_path_planner.h"
#include "openspace_path_planner/core/optimizer/openspace_ocp_bound_parser.h"
#include "openspace_path_planner/core/optimizer/openspace_ocp_optimizer.h"
#include "openspace_path_planner/utils/openspace_common.h"
#include "openspace_path_planner/utils/openspace_tools.h"
#include "path/discretized_path.h"
#include "path/path_data.h"
#include "point/path_pt.h"

namespace gpal::pnc::planning {

// 用于存储点在路径上投影结果的结构体
struct PathProjectionResult {
  size_t closest_segment_start_index = 0;  // 最近点所在线段的起始点索引
  double distance_along_path = 0.0;        // 投影点在整条路径上的累计里程s值
  PathPt projected_point;                  // 投影点的具体信息 (x, y, theta...)
};
class OpenspacePathOptimizer : public BaseOpenspacePathPlanner {
 public:
  OpenspacePathOptimizer() = default;
  virtual ~OpenspacePathOptimizer();
  void clear();
  OpenspaceStatus run(std::any& data) override;
  std::string getDebugInfo() { return debug_info_; };

 private:
  void smoothPathPoints(const DiscretizedPath& path);
  void pathPreProcess(const DiscretizedPath& orin_traj, vector<DiscretizedPath>& target_partition_discret_line,
                      DiscretizedPath& target_discret_line);
  void pathPartition(const DiscretizedPath& orin_traj, vector<DiscretizedPath>& partition_path, bool only_partition);
  void loadKappa(DiscretizedPath& orin_traj);
  void generateReferenceLine(const Eigen::Matrix4d& tf_ego_2_map, const DiscretizedPath& path_ego,
                             const PathPt& init_pt_ego, TrajectoryPt& init_pt_map, ReferenceLine& reference_line_ego,
                             ReferenceLine& reference_line_map);
  void calOptimizerBound(const ReferenceLine& reference_line,
                         const std::vector<std::tuple<double, double, double>>& boundary,
                         std::vector<math::Vec3d>& bound_left, std::vector<math::Vec3d>& bound_right);
  void calOptimizerBounds(const ReferenceLine& reference_line, const PathBoundary& path_boundary,
                          PathData::BoundsVec3dWithId& vis_bd);
  // 计算一个点到路径的投影的辅助函数
  std::optional<PathProjectionResult> findProjectionOnPath(const DiscretizedPath& path,
                                                           const PathPt& point_to_project) const;
  void calculateSteer(DiscretizedPath& path, double wheelbase);

 private:
  enum class SmoothStatus : int32_t { INVALID = -1, SUCCESS = 0, FAILED = 1 };
  OpenspaceOcpBoundParser ocp_bound_parser_;
  OpenspaceOcpOptimizer ocp_optimizer_;
  SmoothStatus smooth_status_ = SmoothStatus::INVALID;
  DiscretizedPath smooth_path_;
  std::vector<DiscretizedPath> optimizer_flu_path_;
  DiscretizedPath optimizer_whole_flu_path_;
  vector<PathData::BoundsVec3dWithId> boundarys_vis_;

 private:
  vector<DiscretizedPath> target_cal_discret_line_;
  int optimizer_index_ = 0;
  OpenspaceStatus optimizer_status_ = OpenspaceStatus::DEFAULT;
  ReferenceLine reference_line_ego_;
  ReferenceLine reference_line_map_;
  TrajectoryPt init_pt_map_;
  PathBoundary boundary_;
  bool is_fallback_attempt_ = false;

 public:
  void setPiecewisePoint(const PathPt& piecewise_point) { piecewise_point_ = piecewise_point; }

  void setScenarioIngoralFlag(const bool& enable_scenario_optimizer) {
    enable_scenario_optimizer_ = enable_scenario_optimizer;
  }

 private:
  void setScenarioSRange();

  PathPt piecewise_point_;
  bool enable_scenario_optimizer_ = false;
  std::vector<std::tuple<std::string, float, float, bool>> scenario_s_range_;
  std::string debug_info_ = "optimizer: ";
};

}  // namespace gpal::pnc::planning