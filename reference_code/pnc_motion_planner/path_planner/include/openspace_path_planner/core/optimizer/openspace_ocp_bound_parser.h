#pragma once

#include <memory>
#include <string>
#include <vector>

#include "config_manager/config_manager.h"
#include "path/path_boundary.h"
#include "path/path_decision.h"
#include "path_bound_parser/path_bound_filter.h"
#include "path_bound_parser/path_bound_parser.h"
#include "point/path_pt.h"
// #include <decision_data/decision_data.h>
#include "basic_algorithm_lib/basic_algorithm_lib.h"
// #include <vehicle_config.pb.h>
#include "base/status.h"
#include "math/math_utils.h"
// #include "local_view/Obstacles.h"
// #include "local_view/Freespace.h"
#include <path/path_data.h>
#include <reference_line_info/reference_line_info.h>

#include <Eigen/Core>

#include "base/singleton.h"
#include "config/path_bound_parser/path_bound_parser_config.pb.h"
#include "config/path_bound_parser/path_bound_points_config.pb.h"
#include "decision_data/decision_result.h"
#include "local_view/local_view.h"
#include "ocp/ocp_model.h"
#include "openspace_path_planner/utils/openspace_common.h"
#include "util/timer.h"

namespace gpal::pnc::planning {

class OpenspaceOcpBoundParser {
 public:
  OpenspaceOcpBoundParser() = default;
  ~OpenspaceOcpBoundParser() = default;

 public:
  bool init();
  void reset();
  void preProcess(const ReferenceLine& reference_line, const TrajectoryPt& init_point,
                  const Eigen::Matrix4d& tf_ego_map);
  Status generateBounds(const Freespace& freespace, const std::vector<ObstaclesLinesegment>& obstacles_linesegments,
                        const ReferenceLine& reference_line, const bool consider_bound, PathBoundary& boundary,
                        const PathPt::Direction direction);
  void setModelName(const string name) { model_name_ = name; }

 private:
  void resetFilter();
  bool initPathBoundary(const ReferenceLine& reference_line, PathBoundary& boundary, const PathPt::Direction direction);
  bool getBoundaryFromFreespace(const Freespace& freespace,
                                std::vector<std::tuple<double, double, double>>* const barrier_boundary,
                                std::vector<std::tuple<double, double, double>>* const soft_boundary,
                                std::pair<bool, double>* block_freespace_info);
  bool getBoundaryFromStaticObstacles(const ReferenceLine& reference_line,
                                      const std::vector<ObstaclesLinesegment>& obstacles_linesegments,
                                      std::vector<std::tuple<double, double, double>>* const barrier_boundary,
                                      std::vector<std::tuple<double, double, double>>* const soft_boundary,
                                      std::string* blocking_obstacle_id);

  std::pair<double, double> getFsLaterSafeBuffer(const bool use_obs_type, const bool use_obs_speed);
  bool updateBoundary(const double config_width, double right_bound, double left_bound,
                      std::tuple<double, double, double>& boundary, const bool return_if_valid = true);
  bool initSamplePoints(const ReferenceLine& reference_line, std::vector<ReferencePoint>* const sample_points);
  std::vector<std::shared_ptr<Decision::DecisionObject>> generateStaticObstacles(
      const std::vector<std::shared_ptr<Decision::DecisionObject>>& all_obstacles, const ReferenceLine& reference_line);
  std::pair<double, double> getObsLaterSafeBuffer(const std::shared_ptr<Decision::DecisionObject> obs,
                                                  const bool use_obs_type, const bool use_obs_speed);
  std::pair<double, double> getObsLaterSafeBufferFromType(const Decision::ObjectType& obs_type,
                                                          const double default_barrier_lateral_buffer,
                                                          const double default_soft_lateral_buffer);
  std::pair<double, double> getObsLaterSafeBufferFromSpeed(const std::shared_ptr<Decision::DecisionObject> obs,
                                                           const double default_barrier_lateral_buffer,
                                                           const double default_soft_lateral_buffer);

  std::pair<double, double> getFsLaterSafeBufferFromSpeed(const double default_barrier_lateral_buffer,
                                                          const double default_soft_lateral_buffer);
  void trimPathBounds(const int path_blocked_idx,
                      std::vector<std::tuple<double, double, double>>* const path_boundaries);
  std::vector<PolylineInput> generatePolylines(const std::vector<ObstaclesLinesegment>& obstacles_linesegments,
                                               const ReferenceLine& reference_line);
  math::Vec2d calculatePedalPoint(const math::Vec2d point, const math::Vec2d& line_start, const math::Vec2d& line_end);

 private:
  bool init_ = false;
  ConfigManager* config_manager_ = nullptr;
  PathBoundPointsConfig config_;
  std::unordered_map<int, PathBoundPointsConfig::ObstacleInfo> obs_type_lateral_distance_map_;

  std::unique_ptr<PathBoundParser> bound_parser_;
  std::unique_ptr<PathBoundFilter> freespace_barrier_bound_filter_;
  std::unique_ptr<PathBoundFilter> freespace_soft_bound_filter_;
  std::unique_ptr<PathBoundFilter> static_obstacle_barrier_bound_filter_;
  std::unique_ptr<PathBoundFilter> static_obstacle_soft_bound_filter_;
  TrajectoryPt planning_start_point_;
  Eigen::Matrix4d tf_ego_map_;

  util::AbstractTable1d<double, double, double> max_allowed_bounds_;
  util::AbstractTable1d<double, double, double> decision_bounds_;
  std::vector<std::pair<double, double>> gate_ranges_;

  string model_name_ = "";

  double adc_frenet_s_ = 0.0;
  double adc_frenet_sd_ = 0.0;
  double adc_frenet_end_s_ = 0.0;
  double adc_frenet_l_ = 0.0;

  double adc_width_ = 0.0;
  double adc_wheel_width_ = 0.0;

  double adc_front_length_ = 0.0;
  double adc_rear_length_ = 0.0;

  PathPt::Direction direction_ = PathPt::Direction::FORWARD;

  int path_blocked_idx_ = -1;

  std::string debug_info_ = "";
  std::vector<PathData::DebugStatusType> debug_status_;

 protected:
  static constexpr double kSpeedEpsilon = 0.1;
  static constexpr double kMaxLateralAccelerations = 1.5;
  static constexpr double kPathResolution = 1.0;
  static constexpr double kPathLength = 100.0;
};
}  // namespace gpal::pnc::planning