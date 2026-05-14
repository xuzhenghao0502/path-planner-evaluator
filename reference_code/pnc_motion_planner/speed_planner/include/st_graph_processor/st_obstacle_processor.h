/**
 * @file st_obstacle_processor.h
 * @brief 障碍物ST投影处理器
 * @details 本类负责障碍物时空边界的精确计算与建模
 */
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "base/status.h"
#include "config/speed_planner/st_graph_processor.pb.h"
#include "config_manager/config_manager.h"
#include "math/interval_data.h"
#include "math/math_utils.h"
#include "obstacle/obstacle.h"
#include "obstacle/st_boundary.h"
#include "path/discretized_path.h"
#include "qp_solver.h"
#include "speed_common/speed_common.h"
#include "util/base_struct.h"

namespace gpal::pnc::planning {
class STObstacleProcessor {
 public:
  STObstacleProcessor(std::vector<double> time_grid, double time_resolution, double time_horizon);
  /// @brief 析构函数
  virtual ~STObstacleProcessor() = default;

  void ComputeSTBoundary(const std::shared_ptr<SpeedPlannerObstacle>& obstacle, const DiscretizedPath& discretized_path,
                         const BehaviorState& behavior_state);
  void ComputeSTBoundary(const std::shared_ptr<SpeedPlannerObstacle>& obstacle, const DiscretizedPath& discretized_path,
                         const BehaviorState& behavior_state, const std::vector<BoxProjectInfo>& box_project_info,
                         const size_t& t_index_min, const size_t& t_index_max);
  void ComputeSTBoundaryLocalPath(
      const std::shared_ptr<SpeedPlannerObstacle>& obstacle, const DiscretizedPath& discretized_path,
      const BehaviorState& behavior_state, const std::vector<BoxProjectInfo>& box_project_info,
      const size_t& t_index_min, const size_t& t_index_max, STBoundary& boundary);

 private:
  // use ocp_risk st compute
  math::IntervalData<SpeedPlannerObstacle::RiskFieldInfo> calcRiskFieldInfos(
      const std::shared_ptr<SpeedPlannerObstacle>& obstacle, const DiscretizedPath& path,
      const BehaviorState& behavior_state, const std::vector<BoxProjectInfo>& box_project_info,
      const size_t& t_index_min, const size_t& t_index_max, std::vector<STPoint>* lower_points,
      std::vector<STPoint>* upper_points, std::vector<double>* lateral_signed_distances);

  double getBoxesRelation(const PathPt& check_pt, const std::vector<BoxInfo>& obs_boxes, double* min_box_distance,
                          double* nearest_s, math::Vec2d* nearest_pt, double* box_dis);
  double getAlphaBetweenBoxes(const std::vector<BoxInfo>& ego_boxes, const std::vector<BoxInfo>& obs_boxes,
                              math::Vec2d* nearest_pt, double* box_dis);
  double calcAlphaBetweenBox(const BoxInfo& obs_box, const BoxInfo& ego_box, const bool& is_obs_expanded,
                             math::Vec2d* nearest_pt);
  double signedDistanceToBoxInfo(const math::Vec2d& pt, const BoxInfo& box_info);
  double getSignedBoxDistance(const std::vector<BoxInfo>& obs_boxes, const DiscretizedPath& path,
                              const double unsigned_distance, const double s);
  void calcTemperalSpatialSpeedLimit(SpeedPlannerObstacle::RiskFieldInfo* risk_field_info,
                                     const std::string& obstacle_id, const Decision::ObjectType& type);
  double getSpeedLimit(const double box_dis, const double longitudinal_speed, const Decision::ObjectType& type);
  void getBoundarySTPoints(math::IntervalData<SpeedPlannerObstacle::RiskFieldInfo>& risk_infos,
                           const std::vector<BoxProjectInfo>& box_project_info, double buffer, std::vector<STPoint>* lower_points,
                           std::vector<STPoint>* upper_points, std::vector<double>* lateral_signed_distances);

  // SpeedPlannerObstacle::LateralRiskInfo getLateralRiskInfo(const BoxProjectInfo& project_info,
  //                                              const proto::TrajectoryPoint& traj_pt, const math::Box2d& obs_box,
  //                                              const double& obs_speed, const double& box_lateral_distance);
  // void CalcLateralRiskSpeedLimit(SpeedPlannerObstacle::LateralRiskInfo* lateral_risk);

  // common
  std::vector<BoxInfo> getEgoBoxes(const PathPt& check_pt, double half_s_buffer = 0.0,
                                   double half_l_buffer = 0.0) const;
  std::vector<BoxInfo> getObsBoxes(const SpeedPlannerObstacle& obs, const double& t, double half_s_buffer = 0.0,
                                   double half_l_buffer = 0.0) const;

  PathPt GetADCCenterPointFromPathPoint(const PathPt& curr_point) const;

  bool checkIfFollowObstacle(const std::shared_ptr<SpeedPlannerObstacle>& obstacle,
                             const std::vector<BoxProjectInfo>& box_project_info);

  // ===== 时间域参数 ===== //
  /// @brief 时间网格划分（单位：秒）
  std::vector<double> time_grid_;
  
  /// @brief 时间分辨率（默认0.1秒）
  double time_resolution_{0.1};
  
  /// @brief 规划时间范围（默认8秒）
  double time_horizon_{8.0};

  // ===== 路径数据 ===== //
  /// @brief 离散化路径数据（用于障碍物投影）
  DiscretizedPath path_;
  
  /// @brief 车辆基础配置参数
  VehicleConfig vehicle_config_;
  
  /// @brief ST图处理器专用配置参数
  StGraphProcessorConfig param_;


  // ===== 优化求解相关 ===== //
  /// @brief 车辆宽度（默认2米）
  double vehicle_width_{2.0};

  /// @brief 二次规划求解器（用于α计算）
  std::shared_ptr<QuadraticProgrammingSolver> alpha_optimizer_{nullptr};
  
  /// @brief 线性目标函数系数（3维向量）
  Eigen::Vector3d c_{0, 0, 1};
  
  /// @brief 二次规划权重矩阵（3x3矩阵）
  Eigen::Matrix<double, 3, 3> Q_ = Eigen::Matrix<double, 3, 3>::Zero(3, 3);
  
  /// @brief 不等式约束矩阵（9x3矩阵）
  Eigen::Matrix<double, 9, 3> G_;
  
  /// @brief 不等式约束边界（9x1向量） 
  Eigen::Matrix<double, 9, 1> h_;
};

}  // namespace gpal::pnc::planning
