/**
 * @file st_boundary_mapper.h
 * @brief 时空边界映射器
 * @details 本类负责将障碍物预测轨迹映射到ST图空间，生成时空约束边界。
 */
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "base/status.h"
// #include "config/tasks/speedmethod/scene_speed_parameter.pb.h"
// #include "config/vehicle_model/vehicle_config.pb.h"
#include "config_manager/config_manager.h"
#include "local_view/local_view.h"
#include "obstacle/obstacle.h"
#include "obstacle/st_boundary.h"
#include "path/path_data.h"
#include "st_graph_processor/st_obstacle_processor.h"
#include "config/speed_planner/speed_planner.pb.h"

namespace gpal::pnc::planning {

class STBoundaryMapper {
 public:
  STBoundaryMapper(std::vector<double> time_grid, double time_resolution, double time_horizon);
  /// @brief 析构函数
  virtual ~STBoundaryMapper() = default;

  void process(const LocalView& local_view, const ObstacleSet& obstacle_set, const PathGroup& path_group,
               const BehaviorState& behavior_state);
  void CaculateLocalPathLowerBoundary(const LocalView& local_view, const ObstacleSet& obstacle_set,
                                      const PathGroup& local_path_group, const BehaviorState& behavior_state,
                                      std::vector<std::pair<STPoint, std::string>>& local_path_lower_boundary);
  std::pair<double, double> calcSLProjection(const math::Vec2d& obs_pt, const DiscretizedPath& path, const double& start_s,
                                             const double& end_s);
 private:
  bool primaryObstacleFilter(const shared_ptr<SpeedPlannerObstacle>& obstacle, const DiscretizedPath& path);

  bool secondObstacleFilter(const shared_ptr<SpeedPlannerObstacle>& obstacle, const DiscretizedPath& path,
                            const vector<PathPt>& path_segmentation, const double& vehicle_speed, size_t& t_index_min, size_t& t_index_max,
                            std::vector<BoxProjectInfo>* box_project_info);

  bool isEgoAwayFromDiscretizedPath(const LocalView& local_view, const DiscretizedPath& discretized_path);

  bool isValidObstacleInParking(const BehaviorState& behavior_state, std::shared_ptr<SpeedPlannerObstacle> obstacle,
                                const std::vector<BoxProjectInfo>& box_project_info);

  // ===== 时间域参数 ===== //
  /// @brief 时间网格划分（单位：秒）
  std::vector<double> time_grid_;
  
  /// @brief 时间分辨率（默认0.1秒）
  double time_resolution_{0.1};
  
  /// @brief 规划时间范围（默认8秒）
  double time_horizon_{8.0};

  // ===== 配置参数 ===== //
  /// @brief 车辆基础配置参数
  VehicleConfig vehicle_config_;
  
  /// @brief 速度规划器全局配置
  SpeedPlannerConfig speed_planner_config_;

  // ===== 优化求解相关 ===== //
  /// @brief 二次规划求解器（用于SL投影计算）
  std::shared_ptr<QuadraticProgrammingSolver> sl_optimizer_{nullptr};
  
  /// @brief 二次规划权重矩阵（1x1矩阵）
  Eigen::Matrix<double, 1, 1> Q_sl_ = Eigen::Matrix<double, 1, 1>::Zero(1, 1);
  
  /// @brief 二次规划线性项（1维向量）
  Eigen::VectorXd c_sl_ = Eigen::VectorXd::Zero(1);
  
  /// @brief 不等式约束矩阵（2x1矩阵）
  Eigen::Matrix<double, 2, 1> G_sl_;
  
  /// @brief 不等式约束边界（2x1矩阵）
  Eigen::Matrix<double, 2, 1> h_sl_;
  
  /// @brief 优化变量初始值（1维零向量）
  Eigen::VectorXd x_init_ = Eigen::VectorXd::Zero(1);

  // common

  bool checkAABoxOverlap(const shared_ptr<SpeedPlannerObstacle>& obstacle,
                         const std::vector<proto::TrajectoryPoint>& obs_trajectory, const size_t& obs_start_idx,
                         const size_t& obs_end_idx, const DiscretizedPath& path, const size_t& adc_start_idx,
                         const size_t& adc_end_idx, const double& buffer);
};

}  // namespace gpal::pnc::planning
