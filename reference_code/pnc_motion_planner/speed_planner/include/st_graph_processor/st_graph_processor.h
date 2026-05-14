/**
 * @file st_graph_processor.h
 * @brief ST图处理器
 * @details 本类负责在ST坐标空间进行动态规划约束生成，及多场景决策结果处理
 */
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "base/status.h"
// #include "config/tasks/speedmethod/scene_speed_parameter.pb.h"
// #include "config/vehicle_model/vehicle_config.pb.h"
#include "config_manager/config_manager.h"
#include "obstacle/obstacle.h"
#include "obstacle/st_boundary.h"
#include "st_graph_processor/st_obstacle_processor.h"
#include "st_graph_processor/st_boundary_mapper.h"
#include "local_view/local_view.h"
#include "path/path_data.h"
#include "decision_data/decision_result.h"

#include "speed/st_graph.h"

namespace gpal::pnc::planning {

class STGraphProcessor {
 public:
  STGraphProcessor(std::vector<double> time_grid, double time_resolution, double time_horizon);
  /// @brief 析构函数
  virtual ~STGraphProcessor() = default;

  void process(const LocalView& local_view, const DecisionResult& decision_result,
               const LateralPath& lateral_path_group, const BehaviorState& behavior_state, const int64_t& time_stamp,
               ObstacleSet& obstacle_map, std::shared_ptr<SpeedResult> speed_result);
  void processCipvStBoundary(const LocalView& local_view, const DecisionResult& decision_result,
                               const LateralPath& lateral_path_group, const int64_t& time_stamp, ObstacleSet& obstacle_map,
                               std::shared_ptr<SpeedResult> speed_result);

 private:
  // follow obstacle
  void FollowObstacleProcess(ObstacleSet& obstacle_map);
  void CalcFollowObstacleSpeedAccelDistChangeRate(const std::shared_ptr<SpeedPlannerObstacle>& obs, double distance,
                                                  double& filtered_speed, double& filtered_accel, double& acc_gain,
                                                  double& distance_change_rate);
  void ObstacleSpeedAndAccelPostProcess(double last_frame_speed, double last_frame_accel, double dt, double& speed,
                                        double& accel);
  void GenerateFollowObstacleSTBoundary(const std::shared_ptr<SpeedPlannerObstacle>& obs, double init_s_lower, double speed,
                                        double accel) const;
  // overtake obstacle 
  void laneChangeOvertakeObstacleProcess(ObstacleSet& obstacle_map, string lc_overtake_id);
  void GenerateLaneChangeOvertakeObstacleSTBoundary(const std::shared_ptr<SpeedPlannerObstacle>& obstacle) ;

  //opposite obstacle 
  void oppositeObstacleProcess(const LocalView& local_view, const ObstacleSet& obstacle_set);
  double calculateConfidentTime(std::shared_ptr<SpeedPlannerObstacle> obstacle, double ego_speed);
  void updateOppositeObstacleBoundary(std::shared_ptr<SpeedPlannerObstacle> obstacle, double ego_speed,
                                                      double confident_time); 
  // obstacle tag
  void updateDecisionObstacleTag(const LocalView& local_view, std::shared_ptr<SpeedPlannerObstacle> obstacle,
                                 std::shared_ptr<SpeedResult> speed_result);
  void updateFollowObstacleTag(std::shared_ptr<SpeedPlannerObstacle> obstacle, std::shared_ptr<SpeedResult> speed_result);
  void updateYieldObstacleTag(std::shared_ptr<SpeedPlannerObstacle> obstacle, std::shared_ptr<SpeedResult> speed_result);
  void updateOvertakeObstacleTag(const LocalView& local_view, std::shared_ptr<SpeedPlannerObstacle> obstacle,
                                 std::shared_ptr<SpeedResult> speed_result);
  void updateRiskyObstacleTag(const LocalView& local_view, std::shared_ptr<SpeedPlannerObstacle> obstacle,
                              std::shared_ptr<SpeedResult> speed_result);
  bool checkOvertakingSafety(const LocalView& local_view, std::shared_ptr<SpeedPlannerObstacle> obstacle,
                             std::shared_ptr<SpeedResult> speed_result, STBoundary::BoundaryType boundary_type);
  bool checkOppositeObstacleRisk(const LocalView& local_view, std::shared_ptr<SpeedPlannerObstacle> obstacle);

  void crossObstacleProcess(const LocalView& local_view, const ObstacleSet& obstacle_set);

  void mergeObstacleProcess(const LocalView& local_view, const ObstacleSet& obstacle_set);

  // st drive boundary
  void calculateDrivableBoundaries(StGraph* st_graph);
  void GetBoundsAtT(const STBoundary* boundary, const double& t_instance, const double& confident_time,
                    STDrivableBoundary& st_drivable_boundary);

  // speed wall
  void updataSpeedWallBoundInStGraph(const LocalView& local_view, const std::vector<SpeedWall> speed_walls,
                                     StGraph* st_graph);

 // ===== 时间域参数 ===== //
  /// @brief 时间网格划分（单位：秒）
  std::vector<double> time_grid_;
  
  /// @brief 时间分辨率（默认0.1秒）
  double time_resolution_{0.1};
  
  /// @brief 规划时间范围（默认8秒）
  double time_horizon_{8.0};

  // ===== 配置管理 ===== //
  /// @brief 配置管理器指针（延迟初始化）
  ConfigManager* config_manager_ = nullptr;
  
  /// @brief 车辆物理参数（轴距/轮距等）
  VehicleParam vehicle_param_;
  
  /// @brief 速度规划器全局配置参数
  SpeedPlannerConfig speed_planner_config_;

  // ===== 处理中间数据 ===== //
  /// @brief 横向路径处理结果集
  PathGroup path_group_;
  
  /// @brief 障碍物集合（已过滤和分类）
  ObstacleSet obstacle_set_;

  // ===== ST边界处理 ===== //
  /// @brief ST边界映射处理器（负责障碍物投影）
  std::shared_ptr<STBoundaryMapper> st_boundary_mapper_;

  // ===== 历史帧数据 ===== //
  /// @brief 当前帧时间戳（微秒单位）
  int64_t time_stamp_;
  
  /// @brief 历史帧时间戳集合（用于速度滤波）
  std::vector<double> history_frame_time_stamps_;
  
  /// @brief 历史跟随障碍物速度记录（按ID索引）
  std::vector<std::unordered_map<std::string, double>> history_frame_follow_obs_speeds_;
  
  /// @brief 历史跟随障碍物加速度记录（按ID索引）
  std::vector<std::unordered_map<std::string, double>> history_frame_follow_obs_accels_;
  
  /// @brief 历史跟随障碍物距离记录（按ID索引）
  std::vector<std::unordered_map<std::string, double>> history_frame_follow_obs_distances_;
  
  /// @brief 历史碰撞保持计数（按ID索引）
  std::unordered_map<std::string, int> history_frame_keep_collision_;
  
  /// @brief 最大历史帧保留数（默认5帧）
  size_t num_history_frames_ = 5UL;
};

}  // namespace gpal::pnc::planning
