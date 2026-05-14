/**
 * @file speed_limit.h
 * @brief 多源速度限制综合处理器
 * @details 本类负责整合道路环境中的各类速度限制条件，生成最终速度规划边界。
 */
#pragma once

#include "config/speed_planner/speed_planner.pb.h"
#include "config/speed_planner/speed_preprocessor.pb.h"
#include "config_manager/config_manager.h"
#include "local_view/local_view.h"
#include "obstacle/obstacle.h"
#include "path/discretized_path.h"
#include "reference_line_info/reference_line_info.h"
#include "speed_common/speed_common.h"

namespace gpal::pnc::planning {

class SpeedLimitProcessor {
 public:
  /// @brief 默认构造函数
  SpeedLimitProcessor() {};
  /// @brief 析构函数
  ~SpeedLimitProcessor() = default;

  void getSpeedLimit(const LocalView& local_view, const DecisionResult& decision_result,
                     const ReferenceLineInfo* reference_line_info, const DiscretizedPath& path, const int& turn_state,
                     const BehaviorState& behavior_state,const SpeedState& init_speed_state, SpeedLimitResult* speed_limit_result);

 private:
  void init();
  // speed limit f(s) = v

  void getMapRelatedSpeedlimit(const ReferenceLineInfo* reference_line_info, const DiscretizedPath& path);
  void getTurnSpeedLimit(const LocalView& local_view, const ReferenceLineInfo* reference_line_info,
                         const DiscretizedPath& path, const int& turn_state);
  void getDecisionSpeedLimit(const LocalView& local_view, const ReferenceLineInfo* reference_line_info,
                             const DecisionResult& decision_result);
  void getCurveSpeedLimit(const LocalView& local_view, const DecisionResult& decision_result,
                          const DiscretizedPath& path);

  // speed limit f(s) = const_v
  void getConstSpeedLimit(const LocalView& local_view, const int& turn_state, const BehaviorState& behavior_state);
  pair<string, double> getDrivingSpeedLimit(const LocalView& local_view);
  pair<string, double> getAccDrivingSpeedLimit(const LocalView& local_view);
  pair<string, double> getParkingSpeedLimit(const LocalView& local_view, const BehaviorState& behavior_state);
  pair<string, double> getTurnScenarioSpeedLimit(const LocalView& local_view, const int& turn_state);  // tmp

  void updateSpeedLimit(double speed_limit, double speed_limit_dis, string speed_limit_type);
  //
  void calculatePathVT(const LocalView& local_view, const BehaviorState& behavior_state, const ReferenceLineInfo* reference_line_info);

  //acc speed limit
  void caculateAccSpeedLimit(const LocalView& local_view);
  void steeringAngleAndYawRateFiliter(const double& steering_angle, const double& yaw_rate,
                                                         double& fliter_steering_angle, double& fliter_yaw_rate,
                                                         double& steering_angle_change_rate,
                                                         double& yaw_rate_change_rate);
  double movingAverageFilter(const std::vector<double>& data);
  double ewmaFilter(double newValue, double oldValue, double alpha);
  double calculateKappaBasedSteeringAngle(double steering_angle);
  double calcaulateKappaBasedYawRate(double yaw_rate, double ego_speed);

 private:
  /// @brief 配置管理器指针
  ConfigManager* config_manager_ = nullptr;
  
  /// @brief 车辆物理参数（轴距/轮距等）
  VehicleParam vehicle_param_;
  
  /// @brief 速度规划器全局配置参数
  SpeedPlannerConfig speed_planner_config_;
  
  /// @brief 速度预处理专用配置参数
  SpeedPreProcessorConfig speed_preprocessor_config_;
  
  /// @brief 项目名称标识（用于多场景配置区分）
  std::string project_name_ = "";

  /// @brief 初始速度规划状态
  SpeedState init_speed_state_;

  // ===== 速度限制计算中间状态 ===== //
  /// @brief 空间速度限制集合（按路径s分布）
  vector<SpatialSpeedLimit> speed_limit_;
  
  /// @brief 基于曲率计算的速度曲线（v(s)曲线）
  vector<proto::TrajectoryPoint> curvature_speed_path_;
  
  /// @brief 路径速度-时间曲线（v(t)曲线）
  vector<proto::TrajectoryPoint> path_v_t_;
  
  /// @brief 转向角曲线限速激活标志位
  bool last_steer_curve_acc_limit_{false};

  // ===== 实时状态记录 ===== //
  /// @brief 当前最大允许速度（用于约束规划器）
  double max_speed_limit_ = 0;
  
  /// @brief 方向盘转角历史数据（用于滤波处理）
  std::vector<double> history_steering_angle_;
  
  /// @brief 横摆角速度历史数据（用于滤波处理） 
  std::vector<double> history_yaw_rate_;
};

}  // namespace gpal::pnc::planning