/**
 * @file speed_planner_base.h
 * @brief 基于速度优化的最优控制速度规划器
 * @details 本类实现基于最优控制理论的实时速度规划解决方案, 采用速度优化求解器求解速度优化问题。
 */
#pragma once

#include "local_view/function_state.h"

#include "config/speed_planner/speed_planner.pb.h"
#include "config/speed_planner/st_graph_processor.pb.h"
#include "config_manager/config_manager.h"
#include "speed_optimizer/speed_model_param.h"
#include "speed_optimizer/speed_ocp_qp_optimizer.h"
#include "speed_preprocessor/speed_preprocessor.h"
#include "st_graph_processor/st_graph_processor.h"

namespace gpal::pnc::planning {

class SpeedPlannerBase {
 public:
  /// @brief 构造函数
  SpeedPlannerBase() = default;
  /// @brief 析构函数
  ~SpeedPlannerBase() = default;
  bool init();
  virtual gpal::pnc::planning::Status runOnce(const LocalView& local_view, const StageState& stage_state,
                                              const shared_ptr<ReferenceLineInfo> target_reference_line_info,
                                              const shared_ptr<DecisionResult> decision_result,
                                              const shared_ptr<PathData> path_data, int64_t time_stamp,
                                              shared_ptr<SpeedResult> speed_result) {
    return gpal::pnc::planning::Status::OK();
  };

 protected:
  void calcBehaviorState(const StageState& stage_state, const DiscretizedPath& discretized_path,
                         BehaviorState* behavior_state);
  void calcInitState(const LocalView& local_view);

  void setOcpSpeedData(const std::pair<std::vector<OcpVariable>, std::vector<OcpVariable>>& x_u,
                       SpeedData* mutable_speed_data);

  void resetSpeedData(const double& desire_acc, const double& desire_speed, SpeedData* mutable_speed_data);

  void getTrajectory(const std::shared_ptr<Localization>& localization, const DiscretizedPath& discretized_path,
                     const SpeedData& speed_data, proto::Trajectory* trajectory_result);
  void resetTrajectory(const std::shared_ptr<Localization>& localization, proto::Trajectory* trajectory_result);
  void stopReasonCheck(const LocalView& local_view, const InvasionObstacle& nearest_invasion_obstacle,
                       SpeedData* speed_data, StopReason* stop_reason);

  void resetTrajectoryResultBasedAbortTask(const LocalView& local_view, const DiscretizedPath& discretized_path,
                                           const StageState& stage_state, shared_ptr<SpeedResult> speed_result);
  ///< 速度规划器初始化标志位
  bool init_ = false;

  ///< 配置管理模块（单例模式）
  ConfigManager* config_manager_ = nullptr;

  VehicleParam vehicle_param_;

  ///< 速度规划器配置参数容器
  SpeedPlannerConfig speed_planner_config_;

  ///< 速度预处理模块（轨迹校验/障碍物预处理/限速计算）
  std::shared_ptr<SpeedPreprocessor> speed_preprocessor_ptr_;

  ///< ST图处理器（障碍物投影/安全走廊生成）
  std::shared_ptr<STGraphProcessor> st_graph_processor_ptr_;

  ///< 优化模型参数生成器（约束条件/权重系数）
  std::shared_ptr<SpeedModelParam> speed_model_param_ptr_;

  ///< 二次规划求解器（OCP问题数值求解）
  std::shared_ptr<SpeedOCPQPOptimizer> speed_ocp_qp_optimizer_;

  // const param
  // const std::vector<double> time_grid_ = {0.0, 0.1, 0.2, 0.4, 0.8, 1.2, 1.6, 2.0, 2.4, 2.8, 3.2, 3.6,
  //                                         4.0, 4.4, 4.8, 5.2, 5.6, 6.0, 6.4, 6.8, 7.2, 7.6, 8.0};
  ///< 时间网格序列（单位：秒，用于离散化规划问题）
  const std::vector<double> time_grid_ = {0.0, 0.1, 0.2, 0.4, 0.6, 1.0, 1.4, 1.8,
                                          2.2, 2.6, 3.0, 3.4, 3.8, 4.2, 4.6, 5.0};
  ///< 时间分辨率（单位：秒，控制规划精度）
  const double time_resolution_{0.1};

  ///< 规划时间范围（单位：秒，总前瞻时间）
  const double time_horizon_{5.0};

  // init data
  ///< 初始状态量（t=0时的运动状态）
  SpeedState x_0_;  // The initial state of our planning process, with t = 0

  ///< 首帧标志位（用于初始化逻辑判断）
  bool in_first_frame_ = true;

  // Historical variables
  /// 上一帧时间戳（单位：纳秒，用于递推计算）
  double last_time_stamp_;

  ///< 历史速度数据缓存（用于状态连续性保证）
  SpeedData last_speed_data_;  // The planning result of last frame

  ///< 上一帧车辆位置（用于位移递推校验）
  math::Vec2d last_pos_;
};

}  // namespace gpal::pnc::planning
