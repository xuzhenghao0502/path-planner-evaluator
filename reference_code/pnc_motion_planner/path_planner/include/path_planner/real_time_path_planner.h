/**
 * @file real_time_path_planner.h
 * @brief 实时路径规划核心模块，包含路径边界生成、优化决策、碰撞检测等功能
 * @struct RealTimePathPlannerConfig 路径规划配置参数
 * @var vehicle_config_ 车辆配置参数
 * @var path_bound_points_config_ 路径边界点配置
 * @uml{flow}
 * start
 * :初始化配置参数;
 * :创建路径边界解析器;
 * :初始化各类型滤波器;
 * :加载障碍物距离映射表;
 * end
 * @enduml
 */

#pragma once

#include "base/log.h"
#include "config/path_bound_parser/path_bound_points_config.pb.h"
#include "config/path_planner/real_time_path_planner_config.pb.h"
#include "path/path_boundary.h"
#include "path/path_decision.h"
#include "path_bound_parser/path_bound_filter.h"
#include "path_bound_parser/path_bound_parser.h"
#include "path_planner_base.h"
#include "ocp_path_optimizer.h"
#include "decision_data/common_utils.h"
#include "decision_data/decision_result.h"

namespace gpal::pnc::planning {

/**
 * @brief 基于横向兜底场景的处理.
 */
enum FallbackStatus {
  INVALID = -1,
  PROTECTION = 0,    ///< PROTECTION 兜底轨迹：1) 采用BicycleTrajectoryTracker ocp模型，模拟横向控制的行驶轨迹，但规划起点在自车当前位置，不利于横向控制跟踪.2) 若1）失败导致横向轨迹为空时，返回fall back结果，即横向规划的初解(第一帧为参考线，其他为上帧横向轨迹)，此方法优先级最低
  UNCONSTRAINED = 1, ///< 无约束求解轨迹：模型及规划起点与有约束轨迹一致，但无横向硬边界约束，仅考虑软边界约束，利于横向控制跟踪；优先级最高.
  HISTORY = 2        ///< 历史轨迹：refine path
}; 

/**
 * @brief 实时路径规划核心模块，包含路径边界生成、优化决策、碰撞检测等功能
 */
class RealTimePathPlanner : public PathPlannerBase {
 public:
  /**
   * @brief 默认构造函数
   */
  RealTimePathPlanner() = default;
  /**
   * @brief 默认析构函数
   */
  ~RealTimePathPlanner() = default;

  bool init() override;
  bool reset() override;
  PathData::StatusType runOnce(const ReferenceLineInfo& target_reference_line_info,
                               const ReferenceLineInfo& current_reference_line_info, 
                               const LocalView& local_view,
                               const DecisionResult& decision_result, const SpeedData& prev_speed_data,
                               const int& stage_state,
                               const int64_t& time_stamp,
                               PathBoundary* path_boundary,
                               PathData* const path_data) override;

 protected:
  void preProcess(const ReferenceLineInfo& target_reference_line_info, const ReferenceLineInfo& current_reference_line_info,
                  const Localization& localization, const Chassis& chassis, const DecisionResult& decision_result, const SpeedData& prev_speed_data);
  bool hasChangeReference(const std::pair<double, double>& s_range, const ReferenceLineInfo& ref_info);
  void resetFilter();
  void updateIgnoreRanges(const std::vector<IgnoreRangeInfo>& ignore_ranges);
  bool isInSpecificScene(const std::string& scene_tag, const float& s);

  Status generateFibonacciCVTSRange(const double& t0, const double& t1, const double& dt);
  std::vector<double> getFibonacciVec(const double& dt, const double& tmax, const size_t n_max = 100);

  Status generateRegularPathBound(const Console& console, const Freespace& freespace,
                                  const ReferenceLineInfo& reference_line_info,
                                  const LateralBoundDecision& decision_boundary,
                                  const LongitudinalBoundDecision& longitudinal_decision_boundary,
                                  PathBoundary* boundary);
  Status generateLaneChangePathBound(const Console& console, const Freespace& freespace,
                                     const ReferenceLineInfo& reference_line_info,
                                     const LateralBoundDecision& decision_boundary,
                                     const LongitudinalBoundDecision& longitudinal_decision_boundary,
                                     PathBoundary* boundary);
  Status generateForceBackPathBound(const ReferenceLineInfo& reference_line_info,
                                    const LateralBoundDecision& decision_boundary, PathBoundary* boundary);
  bool initPathBoundary(const ReferenceLineInfo& reference_line_info, const LateralBoundDecision& decision_boundary, 
                        const LongitudinalBoundDecision& longitudinal_decision_boundary,
                        PathBoundary* path_bound);
  bool ignoreBoundary(const LongitudinalBoundDecision& longitudinal_bound_decision, double& ignore_start_s,
                      double& ignore_end_s);
  bool updatePathSegmentInfo(const double& start_s, const double& end_s, PathBoundary* path_bound);
  bool calcBoundaryFromFreespace(const Freespace& freespace, PathBoundary* boundary);
  bool calcBoundaryFromStaticObstacles(PathBoundary* boundary);
  std::pair<double, double> getObsLaterSafeBuffer(const std::shared_ptr<Decision::DecisionObject> obs, const bool& use_obs_type,
                                                  const bool& use_obs_speed);
  std::pair<double, double> getObsLaterSafeBufferFromType(const Decision::ObjectType& obs_type,
                                                          const double& default_barrier_lateral_buffer,
                                                          const double& default_soft_lateral_buffer);
  std::pair<double, double> getObsLaterSafeBufferFromSpeed(const std::shared_ptr<Decision::DecisionObject> obs,
                                                           const double& default_barrier_lateral_buffer,
                                                           const double& default_soft_lateral_buffer);
  bool updateBoundary(const double& lat_buffer, const double& right_bound, const double& left_bound,
                      std::tuple<double, double, double>& boundary, const bool return_if_valid);
  bool updateBoundaryFromObstacle(const double& right_bound, const double& left_bound, const Decision::DecisionObject* const ob_left,
                                  const Decision::DecisionObject* const ob_right, std::tuple<double, double, double>* boundary,
                                  std::optional<PathBoundary::ObstacleMap::iterator> key_ob);
  void trimPathBounds(const int& path_blocked_idx,
                      std::vector<std::tuple<double, double, double>>* const path_boundaries);
  bool calcBoundaryFromDynamicObstacles(const ReferenceLineInfo& reference_line_info, PathBoundary* boundary);
  std::pair<double, double> getDynamicObsLaterSafeBuffer(const std::shared_ptr<Decision::DecisionObject> obs,
                                                         const bool use_obs_type,
                                                         const bool use_obs_speed);
  bool refineBoundaryUnderSpecialScene(std::vector<std::tuple<double, double, double>>* const barrier_boundary,
                                       std::vector<std::tuple<double, double, double>>* const soft_boundary);
  void getSpecialSceneSrange(std::vector<std::tuple<double, double, double>>* const barrier_boundary,
                             std::vector<std::tuple<double, double, double>>* const soft_boundary);
  void bindBoundary(PathBoundary* boundary, string mode = "regular");

  void getCurveScene();
  void updateTagRangesDebugInfo();


  void calcLaneKeepStartS(const DecisionResult& decision_result, PathData* const path_data);

  Status processPathOptimizer(const DecisionResult& decision_result, 
                              const SpeedData& prev_speed_data, const int64_t& time_stamp, 
                              PathData* const path_data);
  bool solvedPathValidityCheck(const DiscretizedPath& path, const PathBoundary& boundary,
                               const ReferenceLine& ref_line);
  void generateUnconstrainedPath(PathData* const path_data);
  void generateProtectPath(const SpeedData& prev_speed_data, const int64_t& time_stamp, PathData* const path_data);

  bool isPedestrian(const Decision::DecisionObject& obs);

  void CollisionPostProcess(const Freespace& freespace, const std::shared_ptr<Localization> loc, PathData* const path_data);

  void visualization(PathData* const path_data);

  void generateRealtimeTrajBoundaryInfo(std::vector<PathBoundary::PathBoundaryUnitInfo>* path_boundary_info, PathData* path_data);

  PathBoundary::BoundaryUnitTypeInfo ConvertBoundaryType(BoundaryPointTypeInfo decision_type);

  void updateRefOffsetInfo(const DecisionResult& decision_result, bool condition);

 protected:
  RealTimePathPlannerConfig real_time_path_planner_config_; ///< 实时路径规划器配置
  PathBoundPointsConfig path_bound_points_config_;  ///< 路径边界配置
  
  std::unique_ptr<ReferenceLineInfo> target_ref_line_info_; ///< 目标参考线信息
  std::unique_ptr<ReferenceLineInfo> current_ref_line_info_;  ///< 当前参考线信息
  std::string pre_ref_id_ = "";  ///< 历史参考ID
  std::vector<std::pair<std::string, std::vector<std::string>>> pre_lane_id_; ///< 历史车道ID
  ReferenceLine::SmoothType pre_smooth_type_ = ReferenceLine::SmoothType::RAW; ///< 历史参考线的平滑类型
  ReferenceLine::LineType pre_line_type_ = ReferenceLine::LineType::UNKNOWN; ///< 历史参考线类型
  double prev_global_start_s_ = 0.0; ///< 历史参考线全局系下起点对应的s值
  math::Vec3d prev_start_pt_;  ///< 历史起始点
  bool change_ref_ = false; ///< 是否切换参考线
  bool pre_change_ref_ = false; ///< 上帧是否切换参考线
  TrajectoryPt planning_start_point_; ///< 规划起始点
  Eigen::Matrix4d tf_map_2_ego_; ///< 地图系到自车系的变换矩阵
  FsmState behavior_ = FsmState::KEEP;  ///< 行为状态
  std::vector<std::shared_ptr<Decision::DecisionObject>> decision_static_obstacles_;  ///< 决策静态障碍物
  std::vector<std::shared_ptr<Decision::DecisionObject>> decision_dynamic_obstacles_; ///< 决策动态障碍物
  int stage_state_ = 0;  ///< 当前stage
  
  std::unique_ptr<PathBoundParser> bound_parser_; ///< 路径边界解析器
  std::unique_ptr<PathBoundFilter> prior_physical_barrier_bound_filter_; ///< 物理不可通行障碍物硬边界过滤器
  std::unique_ptr<PathBoundFilter> prior_physical_soft_bound_filter_;  ///< 物理不可通行障碍物软边界过滤器
  std::unique_ptr<PathBoundFilter> freespace_barrier_bound_filter_; ///< fs硬边界过滤器
  std::unique_ptr<PathBoundFilter> freespace_soft_bound_filter_; ///< fs软边界过滤器
  std::unique_ptr<PathBoundFilter> static_obstacle_barrier_bound_filter_; ///< 静态障碍物硬边界过滤器
  std::unique_ptr<PathBoundFilter> static_obstacle_soft_bound_filter_; ///< 静态障碍物软边界过滤器
  std::unique_ptr<PathBoundFilter> dynamic_obstacle_soft_bound_filter_; ///< 动态障碍物硬边界过滤器
  std::unordered_map<int, PathBoundPointsConfig::ObstacleInfo> obs_type_lateral_distance_map_; ///< 基于障碍物类型的横向安全距离map表
  std::vector<std::tuple<double, PathBoundary::BoundaryUnitTypeInfo, PathBoundary::BoundaryUnitTypeInfo>> realtime_traj_boundary_type_info_;  ///< 实时轨迹边界类型信息
  int path_blocked_idx_ = -1; ///< 路径block位置索引
  size_t path_segment_size_ = 0; ///< 路径段数
  double path_resolution_ = 0.0; ///< 路径分辨率

  double adc_width_ = 0.0; ///< 自车宽度
  double adc_wheel_width_ = 0.0; ///< 自车轮距
  double adc_front_length_ = 0.0; ///< 自车前部长度
  double adc_rear_length_ = 0.0; ///< 自车后部长度

  double adc_driven_frenet_s_ = 0.0; ///< 自车行驶的Frenet坐标系下的s值
  double adc_driven_frenet_l_ = 0.0; ///< 自车行驶的Frenet坐标系下的l值
  double adc_frenet_s_ = 0.0; ///< 自车的Frenet坐标系下的s值
  double adc_frenet_sd_ = 0.0; ///< 自车的Frenet坐标系下的s速度
  double adc_frenet_end_s_ = 0.0; ///< 自车的Frenet坐标系下的结束s值
  double adc_frenet_l_ = 0.0; ///< 自车的Frenet坐标系下的l值
  double curr_right_bound_adc_ = -0.75; ///< 基于当前自车位置的右侧边界
  double curr_left_bound_adc_ = 0.75;  ///< 基于当前自车位置的左侧边界

  bool l_offset_behavior_valid_  = false;  ///< 决策横向offset信息有效性标志

  double ignore_start_s = -1.0;
  double ignore_end_s = -1.0;
  bool ignore_boundary = false;
  
  std::vector<IgnoreRangeInfo> ignore_ranges_; ///< 忽略范围
  std::vector<std::pair<double, double>> gate_ranges_;  ///< 闸机范围
  std::vector<std::tuple<string, float, float>> special_scene_s_range_sets_;  ///< 特殊场景s值范围集合
  std::vector<std::pair<double, double>> ref_offsets_info_;  ///< 基于参考线的横向偏移信息

  OcpPathOptimizer optimizer_; ///< OCP路径优化求解器
  OcpPathOptimizer::AsyncStatus pre_async_status_ = OcpPathOptimizer::AsyncStatus::ASYNC_SOLVED; ///< 上帧异步状态
  Status last_optimizer_status_; ///< 上帧OCP路径优化求解器状态
  std::string debug_info_ = ""; ///< 调试信息
  std::unordered_set<PathData::DebugStatusType> debug_status_; ///< 调试状态集合

 protected:
  static constexpr double kSpeedEpsilon = 0.1; ///< 速度容差
  static constexpr double kPathResolution = 1.0;  ///< 默认路径分辨率
  static constexpr double kPathResolutionTol = 0.1; ///< 路径分辨率容差
};

}
