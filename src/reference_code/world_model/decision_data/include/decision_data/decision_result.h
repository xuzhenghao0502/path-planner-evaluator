/**
 * @file decision_result.h
 * @brief 定义决策结果相关的结构体和枚举，用于存储和获取决策输出
 * @author 智能驾驶决策规划团队
 * @date 2024-03-01
 * @version 1.0
 * @copyright Copyright (C) 2024 Shanghai Geometrical Perception and Learning Co., Ltd. All rights reserved.
 * @note 模块功能：
 * - 整合 OD 决策、边界决策、FSM 状态等信息，提供决策结果的接口
 */

#ifndef _WORLD_MODEL_LOCAL_VIEW_DECISION_RESULT_
#define _WORLD_MODEL_LOCAL_VIEW_DECISION_RESULT_

#include <cfloat>
#include <map>
#include <string>
#include <vector>

#include "decision_data/decision_define.h"
#include "ellipse_object_info/ellipse_object_info.h"
#include "gpal-interface/planning/trajectory.pb.h"
#include "proto/common/sl_boundary.pb.h"
#include "util/base_struct.h"
#include "trajectory_boundary/time_related_boundary.h"
#include "trajectory_boundary/velocity_related_boundary.h"
#include "trajectory_boundary/trajectory_boundary.h"

namespace gpal::pnc::planning {
// ===== 类型别名定义 ===== //
using LateralOdTag = gpal::pnc::planning::Decision::LateralOdTag;
using LongitudinalOdTag = gpal::pnc::planning::Decision::LongitudinalOdTag;
using RefTrajInfo = gpal::pnc::planning::Decision::RefTrajInfo;
using ObjectGameType = gpal::pnc::planning::Decision::ObjectGameType;
using TrafficLightDecision = gpal::pnc::planning::Decision::TrafficLightDecision;
using RefLineDecesionInfo = gpal::pnc::planning::Decision::RefLineDecesionInfo;

// ===== 边界决策相关参数 ===== //
/*============================== Boundary Decisions =======================================*/

/// @brief 边界点类型信息枚举，描述边界点的类型
enum class BoundaryPointTypeInfo {
  INVALID = 0,                /// @brief 未赋值
  DASHED_LANE_LINE = 1,       /// @brief 车道虚线
  SOLID_LANE_LINE = 2,        /// @brief 车道实线
  PHYSICALLY_IMPASSABLE = 3,  /// @brief 物理不可逾越
};

/// @brief 边界点结构体，记录边界点的位置、速度和类型等信息
struct BoundaryPoint {  // BoundaryPoint. s and l are based on target ref line
  /// @brief 用于表示较大值的常量，待进一步调整
  static constexpr double kLargeValue = 6.0;  // TODO
  /// @brief 边界点的 s 值
  double s = 0.0;
  /// @brief 边界点的速度值
  double v = 0.0;
  /// @brief 边界点的左 l 值
  double l_left = kLargeValue;
  /// @brief 边界点的右 l 值
  double l_right = -kLargeValue;
  /// @brief 边界点左侧的类型
  BoundaryPointTypeInfo left_type = BoundaryPointTypeInfo::INVALID;
  /// @brief 边界点右侧的类型
  BoundaryPointTypeInfo right_type = BoundaryPointTypeInfo::INVALID;
  /// @brief 边界点左侧端点的 xy 坐标
  std::pair<double, double> xy_left{0.0, 0.0};
  /// @brief 边界点右侧端点的 xy 坐标
  std::pair<double, double> xy_right{0.0, 0.0};
};

/// @brief 边界点列表类型别名，用于存储多个边界点信息
using BoundPointList = std::vector<BoundaryPoint>;

/// @brief 横向边界类型枚举，描述横向边界的类型
enum class LateralBoundaryType {
  INVALID = 0,   /// @brief 未赋值
  LAT_HARD = 1,  /// @brief 硬横向边界，通常表示物理上不可逾越的边界
  LAT_SOFT = 2,  /// @brief 软横向边界，一般是有一定限制但并非绝对不可逾越的边界
};

/// @brief 横向边界约束结构体，记录横向边界的类型和边界点信息
struct LateralBoundaryConstraint {
  /// @brief 横向边界类型，默认值为无效类型
  LateralBoundaryType type = LateralBoundaryType::INVALID;
  /// @brief 边界点列表
  BoundPointList points;
};

/// @brief 横向边界约束插值函数声明，根据 s 值对边界约束进行插值计算
/// @param[in] s 用于插值的 s 值
/// @param[in] points 边界点列表
/// @return 包含插值后的 l_left、l_right、left_type 和 right_type 的元组
std::tuple<double, double, BoundaryPointTypeInfo, BoundaryPointTypeInfo> LateralBoundConsInterpolate(
    double s, const BoundPointList& points);

/// @brief 墙类型枚举，描述纵向约束墙的类型
enum class WallType {
  LONG_UNKNOWN_WALL = 0,
  LONG_RSA_WALL = 1,           /// @brief 闸机
  LONG_TFL_WALL = 2,           /// @brief 红绿灯停止线
  LONG_DESTINATION_WALL = 3,   /// @brief 终点墙
  LONG_FORCE_LANE_CHANGE = 4,  /// @brief 强制换道终点
  LONG_JUNCTION_STOP = 5,      /// @brief 路口前停车等待
  LONG_CLEAR_ZONE = 6,         /// @brief 禁停区
  LONG_DECC_WALL = 7,          /// @brief 减速墙
  LONG_VIRTUAL_STOP_WALL = 8,  /// @brief 虚拟停车墙  此属性下s表示剩余距离
};

/// @brief 墙约束结构体，记录纵向约束墙的类型、位置和速度信息
struct WallConstraint {
  /**
   * @brief 默认构造函数
   * @details 初始化一个 `WallConstraint` 对象，各成员变量使用默认值。墙类型默认为 `LONG_UNKNOWN_WALL`，墙的位置 s
   * 值和速度值默认为 0.0。
   */
  WallConstraint() = default;
  /// @brief 构造函数，使用墙类型、位置和速度初始化墙约束
  /// @param[in] type 墙类型
  /// @param[in] s 墙的位置 s 值
  /// @param[in] v 墙的速度值
  WallConstraint(WallType type, double s, double v) : type(type), s(s), v(v) {}
  /// @brief 墙类型，默认值为未知墙
  WallType type = WallType::LONG_UNKNOWN_WALL;
  /// @brief 墙的位置 s 值
  double s = 0.0;
  /// @brief 墙的速度值
  double v = 0.0;
};

// BoundaryDecisions contains a vector of BoundaryConstraint.
/// @brief 横向边界决策类型别名，用于存储多个横向边界约束信息
using LateralBoundDecision = std::vector<LateralBoundaryConstraint>;
/// @brief 纵向边界决策类型别名，用于存储多个纵向墙约束信息
using LongitudinalBoundDecision = std::vector<WallConstraint>;

struct DecisionBoundaryInfo {
  string target_lane_id = "";  ///< 目标车道 ID
  // 边界解析的输出
  std::shared_ptr<TrajectoryBoundary> trajectory_boundary = nullptr;             ///< 路径相关边界信息
  std::shared_ptr<TimeRelatedBoundary> time_related_boundary = nullptr;          ///< 时间相关边界信息
  std::shared_ptr<VelocityRelatedBoundary> velocity_related_boundary = nullptr;  ///< 速度相关边界信息

  // void reset() {
  //   // 重置边界信息
  //   trajectory_boundary.reset();
  //   time_related_boundary.reset();
  //   velocity_related_boundary.reset();
  // }
};

// ===== FSM 状态相关参数 ===== //
/*============================== FSM State =======================================*/
/// @brief 有限状态机状态枚举，描述车辆的行驶状态
enum class FsmState {
  KEEP = 0,  /// @brief 保持当前车道行驶状态

  LEFT_CHANGE = 11,    /// @brief 向左变更车道状态
  RIGHT_CHANGE = 12,   /// @brief 向右变更车道状态
  LEFT_ATTEMPT = 13,   /// @brief 尝试向左变更车道状态
  RIGHT_ATTEMPT = 14,  /// @brief 尝试向右变更车道状态
  LEFT_HOLD = 15,      /// @brief 保持向左变更车道过程中的状态
  RIGHT_HOLD = 16,     /// @brief 保持向右变更车道过程中的状态
  LEFT_RETURN = 17,    /// @brief 换道撤销，target lane 会变成 current lane 并保持一段时间，直到回 current 或再次 change
  RIGHT_RETURN = 18,  /// @brief 换道撤销，向右换道撤销，target lane 会变成 current lane 并保持一段时间，直到回 current
                      /// 或再次 change
};

/// @brief CIPO 候选对象类型别名，存储对象 ID 和对应的 s 值
using CipoCandidates = std::vector<std::pair<std::string, double>>;  // obj id and s

/// @brief 借道绕行情况枚举，描述障碍物是否需要借道绕行
enum class LaneBorrowingBypassType {
  NO_BORROWING = 0,     // 不借道绕行
  LEFT_BORROWING = 1,   // 左借道绕行
  RIGHT_BORROWING = 2,  // 右借道绕行
};

/// @brief 借道绕行信息结构体，记录障碍物是否需要借道绕行的详细信息，供下游规划计算精确路径
struct LaneBorrowingBypassInfo {
  /**
   * @brief 默认构造函数
   */
  LaneBorrowingBypassInfo() = default;
  LaneBorrowingBypassInfo(double start_s, double end_s, LaneBorrowingBypassType borrowing_type,
                          const std::string& obstacle_id, double remaining_space)
      : start_s(start_s),
        end_s(end_s),
        borrowing_type(borrowing_type),
        obstacle_id(obstacle_id),
        remaining_space(remaining_space) {}

  double start_s = 0.0;                                                            // 借道绕行区间的起始 s 值
  double end_s = 0.0;                                                              // 借道绕行区间的结束 s 值
  LaneBorrowingBypassType borrowing_type = LaneBorrowingBypassType::NO_BORROWING;  // 借道绕行类型
  std::string obstacle_id = "";                                                    // 对应的障碍物 ID
  double remaining_space = 0.0;                                                    // 距离绕行方向边界的剩余空间
};

/// @brief 借道绕行信息列表类型别名，用于存储多个障碍物的借道绕行信息
using LaneBorrowingBypassList = std::vector<LaneBorrowingBypassInfo>;

// ===== 最终决策结果相关参数 ===== //
/*============================== Final Output: Decision =======================================*/
/// @brief 决策结果类，整合了 OD 决策、边界决策、FSM 状态等信息，提供获取和修改这些信息的接口
class DecisionResult {
 public:
  /**
   * @brief 获取 OD 决策信息的常量引用。
   * @return OD 决策信息的常量引用。
   * @par 处理流程：
   * @startuml
   * start
   * :返回成员变量 od_decision_ 的常量引用;
   * stop
   * @enduml
   */
  const std::shared_ptr<Decision::DecisionObjectMap>& getOdDecisions() const { return od_decision_; }

  /**
   * @brief 获取 OD 决策信息的可变引用。
   * @return OD 决策信息的可变引用。
   * @par 处理流程：
   * @startuml
   * start
   * :返回成员变量 od_decision_ 的可变引用;
   * stop
   * @enduml
   */
  std::shared_ptr<Decision::DecisionObjectMap>& getMutableOdDecisions() { return od_decision_; }

  /**
   * @brief 获取上一次 OD 决策信息的常量引用。
   * @return 上一次 OD 决策信息的常量引用。
   * @par 处理流程：
   * @startuml
   * start
   * :返回成员变量 last_od_decision_ 的常量引用;
   * stop
   * @enduml
   */
  const std::shared_ptr<Decision::DecisionObjectMap>& getLastOdDecisions() const { return last_od_decision_; }

  /**
   * @brief 获取上一次 OD 决策信息的可变引用。
   * @return 上一次 OD 决策信息的可变引用。
   * @par 处理流程：
   * @startuml
   * start
   * :返回成员变量 last_od_decision_ 的可变引用;
   * stop
   * @enduml
   */
  std::shared_ptr<Decision::DecisionObjectMap>& getMutableLastOdDecisions() { return last_od_decision_; }

  /**
   * @brief 获取横向边界决策信息的常量引用。
   * @return 横向边界决策信息的常量引用。
   * @par 处理流程：
   * @startuml
   * start
   * :返回成员变量 lateral_bound_decision_ 的常量引用;
   * stop
   * @enduml
   */
  const LateralBoundDecision& getLateralBoundaryDecision() const { return lateral_bound_decision_; }

  /**
   * @brief 获取横向边界决策信息的可变指针。
   * @return 横向边界决策信息的可变指针。
   * @par 处理流程：
   * @startuml
   * start
   * :返回成员变量 lateral_bound_decision_ 的地址;
   * stop
   * @enduml
   */
  LateralBoundDecision* getMutableLateralBoundaryDecision() { return &lateral_bound_decision_; }

  /**
   * @brief 获取纵向边界决策信息的常量引用。
   * @return 纵向边界决策信息的常量引用。
   * @par 处理流程：
   * @startuml
   * start
   * :返回成员变量 longitudinal_bound_decision_ 的常量引用;
   * stop
   * @enduml
   */
  const LongitudinalBoundDecision& getLongitudinalBoundaryDecision() const { return longitudinal_bound_decision_; }

  /**
   * @brief 获取纵向边界决策信息的可变指针。
   * @return 纵向边界决策信息的可变指针。
   * @par 处理流程：
   * @startuml
   * start
   * :返回成员变量 longitudinal_bound_decision_ 的地址;
   * stop
   * @enduml
   */
  LongitudinalBoundDecision* getMutableLongitudinalBoundaryDecision() { return &longitudinal_bound_decision_; }

  const DecisionBoundaryInfo& getBaseDecisionBoundaryInfo() const { return base_decison_boundary_info_; }
  DecisionBoundaryInfo* getMutableBaseDecisionBoundaryInfo() { return &base_decison_boundary_info_; }

  const DecisionBoundaryInfo& getOdDecisionBoundaryInfo() const { return od_decison_boundary_info_; }
  DecisionBoundaryInfo* getMutableOdDecisionBoundaryInfo() { return &od_decison_boundary_info_; }

  const DecisionBoundaryInfo& getFinalDecisionBoundaryInfo() const { return final_decison_boundary_info_; }
  DecisionBoundaryInfo* getMutableFinalDecisionBoundaryInfo() { return &final_decison_boundary_info_; }

  const std::vector<TrajectoryPt>& getMultiAgentsTrajectory() const { return multi_agents_trajectory_; }
  std::vector<TrajectoryPt>* getMutableMultiAgentsTrajectory() { return &multi_agents_trajectory_; }

  const std::vector<ObjectInfo>& getDecisionObjectInfo() const { return decision_object_info_; }
  std::vector<ObjectInfo>* getMutableDecisionObjectInfo() { return &decision_object_info_; }
  /**
   * @brief 获取当前 FSM 状态的常量引用。
   * @return 当前 FSM 状态的常量引用。
   * @par 处理流程：
   * @startuml
   * start
   * :返回成员变量 curr_fsm_state_ 的常量引用;
   * stop
   * @enduml
   */
  const FsmState& getCurrFsmState() const { return curr_fsm_state_; }

  /**
   * @brief 获取当前 FSM 状态的可变指针。
   * @return 当前 FSM 状态的可变指针。
   * @par 处理流程：
   * @startuml
   * start
   * :返回成员变量 curr_fsm_state_ 的地址;
   * stop
   * @enduml
   */
  FsmState* getMutableCurrFsmState() { return &curr_fsm_state_; }

  /**
   * @brief 获取上一次 FSM 状态的常量引用。
   * @return 上一次 FSM 状态的常量引用。
   * @par 处理流程：
   * @startuml
   * start
   * :返回成员变量 last_fsm_state_ 的常量引用;
   * stop
   * @enduml
   */
  const FsmState& getLastFsmState() const { return last_fsm_state_; }

  /**
   * @brief 获取上一次 FSM 状态的可变指针。
   * @return 上一次 FSM 状态的可变指针。
   * @par 处理流程：
   * @startuml
   * start
   * :返回成员变量 last_fsm_state_ 的地址;
   * stop
   * @enduml
   */
  FsmState* getMutableLastFsmState() { return &last_fsm_state_; }

  /**
   * @brief 获取主路径历史信息的常量引用。
   * @return 主路径历史信息的常量引用。
   * @par 处理流程：
   * @startuml
   * start
   * :返回成员变量 main_path_history_ 的常量引用;
   * stop
   * @enduml
   */
  const MainPathHistory& getMainPathHistory() const { return main_path_history_; }

  /**
   * @brief 获取主路径历史信息的可变指针。
   * @return 主路径历史信息的可变指针。
   * @par 处理流程：
   * @startuml
   * start
   * :返回成员变量 main_path_history_ 的地址;
   * stop
   * @enduml
   */
  MainPathHistory* getMutableMainPathHistory() { return &main_path_history_; }

  /**
   * @brief 获取最佳参考线 ID 的常量引用。
   * @return 最佳参考线 ID 的常量引用。
   * @par 处理流程：
   * @startuml
   * start
   * :返回成员变量 best_ref_line_id_ 的常量引用;
   * stop
   * @enduml
   */
  const std::string& getBestRefLineId() const { return best_ref_line_id_; }

  /**
   * @brief 获取最佳参考线 ID 的可变指针。
   * @return 最佳参考线 ID 的可变指针。
   * @par 处理流程：
   * @startuml
   * start
   * :返回成员变量 best_ref_line_id_ 的地址;
   * stop
   * @enduml
   */
  std::string* getMutableBestRefLineId() { return &best_ref_line_id_; }

  /**
   * @brief 获取当前参考线 ID 的常量引用。
   * @return 当前参考线 ID 的常量引用。
   * @par 处理流程：
   * @startuml
   * start
   * :返回成员变量 current_ref_line_id_ 的常量引用;
   * stop
   * @enduml
   */
  const std::string& getCurrentRefLineId() const { return current_ref_line_id_; }

  /**
   * @brief 获取当前参考线 ID 的可变指针。
   * @return 当前参考线 ID 的可变指针。
   * @par 处理流程：
   * @startuml
   * start
   * :返回成员变量 current_ref_line_id_ 的地址;
   * stop
   * @enduml
   */
  std::string* getMutableCurrentRefLineId() { return &current_ref_line_id_; }

  /**
   * @brief 获取交通灯决策信息的常量引用。
   * @return 交通灯决策信息的常量引用。
   * @par 处理流程：
   * @startuml
   * start
   * :返回成员变量 traffic_light_info_ 的常量引用;
   * stop
   * @enduml
   */
  const TrafficLightDecision& getTrafficLightDecision() const { return traffic_light_info_; }

  /**
   * @brief 获取交通灯决策信息的可变指针。
   * @return 交通灯决策信息的可变指针。
   * @par 处理流程：
   * @startuml
   * start
   * :返回成员变量 traffic_light_info_ 的地址;
   * stop
   * @enduml
   */
  TrafficLightDecision* getMutableTrafficLightDecision() { return &traffic_light_info_; }

  /**
   * @brief 获取 CIPO 候选对象信息的常量引用。
   * @return CIPO 候选对象信息的常量引用。
   * @par 处理流程：
   * @startuml
   * start
   * :返回成员变量 cipo_candidates_ 的常量引用;
   * stop
   * @enduml
   */
  const CipoCandidates& getCipoCandidates() const { return cipo_candidates_; }

  /**
   * @brief 获取 CIPO 候选对象信息的可变指针。
   * @return CIPO 候选对象信息的可变指针。
   * @par 处理流程：
   * @startuml
   * start
   * :返回成员变量 cipo_candidates_ 的地址;
   * stop
   * @enduml
   */
  CipoCandidates* getMutableCipoCandidates() { return &cipo_candidates_; }

  /**
   * @brief 获取调试信息的可变指针。
   * @return 调试信息的可变指针。
   * @par 处理流程：
   * @startuml
   * start
   * :返回成员变量 debug_info_ 的地址;
   * stop
   * @enduml
   */
  std::string* mutableDebugInfo() { return &debug_info_; }

  /**
   * @brief 获取调试信息的常量引用。
   * @return 调试信息的常量引用。
   * @par 处理流程：
   * @startuml
   * start
   * :返回成员变量 debug_info_ 的常量引用;
   * stop
   * @enduml
   */
  const std::string& getDebugInfo() const { return debug_info_; }

  /**
   * @brief 获取参考轨迹信息的常量引用。
   * @return 参考轨迹信息的常量引用。
   * @par 处理流程：
   * @startuml
   * start
   * :返回成员变量 ref_traj_info_ 的常量引用;
   * stop
   * @enduml
   */
  const std::shared_ptr<RefTrajInfo>& getRefTrajInfo() const { return ref_traj_info_; }

  /**
   * @brief 获取参考轨迹信息的可变引用。
   * @return 参考轨迹信息的可变引用。
   * @par 处理流程：
   * @startuml
   * start
   * :返回成员变量 ref_traj_info_ 的可变引用;
   * stop
   * @enduml
   */
  std::shared_ptr<RefTrajInfo>& mutableRefTrajInfo() { return ref_traj_info_; }

  /**
   * @brief 清空参考轨迹信息。
   * @par 处理流程：
   * @startuml
   * start
   * :将 ref_traj_info_ 中的 traj_duration 置为 0.0;
   * :清空 ref_traj_info_ 中的 traj_points;
   * :将 ref_traj_info_ 中的 use_ref_longitudinal 置为 false;
   * :将 ref_traj_info_ 中的 use_ref_lateral 置为 false;
   * stop
   * @enduml
   */
  void clearRefTrajInfo() {
    ref_traj_info_->traj_duration = 0.0;
    ref_traj_info_->traj_points.clear();
    ref_traj_info_->use_ref_longitudinal = false;
    ref_traj_info_->use_ref_lateral = false;
  }
  /**
   * @brief 获取联合优化约束障碍物ID列表
   * @return 联合优化约束障碍物ID列表
   */
  const std::vector<std::string>& getJointOptObstacles() const { return joint_opt_obstacles_ids_; }
  std::vector<std::string>* getMutableJointOptObstacles() { return &joint_opt_obstacles_ids_; }
  void setJointOptObstacles(const std::vector<std::string>& ids) { joint_opt_obstacles_ids_ = ids; }
  void setJointOptObstacles(std::vector<std::string>&& ids) { joint_opt_obstacles_ids_ = std::move(ids); }
  void clearJointOptObstacles() { joint_opt_obstacles_ids_.clear(); }
  /**
   * @brief 获取联合优化博弈障碍物ID列表
   * @return 联合优化博弈障碍物ID列表
   */
  const std::vector<std::string>& getJointOptAgents() const { return joint_opt_agents_ids_; }
  std::vector<std::string>* getMutableJointOptAgents() { return &joint_opt_agents_ids_; }
  void setJointOptAgents(const std::vector<std::string>& ids) { joint_opt_agents_ids_ = ids; }
  void setJointOptAgents(std::vector<std::string>&& ids) { joint_opt_agents_ids_ = std::move(ids); }
  void clearJointOptAgents() { joint_opt_agents_ids_.clear(); }

  /**
   * @brief 获取中间态局部路径参考线（intermediate local path reference line, 对应lpr_1）
   * @return 中间态局部路径参考线
   */
  const std::shared_ptr<ReferenceLine>& getIntermediateLocalPathRefLine() const { return intermediate_local_path_ref_line_; }
  /**
   * @brief 获取中间态局部路径参考线的可变指针
   * @return 中间态局部路径参考线的可变指针
   */
  std::shared_ptr<ReferenceLine>& mutableIntermediateLocalPathRefLine() { return intermediate_local_path_ref_line_; }

  /**
   * @brief 获取最终局部路径参考线（final local path reference line, 对应lpr_2）
   * @return 最终局部路径参考线
   */
  const std::shared_ptr<ReferenceLine>& getFinalLocalPathRefLine() const { return final_local_path_ref_line_; }
  /**
   * @brief 获取最终局部路径参考线的可变指针
   * @return 最终局部路径参考线的可变指针
   */
  std::shared_ptr<ReferenceLine>& mutableFinalLocalPathRefLine() { return final_local_path_ref_line_; }

  /**
   * @brief 获取借道绕行信息列表
   * @return 借道绕行信息列表的常量引用
   */
  const LaneBorrowingBypassList& getLaneBorrowingBypassList() const { return lane_borrowing_bypass_list_; }

  /**
   * @brief 获取借道绕行信息列表的可变指针
   * @return 借道绕行信息列表的可变指针
   */
  LaneBorrowingBypassList* mutableLaneBorrowingBypassList() { return &lane_borrowing_bypass_list_; }

  const double& getMaxSpeedLimit() const { return max_speed_limit_; }
  void setMaxSpeedLimit(double max_speed_limit) { max_speed_limit_ = max_speed_limit; }

  const std::string& getEgoRefLineIdHd() const { return ego_ref_line_id_hd_; }
  std::string* getMutableEgoRefLineIdHd() { return &ego_ref_line_id_hd_; }

  const std::string& getEgoRefLineIdSd() const { return ego_ref_line_id_sd_; }
  std::string* getMutableEgoRefLineIdSd() { return &ego_ref_line_id_sd_; }

 private:
  /// @brief 当前 OD 决策信息指针
  std::shared_ptr<Decision::DecisionObjectMap> od_decision_ = std::make_shared<Decision::DecisionObjectMap>();
  /// @brief 上一次 OD 决策信息指针
  std::shared_ptr<Decision::DecisionObjectMap> last_od_decision_ = std::make_shared<Decision::DecisionObjectMap>();
  /// @brief 横向边界决策信息
  LateralBoundDecision lateral_bound_decision_;
  /// @brief 纵向边界决策信息
  LongitudinalBoundDecision longitudinal_bound_decision_;
  /// @brief 分层边界信息--包含road info and polyline
  DecisionBoundaryInfo base_decison_boundary_info_;
  /// @brief 分层边界信息--add static obs
  DecisionBoundaryInfo od_decison_boundary_info_;
  /// @brief 分层边界信息3 ......
  /// @brief 分层边界信息final
  DecisionBoundaryInfo final_decison_boundary_info_;

  /// @brief 多智能体轨迹信息
  std::vector<TrajectoryPt> multi_agents_trajectory_;

  std::vector<ObjectInfo> decision_object_info_;

  /// @brief 当前 FSM 状态
  FsmState curr_fsm_state_;
  /// @brief 上一次 FSM 状态
  FsmState last_fsm_state_;
  /// @brief 主路径历史信息
  MainPathHistory main_path_history_;
  /// @brief 交通灯决策信息
  TrafficLightDecision traffic_light_info_{};
  /// @brief 最佳参考线 ID
  std::string best_ref_line_id_ = "";
  /// @brief 当前参考线 ID
  std::string current_ref_line_id_ = "";
  /// @brief 调试信息
  std::string debug_info_ = "";
  /// @brief 参考轨迹信息指针
  std::shared_ptr<RefTrajInfo> ref_traj_info_ = std::make_shared<RefTrajInfo>();
  /// @brief CIPO 候选对象信息
  CipoCandidates cipo_candidates_{};
  /// @brief 参考线决策信息的智能指针
  std::shared_ptr<RefLineDecesionInfo> ref_line_decision_info_ =
      std::make_shared<RefLineDecesionInfo>();
  /// @brief 联合优化约束障碍物ID列表
  std::vector<std::string> obstacles_{};
  /// @brief 联合优化博弈障碍物ID列表
  std::vector<std::string> agents_{};
  /// @brief 联合优化约束障碍物ID列表（新接口使用）
  std::vector<std::string> joint_opt_obstacles_ids_{};
  /// @brief 联合优化博弈障碍物ID列表（新接口使用）
  std::vector<std::string> joint_opt_agents_ids_{};
  /// @brief 中间态局部路径参考线(对应lpr_1)，基于源目标参考线和道路边界计算得到，用于时空联合决策规划
  std::shared_ptr<ReferenceLine> intermediate_local_path_ref_line_ = std::make_shared<ReferenceLine>();
  /// @brief 最终局部路径参考线(对应lpr_2)，基于中间态参考线和障碍物边界进计算得到，用于时空联合决策规划
  std::shared_ptr<ReferenceLine> final_local_path_ref_line_ = std::make_shared<ReferenceLine>();
  /// @brief 借道绕行信息列表，存储多个障碍物的借道绕行信息，用于下游规划模块调用
  LaneBorrowingBypassList lane_borrowing_bypass_list_{};
   
  /// @brief HD地图参考线 ID
  std::string ego_ref_line_id_hd_;
  /// @brief SD感知参考线 ID
  std::string ego_ref_line_id_sd_;
  /// @brief 最大速度限制
  double max_speed_limit_ = kMaxSpeedMS;
};

}  // namespace gpal::pnc::planning

#endif