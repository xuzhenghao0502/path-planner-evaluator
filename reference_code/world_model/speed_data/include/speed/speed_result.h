/**
 * @file speed_result.h
 * @brief 速度规划结果容器
 * @details 本类集成速度规划全流程输出数据，作为模块间数据交换的统一接口
 */
#pragma once

#include <unordered_map>
#include <vector>

#include "math/vec2d.h"
#include "speed/speed_data.h"
#include "speed/speed_planner_obstacle.h"
#include "speed/st_graph.h"

namespace gpal::pnc::planning {

struct StopReason {
  enum StopReasonType {
    INVALID,        // 无效
    DRIVING,        // 正常行驶
    MANUAL,         // 人工
    FALLBACK,       // 异常降级导致的停车
    BLOCK_OD,       // 障碍物逼停
    BLOCK_FS,       // freespace 逼停
    GATE,           // 闸机前停车
    STOP_LINE,      // 红绿灯前停车
    DESTINATION,    // 终点停车
    RISKY_SPEED,    // 风险限速导致的停车
    CLEAR_ZONE,     // 禁停区中存在其他障碍物导致的停车
    VIRTUAL_BLOCK,  // 虚拟障碍物导致的停车
    PRECISE_STOP,   // 精停
    REF_LINE_END,   // 参考线终点导致的停车
    SUSPEND_TASK,   // 暂停任务
  };
  ///< 触发停车的目标ID（障碍物ID/交通灯ID等）
  std::string stop_id = "";
  ///< 当前生效的停车原因类型
  StopReasonType stop_reason_type = StopReasonType::INVALID;
  ///< 停车点距离自车的纵向距离（单位：米）
  double stop_distance = 0.0;
  /// @brief 停车原因优先级映射表（数值越大优先级越高）
  std::unordered_map<StopReasonType, int> stop_reason_rank_map = {
      // 0: invalid, 1: soft bound, 2: block, 3: abnormal, 4: driving/manual
      {StopReasonType::INVALID, 0},      {StopReasonType::DRIVING, 4},     {StopReasonType::MANUAL, 4},
      {StopReasonType::FALLBACK, 3},     {StopReasonType::BLOCK_OD, 2},    {StopReasonType::BLOCK_FS, 2},
      {StopReasonType::GATE, 2},         {StopReasonType::STOP_LINE, 2},   {StopReasonType::DESTINATION, 2},
      {StopReasonType::RISKY_SPEED, 1},  {StopReasonType::CLEAR_ZONE, 2},  {StopReasonType::VIRTUAL_BLOCK, 2},
      {StopReasonType::PRECISE_STOP, 2}, {StopReasonType::REF_LINE_END, 2}, {StopReasonType::SUSPEND_TASK, 3}};
  /**
   * @brief 清除所有停车原因数据
   */
  void clear() {
    stop_id = "";
    stop_reason_type = StopReasonType::INVALID;
    stop_distance = 0.0;
  }
  /**
   * @brief 更新停车原因（带优先级逻辑）
   * @param new_id 新停车目标的唯一标识
   * @param new_stop_reason_type 新停车原因类型
   * @param new_stop_distance 新停车距离（单位：米）
   * @return 是否成功更新（当新原因优先级低于当前原因时返回false）
   *
   * 更新逻辑：
   * 1. 比较新旧原因的优先级数值
   * 2. 高优先级原因覆盖低优先级
   * 3. 同优先级时：
   *   - 异常停车（FALLBACK）累积多个原因
   *   - 障碍物停车取最近距离的原因
   */
  bool updateStopReason(const std::string& new_id, const StopReasonType& new_stop_reason_type,
                        const double& new_stop_distance = 0.0) {
    bool update_success = false;
    int new_priority_rank = stop_reason_rank_map.find(new_stop_reason_type) == stop_reason_rank_map.end()
                                ? 0
                                : stop_reason_rank_map.at(new_stop_reason_type);
    int old_priority_rank = stop_reason_rank_map.find(stop_reason_type) == stop_reason_rank_map.end()
                                ? 0
                                : stop_reason_rank_map.at(stop_reason_type);
    if (new_priority_rank < old_priority_rank) {
      return update_success;
    } else if (new_priority_rank == old_priority_rank && new_priority_rank == 3) {  // abnormal 会保留全部信息
      stop_id += "|" + new_id;
    } else if (new_priority_rank == old_priority_rank &&
               new_stop_distance < stop_distance) {  // block/soft bound 只保留距离自车最近的信息
      stop_id = new_id;
      stop_distance = new_stop_distance;
    } else {
      stop_id = new_id;
      stop_reason_type = new_stop_reason_type;
      stop_distance = new_stop_distance;
    }
    return update_success;
  }
};

/// @brief 速度限制墙类型枚举
enum SpeedWallType {
  UNKNOWN_WALL = 0,         ///< 未知限制类型
  RSA_WALL = 1,             ///< RSA限速区域（Road Speed Advisory）
  TFL_WALL = 2,             ///< 交通信号灯限制（Traffic Light）
  DESTINATION_WALL = 3,     ///< 目的地终点限制
  FREESPACE_WALL = 4,       ///< 自由空间边界限制
  RISKY_TAIL_WALL = 5,      ///< 风险车尾区域限制
  JUNCTION_STOP = 6,        ///< 交叉口停车限制
  END_POINT = 7,            ///< 路径终点限制
  CLEAR_ZONE = 8,           ///< 净空区域限制
  CRANE_TRANSFER_STOP = 9,  ///< 吊运区域停车限制
  REF_LINE_END = 10,        ///< 参考线终点限制
  LANE_CHANGE = 11,         ///< 换道区域限制
  VIRTUAL_LINE = 12,        ///< 虚拟车道线限制
  WEIGH_BRIDGE_STOP = 13,   ///< 地磅区域停车限制
  SG_WEIGHBRIDGE = 14,      ///< 特殊地磅限制
};
static const std::unordered_map<SpeedWallType, std::string> speed_wall_id_map{{UNKNOWN_WALL, "unknown"},
                                                                              {RSA_WALL, "rsa"},
                                                                              {TFL_WALL, "tsr"},
                                                                              {DESTINATION_WALL, "destination"},
                                                                              {FREESPACE_WALL, "fs"},
                                                                              {RISKY_TAIL_WALL, "risky_tail"},
                                                                              {JUNCTION_STOP, "junction_stop"},
                                                                              {END_POINT, "end_point"},
                                                                              {CLEAR_ZONE, "clear_zone"},
                                                                              {CRANE_TRANSFER_STOP, "crane_transfer"},
                                                                              {REF_LINE_END, "ref_line_end"},
                                                                              {LANE_CHANGE, "lane_change"},
                                                                              {VIRTUAL_LINE, "virtual_line"},
                                                                              {WEIGH_BRIDGE_STOP, "weigh_bridge"},
                                                                              {SG_WEIGHBRIDGE, "sg_rsa"}};
/**
 * @brief 速度限制墙描述结构体
 *
 * 用于表示在ST坐标系中影响速度规划的各种限制区域
 */
struct SpeedWall {
  /// @brief 速度墙类型标识（参见SpeedWallType枚举定义）
  SpeedWallType type;

  /// @brief ST坐标系中的限制区域边界点集合
  /// @details 每个Vec2d表示(s,t)坐标点：
  ///          - s: 纵向距离（单位：米）
  ///          - t: 时间坐标（单位：秒）
  std::vector<math::Vec2d> st_wall;

  /// @brief 自车到该限制墙的最近纵向距离（单位：米）
  double stop_distance;
};

/**
 * @brief 速度限制结果集合
 *
 * 存储速度规划过程中产生的各类限制信息
 */
struct SpeedLimitResult {
  /// @brief 全局最大速度限制值（单位：m/s）
  double max_speed_limit_ = 0.0;

  /// @brief 空间速度限制集合（包含位置相关的动态限速）
  std::vector<SpatialSpeedLimit> speed_limit_;

  /// @brief 曲率相关速度限制点集合（基于道路曲率计算得出）
  std::vector<proto::TrajectoryPoint> curvature_speed_limit_;

  /// @brief 路径速度-时间剖面数据（用于可视化调试）
  std::vector<proto::TrajectoryPoint> path_v_t_;

  /**
   * @brief 清除所有限制数据
   *
   * 重置内容包括：
   * - 最大速度限制归零
   * - 清空三个限制数据容器
   */
  void clear() {
    max_speed_limit_ = 0.0;
    speed_limit_.clear();
    curvature_speed_limit_.clear();
    path_v_t_.clear();
  }
};

/**
 * @brief 入侵障碍物描述结构体
 *
 * 用于表示侵入规划路径的障碍物信息
 */
struct InvasionObstacle {
  /// @brief 障碍物唯一标识（"None"表示无入侵障碍物）
  std::string obj_id_ = "None";

  /// @brief 障碍物入侵路径的起始纵向距离（单位：米，无穷大表示无入侵）
  double invasion_s_ = std::numeric_limits<double>::infinity();

  /// @brief 障碍物预计入侵路径的时间点（单位：秒）
  double invasion_t_ = 0;

  /// @brief 障碍物在入侵时刻的预测速度（单位：m/s）
  double invasion_v_ = 0;

  /// @brief 是否为真实物理障碍物（false表示虚拟/预测障碍物）
  bool is_obstacle_ = false;

  /**
   * @brief 清除所有障碍物信息
   *
   * 重置内容：
   * - 障碍物ID设为"None"
   * - 入侵距离恢复为无穷大
   * - 时间/速度归零
   * - 障碍物标识复位
   */
  void clear() {
    obj_id_ = "None";
    invasion_s_ = std::numeric_limits<double>::infinity();
    invasion_t_ = 0;
    invasion_v_ = 0;
    is_obstacle_ = false;
  }
};

/**
 * @brief 车辆行为状态描述结构体
 *
 * 记录与停车、换挡相关的行为状态标志位
 */
struct BehaviorState {
  /// @brief 停车入位状态标识（true表示正在执行停车入位）
  bool park_in_state_ = false;

  /// @brief 停车位搜索状态标识（true表示正在寻找可用停车位）
  bool search_parklot_state_ = false;

  /// @brief 倒挡状态标识（true表示当前处于R挡）
  bool is_r_gear_ = false;

  /// @brief acc状态标识（true表示当前处于ACC状态）
  bool is_acc_state_ = false;

  /// @brief lcc状态标识（true表示当前处于LCC状态）
  bool is_lcc_state_ = false;

  /**
   * @brief 清除所有行为状态
   *
   * 重置内容：
   * - 停车入位状态复位
   * - 停车位搜索状态复位
   * - 挡位状态复位
   * - 加速状态复位
   */
  void clear() {
    park_in_state_ = false;
    search_parklot_state_ = false;
    is_r_gear_ = false;
    is_acc_state_ = false;
    is_lcc_state_ = false;
  }
};

class SpeedResult {
 public:
  /**
   * @brief 清空所有规划结果数据
   */
  void clear();

  /// @name 行为状态访问接口
  /// @{
  const BehaviorState& getBehaviorState() const { return behavior_state_; }
  BehaviorState* mutableBehaviorState() { return &behavior_state_; }
  /// @}

  /// @name 速度限制结果访问接口
  /// @{
  const SpeedLimitResult& speedLimitResult() const { return speed_limit_result_; }
  SpeedLimitResult* mutableSpeedLimiteResult() { return &speed_limit_result_; }
  /// @}

  /// @name 障碍物集合访问接口
  /// @{
  const ObstacleSet& obstacleSet() const { return obstacle_set_; }
  ObstacleSet* mutableObstacleSet() { return &obstacle_set_; }
  /// @}

  /// @name 速度规划数据访问接口
  /// @{
  const SpeedData& speed_data() const { return speed_data_; }
  SpeedData* const mutable_speed_data() { return &speed_data_; }
  /// @}

  /// @name 调试速度数据访问接口
  /// @{
  const SpeedData& debug_speed_data() const { return debug_speed_data_; }
  SpeedData* const mutable_debug_speed_data() { return &debug_speed_data_; }
  /// @}

  /// @name 停车原因访问接口
  /// @{
  const StopReason& stop_reason() const { return stop_reason_; }
  StopReason* mutable_stop_reason() { return &stop_reason_; }
  /// @}

  /// @name 速度限制墙访问接口
  /// @{
  const std::vector<SpeedWall>& speed_walls() const { return speed_walls_; }
  std::vector<SpeedWall>* mutable_speed_walls() { return &speed_walls_; }
  /// @}

  /// @name ST图数据访问接口
  /// @{
  const StGraph& StGraphData() const { return st_graph_; }
  StGraph* mutableStGraphData() { return &st_graph_; }
  /// @}

  /// @name 入侵障碍物访问接口
  /// @{
  const InvasionObstacle& nearestInvasionObstacle() const { return nearest_invasion_obstacle_; }
  InvasionObstacle* mutableNearestInvasionObstacle() { return &nearest_invasion_obstacle_; }
  /// @}

  /// @name 转向控制相关
  /// @{
  const int& getTurnFlag() const { return turn_flag_; }
  void setTurnFlag(int turn_flag) { turn_flag_ = turn_flag; }
  /// @}

  /// @name 目的地停车标志
  /// @{
  const bool& getDestinationStopFlag() const { return destination_stop_flag_; }
  void setDestinationStopFlag(bool destination_stop_flag) { destination_stop_flag_ = destination_stop_flag; }
  /// @}

  /// @name 轨迹结果访问接口
  /// @{
  const proto::Trajectory& trajectoryResult() const { return trajectory_result_; }
  proto::Trajectory* mutableTrajectoryResult() { return &trajectory_result_; }
  /// @}

  /// @name 局部路径下边界访问接口
  /// @{
  const std::vector<std::pair<STPoint, std::string>> localPathLowerBoundary() const {
    return local_path_lower_boundary;
  }
  std::vector<std::pair<STPoint, std::string>>* mutableLocalPathLowerBoundary() { return &local_path_lower_boundary; }
  /// @}

  /// @name 障碍物剩余距离访问接口
  /// @{
  const double& getObstacleRemainDistance() const { return obstacle_remain_distance_; }
  void setObstacleRemainDistance(double obstacle_remain_distance) { obstacle_remain_distance_ = obstacle_remain_distance; }
  /// @}

 private:
  BehaviorState behavior_state_;                ///< 车辆行为状态（停车/换挡等）
  SpeedLimitResult speed_limit_result_;         ///< 速度限制计算结果集合
  ObstacleSet obstacle_set_;                    ///< 障碍物信息集合
  InvasionObstacle nearest_invasion_obstacle_;  ///< 最近入侵障碍物信息
  int turn_flag_ = 0;                           ///< 转向控制标志位

  StGraph st_graph_;                                                       ///< ST图规划数据
  std::vector<SpeedWall> speed_walls_;                                     ///< 速度限制墙集合
  std::vector<std::pair<STPoint, std::string>> local_path_lower_boundary;  ///< 局部路径下边界数据

  SpeedData speed_data_;        ///< 最终速度规划数据
  SpeedData debug_speed_data_;  ///< 调试用速度数据

  StopReason stop_reason_;               ///< 停车原因详细信息
  proto::Trajectory trajectory_result_;  ///< 最终规划轨迹

  bool destination_stop_flag_ = false;  ///< 到达目的地停车标志
  double obstacle_remain_distance_ = 10000.0;  ///< 障碍物剩余距离
};

}  // namespace gpal::pnc::planning
