/**
 * @file path_decision.h
 * @brief 路径决策信息结构体定义
 * @details 包含路径边界信息、忽略区域信息、路径收缩区域信息等
 */

#pragma once

#include <limits>
#include <string>

#include "base/indexed_list.h"
#include "obstacle/obstacle.h"
// #include "proto/decision/main_decision.pb.h"
// #include "proto/decision/object_decisions.pb.h"
// #include "local_view/polyline_fs.h"
#include "math/line_segment2d.h"
#include "proto/common/pnc_point.pb.h"
// #include "proto/decision/object_decisions.pb.h"

namespace gpal::pnc::planning {

/**
 * @brief 路径边界信息结构体
 * @details 描述单个路径点的横向边界约束
 *
 * @note 用于路径优化模块的边界约束
 *
 * @warning 边界值需通过平滑处理
 */
struct PathBoundInfo {
  double s;            ///< 纵向坐标，单位：米
  double left_bound;   ///< 左边界距离，单位：米
  double right_bound;  ///< 右边界距离，单位：米
};

/**
 * @brief 忽略范围信息结构体
 * @details 定义需要特殊处理的路径区域
 *
 * @note 与高精度地图数据同步更新
 * @warning 范围重叠将导致优先级冲突
 */
struct IgnoreRangeInfo {
  enum IgnoreRangeType {
    DISABLED = 0,         ///< 未激活状态，不参与路径计算
    LSTP = 1,             ///< 停车场寻库区域
    GATE = 2,             ///< 自动闸机区域
    TRAFFIC_LIGHTS = 3,   ///< 交通灯控制区域
    TARGET_LINE_END = 4,  ///< 路径终点过渡区
    DESTINATION = 5,      ///< 目的地到达区域
    LONGITUDE = 8,        ///< 经度道路区域
    BLOCKED_FS = 9,       ///< 完全封锁区域

    ALL_LANE_BLOCKED = 10,      ///< 全车道封锁状态
    WEIGH_BRIDGE = 21,          ///< 地磅区域
    PNC_SCENE_URBAN_ROAD = 22,  ///< 城市道路场景
  };

  IgnoreRangeType type;  ///< 区域类型标识
  double start_s;        ///< 起始纵向坐标\n单位:米，
  double end_s;          ///< 结束纵向坐标\n约束: end_s ≥ start_s + delta米
};

/**
 * @brief 路径收缩区域信息
 * @details 描述需要横向收缩的路径区域，用于避障和车道保持
 *
 * @par 区域类型说明:
 * - LEFT  : 左侧收缩区域
 * - RIGHT : 右侧收缩区域
 *
 * @note 收缩系数需通过平滑处理
 */
struct RangeInfo {
  enum RangeType {
    NONE = 0,   ///< 无收缩状态
    LEFT = 1,   ///< 左侧收缩区域
    RIGHT = 2,  ///< 右侧收缩区域
  };

  RangeType type;        ///< 收缩区域类型
  double s;              ///< 当前路径点s坐标
  double start_s;        ///< 起始s坐标
  double end_s;          ///< 结束s坐标
  double contract_coff;  ///< 收缩系数
};

bool isInRange(const std::vector<IgnoreRangeInfo>& ranges, const double s);
bool isInRange(const std::vector<IgnoreRangeInfo>& ranges, const double start_s, const double end_s);
bool isInRange(const std::vector<RangeInfo>& ranges, const double s);
/**
 * @class 路径决策类
 * @details 管理路径上的所有障碍物决策和控制策略
 *
 * @brief PathDecision represents all obstacle decisions on one path.
 *        目前只有 nudge 用，用于 BP 输出最大允许范围、障碍物横向 tag，与 lane_keep_start_s 给下游
 *
 */
class PathDecision {
 public:
  /**
   * @brief 路径偏移量集合类型
   * @details 存储(s,l)坐标对的容器，用于路径横向偏移计算
   *
   * @par 数据结构:
   * - 每个元素为std::array<double,2>
   * - array[0]: s坐标 (单位：米)
   * - array[1]: l坐标 (单位：米)
   */
  typedef std::vector<std::array<double, 2>> RefOffsetsType;  ///< 路径偏移量集合

  /**
   * @brief 行为类型枚举
   * @details 定义路径决策模块的不同工作模式
   *
   */
  enum class BehaviorType {
    NONE = 0,
    PULL_OVER_LONGI = 1,    ///< 纵向停车模式
    PULL_OVER_LATERAL = 2,  ///< 侧方停车模式
  };

  /**
   * @brief 默认构造函数
   * @details 初始化决策容器和状态标志
   *
   */
  PathDecision() = default;

  /**
   * @brief 获取忽略范围列表（只读）
   * @details 返回当前所有需要忽略的路径区域集合
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :返回ignore_range_;
   * stop
   * @enduml
   *
   * @return const std::vector<IgnoreRangeInfo>& 忽略区域列表的不可变引用
   * @warning 返回的引用在对象生命周期结束后失效
   */
  const std::vector<IgnoreRangeInfo>& ignoreRange() const { return ignore_range_; };

  /**
   * @brief 获取可修改的忽略范围列表指针
   * @details 返回可用于直接修改忽略区域集合的指针
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :返回&ignore_range_;
   * stop
   * @enduml
   *
   * @note 调用后需自行处理线程安全问题
   *
   * @return std::vector<IgnoreRangeInfo>* 可修改的指针
   * @warning 多线程访问需要自行加锁
   */
  std::vector<IgnoreRangeInfo>* const mutableIgnoreRange() { return &ignore_range_; };

  // 1. nudge tag 风险等级由大到小排列为：dynamic nudge > risky nudge > nudge；
  // 2. 风险低到风险高当前帧立马响应，比如前一帧是nudge，这一帧判断为risky，立马是risky；
  // 3. 风险高到风险低需要连续判断5帧都成功（Dynamic nudge 到
  // nudge不需要），比如前一帧是risky，这一帧判断为nudge，则还是risky，连续5帧是nudge后才会变成nudge；
  // std::pair<std::shared_ptr<Obstacle>, bool> addStaticObstacle(const Obstacle& obstacle);
  // std::pair<std::shared_ptr<Obstacle>, bool> addRiskyObstacle(const Obstacle& obstacle);  // 仅针对出库蠕行cut-in车辆
  // std::pair<std::shared_ptr<Obstacle>, bool> addDynamicObstacle(const Obstacle& obstacle);

  /**
   * @brief 获取静态障碍物容器（只读）
   * @details 包含静态障碍物和近场障碍物集合
   * @par 更新流程图:
   * @startuml
   * start
   * :返回static_obstacles_;
   * stop
   * @enduml
   *
   * @return const IndexedList<Obstacle>& 不可变引用
   * @note 包含nudge风险等级最高的障碍物
   * @warning 直接操作容器可能破坏风险等级一致性
   */
  const IndexedList<Obstacle>& static_obstacles() const { return static_obstacles_; }

  /**
   * @brief 获取可修改的静态障碍物容器指针
   * @warning 修改后需调用refreshObstacleTags()刷新障碍物标签
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :返回&static_obstacles_;
   * stop
   * @enduml
   *
   * @return IndexedList<Obstacle>* 可修改指针
   * @warning 多线程访问需自行处理
   */
  IndexedList<Obstacle>* mutable_static_obstacles() { return &static_obstacles_; }

  /**
   * @brief 获取动态障碍物容器（只读）
   * @details 包含需要实时避让的动态障碍物
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :返回dynamic_obstacles_;
   * stop
   * @enduml
   *
   * @return const IndexedList<Obstacle>& 不可变引用
   * @warning 直接操作容器可能破坏风险等级一致性
   */
  const IndexedList<Obstacle>& dynamic_obstacles() const { return dynamic_obstacles_; }

  /**
   * @brief 获取可修改的动态障碍物容器指针
   * @warning 修改需触发路径重规划
   * @par 更新流程图:
   * @startuml
   * start
   * :返回&dynamic_obstacles_;
   * stop
   * @enduml
   *
   * @return IndexedList<Obstacle>* 可修改指针
   * @warning 多线程访问需自行处理
   */
  IndexedList<Obstacle>* mutable_dynamic_obstacles() { return &dynamic_obstacles_; }

  /**
   * @brief 获取高风险障碍物容器（只读）
   * @details 特定场景下的高风险障碍物（如出库蠕行车辆）
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :返回risky_obstacles_;
   * stop
   * @enduml
   *
   * @return const IndexedList<Obstacle>& 不可变引用
   * @note 风险等级介于static和dynamic之间
   */
  const IndexedList<Obstacle>& risky_obstacles() const { return risky_obstacles_; }

  /**
   * @brief 获取可修改的高风险障碍物容器指针
   * @details 返回可用于直接修改高风险障碍物集合的指针
   * @par 更新流程图:
   * @startuml
   * start
   * :返回&risky_obstacles_;
   * stop
   * @enduml
   * @warning 多线程访问需自行处理
   *
   * @return IndexedList<Obstacle>* 可修改指针
   */
  IndexedList<Obstacle>* mutable_risky_obstacles() { return &risky_obstacles_; }

  // key: dynamic obs id; value: nudge tag, obs_info->start_t, obs_info->end_t
  // const DynamicObsIdInfoMap& dynamic_obstacle_info() const { return dynamic_obstacle_info_; };
  // DynamicObsIdInfoMap* mutable_dynamic_obstacle_info() { return &dynamic_obstacle_info_; };

  // bool = if borrowed line

  /**
   * @brief 获取最大允许路径边界（只读）
   * @details 返回当前路径的最大可行驶边界信息
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :返回max_allowed_bounds_;
   * stop
   * @enduml
   *
   * @return const std::pair<bool, std::vector<PathBoundInfo>>&
   *         - bool: 是否借用对向车道
   *         - vector: 按s排序的边界点集合
   * @note 边界信息需与障碍物决策同步更新
   */
  const std::pair<bool, std::vector<PathBoundInfo>>& max_allowed_bounds() const { return max_allowed_bounds_; };

  /**
   * @brief 获取可修改的最大允许路径边界指针
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :返回&max_allowed_bounds_;
   * stop
   * @enduml
   *
   * @return std::pair<bool, std::vector<PathBoundInfo>>*
   * @note 建议通过addObstacle方法间接修改边界
   * @warning 多线程访问需自行处理
   */
  std::pair<bool, std::vector<PathBoundInfo>>* mutable_max_allowed_bounds() { return &max_allowed_bounds_; };

  void setLaneKeepStartS(const double lane_keep_start_s);

  void setStopReferenceLineS(const double stop_reference_line_s);

  /**
   * @brief 获取路径保持起始点
   * @details 返回当前设置的车道保持功能起始纵向坐标
   * @return double 起始s坐标（单位：米）
   * @par 更新流程图:
   * @startuml
   * start
   * :访问lane_keep_start_s_成员;
   * :返回double类型值;
   * stop
   * @enduml
   *
   * @note 车道保持起始点s坐标
   * @warning 需通过setLaneKeepStartS()设置有效值
   */
  double lane_keep_start_s() const { return lane_keep_start_s_; }

  /**
   * @brief 获取停止参考线位置
   * @details 返回当前设置的停止参考线s坐标
   * @return double 停止点s坐标（单位：米）
   * @par 更新流程图:
   * @startuml
   * start
   * :访问stop_reference_line_s_成员;
   * :返回double类型值;
   * stop
   * @enduml
   *
   * @note 停止参考线s坐标
   * @warning 需通过setStopReferenceLineS()设置有效值
   */
  double stop_reference_line_s() const { return stop_reference_line_s_; }

  /**
   * @brief 设置决策行为类型
   * @param[in] behavior_type 行为类型枚举值
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :更新behavior_type_;
   * stop
   * @enduml
   *
   * @warning 该函数仅用于设置行为类型，不涉及路径优化
   */
  void set_behavior_type(const BehaviorType& behavior_type) { behavior_type_ = behavior_type; };

  /**
   * @brief 获取当前决策行为类型
   * @details 返回当前生效的行为模式
   * @param[in] behavior_type 行为类型枚举值
   *
   * @return BehaviorType 当前生效的行为模式
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :返回behavior_type_;
   * stop
   * @enduml
   *
   * @note NONE表示常规驾驶模式
   */
  BehaviorType get_behavior_type() const { return behavior_type_; };

  /**
   * @brief 获取路径偏移量集合（只读）
   * @details 返回当前路径的横向偏移点集合
   * @return const RefOffsetsType& 路径偏移点集(s,l)的不可变引用
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :返回ref_offsets_;
   * stop
   * @enduml
   *
   * @warning 返回的引用在对象生命周期结束后失效
   */
  const RefOffsetsType& ref_offsets() const { return ref_offsets_; }

  /**
   * @brief 获取可修改的路径偏移量指针
   * @details 返回可用于直接修改路径偏移量集合的指针
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :返回&ref_offsets_;
   * stop
   * @enduml
   *
   * @return RefOffsetsType* 可修改指针
   * @warning 多线程访问需自行处理
   */
  RefOffsetsType* mutable_ref_offsets() { return &ref_offsets_; }

  /**
   * @brief 设置停车拉近标志
   * @details 控制是否启用停车位置微调功能
   * @param[in] pull_over_stop true: 启用停车位置微调
   * @par 更新流程图:
   * @startuml
   * start
   * :更新pull_over_stop_;
   * stop
   * @enduml
   *
   * @note 启用后将在停车位置附近进行微调
   */
  void set_pull_over_stop(const bool pull_over_stop) { pull_over_stop_ = pull_over_stop; }

  /**
   * @brief 获取停车拉近状态
   * @details 检查是否处于停车位置微调模式
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :返回pull_over_stop_;
   * stop
   * @enduml
   *
   * @return bool 是否处于停车位置微调模式
   * @note 默认返回false
   */
  bool pull_over_stop() const { return pull_over_stop_; }

  /**
   * @brief 启用逆向路径规划模式
   * @details 控制是否开启倒车路径生成功能，用于狭窄场景下的倒车避让
   *
   * @param[in] enable_backward_planning true: 启用逆向规划
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :更新enable_backward_planning_;
   * stop
   * @enduml
   *
   * @note 启用时需确保已设置有效的倒车参考线
   * @warning 倒车模式下需关闭自动加速功能
   */
  void set_enable_backward_planning(const bool enable_backward_planning) {
    enable_backward_planning_ = enable_backward_planning;
  }

  /**
   * @brief 获取逆向规划状态
   * @return bool 是否处于倒车路径生成模式
   * @par 更新流程图:
   * @startuml
   * start
   * :返回enable_backward_planning_;
   * stop
   * @enduml
   *
   * @note 默认返回false表示常规前向规划
   */
  bool enable_backward_planning() const { return enable_backward_planning_; }

  std::string debug_info{""};  ///< 调试信息输出缓冲区

 private:
  std::vector<IgnoreRangeInfo> ignore_range_;  ///< 忽略区域集合
  IndexedList<Obstacle> static_obstacles_;     ///< 静态障碍物容器
  IndexedList<Obstacle> dynamic_obstacles_;    ///< 动态障碍物容器
  IndexedList<Obstacle> risky_obstacles_;      ///< 高风险障碍物容器
  // DynamicObsIdInfoMap dynamic_obstacle_info_;    ///< [预留]动态障碍物追踪信息
  std::pair<bool, std::vector<PathBoundInfo>> max_allowed_bounds_;     ///< 最大允许边界
  double lane_keep_start_s_ = 0;                                       ///< 车道保持起点s坐标
  double stop_reference_line_s_ = std::numeric_limits<double>::max();  ///< 停止参考线位置
  BehaviorType behavior_type_ = BehaviorType::NONE;                    ///< 当前决策行为模式
  RefOffsetsType ref_offsets_;                                         ///< 路径偏移量集合
  bool pull_over_stop_ = false;                                        ///< 停车位置微调标志
  bool enable_backward_planning_ = false;                              ///< 逆向规划模式开关
};
}  // namespace gpal::pnc::planning
