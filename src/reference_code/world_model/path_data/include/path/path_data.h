/**
 * @file path_data.h
 * @brief 路径数据容器类定义
 * @details 提供路径数据容器类，用于存储和管理路径相关数据，包括离散路径、Frenet坐标系路径、路径边界约束等
 */

#pragma once

#include <list>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "discretized_path.h"
#include "frenet_frame_path.h"
#include "gpal-interface/planning/trajectory.pb.h"
#include "line_segments.h"
#include "math/line_segment2d.h"
#include "obstacle/obstacle.h"
#include "path_boundary.h"
#include "point/path_pt.h"
#include "reference_line/reference_line.h"

namespace gpal::pnc::planning {

/**
 * @class PathData
 * @brief 路径数据容器类
 * @details 管理以下核心数据：
 * - 离散路径点集合
 * - Frenet坐标系路径
 * - 路径边界约束
 * - 规划状态与调试信息
 *
 * @par 典型数据流:
 * @startuml
 * start
 * :生成原始路径;
 * :坐标转换处理;
 * if (Frenet转换?) then (是)
 *   :存储Frenet路径;
 * else (否)
 *   :存储笛卡尔路径;
 * endif
 * :添加边界约束;
 * :记录规划状态;
 * stop
 * @enduml
 */
class PathData {
 public:
  /**
   * @enum DebugStatusType
   * @brief 路径规划调试状态枚举
   * @details 记录规划过程中不同阶段的异常状态，用于问题诊断和状态跟踪
   */
  enum class DebugStatusType {
    REGULAR = 1,             ///< 常规运行状态
    LANE_CHANGE = 2,         ///< 换道过程激活
    FORCE_BACK = 3,          ///< 强制倒车模式
    LANE_CHANGE_RETURN = 4,  ///< 换道返回状态
    REF_OFFSETS = 5,         ///< 参考线偏移异常

    // 路径边界阻塞相关 (11-19)
    BLOCK_FS_BOUND = 11,                      ///< 路径边界约束导致阻塞
    BLOCK_FS_POST = 12,                       ///< 后处理阶段路径阻塞
    BLOCK_FS_LOCAL_PATH = 13,                 ///< 局部路径生成阻塞
    RISKY_FS_POST = 14,                       ///< 风险路径后处理警告
    RISKY_FS_LOCAL_PATH = 15,                 ///< 风险局部路径警告
    BLOCK_STATIC_OBSTACLE = 16,               ///< 静态障碍物阻塞
    BLOCK_DECIDE_BOUND = 17,                  ///< 决策边界阻塞
    BLOCK_DRIVABLE_BOUNDARY_POST = 18,        ///< 可行域后处理阻塞
    BLOCK_DRIVABLE_BOUNDARY_LOCAL_PATH = 19,  ///< 可行域局部路径阻塞

    // 求解器异常相关 (21-30)
    IGNORE_RANGE_NUDGE = 21,           ///< 忽略横向偏移范围
    IGNORE_RANGE_REF = 22,             ///< 忽略参考线范围
    REF_NULL = 23,                     ///< 参考线为空
    NUDGE_DISABLED_REF = 24,           ///< 参考线横向偏移禁用
    SOLVER_FAILED_REFINE = 25,         ///< 轨迹优化求解失败
    SOLVER_FAILED_REF = 26,            ///< 参考线求解失败
    SOLVER_FAILED = 27,                ///< 通用求解器失败
    SOLVER_FAILED_UNCONSTRAINED = 28,  ///< 无约束求解失败
    SOLVER_UNDERLOCK_REFINE = 29,      ///< 优化器欠锁定
    SOLVER_WRONG_STEER_ANGEL = 30,     ///< 方向盘角度计算异常

    // 边界约束相关 (31-40)
    MAX_BOUND_EMPTY = 31,                   ///< 最大边界为空
    MAX_BOUND_CLEARED = 32,                 ///< 最大边界已清除
    REF_CHANGED = 33,                       ///< 参考线变更
    PREV_PATH_INVALID = 34,                 ///< 前序路径无效
    PREV_PATH_TOO_SHORT = 35,               ///< 前序路径过短
    PREV_PATH_TOO_FAR_REFINE_FAILED = 36,   ///< 前序路径过远优化失败
    NEAR_DEST_SOLVER_FAILED = 37,           ///< 接近终点求解失败
    NO_FS_NURGE_ON_BRIDGE = 38,             ///< 桥上禁止横向偏移
    LANE_CHANGE_SOLVER_FAILED_REFINE = 39,  ///< 换道轨迹优化失败
    SOLVED_PATH_VALIDITY_CHECK = 40,        ///< 求解路径有效性检查

    // 绕行相关状态 (41-49)
    DETOUR = 41,                      ///< 绕行模式激活
    DETOUR_BACK = 42,                 ///< 绕行返回状态
    DETOUR_BACK_GATE = 43,            ///< 绕行返回闸道
    DETOUR_BACK_INTENTION = 44,       ///< 绕行返回意图
    DETOUR_REFINE = 45,               ///< 绕行轨迹优化
    DETOUR_FALLBACK = 46,             ///< 绕行降级处理
    DETOUR_FALLBACK_FAILED = 47,      ///< 绕行降级失败
    DETOUR_INIT_PATH_TOO_SHORT = 48,  ///< 绕行初始路径过短
    DETOUR_OBS_CHANGE = 49,           ///< 绕行障碍物变化

    // 轨迹优化相关 (50-63)
    OCP_SIZE_SMALL = 50,      ///< 规划轨迹点数不足（至少需要5个点）
    POST_CHECK_INVALID = 51,  ///< 后验证检查失败
    REF_RAW = 60,             ///< 参考线原始数据异常
    REF_OCP = 61,             ///< 参考线最优控制问题求解成功
    REF_DISCRETE_POINT = 62,  ///< 参考线离散点优化成功
    REF_MODIFY = 63,          ///< 参考线动态修改

    // 车辆模型相关 (81-83)
    MODEL_LATERAL_TRACTOR_TRAILER = 81,  ///< 拖挂车横向模型
    MODEL_LATERAL_SIMPLE = 82,           ///< 简单横向模型
    MODEL_LATERAL_GENERAL = 83,          ///< 通用横向模型

    // 求解器模式 (90-91)
    SOLVER_UNCONSTRAINED = 90,  ///< 无约束求解模式
    SOLVER_PROTECT = 91         ///< 保护模式求解
  };

  /**
   * @enum ParkDebugStatusType
   * @brief 泊车调试状态枚举
   * @details 记录自动泊车过程中的关键状态节点
   */
  enum class ParkDebugStatusType {
    PARKIN = 1,                ///< 泊入过程激活
    PARKOUT = 2,               ///< 泊出过程激活
    PARKOUT_LEFT = 8,          ///< 左侧泊出尝试
    PARKOUT_RIGHT = 9,         ///< 右侧泊出尝试
    PARKOUT_PLAN_FAILED = 15,  ///< 泊出路径规划失败
    PARKIN_FINISHED = 25,      ///< 泊入完成
    PARKOUT_FINISHED = 26      ///< 泊出完成
  };

  /**
   * @enum StatusType
   * @brief 路径规划状态枚举
   * @details 表示路径规划过程的整体执行状态
   */
  enum class StatusType {
    INVALID = 0,   ///< 无效状态（未初始化）
    RUNNING = 1,   ///< 运行中状态
    FAILED = 2,    ///< 规划失败状态
    FINISHED = 3,  ///< 规划完成状态
    HOLD_ON = 4,   ///< 保持状态（规划完成但仍运行状态）
    CAL_SUCCESS = 5, ///< 计算成功状态
  };

  /**
   * @enum PathType
   * @brief 路径类型枚举
   * @details 区分不同层级的路径规划结果
   */
  enum class PathType {
    NONE = 0,     ///< 无有效路径
    LATERAL = 1,  ///< 横向规划路径
    LOCAL = 2     ///< 局部规划路径
  };

  /**
   * @enum BlockPointDirection
   * @brief 阻塞点方向枚举
   * @details 标识路径阻塞点的相对方向
   */
  enum class BlockPointDirection {
    NONE = 0,  ///< 无明确方向
    LEFT = 1,  ///< 左侧阻塞
    RIGHT = 2  ///< 右侧阻塞
  };

  /**
   * @struct BlockFSInfo
   * @brief 路径阻挡点详细信息结构体
   * @details 记录导致路径规划失败的阻挡点空间信息及关联参数
   *
   */
  struct BlockFSInfo {
    PathType path_type = PathType::NONE;            ///< 路径类型标识 (LATERAL/LOCAL)
    double s = std::numeric_limits<double>::max();  ///< 阻挡点s坐标（沿路径方向），范围[0, path_length]，单位：米
    bool is_valid = false;                          ///< 数据有效性标志，true表示有效阻挡点
    PathPt check_point;                             ///< 路径检查点信息（包含x,y,theta等属性）
    math::Vec2d block_point;                        ///< 阻挡点笛卡尔坐标（世界坐标系）
    BlockPointDirection block_point_direction = BlockPointDirection::NONE;  ///< 阻挡点相对方向
    std::vector<math::Vec2d> vis_pts;                                       ///< 可视化点集合（用于调试显示）

    /**
     * @brief 清空阻挡点信息
     * @note 在以下情况必须调用：
     * 1. 路径规划重新开始前
     * 2. 收到新的感知数据时
     *
     * @par 更新流程图:
     * @startuml
     * start
     * :初始化数据;
     * stop
     * @enduml
     *
     * @warning 必须确保:
     * 1. 清空后数据状态为默认值
     * 2. 后续调用需重新设置有效信息
     */
    void clear() {
      path_type = PathType::NONE;
      s = std::numeric_limits<double>::max();
      is_valid = false;
      block_point_direction = BlockPointDirection::NONE;
      vis_pts.clear();
    }
  };

  /**
   * @struct OcpPathInfo
   * @brief 最优控制问题路径信息结构体
   * @details 存储轨迹优化过程中的松弛变量及障碍物关联信息
   *
   */
  struct OcpPathInfo {
    int ind;                              ///< 路径点索引号
    float lateral_dis = 0.0;              ///< 横向参考线偏移量
    float slack_trailer_front = 0.0;      ///< 拖车前部松弛变量
    float slack_trailer_rear_axis = 0.0;  ///< 拖车后轴松弛变量
    float slack_trailer_back = 0.0;       ///< 拖车尾部松弛变量
    std::string ob_id_left;               ///< 左侧关联障碍物ID
    std::string ob_id_right;              ///< 右侧关联障碍物ID
  };

  /**
   * @struct BoundsVec3d
   * @brief 三维边界集合结构体
   * @details 存储路径规划中的多类型三维边界信息
   *
   */
  struct BoundsVec3d {
    std::vector<math::Vec3d> barrier_bound_right;  ///< 右侧硬边界点集
    std::vector<math::Vec3d> barrier_bound_left;   ///< 左侧硬边界点集
    std::vector<math::Vec3d> soft_bound_right;     ///< 右侧软约束点集
    std::vector<math::Vec3d> soft_bound_left;      ///< 左侧软约束点集

    /**
     * @brief 清空所有边界数据
     * @details 清除所有边界点集，重置为默认状态
     * @par 更新流程图:
     * @startuml
     * start
     * :清空所有边界点集;
     * stop
     * @enduml
     * @warning 必须确保:
     * 1. 清空后所有边界集合为空
     * 2. 后续调用需重新设置有效边界信息
     */
    void clear() {
      barrier_bound_right.clear();
      barrier_bound_left.clear();
      soft_bound_right.clear();
      soft_bound_left.clear();
    }
  };

  /**
   * @struct BoundsVec3dWithId
   * @brief 带标识的三维边界集合
   * @details 扩展边界集合结构，支持多组边界数据标识管理
   *
   * @par 主要应用场景:
   * - 多障碍物独立边界存储
   * - 动态边界版本管理
   */
  struct BoundsVec3dWithId {
    std::string id = "";                           ///< 边界集合唯一标识符
    std::vector<math::Vec3d> barrier_bound_right;  ///< 右侧硬边界点集
    std::vector<math::Vec3d> barrier_bound_left;   ///< 左侧硬边界点集
    std::vector<math::Vec3d> soft_bound_right;     ///< 右侧软约束点集
    std::vector<math::Vec3d> soft_bound_left;      ///< 左侧软约束点集

    /**
     * @brief 清空边界数据及标识
     * @details 重置边界集合和标识符，准备接受新的边界信息
     * @par 更新流程图:
     * @startuml
     * start
     * :清空边界集合;
     * :重置标识符;
     * stop
     * @enduml
     *
     * @warning 必须确保:
     * 1. 清空后所有边界集合为空
     * 2. 标识符重置为空字符串
     * 3. 后续调用需重新设置有效边界信息
     */
    void clear() {
      id.clear();
      barrier_bound_right.clear();
      barrier_bound_left.clear();
      soft_bound_right.clear();
      soft_bound_left.clear();
    }
  };

  /**
   * @brief PathData构造函数
   * @details 初始化路径数据对象，设置默认值
   */
  PathData() = default;

 public:
  void clear();

  /**
   * @brief 设置参考线对象
   * @param[in] reference_line 参考线指针
   * @return bool 设置成功状态
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :接收参考线输入;
   * :更新reference_line_成员;
   * end
   * @enduml
   *
   * @warning 调用此方法后，路径数据将被覆盖
   */
  bool setReferenceLine(const ReferenceLine* reference_line) noexcept {
    reference_line_ = reference_line;
    return true;
  }

  bool setFrenetPath(FrenetFramePath frenet_path);

  /**
   * @brief 设置离散路径
   * @param[in] discretized_path 离散路径数据
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :接收离散路径输入;
   * :更新discretized_path_成员;
   * stop
   * @enduml
   *
   * @warning 调用此方法后，路径数据将被覆盖
   */
  void setDiscretizedPath(const DiscretizedPath& discretized_path) { discretized_path_ = discretized_path; }

  /**
   * @brief 设置局部路径数据
   * @param[in] local_path 局部路径对象
   *
   * @par 参数要求:
   * - 必须经过平滑处理
   * - 路径点间距≤0.2米
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :接收局部路径输入;
   * :更新local_path_成员;
   * stop
   * @enduml
   *
   * @warning 调用此方法后，路径数据将被覆盖
   */
  void setLocalPath(const DiscretizedPath& local_path) { local_path_ = local_path; }

  /**
   * @brief 设置路径边界约束
   * @param[in] bounds 边界线段集合
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :接收边界线段输入;
   * :更新bounds_成员;
   * stop
   * @enduml
   * @note 边界线段必须按s升序排列
   *
   * @warning 调用此方法后，路径数据将被覆盖
   */
  void setBounds(const std::vector<LineSegments>& bounds) { bounds_ = bounds; }

  /**
   * @brief 设置路径标签标识
   * @details 更新路径的分类标签，用于标识路径的生成场景
   * @param[in] label 路径分类标签
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :接收标签输入;
   * :更新path_label_成员;
   * stop
   * @enduml
   *
   * @warning 输入约束:
   * - 大小写敏感
   * - 禁止使用空字符串
   */
  void setPathLabel(const std::string& label) { path_label_ = label; };

  /**
   * @brief 获取当前路径标签
   * @details 返回路径边界的类型标签，用于判断当前路径的生成场景
   * @return const std::string& 路径标签常量引用
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :访问path_label_成员;
   * :返回常量引用;
   * stop
   * @enduml
   *
   * @warning 使用限制:
   * - 需确保对象生命周期有效
   * - 禁止通过引用修改值
   */
  const std::string& path_label() const { return path_label_; }

  /**
   * @brief 设置主阻塞障碍物ID
   * @param[in] obs_id 障碍物唯一标识符
   *
   * @par 参数规范:
   * - 关联障碍物必须存在于当前感知列表
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :接收障碍物ID输入;
   * :更新blocking_obstacle_id_;
   * endif
   * stop
   * @enduml
   *
   * @warning 调用此方法后，路径数据将被覆盖
   */
  void setBlockingObstacleId(const std::string& obs_id) { blocking_obstacle_id_ = obs_id; }

  /**
   * @brief 获取当前阻塞障碍物ID
   * @return const std::string& 障碍物ID常量引用
   * @par 更新流程图:
   * @startuml
   * start
   * :访问blocking_obstacle_id_成员;
   * :返回常量引用;
   * stop
   * @enduml
   *
   * @note 在多线程场景下需加锁访问
   */
  const std::string& blocking_obstacle_id() const { return blocking_obstacle_id_; }

  /**
   * @brief 获取路径阻塞信息集合
   * @return const std::vector<BlockFSInfo>& 阻塞信息常量引用
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :访问block_fs_info_成员;
   * :返回常量引用;
   * stop
   * @enduml
   *
   * @warning 禁止通过返回的引用修改数据内容
   */
  const std::vector<BlockFSInfo>& blockFSInfo() const { return block_fs_info_; }

  /**
   * @brief 获取可修改的阻塞信息指针
   * @return std::vector<BlockFSInfo>* 阻塞信息向量指针
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :访问block_fs_info_成员;
   * :返回可修改指针;
   * stop
   * @enduml
   *
   * @warning 需确保调用者持有有效指针
   * @note 调用此方法后，路径数据将被覆盖
   */
  std::vector<BlockFSInfo>* mutableBlockFSInfo() { return &block_fs_info_; }

  /**
   * @brief 获取离散化路径数据（只读访问）
   * @details 返回离散化路径数据的常量引用
   * @return const DiscretizedPath& 路径数据常量引用
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :请求路径数据;
   * :返回有效引用;
   * stop
   * @enduml
   *
   * @warning 返回引用的生命周期与PathData对象绑定
   */
  const DiscretizedPath& discretized_path() const { return discretized_path_; }

  /**
   * @brief 获取可修改的离散路径指针
   * @details 返回离散化路径数据的可修改指针
   * @return DiscretizedPath* 路径对象指针
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :请求路径数据;
   * :返回有效指针;
   * stop
   * @enduml
   *
   * @warning 调用者需确保指针有效
   * @note 调用此方法后，路径数据将被覆盖
   */
  DiscretizedPath* mutableDiscretizedPath() { return &discretized_path_; }

  /**
   * @brief 获取局部路径数据（只读访问）
   * @details 返回局部路径数据的常量引用
   * @return const DiscretizedPath& 局部路径常量引用
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :请求局部路径数据;
   * :返回有效引用;
   * stop
   * @enduml
   *
   * @warning 返回路径可能为空
   *
   */
  const DiscretizedPath& local_path() const { return local_path_; }

  /**
   * @brief 获取可修改的局部路径指针
   * @details 返回局部路径数据的可修改指针
   * @return DiscretizedPath* 局部路径对象指针
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :请求局部路径数据;
   * :返回有效指针;
   * stop
   * @enduml
   *
   * @warning 调用者需确保指针有效
   */
  DiscretizedPath* mutableLocalPath() { return &local_path_; }

  /**
   * @brief 获取Frenet坐标系路径（只读访问）
   * @details 返回Frenet坐标系路径的常量引用
   * @return const FrenetFramePath& Frenet路径常量引用
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :请求Frenet路径数据;
   * :返回有效引用;
   * stop
   * @enduml
   *
   * @warning 返回路径可能为空
   */
  const FrenetFramePath& frenet_path() const { return frenet_path_; }

  /**
   * @brief 获取路径边界约束集合
   * @details 返回路径边界约束的常量引用
   * @return const std::vector<LineSegments>& 边界线段集合常量引用
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :请求边界约束;
   * :返回有效引用;
   * stop
   * @enduml
   *
   * @warning 返回数据可能为空（未设置边界时）
   */
  const std::vector<LineSegments> bounds() const { return bounds_; }

  /**
   * @brief 获取全量泊车路径集合（只读访问）
   * @details 返回全量泊车路径的常量引用
   * @return const vector<vector<PathPt>>& 二维路径点集合常量引用
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :请求全量泊车路径数据;
   * :返回有效引用;
   * stop
   * @enduml
   *
   * @warning 返回数据可能为空（未生成泊车路径时）
   */
  const vector<vector<PathPt>>& park_full_discretized_path() const { return park_full_discretized_path_; }

  /**
   * @brief 获取可修改的泊车路径指针
   * @return vector<vector<PathPt>>* 二维路径点集合指针
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :请求全量泊车路径数据;
   * :return有效指针;
   * stop
   * @enduml
   *
   * @warning 调用者需确保指针有效
   */
  vector<vector<PathPt>>* mutableParkFullDiscretizedPath() { return &park_full_discretized_path_; }

  /**
   * @brief 设置规划器状态
   * @details 更新路径规划器的当前状态
   * @param[in] status 规划器状态枚举值
   *
   * @par 状态转换流程图:
   * @startuml
   * start
   * :接收状态输入;
   * :更新planner_status_;
   * stop
   * @enduml
   *
   * @warning 调用此方法后，路径数据将被覆盖
   */
  void setPlannerStatus(const StatusType& status) { planner_status_ = status; }

  /**
   * @brief 获取当前规划器状态
   * @return StatusType 规划器状态枚举值
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :访问planner_status_成员;
   * :返回当前状态;
   * stop
   * @enduml
   *
   * @warning 调用此方法后，路径数据将被覆盖
   *
   */
  StatusType getPlannerStatus() { return planner_status_; }

  /**
   * @brief 获取可修改的调试信息指针
   * @details 返回调试信息字符串的可修改指针
   * @return std::string* 调试信息字符串指针
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :请求调试信息;
   * :返回可修改指针;
   * stop
   * @enduml
   *
   * @warning 需确保调用者持有有效指针
   * @note 调用此方法后，路径数据将被覆盖
   */
  std::string* mutableDebugInfo() { return &debug_info_; }

  /**
   * @brief 获取调试信息（只读访问）
   * @return const std::string& 调试信息常量引用
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :请求调试信息;
   * :返回有效引用;
   * stop
   * @enduml
   *
   * @warning 返回引用的生命周期与PathData对象绑定
   */
  const std::string& debugInfo() const { return debug_info_; }

  /**
   * @brief 获取可修改的局部路径调试信息指针
   * @details 返回局部路径调试信息的可修改指针
   * @return std::string* 调试信息字符串指针
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :请求局部路径调试信息;
   * :返回可修改指针;
   * stop
   * @enduml
   *
   * @warning 需确保调用者持有有效指针
   * @note 调用此方法后，路径数据将被覆盖
   */
  std::string* mutableLocalPathDebugInfo() { return &local_path_debug_info_; }

  /**
   * @brief 获取局部路径调试信息（只读访问）
   * @details 返回局部路径调试信息的常量引用
   * @return const std::string& 调试信息常量引用
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :请求局部路径调试信息;
   * :返回有效引用;
   * stop
   * @enduml
   *
   * @warning 返回引用的生命周期与PathData对象绑定
   */
  const std::string& localPathDebugInfo() const { return local_path_debug_info_; }

  /**
   * @brief 获取可修改的泊车调试信息集合指针
   * @details 返回泊车调试信息集合的可修改指针
   * @return std::unordered_set<std::string>* 调试信息哈希集合指针
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :请求泊车调试信息;
   * :返回可修改指针;
   * stop
   * @enduml
   *
   * @warning 需确保调用者持有有效指针
   * @note 调用此方法后，路径数据将被覆盖
   *
   */
  std::unordered_set<std::string>* mutableParkDebugInfo() { return &park_debug_info_; }

  /**
   * @brief 获取泊车调试信息集合（只读访问）
   * @details 返回泊车调试信息集合的常量引用
   * @return const std::unordered_set<std::string>& 调试信息哈希集合常量引用
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :请求泊车调试信息;
   * :返回有效引用;
   * stop
   * @enduml
   *
   * @warning 返回引用的生命周期与PathData对象绑定
   * @note 调用此方法后，路径数据将被覆盖
   */
  const std::unordered_set<std::string>& parkDebugInfo() const { return park_debug_info_; }

  /**
   * @brief 获取可修改的规划器调试状态集合指针
   * @details 返回规划器调试状态集合的可修改指针
   * @return std::vector<DebugStatusType>* 调试状态向量指针
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :请求状态修改权限;
   * :返回可写指针;
   * stop
   * @enduml
   *
   * @warning 需确保调用者持有有效指针
   * @note 调用此方法后，路径数据将被覆盖
   */
  std::vector<DebugStatusType>* mutablePlannerDebugStatus() { return &planner_debug_status_; }

  /**
   * @brief 获取规划器调试状态集合（只读访问）
   * @details 返回规划器调试状态集合的常量引用
   * @return const std::vector<DebugStatusType>& 调试状态常量引用
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :请求规划器调试状态;
   * :返回有效引用;
   * stop
   * @enduml
   *
   * @note 用于规划器性能分析和异常回溯
   * @warning 返回引用的生命周期与PathData对象绑定
   */
  const std::vector<DebugStatusType>& plannerDebugStatus() const { return planner_debug_status_; }

  /**
   * @brief 获取可修改的泊车调试状态集合指针
   * @details 返回泊车调试状态集合的可修改指针
   * @return std::unordered_set<ParkDebugStatusType>* 调试状态哈希集合指针
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :请求泊车调试状态修改权限;
   * :返回可写指针;
   * stop
   * @enduml
   *
   * @note 用于泊车状态的动态更新和监控
   * @warning 需确保调用者持有有效指针
   */
  std::unordered_set<ParkDebugStatusType>* mutableParkDebugStatus() { return &park_debug_status_; }

  /**
   * @brief 获取泊车调试状态集合（只读访问）
   * @return const std::unordered_set<ParkDebugStatusType>& 调试状态常量引用
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :请求泊车调试状态;
   * :返回有效引用;
   * stop
   * @enduml
   *
   * @note 用于泊车状态的动态更新和监控
   * @warning 返回引用的生命周期与PathData对象绑定
   */
  const std::unordered_set<ParkDebugStatusType>& parkDebugStatus() const { return park_debug_status_; }

  /**
   * @brief 设置剩余距离信息
   * @details 更新剩余距离信息，包含有效性标志和距离值
   * @param[in] remain_dis_info 包含有效性标志和距离值的pair对象
   *

   * @par 更新流程图:
   * @startuml
   * start
   * :接收剩余距离输入;
   * :更新remain_dis_info_;
   * stop
   * @enduml
   *
   * @warning 调用此方法后，路径数据将被覆盖
   */
  void setRemainDisInfo(std::pair<bool, double>& remain_dis_info) { remain_dis_info_ = remain_dis_info; }

  /**
   * @brief 设置剩余距离信息
   * @details 更新剩余距离信息，包含有效性标志和距离值
   * @param[in] remain_dis_info 包含有效性标志和距离值的pair对象
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :接收剩余距离输入;
   * :更新remain_dis_info_;
   * stop
   * @enduml
   *
   * @warning 负值输入将自动重置为默认无效状态
   */
  const std::pair<bool, double> getRemainDisInfo() const { return remain_dis_info_; }

  /**
   * @brief 设置剩余轨迹段数
   * @details 更新剩余轨迹段数
   * @param[in] num 剩余轨迹段数
   *

   * @par 更新流程图:
   * @startuml
   * start
   * :接收剩余轨迹段数输入;
   * :更新remain_traj_num_;
   * stop
   * @enduml
   *
   * @warning 调用此方法后，路径数据将被覆盖
   */
  void setRemainTrajNum(int num) { remain_traj_num_ = num; }

  /**
   * @brief 获取剩余轨迹段数
   * @details 更新剩余轨迹段数
   * @param[in] num 剩余轨迹段数
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :接收剩余轨迹段数输入;
   * :更新remain_traj_num_;
   * stop
   * @enduml
   *
   * @warning 负值输入将自动重置为默认无效状态
   */
  const int getRemainTrajNum() const { return remain_traj_num_; }

  /**
   * @brief 获取可修改的OCP路径信息指针
   * @return std::vector<OcpPathInfo>* OCP路径向量指针
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :请求OCP路径数据;
   * :返回可修改指针;
   * stop
   * @enduml
   *
   * @warning 需确保调用者持有有效指针
   * @note 调用此方法后，路径数据将被覆盖
   */
  std::vector<OcpPathInfo>* mutableOcpPathInfo() { return &ocp_path_info_; }

  /**
   * @brief 获取OCP路径信息（只读访问）
   * @details 返回OCP路径信息的常量引用
   * @return const std::vector<OcpPathInfo>& OCP路径常量引用
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :请求OCP路径数据;
   * :返回有效引用;
   * stop
   * @enduml
   * @warning 返回引用的生命周期与PathData对象绑定
   * @note 用于OCP路径的动态更新和监控
   */
  const std::vector<OcpPathInfo>& ocpPathInfo() const { return ocp_path_info_; }

  /**
   * @brief 获取可修改的三维边界数据指针
   * @details 返回三维边界数据的可修改指针
   * @return BoundsVec3d* 三维边界数据指针
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :接收新边界数据;
   * :更新bounds_vec3d_;
   * stop
   * @enduml
   *
   * @warning 需确保调用者持有有效指针
   * @note 调用此方法后，路径数据将被覆盖
   */
  BoundsVec3d* mutableBoundsVec3d() { return &bounds_vec3d_; }

  /**
   * @brief 获取三维边界数据（只读访问）
   * @return const BoundsVec3d& 三维边界常量引用
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :请求三维边界数据;
   * :返回有效引用;
   * stop
   * @enduml
   *
   * @warning 返回引用的生命周期与PathData对象绑定
   * @note 用于三维边界的动态更新和监控
   */
  const BoundsVec3d& boundsVec3d() const { return bounds_vec3d_; }

  /**
   * @brief 获取可修改的冗余边界数据指针
   * @details 返回冗余边界数据的可修改指针
   * @return BoundsVec3d* 冗余边界数据指针
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :接收新边界数据;
   * :更新redundant_bounds_vec3d_;
   * stop
   * @enduml
   *
   * @warning 需确保调用者持有有效指针
   * @note 调用此方法后，路径数据将被覆盖
   */
  BoundsVec3d* mutableRedundantBoundsVec3d() { return &redundant_bounds_vec3d_; }
  /**
   * @brief 获取冗余边界数据（只读访问）
   * @details 返回冗余边界数据的常量引用
   * @return const BoundsVec3 & 冗余边界常量引用
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :请求冗余边界数据;
   * :返回有效引用;
   * stop
   * @enduml
   *
   * @warning 返回引用的生命周期与PathData对象绑定
   * @note 用于冗余边界的动态更新和监控
   */
  const BoundsVec3d& redundantBoundsVec3d() const { return redundant_bounds_vec3d_; }

  /**
   * @brief 获取可修改的决策边界数据指针
   * @details 返回决策边界数据的可修改指针
   * @return BoundsVec3d* 决策边界数据指针
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :接收新边界数据;
   * :更新decision_bounds_vec3d_;
   * stop
   * @enduml
   *
   * @warning 需确保调用者持有有效指针
   * @note 调用此方法后，路径数据将被覆盖
   */
  BoundsVec3d* mutableDesicionBoundsVec3d() { return &decision_bounds_vec3d_; }

  /**
   * @brief 获取决策边界数据（只读访问）
   * @details 返回决策边界数据的常量引用
   * @return const BoundsVec3d& 决策边界常量引用
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :请求决策边界数据;
   * :返回有效引用;
   * stop
   * @enduml
   *
   * @warning 返回引用的生命周期与PathData对象绑定
   * @note 用于决策边界的动态更新和监控
   */
  const BoundsVec3d& decisionBoundsVec3d() const { return decision_bounds_vec3d_; }

  /**
   * @brief 获取带ID的三维边界集合指针
   * @details 返回可修改的三维边界集合指针
   * @return std::vector<BoundsVec3dWithId>* 边界集合指针
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :接收新边界数据;
   * :更新bounds_vec3d_with_id_;
   * stop
   * @enduml
   * @warning 需确保调用者持有有效指针
   * @note 调用此方法后，路径数据将被覆盖
   *
   */
  std::vector<BoundsVec3dWithId>* mutableBoundsVec3dWithId() { return &bounds_vec3d_with_id_; }

  /**
   * @brief 获取带ID的三维边界集合（只读访问）
   * @details 返回三维边界集合的常量引用
   * @return const std::vector<BoundsVec3dWithId>& 边界集合常量引用
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :请求三维边界数据;
   * :返回有效引用;
   * stop
   * @enduml
   *
   * @warning 返回引用的生命周期与PathData对象绑定
   * @note 用于三维边界的动态更新和监控
   */
  const std::vector<BoundsVec3dWithId>& boundsVec3dWithId() const { return bounds_vec3d_with_id_; }

  /**
   * @brief 获取可修改的路径硬边界信息指针
   * @details 返回路径硬边界信息的可修改指针
   * @return std::vector<PathBoundary::PathBoundaryUnitInfo>* 边界单元信息向量指针
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :更新path_barrier_boundary_info_;
   * stop
   * @enduml
   *
   * @warning 需确保调用者持有有效指针
   * @note 调用此方法后，路径数据将被覆盖
   */
  std::vector<PathBoundary::PathBoundaryUnitInfo>* mutablePathBarrierBoundaryInfo() {
    return &path_barrier_boundary_info_;
  }

  /**
   * @brief 获取路径硬边界信息（只读访问）
   * @details 返回路径硬边界信息的常量引用
   * @return const std::vector<PathBoundary::PathBoundaryUnitInfo>& 边界单元信息常量引用
   *
   * @par 更新流程图:
   * @startuml
   * start
   * :请求路径硬边界信息;
   * :返回有效引用;
   * stop
   * @enduml
   *
   * @warning 返回引用的生命周期与PathData对象绑定
   * @note 用于路径硬边界的动态更新和监控
   */
  const std::vector<PathBoundary::PathBoundaryUnitInfo>& pathBarrierBoundaryInfo() const {
    return path_barrier_boundary_info_;
  }

 private:
  bool sl2xy(const FrenetFramePath& frenet_path, DiscretizedPath* const discretized_path);
  bool xy2sl(const DiscretizedPath& discretized_path, FrenetFramePath* const frenet_path);

  const ReferenceLine* reference_line_ = nullptr;                               ///< 参考线对象指针
  DiscretizedPath discretized_path_;                                            ///< 离散化路径点集合
  DiscretizedPath local_path_;                                                  ///< 局部路径点集合
  FrenetFramePath frenet_path_;                                                 ///< Frenet坐标系路径
  std::vector<vector<PathPt>> park_full_discretized_path_;                      ///< 二维路径点集合
  std::vector<LineSegments> bounds_;                                            ///< 二维边界数据
  BoundsVec3d bounds_vec3d_;                                                    ///< 三维边界数据
  BoundsVec3d redundant_bounds_vec3d_;                                          ///< 冗余边界数据
  BoundsVec3d decision_bounds_vec3d_;                                           ///< 决策边界数据
  std::vector<OcpPathInfo> ocp_path_info_;                                      ///< OCP路径信息集合
  std::vector<BoundsVec3dWithId> bounds_vec3d_with_id_;                         ///< 带ID的三维边界集合
  std::vector<PathBoundary::PathBoundaryUnitInfo> path_barrier_boundary_info_;  ///< 实时横向硬边界信息
  std::pair<bool, double> remain_dis_info_ = std::make_pair(false, 1e4);        ///< 剩余距离信息
  int remain_traj_num_ = 1;                                                     ///< 剩余轨迹段数目（含当前轨迹）
  std::string path_label_ = "";                                                 ///< 路径标签标识
  std::string blocking_obstacle_id_ = "";                                       ///< 阻塞障碍物ID
  std::vector<BlockFSInfo> block_fs_info_;                                      ///< 阻塞障碍物信息集合
  StatusType planner_status_ = StatusType::INVALID;                             ///< 规划器状态
  std::vector<DebugStatusType> planner_debug_status_;                           ///< 规划器调试状态集合
  std::unordered_set<ParkDebugStatusType> park_debug_status_;                   ///< 泊车调试状态集合
  std::string debug_info_ = "";                                                 ///< 调试信息字符串
  std::string local_path_debug_info_ = "";                                      ///< 局部路径调试信息
  std::unordered_set<std::string> park_debug_info_;                             ///< 泊车调试信息集合

 public:
  bool is_park_out_finished_ = false;  ///< 泊出流程完成标志
};
}  // namespace gpal::pnc::planning
