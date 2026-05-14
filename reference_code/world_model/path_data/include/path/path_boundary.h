/**
 * @file path_boundary.h
 * @brief 路径边界数据容器
 * @details 管理路径规划中多种类型的横向边界信息，包含道路边界、障碍物边界等
 */

#ifndef PATH_BOUNDARY_H
#define PATH_BOUNDARY_H

#pragma once

#include <limits>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
// include <obstacle/obstacle.h>

namespace gpal::pnc::planning {
/**
 * @brief 路径边界数据容器
 * @details 管理路径规划中多种类型的横向边界信息，包含道路边界、障碍物边界等
 * @note 该类主要用于路径优化模块的边界约束处理
 *
 */
class PathBoundary {
 public:
  /**
   * @brief 障碍物关联信息结构
   * @details 存储与障碍物相关的ID信息，用于关联障碍物边界
   */
  struct ObstacleInfo {
    std::string ob_id_left = "";   ///< 左侧关联障碍物ID
    std::string ob_id_right = "";  ///< 右侧关联障碍物ID
  };

  /**
   * @brief 边界单元类型枚举
   * @details 定义路径上某一点的边界类型，包括决策边界、车道线、物理不可逾越等
   */
  enum class BoundaryUnitTypeInfo {
    INVALID = 0,                ///< 未赋值
    DECISION = 1,               ///< 决策边界
    DASHED_LANE_LINE = 2,       ///< 车道虚线
    SOLID_LANE_LINE = 3,        ///< 车道实线
    PHYSICALLY_IMPASSABLE = 4,  ///< 物理不可逾越
    STATIC_OD = 10,             ///< 障碍物边界
    DYNAMIC_OD = 11,            ///< 动态障碍物边界
    FS = 12,                    ///< FS边界
  };

  /**
   * @brief 路径边界单元数据结构
   * @details 存储路径上某一点的横向边界信息，包含左右边界偏移量及边界类型
   *
   * @note 典型构造过程:
   * 1. 从感知模块获取原始边界数据
   * 2. 转换坐标系到Frenet框架
   * 3. 标注边界类型属性
   *
   * @warning 必须确保:
   * 1. 左右边界值符合物理约束
   * 2. 边界类型与实际场景一致
   */
  struct PathBoundaryUnitInfo {
    double s = 0.0;                                                     ///< 纵向坐标
    double l_left = 0.0;                                                ///< 左侧横向偏移量
    double l_right = 0.0;                                               ///< 右侧横向偏移量
    BoundaryUnitTypeInfo l_left_type = BoundaryUnitTypeInfo::INVALID;   ///< 左侧边界类型
    BoundaryUnitTypeInfo l_right_type = BoundaryUnitTypeInfo::INVALID;  ///< 右侧边界类型

    /**
     * @brief 构造函数
     * @param[in] s 纵向坐标
     * @param[in] l_left 左侧横向偏移量
     * @param[in] l_right 右侧横向偏移量
     * @param[in] l_left_type 左侧边界类型
     * @param[in] l_right_type 右侧边界类型
     */
    PathBoundaryUnitInfo(const double& s, const double& l_left, const double& l_right,
                         const BoundaryUnitTypeInfo& l_left_type, const BoundaryUnitTypeInfo& l_right_type)
        : s(s), l_left(l_left), l_right(l_right), l_left_type(l_left_type), l_right_type(l_right_type) {}
  };
  using ObstacleMap = std::map<double, ObstacleInfo>;  ///< 障碍物的映射

  PathBoundary(const double start_s = 0, const double delta_s = 2, const size_t size = 100);

  /**
   * @brief 虚析构函数
   * @details 默认实现的虚析构函数，确保派生类对象能正确释放资源
   *
   * @par 销毁流程:
   * @startuml
   * start
   * partition 资源释放 {
   *   :析构所有成员变量;
   *   :释放容器内存;
   * }
   * stop
   * @enduml
   *
   * @note 特性说明:
   * 1. 自动调用成员变量的析构函数
   * 2. 支持多态销毁
   *
   * @warning
   * 1. 确保所有资源都能正确释放
   */
  virtual ~PathBoundary() = default;

  void reset(const double start_s = 0, const double delta_s = 0, const size_t size = 0);

  /**
   * @brief 获取路径起始纵向坐标
   * @details 返回路径边界容器的起始s坐标值，该值表示路径规划起始点在Frenet坐标系中的纵向位置
   *
   * @par 关键数据说明:
   * - 物理意义: 路径规划起始点的纵向位置
   * - 单位: 米
   * - 取值范围: [0, +∞)
   *
   * @par 流程图:
   * @startuml
   * start
   * :访问start_s_成员;
   * :返回double类型值;
   * stop
   * @enduml
   *
   * @return double 当前路径起始s坐标
   *
   * @note 典型使用场景:
   * 1. 计算路径覆盖范围
   * 2. 与其他模块坐标系对齐
   *
   * @warning 返回值有效性依赖:
   * 1. 需通过reset()或构造函数正确初始化
   * 2. 不应在对象销毁后调用
   */
  double start_s() const { return start_s_; }

  /**
   * @brief 获取路径采样间隔
   * @details 返回相邻边界点间的纵向距离，该值决定路径边界的分辨率和计算精度
   *
   * @par 关键数据说明:
   * - 物理意义: 相邻采样点的纵向间距
   * - 单位: 米
   * - 取值范围: >0
   * - 默认值: 构造函数初始化值
   *
   * @par 流程图:
   * @startuml
   * start
   * :访问delta_s_成员;
   * :返回double类型值;
   * stop
   * @enduml
   *
   * @return double 当前采样间隔值
   *
   * @note 典型应用场景:
   * 1. 计算路径总长度: start_s + delta_s*(size-1)
   * 2. 调整路径规划分辨率
   *
   * @warning 重要约束:
   * 1. 必须为正值才能保证路径有效性
   * 2. 修改需通过reset()方法
   * 3. 与size共同决定内存消耗
   */
  double delta_s() const { return delta_s_; }

  /**
   * @brief 获取路径终止纵向坐标
   * @details 计算并返回路径边界容器的终止s坐标，通过起始坐标和路径长度计算得出
   *
   * @par 关键数据说明:
   * - 单位: 米
   * - 取值范围: [start_s_, +∞)
   *
   * @par 流程图:
   * @startuml
   * start
   * :获取start_s值;
   * :调用length()计算长度;
   * :相加得到end_s;
   * stop
   * @enduml
   *
   * @return double 路径终点s坐标
   *
   * @note 典型应用场景:
   * 1. 路径有效性检查（end_s > start_s）
   * 2. 模块间数据对齐校验
   *
   * @warning 计算约束:
   * 1. size=0时返回start_s
   * 2. 需确保delta_s和size的有效性
   */
  double end_s() const { return start_s_ + length(); }

  /**
   * @brief 计算路径总长度
   * @details 根据采样间隔和边界点数量计算路径的物理长度，处理空容器和无效参数情况
   *
   * @par 计算公式说明:
   * - 有效长度: |delta_s_ * (size_ - 1)| (当size_>0时)
   * - 无效长度: 0.0 (当size_=0时)
   * - 单位: 米
   * - 取值范围: [0, +∞)
   *
   * @par 计算流程图:
   * @startuml
   * start
   * if (size_ > 0?) then (是)
   *   :计算delta_s*(size-1);
   *   :取绝对值;
   * else (否)
   *   :返回0.0;
   * endif
   * stop
   * @enduml
   *
   * @return double 路径实际覆盖的纵向距离
   *
   * @note 典型应用场景:
   * 1. 路径规划精度评估
   * 2. 路径跟踪算法中距离计算
   * 3. 与start_s共同决定路径范围
   *
   * @warning 计算前提:
   * 1. delta_s_符号不影响结果(取绝对值)
   * 2. size_=1时返回0长度
   * 3. 需通过reset()维护参数有效性
   */
  double length() const { return size_ > 0 ? std::abs(delta_s_ * (size_ - 1)) : 0.0; }

  /**
   * @brief 获取边界点数量
   * @details 返回路径边界容器中存储的边界点总数，表示路径上采样点的数量
   *
   * @par 关键数据说明:
   * - 物理意义: 边界点的数量
   * - 取值范围: [0, +∞)
   *
   * @par 流程图:
   * @startuml
   * start
   * :访问size_成员;
   * :返回size_t类型值;
   * stop
   * @enduml
   *
   * @return size_t 当前存储的边界点数量
   *
   * @note 典型应用场景:
   * 1. 路径长度评估
   * 2. 边界点遍历操作
   * 3. 与delta_s共同决定内存占用
   *
   * @warning 重要约束:
   * 1. 必须通过reset()更新
   * 2. size=0时容器为空
   * 3. 与delta_s共同决定路径长度
   */
  size_t size() const { return size_; }

  void trim(const double extra_tail_length = 0);

  /**
   * @brief 设置车道保持起始点
   * @details 更新车道保持功能起始的纵向坐标，用于控制车辆开始执行车道保持的位置
   *
   * @param[in] lane_keep_start_s 新的车道保持起始s坐标
   * - 类型: double
   * - 单位: 米
   * - 取值范围: [0, end_s]
   * - 特殊值说明: 0表示立即开始车道保持
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :更新lane_keep_start_s_;
   * endif
   * stop
   * @enduml
   *
   * @note 典型应用场景:
   * 1. 动态调整车道保持位置
   * 2. 响应交通规则变化
   *
   * @warning 重要约束:
   * 1. 需在路径坐标系有效范围内设置
   * 2. 设置后需重新计算车道保持轨迹
   * 3. 无效值可能导致车道保持功能异常
   */
  void setLaneKeepStartS(const double lane_keep_start_s) { lane_keep_start_s_ = lane_keep_start_s; }

  /**
   * @brief 获取车道保持起始点
   * @details 返回当前设置的车道保持功能起始纵向坐标，用于确定车辆开始执行车道保持的s位置
   *
   * @par 关键数据说明:
   * - 物理意义: 车道保持起始点s坐标
   * - 单位: 米
   * - 取值范围: [0, end_s]
   * - 更新机制: 通过setLaneKeepStartS()设置
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :访问lane_keep_start_s_成员;
   * :返回double类型值;
   * stop
   * @enduml
   *
   * @return double 当前车道保持起始s坐标
   *
   * @note 典型应用场景:
   * 1. 动态调整车道保持起始位置
   * 2. 响应交通规则变化
   *
   * @warning 使用前提:
   * 1. 需通过setLaneKeepStartS()预先设置有效值
   * 2. 返回值应小于等于路径终点坐标
   * 3. 无效值会导致车道保持功能异常
   */
  double lane_keep_start_s() const { return lane_keep_start_s_; }

  /**
   * @brief 获取决策边界数据
   * @details 返回决策模块生成的原始边界信息容器，包含(s, left_bound, right_bound)元组序列
   *
   * @par 数据结构说明:
   * - 容器类型: vector<tuple<double, double, double>>
   * - 元组结构: (s坐标, 左侧横向偏移量, 右侧横向偏移量)
   * - 单位: 米
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :访问decision_boundary_成员;
   * :返回常量引用;
   * stop
   * @enduml
   *
   * @return const std::vector<std::tuple<double, double, double>>& 不可修改的决策边界数据引用
   *
   * @note 典型应用场景:
   * 1. 路径优化模块读取原始决策输入
   * 2. 可视化调试工具展示边界数据
   *
   * @warning 重要说明:
   * 1. 返回值为引用，需确保对象生命周期
   * 2. 修改应通过mutable_decision_boundary()
   * 3. 数据有效性取决于决策模块输出
   */
  const std::vector<std::tuple<double, double, double>>& decision_boundary() const { return decision_boundary_; };

  /**
   * @brief 获取可修改的决策边界数据指针
   * @details 返回决策边界容器的可修改指针，用于直接修改决策边界数据
   *
   * @par 数据结构说明:
   * - 容器类型: vector<tuple<double, double, double>>*
   * - 元组结构: (s坐标, 左侧横向偏移量, 右侧横向偏移量)
   * - 修改约束:
   *   1. 需保持s坐标升序排列
   *   2. 左侧偏移量 ≤ 右侧偏移量
   *   3. s ∈ [start_s_, end_s]
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :获取decision_boundary_地址;
   * :返回指针;
   * stop
   * @enduml
   *
   * @return std::vector<std::tuple<double, double, double>>* 可修改的决策边界数据指针
   *
   * @note 典型应用场景:
   * 1. 决策模块在线更新边界
   * 2. 测试用例注入模拟数据
   *
   * @warning 重要说明:
   * 1. 修改后需调用trim()维护容器大小
   * 2. 需确保数据一致性
   * 3. 仅在必要时使用, 不建议频繁修改
   * 4. 修改后需验证数据有效性
   */
  std::vector<std::tuple<double, double, double>>* mutable_decision_boundary() { return &decision_boundary_; };

  /**
   * @brief 获取软边界数据
   * @details 返回综合软边界信息容器，包含可穿越的边界信息（如车道虚线、虚拟决策边界等）
   *
   * @par 数据结构说明:
   * - 容器类型: vector<tuple<double, double, double>>
   * - 元组结构: (s坐标, 左侧最小偏移, 右侧最大偏移)
   * - 单位: 米
   * - 取值范围:
   *   - s ∈ [start_s_, end_s]
   *   - 左侧偏移 ≥ 硬边界左侧值
   *   - 右侧偏移 ≤ 硬边界右侧值
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :访问soft_boundary_成员;
   * :返回常量引用;
   * stop
   * @enduml
   *
   * @return const std::vector<std::tuple<double, double, double>>& 不可修改的软边界数据引用
   *
   * @note 典型应用场景:
   * 1. 路径优化算法读取可行驶区域
   * 2. 舒适性轨迹生成
   *
   * @warning 重要说明:
   * 1. 返回值为引用，需确保对象生命周期
   * 2. 需与硬边界配合使用验证有效性
   */
  const std::vector<std::tuple<double, double, double>>& soft_boundary() const { return soft_boundary_; };

  /**
   * @brief 获取可修改的软边界指针
   * @details 返回软边界容器的可修改指针，用于紧急避障等特殊场景的边界调整
   *
   * @par 修改约束:
   * - 需保持与硬边界的包含关系:
   *   soft_left ≥ barrier_left
   *   soft_right ≤ barrier_right
   * - s坐标需严格递增
   * - 修改后需调用trim()同步容器大小
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :返回指针;
   * stop
   * @enduml
   *
   * @return std::vector<std::tuple<double, double, double>>* 可修改的软边界指针
   *
   * @note 典型应用场景:
   * 1. 紧急避障场景动态调整边界
   * 2. 测试用例注入模拟数据
   *
   * @warning 重要说明:
   * 1. 修改后必须验证与硬边界的关系
   * 2. 需谨慎使用，避免影响路径规划
   * 3. 修改后需调用trim()维护容器大小
   */
  std::vector<std::tuple<double, double, double>>* mutable_soft_boundary() { return &soft_boundary_; };

  /**
   * @brief 获取硬边界数据
   * @details 返回不可逾越的物理边界信息容器，包含(s, left_limit, right_limit)元组序列
   *
   * @par 数据结构说明:
   * - 容器类型: vector<tuple<double, double, double>>
   * - 元组结构: (s坐标, 左侧绝对限制, 右侧绝对限制)
   * - 单位: 米
   * - 取值范围:
   *   - s ∈ [start_s_, end_s]
   *   - left_limit ≤ right_limit
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :访问barrier_boundary_成员;
   * :返回常量引用;
   * stop
   * @enduml
   *
   * @return const std::vector<std::tuple<double, double, double>>& 不可修改的硬边界数据引用
   *
   * @note 典型应用场景:
   * 1. 路径安全边界校验
   * 2. 碰撞风险评估
   *
   * @warning 重要说明:
   * 1. 边界数据具有最高优先级
   * 2. 修改需通过mutable_barrier_boundary()
   */
  const std::vector<std::tuple<double, double, double>>& barrier_boundary() const { return barrier_boundary_; };

  /**
   * @brief 获取可修改的硬边界指针
   * @details 返回硬边界容器的可修改指针，用于更新物理不可逾越的边界信息
   *
   * @par 修改约束:
   * - 需保持s坐标严格递增
   * - 左侧限制 ≤ 右侧限制
   * - 修改后需调用trim()同步容器大小
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :获取barrier_boundary_地址;
   * :返回指针;
   * @enduml
   *
   * @return std::vector<std::tuple<double, double, double>>* 可修改的硬边界指针
   *
   * @note 典型应用场景:
   * 1. 动态障碍物边界更新
   * 2. 测试用例注入模拟数据
   *
   * @warning 重要说明:
   * 1. 修改需谨慎，直接影响车辆安全
   * 2. 修改后需调用trim()维护容器大小
   * 3. 禁止在路径跟踪过程中修改
   */
  std::vector<std::tuple<double, double, double>>* mutable_barrier_boundary() { return &barrier_boundary_; };

  /**
   * @brief 获取环境感知软边界数据
   * @details 返回融合动静态障碍物和自由空间的感知系统软边界信息，表示可谨慎穿越的感知边界
   *
   * @par 数据结构说明:
   * - 容器类型: vector<tuple<double, double, double>>
   * - 元组结构: (s坐标, 左侧安全偏移, 右侧安全偏移)
   * - 单位: 米
   * - 取值范围:
   *   - s ∈ [start_s_, end_s]
   *   - 左侧偏移 ≥ 硬边界左侧值
   *   - 右侧偏移 ≤ 硬边界右侧值
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :访问env_perception_soft_boundary_成员;
   * :返回常量引用;
   * stop
   * @enduml
   *
   * @return const std::vector<std::tuple<double, double, double>>& 不可修改的环境感知软边界引用
   *
   * @note 典型应用场景:
   * 1. 路径优化算法读取感知安全区域
   * 2. 舒适性轨迹生成
   *
   * @warning 重要说明:
   * 1. 返回值为引用，需确保对象生命周期
   * 2. 需与硬边界配合使用验证有效性
   * 3. 需确保数据一致性
   */
  const std::vector<std::tuple<double, double, double>>& env_perception_soft_boundary() const {
    return env_perception_soft_boundary_;
  };

  /**
   * @brief 获取可修改的环境感知软边界指针
   * @details 返回感知系统软边界的可修改指针，用于在线校准感知边界数据
   *
   * @par 修改约束:
   * - 需保持与感知坐标系一致
   * - 修改范围需在硬边界内
   * - 修改后需调用trim()同步容器
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :获取env_perception_soft_boundary_地址;
   * :返回指针;
   * stop
   * @enduml
   *
   * @return std::vector<std::tuple<double, double, double>>* 可修改的环境感知软边界指针
   *
   * @note 典型应用场景:
   * 1. 测试用例注入模拟数据
   * 2. 动态障碍物边界更新
   *
   * @warning 重要说明:
   * 1. 修改需谨慎，直接影响车辆安全
   * 2. 修改后需调用trim()维护容器大小
   * 3. 需确保数据一致性
   */
  std::vector<std::tuple<double, double, double>>* mutable_env_perception_soft_boundary() {
    return &env_perception_soft_boundary_;
  };

  /**
   * @brief 获取环境感知硬边界数据
   * @details 返回融合动静态障碍物感知的不可逾越边界信息，包含绝对物理限制的边界数据
   *
   * @par 数据结构说明:
   * - 容器类型: vector<tuple<double, double, double>>
   * - 元组结构: (s坐标, 左侧绝对限制, 右侧绝对限制)
   * - 单位: 米
   * - 取值范围:
   *   - s ∈ [start_s_, end_s]
   *   - 左侧限制 ≤ 右侧限制
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :访问env_perception_barrier_boundary_成员;
   * :返回常量引用;
   * stop
   * @enduml
   *
   * @return const std::vector<std::tuple<double, double, double>>& 不可修改的环境感知硬边界引用
   *
   * @note 典型应用场景:
   * 1. 路径安全边界校验
   * 2. 碰撞风险评估
   *
   * @warning 重要说明:
   * 1. 修改需通过mutable_env_perception_barrier_boundary()
   * 2. 需与软边界配合使用验证有效性
   */
  const std::vector<std::tuple<double, double, double>>& env_perception_barrier_boundary() const {
    return env_perception_barrier_boundary_;
  };

  /**
   * @brief 获取可修改的环境感知硬边界指针
   * @details 返回感知系统硬边界的可修改指针，用于紧急场景下的边界覆盖
   *
   * @par 修改约束:
   * - 需保持与物理世界一致性
   * - s坐标严格递增
   * - 修改后需调用trimBoundary()
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :获取env_perception_barrier_boundary_地址;
   * :返回指针;
   * stop
   * @enduml
   *
   * @return std::vector<std::tuple<double, double, double>>* 可修改的环境感知硬边界指针
   *
   * @note 典型应用场景:
   * 1. 紧急避障场景动态调整边界
   * 2. 测试用例注入模拟数据
   *
   * @warning 重要说明:
   * 1. 修改需谨慎，直接影响车辆安全
   * 2. 修改后需调用trimBoundary()维护容器大小
   * 3. 需确保数据一致性
   */
  std::vector<std::tuple<double, double, double>>* mutable_env_perception_barrier_boundary() {
    return &env_perception_barrier_boundary_;
  };

  /**
   * @brief 获取先验物理软边界数据
   * @details 返回基于道路基础设施的可行驶区域信息，包含(s, left_margin, right_margin)元组序列
   *
   * @par 数据结构说明:
   * - 容器类型: vector<tuple<double, double, double>>
   * - 元组结构: (s坐标, 左侧安全余量, 右侧安全余量)
   * - 单位: 米
   * - 取值范围:
   *   - s ∈ [start_s_, end_s]
   *   - 左侧余量 ≥ 物理硬边界左侧值
   *   - 右侧余量 ≤ 物理硬边界右侧值
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :访问prior_physical_soft_boundary_成员;
   * :返回常量引用;
   * stop
   * @enduml
   *
   * @return const std::vector<std::tuple<double, double, double>>& 不可修改的先验物理软边界引用
   *
   * @note 典型应用场景:
   * 1. 路径优化算法读取先验安全区域
   * 2. 舒适性轨迹生成
   *
   * @warning 重要说明:
   * 1. 返回值为引用，需确保对象生命周期
   * 2. 需与硬边界配合使用验证有效性
   * 3. 需确保数据一致性
   */
  const std::vector<std::tuple<double, double, double>>& prior_physical_soft_boundary() const {
    return prior_physical_soft_boundary_;
  };

  /**
   * @brief 获取可修改的先验物理软边界指针
   * @details 返回先验物理软边界的可修改指针，用于人工标注特殊路况的可行驶区域
   *
   * @par 修改约束:
   * - 需保持与先验硬边界的包含关系
   * - s坐标严格单调递增
   * - 修改后需调用trimBoundary()
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :获取prior_physical_soft_boundary_地址;
   * :返回指针;
   * stop
   * @enduml
   *
   * @return std::vector<std::tuple<double, double, double>>* 可修改的先验物理软边界指针
   *
   * @note 典型应用场景:
   * 1. 人工标注特殊路况
   * 2. 测试用例注入模拟数据
   *
   * @warning 重要说明:
   * 1. 修改需谨慎，直接影响车辆安全
   * 2. 修改后需调用trimBoundary()维护容器大小
   * 3. 需确保数据一致性
   */
  std::vector<std::tuple<double, double, double>>* mutable_prior_physical_soft_boundary() {
    return &prior_physical_soft_boundary_;
  };

  /**
   * @brief 获取先验物理硬边界数据
   * @details 返回基于道路基础设施的不可逾越物理边界信息（如护栏、路缘石等）
   *
   * @par 数据结构说明:
   * - 容器类型: vector<tuple<double, double, double>>
   * - 元组结构: (s坐标, 左侧绝对限制, 右侧绝对限制)
   * - 单位: 米
   * - 取值范围:
   *   - s ∈ [start_s_, end_s]
   *   - 左侧限制 ≤ 右侧限制
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :访问prior_physical_barrier_boundary_成员;
   * :返回常量引用;
   * stop
   * @enduml
   *
   * @return const std::vector<std::tuple<double, double, double>>& 不可修改的先验物理硬边界引用
   *
   * @note 典型应用场景:
   * 1. 路径安全边界校验
   * 2. 碰撞风险评估
   *
   * @warning 重要说明:
   * 1. 修改需通过mutable_prior_physical_barrier_boundary()
   * 2. 需与软边界配合使用验证有效性
   */
  const std::vector<std::tuple<double, double, double>>& prior_physical_barrier_boundary() const {
    return prior_physical_barrier_boundary_;
  };

  /**
   * @brief 获取可修改的先验物理硬边界指针
   * @details 返回先验物理硬边界的可修改指针，用于更新基础设施变更后的绝对限制
   *
   * @par 修改约束:
   * - 需保持与物理世界完全一致
   * - s坐标严格递增
   * - 修改后需调用trimBoundary()
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :获取prior_physical_barrier_boundary_地址;
   * :返回指针;
   * stop
   * @enduml
   *
   * @return std::vector<std::tuple<double, double, double>>* 可修改的先验物理硬边界指针
   *
   * @note 典型应用场景:
   * 1. 道路基础设施变更
   * 2. 测试用例注入模拟数据
   *
   * @warning 重要说明:
   * 1. 修改需谨慎，直接影响车辆安全
   * 2. 修改后需调用trimBoundary()维护容器大小
   * 3. 需确保数据一致性
   */
  std::vector<std::tuple<double, double, double>>* mutable_prior_physical_barrier_boundary() {
    return &prior_physical_barrier_boundary_;
  };

  /**
   * @brief 获取静态障碍物软边界数据
   * @details 返回静态障碍物膨胀后的可穿越边界信息，包含绕过静态障碍物的安全偏移量
   *
   * @par 数据结构说明:
   * - 容器类型: vector<tuple<double, double, double>>
   * - 元组结构: (s坐标, 左侧安全偏移, 右侧安全偏移)
   * - 单位: 米
   * - 取值范围:
   *   - s ∈ [start_s_, end_s]
   *   - 左侧偏移 ≥ 硬边界左侧值
   *   - 右侧偏移 ≤ 硬边界右侧值
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :访问static_od_soft_boundary_成员;
   * :返回常量引用;
   * stop
   * @enduml
   *
   * @return const std::vector<std::tuple<double, double, double>>& 不可修改的静态障碍物软边界引用
   *
   * @note 典型应用场景:
   * 1. 路径优化算法读取静态障碍物安全区域
   * 2. 舒适性轨迹生成
   *
   * @warning 重要说明:
   * 1. 返回值为引用，需确保对象生命周期
   * 2. 需与硬边界配合使用验证有效性
   * 3. 需确保数据一致性
   */
  const std::vector<std::tuple<double, double, double>>& static_od_soft_boundary() const {
    return static_od_soft_boundary_;
  };

  /**
   * @brief 获取可修改的静态障碍物软边界指针
   * @details 返回静态障碍物软边界的可修改指针，用于特殊场景的路径放宽
   *
   * @par 修改约束:
   * - 需保持与障碍物地图一致
   * - s坐标严格递增
   * - 修改后需调用trimBoundary()
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :获取static_od_soft_boundary_地址;
   * :返回指针;
   * stop
   * @enduml
   *
   * @return std::vector<std::tuple<double, double, double>>* 可修改的静态障碍物软边界指针
   *
   * @note 典型应用场景:
   * 1. 特殊场景路径放宽
   * 2. 测试用例注入模拟数据
   *
   * @warning 重要说明:
   * 1. 修改需谨慎，直接影响车辆安全
   * 2. 修改后需调用trimBoundary()维护容器大小
   * 3. 需确保
   */
  std::vector<std::tuple<double, double, double>>* mutable_static_od_soft_boundary() {
    return &static_od_soft_boundary_;
  };

  /**
   * @brief 获取静态障碍物硬边界数据
   * @details 返回静态障碍物不可穿越的绝对边界信息，表示车辆必须避让的物理限制
   *
   * @par 数据结构说明:
   * - 容器类型: vector<tuple<double, double, double>>
   * - 元组结构: (s坐标, 左侧绝对限制, 右侧绝对限制)
   * - 单位: 米
   * - 取值范围:
   *   - s ∈ [start_s_, end_s]
   *   - 左侧限制 ≤ 右侧限制
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :访问static_od_barrier_boundary_成员;
   * :返回常量引用;
   * stop
   * @enduml
   *
   * @return const std::vector<std::tuple<double, double, double>>& 不可修改的静态障碍物硬边界引用
   *
   * @note 典型应用场景:
   * 1. 路径安全边界校验
   * 2. 碰撞风险评估
   *
   * @warning 重要说明:
   * 1. 修改需通过mutable_static_od_barrier_boundary()
   * 2. 需与软边界配合使用验证有效性
   */
  const std::vector<std::tuple<double, double, double>>& static_od_barrier_boundary() const {
    return static_od_barrier_boundary_;
  };

  /**
   * @brief 获取可修改的静态障碍物硬边界指针
   * @details 返回静态障碍物硬边界的可修改指针，用于紧急避障场景的边界调整
   *
   * @par 修改约束:
   * - 需保持与感知数据一致
   * - 仅允许扩大安全边界
   * - 修改后需调用trimBoundary()
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :获取static_od_barrier_boundary_地址;
   * :返回指针;
   * stop
   * @enduml
   *
   * @return std::vector<std::tuple<double, double, double>>* 可修改的静态障碍物硬边界指针
   *
   * @note 典型应用场景:
   * 1. 紧急避障场景动态调整边界
   * 2. 测试用例注入模拟数据
   *
   * @warning 重要说明:
   * 1. 修改需谨慎，直接影响车辆安全
   * 2. 修改后需调用trimBoundary()维护容器大小
   * 3. 需确保数据一致性
   */
  std::vector<std::tuple<double, double, double>>* mutable_static_od_barrier_boundary() {
    return &static_od_barrier_boundary_;
  };

  /**
   * @brief 获取自由空间软边界数据
   * @details 返回自由空间规划生成的动态可行驶区域边界，表示车辆可自主探索的临时通行区域
   *
   * @par 数据结构说明:
   * - 容器类型: vector<tuple<double, double, double>>
   * - 元组结构: (s坐标, 左侧探索边界, 右侧探索边界)
   * - 单位: 米
   * - 取值范围:
   *   - s ∈ [start_s_, end_s]
   *   - 左侧边界 ≥ 硬边界左侧值
   *   - 右侧边界 ≤ 硬边界右侧值
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :访问fs_soft_boundary_成员;
   * :返回常量引用;
   * stop
   * @enduml
   *
   * @return const std::vector<std::tuple<double, double, double>>& 不可修改的自由空间软边界引用
   *
   * @note 典型应用场景:
   * 1. 车道内自由空间探索
   * 2. 非结构化道路探索
   *
   * @warning 重要说明:
   * 1. 返回值为引用，需确保对象生命周期
   * 2. 需与硬边界配合使用验证有效性
   * 3. 需确保数据一致性
   */
  const std::vector<std::tuple<double, double, double>>& fs_soft_boundary() const { return fs_soft_boundary_; };

  /**
   * @brief 获取可修改的自由空间软边界指针
   * @details 返回自由空间软边界的可修改指针，用于紧急路径探索场景
   *
   * @par 修改约束:
   * - 需保持与占据地图一致性
   * - s坐标严格递增
   * - 修改后需调用trimBoundary()
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :获取fs_soft_boundary_地址;
   * :返回指针;
   * stop
   * @enduml
   *
   * @return std::vector<std::tuple<double, double, double>>* 可修改的自由空间软边界指针
   *
   * @note 典型应用场景:
   * 1. 自由空间探索路径规划
   * 2. 测试用例注入模拟数据
   *
   * @warning 重要说明:
   * 1. 修改需谨慎，直接影响车辆安全
   * 2. 修改后需调用trimBoundary()维护容器大小
   * 3. 需确保数据一致性
   */
  std::vector<std::tuple<double, double, double>>* mutable_fs_soft_boundary() { return &fs_soft_boundary_; };

  /**
   * @brief 获取自由空间硬边界数据
   * @details 返回自由空间规划的绝对安全边界，表示基于传感器数据的不可穿越区域
   *
   * @par 数据结构说明:
   * - 容器类型: vector<tuple<double, double, double>>
   * - 元组结构: (s坐标, 左侧绝对限制, 右侧绝对限制)
   * - 单位: 米
   * - 取值范围:
   *   - s ∈ [start_s_, end_s]
   *   - 左侧限制 ≤ 右侧限制
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :访问fs_barrier_boundary_成员;
   * :返回常量引用;
   * stop
   * @enduml
   *
   * @return const std::vector<std::tuple<double, double, double>>& 不可修改的自由空间硬边界引用
   *
   * @note 典型应用场景:
   * 1. 路径安全边界校验
   * 2. 碰撞风险评估
   *
   * @warning 重要说明:
   * 1. 修改需通过mutable_fs_barrier_boundary()
   * 2. 需与软边界配合使用验证有效性
   */
  const std::vector<std::tuple<double, double, double>>& fs_barrier_boundary() const { return fs_barrier_boundary_; };

  /**
   * @brief 获取可修改的自由空间硬边界指针
   * @details 返回自由空间硬边界的可修改指针，用于传感器失效时的应急处理
   *
   * @par 修改约束:
   * - 仅允许扩大安全边界
   * - 需保持s坐标连续性
   * - 修改后需调用trimBoundary()
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :获取fs_barrier_boundary_地址;
   * :返回指针;
   * stop
   * @enduml
   *
   * @return std::vector<std::tuple<double, double, double>>* 可修改的自由空间硬边界指针
   *
   * @note 典型应用场景:
   * 1. 传感器失效场景动态调整边界
   * 2. 测试用例注入模拟数据
   *
   * @warning 重要说明:
   * 1. 修改需谨慎，直接影响车辆安全
   * 2. 修改后需调用trimBoundary()维护容器大小
   * 3. 需确保数据一致性
   */
  std::vector<std::tuple<double, double, double>>* mutable_fs_barrier_boundary() { return &fs_barrier_boundary_; };

  /**
   * @brief 获取动态障碍物软边界数据
   * @details 返回动态障碍物预测轨迹的安全包络边界，表示车辆需谨慎避让的时变区域
   *
   * @par 数据结构说明:
   * - 容器类型: vector<tuple<double, double, double>>
   * - 元组结构: (s坐标, 左侧避让余量, 右侧避让余量)
   * - 单位: 米
   * - 取值范围:
   *   - s ∈ [start_s_, end_s]
   *   - 左侧余量 ≥ 硬边界左侧值
   *   - 右侧余量 ≤ 硬边界右侧值
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :访问dynamic_od_soft_boundary_成员;
   * :返回常量引用;
   * stop
   * @enduml
   *
   * @return const std::vector<std::tuple<double, double, double>>& 不可修改的动态障碍物软边界引用
   *
   * @note 典型应用场景:
   * 1. 路径优化算法读取动态障碍物安全区域
   * 2. 舒适性轨迹生成
   *
   * @warning 重要说明:
   * 1. 返回值为引用，需确保对象生命周期
   * 2. 需与硬边界配合使用验证有效性
   * 3. 需确保数据一致性
   */
  const std::vector<std::tuple<double, double, double>>& dynamic_od_soft_boundary() const {
    return dynamic_od_soft_boundary_;
  };

  /**
   * @brief 获取可修改的动态障碍物软边界指针
   * @details 返回动态障碍物软边界的可修改指针，用于特殊交互场景的避让调整
   *
   * @par 修改约束:
   * - 需保持与预测轨迹同步
   * - s坐标严格递增
   * - 修改后需调用trimBoundary()
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :获取dynamic_od_soft_boundary_地址;
   * :返回指针;
   * stop
   * @enduml
   *
   * @return std::vector<std::tuple<double, double, double>>* 可修改的动态障碍物软边界指针
   *
   * @note 典型应用场景:
   * 1. 特殊交互场景动态避让调整
   * 2. 测试用例注入模拟数据
   *
   * @warning 重要说明:
   * 1. 修改需谨慎，直接影响车辆安全
   * 2. 修改后需调用trimBoundary()维护容器大小
   * 3. 需确保数据一致性
   */
  std::vector<std::tuple<double, double, double>>* mutable_dynamic_od_soft_boundary() {
    return &dynamic_od_soft_boundary_;
  };

  /**
   * @brief 获取实时横向软边界信息
   * @details 返回包含边界类型标注的可调整横向边界信息，用于舒适性路径生成
   *
   * @par 数据结构说明:
   * - 容器类型: vector<PathBoundaryUnitInfo>
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :访问path_soft_boundary_info_成员;
   * :返回常量引用;
   * stop
   * @enduml
   *
   * @return const std::vector<PathBoundaryUnitInfo>& 不可修改的横向软边界信息引用
   *
   * @note 典型应用场景:
   * 1. 舒适性路径生成
   * 2. 车道级路径微调
   *
   * @warning 重要说明:
   * 1. 返回值为引用，需确保对象生命周期
   * 2. 需与硬边界配合使用验证有效性
   * 3. 需确保数据一致性
   */
  const std::vector<PathBoundaryUnitInfo>& path_soft_boundary_info() const { return path_soft_boundary_info_; };

  /**
   * @brief 获取可修改的横向软边界指针
   * @details 返回横向软边界的可修改指针，用于动态车道保持调整
   *
   * @par 修改约束:
   * - 需保持s坐标递增
   * - 边界类型必须为DASHED_LANE_LINE或DECISION
   * - 修改后需调用trimBoundary()
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :获取path_soft_boundary_info_地址;
   * :返回指针;
   * stop
   * @enduml
   *
   * @return std::vector<PathBoundaryUnitInfo>* 可修改的横向软边界指针
   *
   * @note 典型应用场景:
   * 1. 动态车道保持场景
   * 2. 道路变更时的临时调整
   *
   * @warning 重要说明:
   * 1. 修改需谨慎，直接影响车辆安全
   * 2. 修改后需调用trimBoundary()维护容器大小
   * 3. 需确保数据一致性
   */
  std::vector<PathBoundaryUnitInfo>* mutable_path_soft_boundary_info() { return &path_soft_boundary_info_; };

  /**
   * @brief 获取实时横向硬边界信息
   * @details 返回包含绝对限制的横向边界信息，用于安全关键路径生成
   *
   * @par 数据结构说明:
   * - 容器类型: vector<PathBoundaryUnitInfo>
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :访问path_barrier_boundary_info_成员;
   * :返回常量引用;
   * stop
   * @enduml
   *
   * @return const std::vector<PathBoundaryUnitInfo>& 不可修改的横向硬边界信息引用
   *
   * @note 典型应用场景:
   * 1. 安全关键路径生成
   * 2. 碰撞风险评估
   *
   * @warning 重要说明:
   * 1. 返回值为引用，需确保对象生命周期
   * 2. 需与软边界配合使用验证有效性
   * 3. 需确保数据一致性
   */
  const std::vector<PathBoundaryUnitInfo>& path_barrier_boundary_info() const { return path_barrier_boundary_info_; };

  /**
   * @brief 获取可修改的横向硬边界指针
   * @details 返回横向硬边界的可修改指针，用于紧急避障场景
   *
   * @par 修改约束:
   * - 边界类型必须为SOLID_LANE_LINE或PHYSICALLY_IMPASSABLE
   * - 修改范围需包含软边界
   * - 修改后需调用trimBoundary()
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :获取path_barrier_boundary_info_地址;
   * :返回指针;
   * stop
   * @enduml
   *
   * @return std::vector<PathBoundaryUnitInfo>* 可修改的横向硬边界指针
   *
   * @note 典型应用场景:
   * 1. 紧急避障场景
   * 2. 道路变更时的临时调整
   *
   * @warning 重要说明:
   * 1. 修改需谨慎，直接影响车辆安全
   * 2. 修改后需调用trimBoundary()维护容器大小
   * 3. 需确保数据
   */
  std::vector<PathBoundaryUnitInfo>* mutable_path_barrier_boundary_info() { return &path_barrier_boundary_info_; };

  /**
   * @brief 获取关键障碍物映射表
   * @details 返回与路径边界强关联的障碍物信息映射表，用于路径安全校验和避让决策
   *
   * @par 数据结构说明:
   * - 容器类型: map<double, ObstacleInfo>
   * - 键类型: double 表示障碍物关联的s坐标
   * - 值类型: ObstacleInfo 结构体，包含左右侧关联障碍物ID
   * - 单位: 米(s坐标)
   * - 取值范围:
   *   - s ∈ [start_s_, end_s]
   *   - 障碍物ID非空字符串
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :访问key_obstacles_成员;
   * :返回常量引用;
   * stop
   * @enduml
   *
   * @return const ObstacleMap& 不可修改的障碍物映射表引用
   *
   * @note 典型应用场景:
   * 1. 路径安全校验
   * 2. 动态障碍物避让决策
   *
   * @warning 重要说明:
   * 1. 返回值为引用，需确保对象生命周期
   * 2. 需与路径边界同步更新
   * 3. 需确保数据一致性
   */
  const ObstacleMap& key_obstacles() const { return key_obstacles_; }

  /**
   * @brief 获取可修改的关键障碍物映射表指针
   * @details 返回障碍物映射表的可修改指针，用于在线更新障碍物关联信息
   *
   * @par 修改约束:
   * - 需保持s坐标递增
   * - 障碍物ID必须有效
   * - 修改后需触发路径重规划
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :获取key_obstacles_地址;
   * :返回指针;
   * stop
   * @enduml
   *
   * @return ObstacleMap* 可修改的障碍物映射表指针
   *
   * @note 典型应用场景:
   * 1. 动态障碍物感知更新
   * 2. 道路变更时的临时调整
   *
   * @warning 重要说明:
   * 1. 修改需谨慎，直接影响车辆安全
   * 2. 修改后需调用trimBoundary()维护容器大小
   * 3. 需确保数据一致性
   */
  ObstacleMap* mutable_key_obstacles() { return &key_obstacles_; }
  std::optional<ObstacleMap::iterator> findKeyObstacle(const double s);
  std::optional<ObstacleMap::const_iterator> findConstKeyObstacle(const double s) const;

  /**
   * @brief 设置路径边界标签
   * @details 设置路径边界的类型标签，用于区分常规路径、紧急路径等不同场景
   *
   * @param[in] label 路径标签字符串
   * - 类型: std::string
   * - 有效值: "regular", "emergency", "lane_change", "merge"
   * - 长度限制: ≤20字符
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :接收输入标签;
     :移动语义赋值给label_;
   * stop
   * @enduml
   *
   * @note 典型应用场景:
   * 1. 路径生成时根据场景设置标签
   * 2. 路径重规划时根据标签调整策略
   *
   * @warning 输入校验:
   * 1. 空标签处理
   * 2. 大小写敏感
   *
   *
   */
  void set_label(std::string label) { label_ = std::move(label); }

  /**
   * @brief 获取当前路径标签
   * @details 返回路径边界的类型标签，用于判断当前路径的生成场景
   *
   * @par 数据结构说明:
   * - 类型: std::string 常量引用
   * - 有效值: 预定义路径类型枚举字符串
   * - 内存特性: 内部成员直接引用
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :访问label_成员;
   * :返回常量引用;
   * stop
   * @enduml
   *
   * @return const std::string& 不可修改的路径标签引用
   *
   * @note 数据更新机制:
   * 1. 内部成员直接引用
   * 2. 不可修改，需通过set_label()更新
   *
   * @warning 使用限制:
   * 1. 需确保对象生命周期有效
   * 2. 禁止通过引用修改值
   */
  const std::string& label() const { return label_; }

  /**
   * @brief 获取可修改的标签指针
   * @details 返回标签字符串的可修改指针，用于直接内存操作
   *
   * @par 修改约束:
   * - 需符合预定义标签格式
   * - 修改后需触发路径重规划
   * - 最大长度≤20字符
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :获取label_地址;
   * :返回指针;
   * stop
   * @enduml
   *
   * @return std::string* 可修改的标签指针
   *
   * @note 数据更新机制:
   * 1. 内部成员直接引用
   * 2. 需谨慎操作，可能导致数据不一致
   * 3. 禁止修改空字符串
   *
   * @warning 操作风险:
   * 1. 需保证线程安全, 避免产生野指针
   * 2. 修改后需验证路径生成逻辑
   */
  std::string* mutable_label() { return &label_; }

  /**
   * @brief 设置阻塞障碍物ID
   * @details 标记当前主要阻塞路径的障碍物ID，用于避让优先级决策
   *
   * @param[in] obs_id 障碍物唯一标识符
   * - 类型: std::string
   * - 格式要求: UUID或感知系统规范ID
   * - 长度限制: ≤128字符
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :接收输入obs_id;
   * :移动语义赋值给blocking_obstacle_id_;
   * stop
   * @enduml
   *
   * @note 数据更新机制:
   * 1. 内部成员直接赋值
   * 2. 需与路径重规划同步更新
   *
   * @warning 重要说明:
   * 1. 修改需谨慎，直接影响避让策略
   * 2. 修改后需触发路径重规划
   * 3. 禁止使用空字符串
   */
  void set_blocking_obstacle_id(std::string obs_id) { blocking_obstacle_id_ = std::move(obs_id); }

  /**
   * @brief 获取当前阻塞障碍物ID
   * @details 返回当前主要阻塞路径的障碍物ID，用于避让策略决策
   *
   * @par 数据结构说明:
   * - 类型: std::string 常量引用
   * - 有效值: 符合感知系统规范的ID格式
   * - 空值含义: 无阻塞障碍物
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :访问blocking_obstacle_id_成员;
   * :返回常量引用;
   * stop
   * @enduml
   *
   * @return const std::string& 不可修改的障碍物ID引用
   *
   * @note 数据更新机制:
   * 1. 内部成员直接引用
   * 2. 需与路径重规划同步更新
   * 3. 初始值为空字符串
   *
   * @warning 重要说明:
   * 1. 返回值为引用，需确保对象生命周期
   * 2. 禁止通过引用修改值
   */
  const std::string& blocking_obstacle_id() const { return blocking_obstacle_id_; }

  /**
   * @brief 获取可修改的阻塞障碍物ID指针
   * @details 返回障碍物ID的可修改指针，用于紧急场景直接操作内存
   *
   * @par 修改约束:
   * - 需符合ID格式规范
   * - 最大长度≤128字符
   * - 修改后需触发路径重规划
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :获取blocking_obstacle_id_地址;
   * :返回指针;
   * stop
   * @enduml
   *
   * @return std::string* 可修改的障碍物ID指针
   *
   * @note 数据更新机制:
   * 1. 内部成员直接引用
   * 2. 需谨慎操作，可能导致数据不一致
   * 3. 禁止修改空字符串
   *
   * @warning 操作风险:
   * 1. 需保证线程安全, 避免产生野指针
   * 2. 修改后需验证路径生成逻辑
   */
  std::string* mutable_blocking_obstacle_id() { return &blocking_obstacle_id_; }

  void set_blocking_freespace_info(const double s);

  /**
   * @brief 获取自由空间阻塞信息
   * @details 返回当前自由空间阻塞状态及位置信息，用于安全路径验证
   *
   * @par 数据结构说明:
   * - 类型: pair<bool, double> 常量引用
   * - 成员说明:
   *   * first: 阻塞状态标识
   *     - true: 存在阻塞自由空间
   *     - false: 无阻塞
   *   * second: 阻塞点s坐标
   *     - 有效范围: [start_s_, end_s]
   *     - 无效值: double::max()
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :访问blocking_freespace_info_成员;
   * :返回常量引用;
   * stop
   * @enduml
   *
   * @return const std::pair<bool, double>& 不可修改的自由空间阻塞信息引用
   *
   * @note 数据更新机制:
   * 1. 内部成员直接引用
   * 2. 需与路径重规划同步更新
   *
   * @warning 重要说明:
   * 1. 返回值为引用，需确保对象生命周期
   * 2. 禁止通过引用修改值
   */
  const std::pair<bool, double>& blocking_freespace_info() const { return blocking_freespace_info_; }

  /**
   * @brief 获取可修改的自由空间阻塞信息指针
   * @details 返回阻塞信息内存地址，用于紧急场景直接修改状态
   *
   * @par 修改约束:
   * - 需保持数据格式有效性
   *
   * @par 操作流程图:
   * @startuml
   * start
   * :获取blocking_freespace_info_地址;
   * :返回指针;
   * stop
   * @enduml
   *
   * @return std::pair<bool, double>* 可修改的阻塞信息指针
   *
   * @note 数据更新机制:
   * 1. 内部成员直接引用
   * 2. 需谨慎操作，可能导致数据不一致
   * 3. 禁止修改空字符串
   *
   * @warning 重要说明:
   * 1. 修改需谨慎，直接影响车辆安全
   * 2. 修改后需验证路径生成逻辑
   */
  std::pair<bool, double>* mutable_blocking_freespace_info() { return &blocking_freespace_info_; }

 protected:
  void trimBoundary(std::vector<std::tuple<double, double, double>>& boundary);

 private:
  double start_s_ = 0.0;                                                                        ///< 路径起始纵向坐标
  double delta_s_ = 0.0;                                                                        ///< 路径采样间隔
  double lane_keep_start_s_ = 0.0;                                                              ///< 车道保持起始坐标
  size_t size_ = 0;                                                                             ///< 路径边界点数量
  std::string label_ = "regular";                                                               ///< 路径类型标签
  std::string blocking_obstacle_id_ = "";                                                       ///< 阻塞障碍物ID
  std::pair<bool, double> blocking_freespace_info_{false, std::numeric_limits<double>::max()};  ///< 自由空间阻塞信息
  ObstacleMap key_obstacles_;                                                                   ///< 关键障碍物映射表

  std::vector<std::tuple<double, double, double>> decision_boundary_;  ///< 决策原始边界
  std::vector<std::tuple<double, double, double>> soft_boundary_;      ///< 综合软边界
  std::vector<std::tuple<double, double, double>> barrier_boundary_;   ///< 综合硬边界
  std::vector<std::tuple<double, double, double>>
      env_perception_soft_boundary_;  ///< 综合动静态障碍物和fs的感知系统输出边界的软边界
  std::vector<std::tuple<double, double, double>>
      env_perception_barrier_boundary_;  ///< 综合动静态障碍物和fs的感知系统输出边界的硬边界
  std::vector<std::tuple<double, double, double>>
      prior_physical_soft_boundary_;  ///< 基于道路边界信息和决策信息的先验信息的软边界
  std::vector<std::tuple<double, double, double>>
      prior_physical_barrier_boundary_;  ///< 基于道路边界信息和决策信息的先验信息的硬边界
  std::vector<std::tuple<double, double, double>> static_od_soft_boundary_;     ///< 静态障碍物的软边界
  std::vector<std::tuple<double, double, double>> static_od_barrier_boundary_;  ///< 静态障碍物的硬边界
  std::vector<std::tuple<double, double, double>> fs_soft_boundary_;            ///< fs的软边界
  std::vector<std::tuple<double, double, double>> fs_barrier_boundary_;         ///< fs的硬边界
  std::vector<std::tuple<double, double, double>> dynamic_od_soft_boundary_;    ///< 动态障碍物的软边界
  std::vector<PathBoundaryUnitInfo> path_soft_boundary_info_;                   ///< 实时横向软边界信息
  std::vector<PathBoundaryUnitInfo> path_barrier_boundary_info_;                ///< 实时横向硬边界信息
};

}  // namespace gpal::pnc::planning

#endif
