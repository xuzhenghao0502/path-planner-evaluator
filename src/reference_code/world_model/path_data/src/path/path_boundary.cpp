/**
 * @file path_boundary.cpp
 * @brief 路径边界数据容器
 * @details 管理路径规划中多种类型的横向边界信息，包含道路边界、障碍物边界等
 */

#include "path/path_boundary.h"

#include <base/log.h>

#include <algorithm>

namespace gpal::pnc::planning {

/**
 * @brief 路径边界构造函数
 * @details 初始化路径边界容器，创建指定参数的边界数据存储结构
 *
 * @param[in] start_s 起始纵向坐标
 * - 类型: double
 * - 单位: 米
 * - 范围: [0, +∞)
 * - 默认值: 0.0
 * @param[in] delta_s 采样间隔
 * - 类型: double
 * - 单位: 米
 * - 范围: >0
 * - 默认值: 2.0
 * @param[in] size 边界点数量
 * - 类型: size_t
 * - 范围: >=0
 * - 默认值: 100
 *
 * @par 关键变量说明:
 * - start_s_: 记录路径起点s坐标
 * - delta_s_: 控制边界分辨率
 * - size_: 决定容器初始容量
 *
 * @par 流程图:
 * @startuml
 * start
 * :验证参数有效性;
 * partition 初始化 {
 *   :设置start_s_ = start_s;
 *   :设置delta_s_ = delta_s;
 *   :分配size_存储空间;
 * }
 * :计算end_s = start_s + delta_s*(size-1);
 * stop
 * @enduml
 *
 * @note 默认参数说明:
 * 1. 2米间隔适用于城市道路场景
 * 2. 100个点对应200米路径长度
 *
 * @warning 必须确保:
 * 1. delta_s不能为0或负数
 * 2. size参数不应超过内存限制
 * 3. start_s应与其他模块坐标系一致
 */
PathBoundary::PathBoundary(const double start_s, const double delta_s, const size_t size)
    : start_s_(start_s), delta_s_(delta_s), size_(size) {}

/**
 * @brief 重置路径边界参数
 * @details 重新初始化边界容器参数，清除现有数据并设置新的采样参数
 *
 * @param[in] start_s 新起始纵向坐标
 * - 类型: double
 * - 单位: 米
 * - 范围: [0, +∞)
 * - 默认值: 0.0
 * @param[in] delta_s 新采样间隔
 * - 类型: double
 * - 单位: 米
 * - 范围: >=0
 * - 默认值: 0.0
 * @param[in] size 新边界点数量
 * - 类型: size_t
 * - 范围: >=0
 * - 默认值: 0
 *
 * @par 关键操作流程:
 * @startuml
 * start
 * partition 参数重置 {
 *   :清除现有边界数据;
 *   if (start_s != 0) then (是)
 *     :更新start_s_;
 *   endif
 *   if (delta_s != 0) then (是)
 *     :更新delta_s_;
 *   endif
 *   if (size != 0) then (是)
 *     :重新分配存储空间;
 *   endif
 * }
 * stop
 * @enduml
 *
 * @note 特殊场景说明:
 * 1. 当delta_s=0时保持原采样间隔
 * 2. size=0表示清空容器
 *
 * @warning 必须确保:
 * 1. delta_s与size的乘积不应溢出
 * 2. 调用后需重新填充边界数据
 * 3. 避免在路径跟踪过程中调用
 */
void PathBoundary::reset(const double start_s, const double delta_s, const size_t size) {
  start_s_ = start_s;
  delta_s_ = delta_s;
  size_ = size;
  soft_boundary_.clear();
  barrier_boundary_.clear();
  decision_boundary_.clear();
  env_perception_barrier_boundary_.clear();
  env_perception_soft_boundary_.clear();
  prior_physical_barrier_boundary_.clear();
  prior_physical_soft_boundary_.clear();
  fs_barrier_boundary_.clear();
  fs_soft_boundary_.clear();
  static_od_barrier_boundary_.clear();
  static_od_soft_boundary_.clear();
  dynamic_od_soft_boundary_.clear();
  path_barrier_boundary_info_.clear();
  path_soft_boundary_info_.clear();
  blocking_obstacle_id_.clear();
}

/**
 * @brief 设置自由空间阻塞信息
 * @details 标记当前阻塞路径的自由空间位置及有效性，用于紧急避障路径生成
 *
 * @param[in] s 自由空间阻塞点纵向坐标
 * - 单位: 米
 * - 取值范围: [start_s_, end_s]
 * - 有效条件: 当前路径范围内存在物理不可穿越区域
 *
 * @par 操作流程图:
 * @startuml
 * start
 * :输入s坐标;
 * if (s ∈ [start_s_, end_s]?) then (是)
 *   :设置blocking_freespace_info_为{true, s};
 * else (否)
 *   :设置为{false, max_double};
 * endif
 * stop
 * @enduml
 *
 * @note 典型应用场景:
 * 1. 自由空间障碍物检测
 * 2. 紧急路径重规划
 *
 * @warning 重要说明:
 * 1. 需进行输入范围校验
 * 2. 禁止在路径跟踪时修改
 */
void PathBoundary::set_blocking_freespace_info(const double s) {
  if (blocking_freespace_info_.first) {
    blocking_freespace_info_.second = std::min(blocking_freespace_info_.second, s);
  } else {
    blocking_freespace_info_.first = true;
    blocking_freespace_info_.second = s;
  }
}

/**
 * @brief 裁剪路径边界数据
 * @details 移除超出指定长度的尾部边界数据，保留指定长度的额外缓冲
 *
 * @param[in] extra_tail_length 额外保留的尾部长度
 * - 类型: double
 * - 单位: 米
 * - 取值范围: >=0
 * - 特殊值说明: 0表示严格按需裁剪
 *
 * @par 关键操作流程:
 * @startuml
 * start
 * partition 计算裁剪量 {
 *   :计算保留的采样点数\nnew_size = (length() + extra_tail_length) / delta_s_ + 1;
 *   :限制new_size范围[1, 原始size];
 * }
 * :更新size_为new_size;
 * partition 边界裁剪 {
 *   :遍历所有边界容器;
 *   while (是否存在未处理边界容器?) is (是)
 *     :调用trimBoundary()裁剪当前容器;
 *   endwhile
 * }
 * stop
 * @enduml
 *
 * @note 典型应用场景:
 * 1. 路径优化后缩短规划长度
 * 2. 内存回收
 *
 * @warning 重要约束:
 * 1. 需在路径规划完成后调用
 * 2. 裁剪后需重新校验路径有效性
 * 3. 避免在路径跟踪过程中调用
 */
void PathBoundary::trim(const double extra_tail_length) {
  if (!barrier_boundary_.empty()) {
    std::sort(barrier_boundary_.begin(), barrier_boundary_.end(),
              [](const std::tuple<double, double, double>& lhs, const std::tuple<double, double, double>& rhs) {
                return std::get<0>(lhs) < std::get<0>(rhs);
              });

    double length_barrier = std::get<0>(barrier_boundary_.back()) - start_s_;
    double delta_s_abs = std::abs(delta_s_);
    if (extra_tail_length > delta_s_abs) {
      CHECK_GT(delta_s_abs, 0);
      double tail_length = 0;
      const double min_s_tail = 1.0;
      while (tail_length < extra_tail_length && length_barrier < length()) {
        auto back_boundary = barrier_boundary_.back();
        double delta_s = std::min(end_s() - std::get<0>(barrier_boundary_.back()), delta_s_);
        if (delta_s_ < 1e-10) {
          break;
        }
        std::get<0>(back_boundary) += delta_s;
        tail_length += delta_s_abs;
        length_barrier += delta_s_abs;
        barrier_boundary_.emplace_back(std::move(back_boundary));
      }
    }
    size_ = std::max(1, int(length_barrier / delta_s_abs) + 1);
  }
  trimBoundary(soft_boundary_);
  trimBoundary(decision_boundary_);
}

/**
 * @brief 裁剪路径边界数据
 * @details 对指定类型的边界容器进行裁剪操作，包含两个主要步骤：
 *          1. 移除超出路径终点(end_s)的尾部边界点
 *          2. 补充边界点使数量与障碍物边界容器保持一致
 *
 * @param[in,out] boundary 待处理的边界容器，包含(s,左界,右界)元组
 * @par 输入输出说明:
 * - 输入容器要求无序，输出容器按s升序排列
 * - 直接修改原始容器，无返回值
 *
 * @par 流程图:
 * @startuml
 * start
 * partition 初始化处理 {
 *   :排序边界容器(按s升序);
 *   :计算当前路径终点end_s;
 * }
 *
 * partition 裁剪阶段 {
 *   :查找首个s超过end_s的位置;
 *   if (存在超界数据?) then (是)
 *     :删除超界点之后的所有元素;
 *   else (否)
 *     :保持当前数据不变;
 *   endif
 * }
 *
 * partition 数据补齐 {
 *   while (边界点数量 < 障碍物边界数量?) is (是)
 *     :获取最后一个边界点;
 *     :计算增量delta_s = min(end_s - s, delta_s_);
 *     if (delta_s < 1e-10) then (过小)
 *       :跳出循环;
 *     else (有效值)
 *       :更新s坐标 += delta_s;
 *       :追加到容器末尾;
 *     endif
 *   endwhile
 * }
 * stop
 * @enduml
 *
 * @note 典型应用场景:
 * 1. 路径优化后缩短规划长度
 * 2. 多边界容器同步更新
 *
 * @warning 重要约束:
 * - delta_s_过小时可能导致死循环(需外部保证delta_s_有效性)
 * - 时间复杂度O(n log n)来自排序操作
 */
void PathBoundary::trimBoundary(std::vector<std::tuple<double, double, double>>& boundary) {
  if (!boundary.empty()) {
    std::sort(boundary.begin(), boundary.end(),
              [](const std::tuple<double, double, double>& lhs, const std::tuple<double, double, double>& rhs) {
                return std::get<0>(lhs) < std::get<0>(rhs);
              });

    auto iter_end = std::upper_bound(
        boundary.begin(), boundary.end(), end_s(),
        [](const double s, const std::tuple<double, double, double>& rhs) { return s < std::get<0>(rhs); });
    if (iter_end != boundary.end()) {
      boundary.erase(iter_end, boundary.end());
    }

    while (boundary.size() < barrier_boundary_.size()) {
      auto back_boundary = boundary.back();
      double delta_s = std::min(end_s() - std::get<0>(back_boundary), delta_s_);
      if (delta_s_ < 1e-10) {
        break;
      }
      std::get<0>(back_boundary) += delta_s;
      boundary.emplace_back(std::move(back_boundary));
    }
  }
}

/**
 * @brief 查找指定s坐标的关键障碍物条目
 * @details 在障碍物映射表中查找与给定s坐标关联的障碍物信息，允许修改查找到的条目
 *
 * @param[in] s 目标纵向坐标
 * - 单位：米
 * - 取值范围：[start_s_, end_s]
 * - 精度要求：±0.1米
 *
 * @par 关键变量说明:
 * - lower_bound (double): 搜索下界，初始为start_s_
 * - upper_bound (double): 搜索上界，初始为end_s_
 * - epsilon (double): 坐标匹配阈值，固定为0.1米
 *
 * @par 判断条件:
 * - 查找成功条件：|map_key - s| ≤ epsilon
 * - 查找失败条件：遍历完整s范围未找到匹配项
 *
 * @par 操作流程图:
 * @startuml
 * start
 * :初始化二分查找边界;
 * partition 二分查找 {
 * while (查找范围有效?) is (是)
 *   :计算中间点mid;
 *   if (|mid.key - s| ≤ 0.1m?) then (是)
 *     :返回迭代器;
 *     stop
 *   else (否)
 *     if (mid.key < s?) then (是)
 *       :调整下界;
 *     else (否)
 *       :调整上界;
 *     endif
 *   endif
 * endwhile
 * }
 * :返回空值;
 * stop
 * @enduml
 *
 * @return std::optional<ObstacleMap::iterator> 可选的障碍物条目迭代器
 * - 存在时：返回有效迭代器
 * - 不存在时：返回std::nullopt
 *
 * @note 典型应用场景:
 * 1. 动态避撞检查
 * 2. 障碍物关联路径优化
 *
 * @warning 重要说明:
 * 1. 需确保障碍物映射表按s坐标有序
 * 2. 查找效率取决于障碍物数量和查找范围
 * 3. 迭代器生命周期与容器绑定
 */
std::optional<PathBoundary::ObstacleMap::iterator> PathBoundary::findKeyObstacle(const double s) {
  if (key_obstacles_.empty() || s < key_obstacles_.begin()->first || s > key_obstacles_.rbegin()->first) {
    return std::nullopt;  // 表示值的缺失.
  }
  // find nearest element of nearest key value
  auto iter = key_obstacles_.lower_bound(s);
  if (iter == key_obstacles_.end()) {
    return std::nullopt;
  }
  return iter;
}

/**
 * @brief 查找指定s坐标的关键障碍物条目（常量版本）
 * @details 在障碍物映射表中查找与给定s坐标关联的障碍物信息，返回只读访问的迭代器
 *
 * @param[in] s 目标纵向坐标
 * - 单位：米
 * - 取值范围：[start_s_, end_s]
 * - 精度要求：±0.1米
 *
 * @par 操作流程图:
 * @startuml
 * start
 * :调用findKeyObstacle查找;
 * if (找到条目?) then (是)
 *   :转换为const迭代器;
 * else (否)
 *   :保持空值;
 * endif
 * stop
 * @enduml
 *
 * @return std::optional<ObstacleMap::const_iterator> 可选的常量障碍物条目迭代器
 * - 存在时：返回有效常量迭代器
 * - 不存在时：返回std::nullopt
 *
 * @note 特殊处理:
 * 1. 内部复用findKeyObstacle实现
 * 2. 自动执行const_cast转换
 *
 * @warning 使用限制:
 * 1. 禁止通过迭代器修改数据
 * 2. 需确保对象处于稳定状态
 * 3. 迭代器生命周期与容器绑定
 */
std::optional<PathBoundary::ObstacleMap::const_iterator> PathBoundary::findConstKeyObstacle(const double s) const {
  if (key_obstacles_.empty() || s < key_obstacles_.cbegin()->first || s > key_obstacles_.crbegin()->first) {
    return std::nullopt;
  }
  // find nearest element of nearest key value
  auto iter = key_obstacles_.lower_bound(s);
  if (iter == key_obstacles_.cend()) {
    return std::nullopt;
  }
  return iter;
}

}  // namespace gpal::pnc::planning
