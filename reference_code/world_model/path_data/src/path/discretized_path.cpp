/**
 * @file discretized_path.cpp
 * @brief 离散路径类的定义
 * @details 该文件定义了离散路径类DiscretizedPath，用于存储和管理离散路径点序列。
 * 该类提供了路径长度计算、路径点评估、最近点查询等功能。
 */

#include "path/discretized_path.h"

#include <algorithm>

#include "Eigen/Dense"
#include "base/log.h"
#include "math/linear_interpolation.h"

namespace gpal::pnc::planning {

namespace {

/**
 * @brief 数值符号判断函数
 * @details 根据输入值判断其符号方向，考虑浮点数精度容差
 *
 * @param[in] value 输入数值
 * - 类型: double
 * - 范围: (-∞, +∞)
 * @param[in] epsilon 精度容差 (默认1e-10)
 * - 类型: double
 * - 范围: > 0
 *
 * @par 判断逻辑:
 * - value > epsilon: 返回1.0
 * - value < -epsilon: 返回-1.0
 * - 其他情况: 返回0.0
 *
 * @par 流程图:
 * @startuml
 * start
 * if (value > epsilon?) then (是)
 *   :返回1.0;
 * elseif (value < -epsilon?) then (是)
 *   :返回-1.0;
 * else (否)
 *   :返回0.0;
 * endif
 * stop
 * @enduml
 *
 * @note 典型应用场景:
 * 1. 牛顿迭代法中方向判断
 * 2. 数值优化问题中的梯度方向
 *
 * @warning 需确保:
 * - epsilon为正值
 * - 不宜设置过大的epsilon值
 */
double sign(double value, double epsilon = 1e-10) {
  if (value > epsilon) {
    return 1.0;
  } else if (value < -epsilon) {
    return -1.0;
  }
  return 0.0;
}

/**
 * @brief 数值范围限定函数
 * @details 将输入值限制在[lower, upper]区间内
 *
 * @param[in] value 输入数值
 * - 类型: double
 * - 范围: (-∞, +∞)
 * @param[in] lower 区间下界
 * - 类型: double
 * @param[in] upper 区间上界
 * - 类型: double
 *
 * @par 处理逻辑:
 * 1. value = min(value, upper)
 * 2. value = max(value, lower)
 *
 * @par 流程图:
 * @startuml
 * start
 * :输入value, lower, upper;
 * :计算最小值\nvalue = min(value, upper);
 * :计算最大值\nvalue = max(value, lower);
 * :返回处理后的value;
 * stop
 * @enduml
 *
 * @note 特性说明:
 * 1. 自动处理lower > upper的情况
 * 2. 返回值始终满足 lower <= clamp(value) <= upper
 *
 * @warning 注意事项:
 * 1. 当lower > upper时，实际生效区间为[upper, lower]
 * 2. 浮点精度问题可能导致边界值不完全精确
 */
double clamp(double value, double lower, double upper) {
  value = std::min(value, upper);
  value = std::max(value, lower);
  return value;
}

/**
 * @brief 计算牛顿迭代步长
 * @details 根据一阶导数(dJ)和二阶导数(ddJ)计算牛顿步长，并限制最大步长
 *
 * @param[in] dJ 目标函数一阶导数
 * - 类型: double
 * - 物理意义: 梯度方向
 * @param[in] ddJ 目标函数二阶导数
 * - 类型: double
 * - 物理意义: Hessian矩阵元素
 * @param[in] max_step 最大允许步长
 * - 类型: double
 * - 范围: > 0
 * @param[in] epsilon 数值稳定性容差 (默认1e-10)
 * - 类型: double
 * - 范围: > 0
 *
 * @par 计算逻辑:
 * 1. 当|ddJ| < epsilon时: 返回符号与dJ相反的max_step
 * 2. 否则: 计算-dJ/ddJ并夹紧到[-max_step, max_step]
 *
 * @par 流程图:
 * @startuml
 * start
 * if (|ddJ| < epsilon?) then (是)
 *   :计算符号方向\nstep = -sign(dJ) * max_step;
 * else (否)
 *   :计算原始步长\nraw_step = -dJ / ddJ;
 *   :夹紧到[-max_step, max_step];
 * endif
 * :返回step;
 * stop
 * @enduml
 *
 * @note 典型应用场景:
 * 1. 路径最近点搜索中的牛顿迭代
 * 2. 数值优化问题中的步长控制
 *
 * @warning 需确保:
 * 1. max_step必须为正数
 * 2. 避免ddJ接近零时步长过大
 */
double getNewtonStep(const double dJ, const double ddJ, const double max_step, double epsilon = 1e-10) {
  if (std::abs(ddJ) < epsilon) {
    return -sign(dJ) * max_step;
  }
  return clamp(-dJ / ddJ, -max_step, max_step);
}

}  // namespace

/**
 * @brief 离散路径构造函数
 * @details 通过移动语义初始化路径点容器，继承std::vector<PathPt>特性
 *
 * @param[in] path_points 路径点集合
 * - 类型: std::vector<PathPt>
 * - 输入要求:
 *   1. 必须按s坐标升序排列
 *   2. 相邻点s差值必须>0 (建议至少0.1米)
 *   3. 允许空向量（需外部处理异常）
 *
 * @par 关键数据结构:
 * - PathPt::s() (double): 路径点累计距离，范围[0, +∞)
 * - PathPt::x()/y() (double): 全局坐标系坐标，单位米
 * - PathPt::theta() (double): 航向角，范围[-π, π]
 *
 * @par 流程图:
 * @startuml
 * start
 * :接收path_points参数;
 * partition 数据初始化 {
 * :移动语义转移数据所有权;
 * :调用基类vector构造函数;
 * }
 * :完成容器初始化;
 * stop
 * @enduml
 *
 * @note 调用方需确保:
 * 1. 输入路径点已正确排序
 * 2. 相邻点间距满足最小分辨率要求
 *
 * @warning 构造函数不验证输入数据的:
 * - 排序正确性
 * - s值单调性
 * - 坐标连续性
 * 需通过外部校验流程保证数据有效性
 */
DiscretizedPath::DiscretizedPath(std::vector<PathPt> path_points) : std::vector<PathPt>(std::move(path_points)) {}

/**
 * @brief 计算离散路径总长度
 * @details 通过首尾路径点的s坐标差值计算路径总长度
 *
 * @return double 路径总长度（单位：米）
 * - 空路径返回0.0
 * - 非空路径计算为 back().s() - front().s()
 *
 * @par 关键变量说明:
 * - front().s() (double): 起始点s坐标，范围[0, +∞)
 * - back().s() (double): 终止点s坐标，范围[front().s(), +∞)
 *
 * @par 判断条件:
 * - 路径点容器empty()状态检查
 *
 * @par 流程图:
 * @startuml
 * start
 * if (路径为空?) then (是)
 *   :返回0.0;
 * else (否)
 *   :获取首点s坐标;
 *   :获取尾点s坐标;
 *   :计算差值得到长度;
 * endif
 * stop
 * @enduml
 *
 * @note 路径点必须满足:
 * 1. 按s升序排列
 * 2. 相邻点s差值必须>0
 *
 * @warning 当路径为空时返回0.0，调用方需自行处理
 */
double DiscretizedPath::length() const {
  if (empty()) {
    return 0.0;
  }
  return back().s() - front().s();
}

/**
 * @brief 路径点插值评估
 * @details 在离散路径点序列中通过线性插值计算指定s坐标的路径点
 *
 * @param[in] path_s 目标s坐标
 * - 类型: double
 * - 单位: 米
 * - 有效范围: [front().s(), back().s()]
 *
 * @par 关键变量说明:
 * - it_lower (iterator): lower_bound查找结果，范围[begin(), end()]
 * - 插值方法: 线性插值（linear approximation）
 *
 * @par 流程图:
 * @startuml
 * start
 * :输入目标s值;
 * partition 路径检查 {
 * if (路径为空?) then (是)
 *   :触发ACHECK断言;
 * else (否)
 *   :使用lower_bound查找s坐标;
 *   if (位于首点前?) then (是)
 *     :返回第一个路径点;
 *   else if (位于末点后?) then (是)
 *     :返回最后一个路径点;
 *   else (中间位置)
 *     :线性插值前后两个路径点;
 *   endif
 * endif
 * }
 * stop
 * @enduml
 *
 * @note 输入s值建议满足:
 * - 应大于等于front().s()
 * - 应小于等于back().s()
 *
 * @warning 空路径调用将触发断言:
 * - 需确保!empty()
 * - 建议调用前检查length() > 0
 */
PathPt DiscretizedPath::evaluate(const double path_s) const {
  ACHECK(!empty());
  auto it_lower = queryLowerBound(path_s);
  if (it_lower == begin()) {
    return front();
  }
  if (it_lower == end()) {
    return back();
  }
  return gpal::pnc::planning::math::interpolateUsingLinearApproximation(*(it_lower - 1), *it_lower, path_s);
}

/**
 * @brief 查找最近路径点
 * @details 遍历离散路径点集合，计算给定三维点到路径的最近距离
 *
 * @param[in] pt 目标点坐标
 * - 类型: math::Vec3d
 * - 单位: 米
 * - 有效范围: x,y,z ∈ (-∞, +∞)
 * @param[out] min_dist 最近距离引用输出
 * - 类型: double&
 * - 单位: 米
 * - 有效范围: [0, +∞)
 *
 * @par 关键变量说明:
 * - it (iterator): 路径点遍历迭代器，范围[begin(), end())
 * - cur_dist (double): 当前点距离，计算方式: 欧氏距离√(Δx²+Δy²+Δz²)
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化min_dist为极大值;
 * :创建临时最近点对象;
 * partition 路径点遍历 {
 * while (遍历所有路径点?) is (是)
 *   :计算当前点与目标点距离;
 *   if (当前距离 < min_dist?) then (是)
 *     :更新min_dist;
 *     :记录当前路径点;
 *   endif
 * endwhile
 * }
 * :返回记录的最近点;
 * stop
 * @enduml
 *
 * @note 算法特性:
 * 1. 线性时间复杂度O(n)
 * 2. 精确匹配最近点（非插值点）
 *
 * @warning 使用约束:
 * - 空路径调用将触发断言
 * - min_dist参数需预先初始化（建议赋极大值）
 * - 需确保pt坐标与路径点坐标系一致
 */
PathPt DiscretizedPath::getNearestPoint(const gpal::pnc::planning::math::Vec3d& pt, double& min_dist) const {
  return getNearestPoint(pt, front().s(), back().s(), min_dist);
}

/**
 * @brief 区间最近路径点查询
 * @details 在指定s坐标区间内查找距离给定三维点最近的路径点
 *
 * @param[in] pt 目标点坐标
 * - 类型: math::Vec3d
 * - 单位: 米
 * @param[in] start_s 区间起始s坐标
 * - 类型: double
 * - 范围: [front().s(), back().s()]
 * @param[in] end_s 区间结束s坐标
 * - 类型: double
 * - 范围: [start_s, back().s()]
 * @param[out] min_dist 最近距离引用输出
 * - 类型: double&
 * - 单位: 米
 *
 * @par 关键变量说明:
 * - start_iter (iterator): lower_bound(start_s)结果，范围[begin(), end()]
 * - end_iter (iterator): upper_bound(end_s)结果，范围[start_iter, end()]
 *
 * @par 流程图:
 * @startuml
 * start
 * :输入目标点和s区间;
 * partition 区间验证 {
 * if (start_s > end_s || 路径为空?) then (是)
 *   :返回无效点并设置min_dist为极大值;
 * else (否)
 *   :使用lower/upper_bound定位区间起止;
 *   :遍历区间内所有路径点;
 * endif
 * }
 * partition 距离计算 {
 * while (遍历区间路径点?) is (是)
 *   :计算当前点与目标点距离;
 *   if (当前距离 < min_dist?) then (是)
 *     :更新min_dist和最近点;
 *   endif
 * endwhile
 * }
 * :返回区间内最近点;
 * stop
 * @enduml
 *
 * @note 与无区间版本的区别:
 * 1. 搜索范围限制在指定s区间
 * 2. 支持局部路径最近点查询
 *
 * @warning 需确保:
 * 1. start_s <= end_s
 * 2. 区间端点不超过路径总长度
 * 3. 空区间将返回无效结果
 */
PathPt DiscretizedPath::getNearestPoint(const gpal::pnc::planning::math::Vec3d& pt, double u0, double u1,
                                        double& min_dist) const {
  PathPt min_pt;
  if (u0 > u1) {
    std::swap(u0, u1);
  }
  std::pair<double, double> s_range(u0, u1);

  constexpr size_t iter_max = 10;
  constexpr double tol = 1e-3;
  double max_step = 0.5 * (s_range.second - s_range.first);
  min_dist = std::numeric_limits<double>::max();
  std::pair<double, double> newton_range(u0, u1);
  double min_path_s = 0;
  bool min_found = false;
  double ui = s_range.first;
  bool has_extend_front = false;
  bool has_extend_rear = false;
  for (size_t i = 0; i < iter_max; i++) {
    auto path_pt = evaluate(ui);
    Eigen::Vector2d pe(path_pt.x() - pt.x(), path_pt.y() - pt.y());
    double dist = pe.norm();
    if (dist < min_dist) {
      min_dist = dist;
      min_path_s = ui;
    }
    const double cos_theta = cos(path_pt.theta());
    const double sin_theta = sin(path_pt.theta());
    Eigen::Vector2d dp(cos_theta, sin_theta);
    Eigen::Vector2d ddp(-sin_theta * path_pt.kappa(), cos_theta * path_pt.kappa());
    const double v = 1.0;
    const double dJ = v * (dp.transpose() * pe)(0);
    const double ddJ = (v * ddp.transpose() * pe + dp.transpose() * dp)(0);
    const double du = getNewtonStep(dJ, ddJ, max_step);
    if (std::abs(du) < tol) {
      min_found = true;
      break;
    }
    ui = path_pt.s() + du;
    if (du > 0) {
      newton_range.first = std::max(std::max(path_pt.s(), newton_range.first), s_range.first);
      newton_range.second = std::min(std::max(path_pt.s(), newton_range.second), s_range.second);
    } else if (du < 0) {
      newton_range.first = std::max(std::min(path_pt.s(), newton_range.first), s_range.first);
      newton_range.second = std::min(std::min(path_pt.s(), newton_range.second), s_range.second);
    }
    if (ui < s_range.first) {
      if (!has_extend_front) {
        has_extend_front = true;
        ui = min_path_s;
        max_step *= 0.5;
        continue;
      }
      break;
    } else if (ui > s_range.second) {
      if (!has_extend_rear) {
        has_extend_rear = true;
        ui = min_path_s;
        max_step *= 0.5;
        continue;
      }
      break;
    }
  }
  min_pt = evaluate(min_path_s);
  if (!min_found) {
    if (newton_range.first > newton_range.second) {
      std::swap(newton_range.first, newton_range.second);
    }
    auto it_lower = queryLowerBound(newton_range.first);
    auto it_upper = queryLowerBound(newton_range.second);
    for (auto iter = it_lower; iter != it_upper; iter++) {
      double dist = iter->DistanceTo(pt);
      if (dist < min_dist) {
        min_dist = dist;
        min_pt = *iter;
      }
    }
  }
  return min_pt;
}

/**
 * @brief 查找路径点下界
 * @details 使用二分查找获取第一个不小于目标s坐标的路径点迭代器
 *
 * @param[in] path_s 目标s坐标
 * - 类型: double
 * - 单位: 米
 * - 有效范围: (-∞, +∞)
 *
 * @par 关键变量说明:
 * - std::lower_bound: 使用STL算法实现，时间复杂度O(log n)
 * - 比较函数: 基于PathPt::s进行比较
 *
 * @par 流程图:
 * @startuml
 * start
 * partition 二分查找流程 {
 * if (路径为空?) then (是)
 *   :返回end迭代器;
 * else (否)
 *   :初始化首尾迭代器;
 *   while (查找区间有效?) is (是)
 *     :计算中间点s坐标;
 *     if (中间点s < 目标s?) then (是)
 *       :调整查找区间到右侧;
 *     else (否)
 *       :调整查找区间到左侧;
 *     endif
 *   endwhile
 *   :返回第一个不小于目标s的迭代器;
 * endif
 * }
 * stop
 * @enduml
 *
 * @note 算法特性:
 * 1. 要求路径点按s严格升序排列
 * 2. 当所有点s < path_s时返回end()
 *
 * @warning 使用前提:
 * - 路径点必须已排序
 * - 调用前应校验路径非空
 */
std::vector<PathPt>::const_iterator DiscretizedPath::queryLowerBound(const double path_s) const {
  auto func = [](const PathPt& tp, const double path_s) { return tp.s() < path_s; };
  return std::lower_bound(begin(), end(), path_s, func);
}

/**
 * @brief 逆向路径点插值评估
 * @details 从路径末端开始查找，通过线性插值计算指定s坐标的路径点
 *
 * @param[in] path_s 目标s坐标
 * - 类型: double
 * - 单位: 米
 * - 有效范围: [front().s(), back().s()]
 *
 * @par 关键变量说明:
 * - it_upper (iterator): upper_bound查找结果，范围[begin(), end()]
 * - 插值方向: 从末端向前逆向查找
 *
 * @par 流程图:
 * @startuml
 * start
 * :输入目标s值;
 * partition 路径检查 {
 * if (路径为空?) then (是)
 *   :触发ACHECK断言;
 * else (否)
 *   :使用upper_bound逆向查找s坐标;
 *   if (位于末点后?) then (是)
 *     :返回最后一个路径点;
 *   else if (位于首点前?) then (是)
 *     :返回第一个路径点;
 *   else (中间位置)
 *     :逆向线性插值前后两个路径点;
 *   endif
 * endif
 * }
 * stop
 * @enduml
 *
 * @note 与evaluate()的主要区别:
 * 1. 使用upper_bound代替lower_bound
 * 2. 插值方向从路径末端开始
 *
 * @warning 空路径调用将触发断言:
 * - 需确保!empty()
 * - 建议优先使用evaluate()方法
 */
PathPt DiscretizedPath::evaluateReverse(const double path_s) const {
  ACHECK(!empty());
  auto it_upper = queryUpperBound(path_s);
  if (it_upper == begin()) {
    return front();
  }
  if (it_upper == end()) {
    return back();
  }
  return gpal::pnc::planning::math::interpolateUsingLinearApproximation(*(it_upper - 1), *it_upper, path_s);
}

/**
 * @brief 查找路径点上界
 * @details 使用二分查找获取第一个大于目标s坐标的路径点迭代器
 *
 * @param[in] path_s 目标s坐标
 * - 类型: double
 * - 单位: 米
 * - 有效范围: (-∞, +∞)
 *
 * @par 关键变量说明:
 * - std::upper_bound: 使用STL算法实现，时间复杂度O(log n)
 * - 比较函数: 基于PathPt::s进行比较
 *
 * @par 流程图:
 * @startuml
 * start
 * partition 二分查找流程 {
 * if (路径为空?) then (是)
 *   :返回end迭代器;
 * else (否)
 *   :初始化首尾迭代器;
 *   while (查找区间有效?) is (是)
 *     :计算中间点s坐标;
 *     if (目标s < 中间点s?) then (是)
 *       :调整查找区间到左侧;
 *     else (否)
 *       :调整查找区间到右侧;
 *     endif
 *   endwhile
 *   :返回第一个大于目标s的迭代器;
 * endif
 * }
 * stop
 * @enduml
 *
 * @note 算法特性:
 * 1. 返回迭代器总是指向第一个大于path_s的元素
 * 2. 当所有点s <= path_s时返回end()
 *
 * @warning 使用前提:
 * - 路径点必须严格按s升序排列
 * - 通常与queryLowerBound配合使用定义区间[lower, upper)
 */
std::vector<PathPt>::const_iterator DiscretizedPath::queryUpperBound(const double path_s) const {
  auto func = [](const double path_s, const PathPt& tp) { return path_s < tp.s(); };
  return std::upper_bound(begin(), end(), path_s, func);
}

/**
 * @brief 提取路径区间点集
 * @details 获取s坐标区间[lower_s, upper_s)内的所有路径点
 *
 * @param[in] lower_s 区间起始s坐标
 * - 类型: double
 * - 范围: [front().s(), back().s()]
 * @param[in] upper_s 区间结束s坐标
 * - 类型: double
 * - 范围: [lower_s, back().s()]
 * @param[out] path_pts 输出路径点容器指针
 * - 类型: DiscretizedPath*
 * - 要求: 必须预先初始化
 *
 * @par 关键变量说明:
 * - start_iter (iterator): lower_bound查找结果，范围[begin(), end()]
 * - end_iter (iterator): upper_bound查找结果，范围[start_iter, end()]
 *
 * @par 流程图:
 * @startuml
 * start
 * :输入s区间参数;
 * partition 参数校验 {
 * if (lower_s > upper_s || 路径为空?) then (是)
 *   :清空输出容器;
 * else (否)
 *   :使用lower_bound定位起点;
 *   :使用upper_bound定位终点;
 *   :将[start_iter, end_iter)插入输出容器;
 * endif
 * }
 * stop
 * @enduml
 *
 * @note 区间特性:
 * 1. 左闭右开区间: lower_s <= s < upper_s
 * 2. 输出容器会被先清空再填充
 *
 * @warning 需确保:
 * 1. lower_s <= upper_s
 * 2. 输出指针非空
 * 3. 区间端点不超过路径总长度
 */
void DiscretizedPath::getPathPts(const double lower_s, const double upper_s, DiscretizedPath* path_pts) const {
  path_pts->clear();
  constexpr double min_legnth = 1.0;
  if (upper_s < lower_s + 1e-6) {
    return;
  }
  auto iter_lower = queryLowerBound(lower_s);
  if (iter_lower != end()) {
    auto iter_upper = queryUpperBound(upper_s);
    path_pts->assign(iter_lower, iter_upper);
  }
}

}  // namespace gpal::pnc::planning
