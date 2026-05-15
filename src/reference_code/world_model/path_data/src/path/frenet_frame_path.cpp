/**
 * @file frenet_frame_path.cpp
 * @brief Frenet坐标系路径类定义
 * @details 提供Frenet坐标系下的路径点序列及相关操作
 */

#include "path/frenet_frame_path.h"

#include <algorithm>
#include <limits>

#include "base/log.h"
#include "math/linear_interpolation.h"

namespace gpal::pnc::planning {

using gpal::pnc::FrenetFramePoint;

/**
 * @brief 参数化构造函数
 * @details 通过已有Frenet路径点集合初始化路径对象
 *
 * @param[in] points Frenet坐标系点集合
 * - 类型: std::vector<FrenetFramePoint>
 * - 要求: 必须按s坐标严格升序排列
 * - 所有权: 构造后内部数据由本对象管理
 *
 * @par 流程图:
 * @startuml
 * start
 * :输入Frenet点集合;
 * partition 数据校验 {
 * if (points为空?) then (是)
 *   :创建空路径对象;
 * else (否)
 *   :移动语义接管数据;
 * endif
 * }
 * stop
 * @enduml
 *
 * @note 特性说明:
 * 1. 使用移动语义避免数据拷贝
 * 2. 构造后原输入vector变为未定义状态
 *
 * @warning 必须确保:
 * 1. 输入的points已按s升序排列
 * 2. 非空时至少包含两个点
 */
FrenetFramePath::FrenetFramePath(std::vector<FrenetFramePoint> points)
    : std::vector<FrenetFramePoint>(std::move(points)) {}

/**
 * @brief 计算Frenet路径总长度
 * @details 通过首尾点s坐标差值计算路径总长度
 *
 * @return double
 * - 单位: 米
 * - 范围: [0, +∞)
 * - 空路径返回0.0
 *
 * @par 流程图:
 * @startuml
 * start
 * if (路径为空?) then (是)
 *   :返回0.0;
 * else (否)
 *   :计算back().s() - front().s();
 * endif
 * stop
 * @enduml
 *
 * @note 计算前提:
 * 1. 路径点必须按s升序排列
 * 2. 至少包含两个点才有意义
 *
 * @warning 注意:
 * 当路径点未正确排序时，计算结果将不准确
 */
double FrenetFramePath::length() const {
  if (empty()) {
    return 0.0;
  }
  return back().s() - front().s();
}

/**
 * @brief 获取SL边界内的最近点
 * @details 在[s_start, s_end]区间内查找横向距离(l)最小的Frenet点
 *
 * @param[in] sl SL边界约束
 * - s_start: 起始s坐标 (单位: 米)
 * - s_end: 结束s坐标 (单位: 米)
 * - l_lower: 横向下界 (单位: 米)
 * - l_upper: 横向上界 (单位: 米)
 *
 * @par 关键变量说明:
 * - min_l (double): 最小横向距离，初始值设为最大值
 * - min_point (FrenetFramePoint): 当前找到的最小点
 * - search_range (pair<iterator>): 通过二分查找确定的搜索区间
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化min_l为DBL_MAX;
 * partition 确定搜索范围 {
 *   :使用lower_bound查找s_start;
 *   :使用upper_bound查找s_end;
 * }
 * partition 区间遍历 {
 *   while (遍历区间内所有点?) is (是)
 *     if (点.l()在[l_lower, l_upper]区间内?) then (是)
 *       :更新min_l和min_point;
 *     endif
 *   endwhile
 * }
 * :返回min_point;
 * stop
 * @enduml
 *
 * @note 查找策略:
 * 1. 优先返回完全在SL边界内的点
 * 2. 若无则返回区间内l最小的点
 *
 * @warning 必须确保:
 * 1. SLBoundary的s_start <= s_end
 * 2. 路径点已按s升序排列
 */
FrenetFramePoint FrenetFramePath::getNearestPoint(const SLBoundary& sl) const {
  auto it_lower = std::lower_bound(begin(), end(), sl.start_s(), lowerBoundComparator);
  if (it_lower == end()) {
    return back();
  }
  auto it_upper = std::upper_bound(it_lower, end(), sl.end_s(), upperBoundComparator);
  double min_dist = std::numeric_limits<double>::max();
  auto min_it = it_upper;
  for (auto it = it_lower; it != it_upper; ++it) {
    if (it->l() >= sl.start_l() && it->l() <= sl.end_l()) {
      return *it;
    } else if (it->l() > sl.end_l()) {
      double diff = it->l() - sl.end_l();
      if (diff < min_dist) {
        min_dist = diff;
        min_it = it;
      }
    } else {
      double diff = sl.start_l() - it->l();
      if (diff < min_dist) {
        min_dist = diff;
        min_it = it;
      }
    }
  }
  return *min_it;
}

/**
 * @brief 通过s坐标插值路径点
 * @details 在Frenet路径上通过线性插值获取指定s坐标对应的路径点
 *
 * @param[in] s 目标s坐标
 * - 类型: double
 * - 单位: 米
 * - 有效范围: [front().s(), back().s()]
 *
 * @par 关键变量说明:
 * - lower_bound: 第一个不小于s的路径点迭代器
 * - 插值权重: 根据相邻点s坐标计算线性比例
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查路径是否为空;
 * if (路径为空?) then (是)
 *   :触发断言错误;
 * else (否)
 *   partition 查找插值区间 {
 *     :使用lower_bound查找s位置;
 *     if (在起始位置?) then (是)
 *       :返回首点;
 *     elseif (在结束位置?) then (是)
 *       :返回尾点;
 *     else (否)
 *       :在前后点间线性插值;
 *     endif
 *   }
 * endif
 * stop
 * @enduml
 *
 * @note 插值规则:
 * 1. s超出范围时返回端点
 * 2. 使用相邻点s差计算插值比例
 *
 * @warning 必须确保:
 * 1. 路径非空
 * 2. 路径点已按s升序排列
 */
FrenetFramePoint FrenetFramePath::evaluateByS(const double s) const {
  CHECK_GT(size(), 1U);
  auto it_lower = std::lower_bound(begin(), end(), s, lowerBoundComparator);
  if (it_lower == begin()) {
    return front();
  } else if (it_lower == end()) {
    return back();
  }
  const auto& p0 = *(it_lower - 1);
  const auto s0 = p0.s();
  const auto& p1 = *it_lower;
  const auto s1 = p1.s();

  FrenetFramePoint p;
  p.set_s(s);
  p.set_l(gpal::pnc::planning::math::lerp(p0.l(), s0, p1.l(), s1, s));
  p.set_dl(gpal::pnc::planning::math::lerp(p0.dl(), s0, p1.dl(), s1, s));
  p.set_ddl(gpal::pnc::planning::math::lerp(p0.ddl(), s0, p1.ddl(), s1, s));
  return p;
}

}  // namespace gpal::pnc::planning
