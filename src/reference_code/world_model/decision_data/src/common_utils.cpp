/**
 * @file common_utils.cpp
 * @brief 包含决策数据相关的通用工具函数，主要实现边界信息的线性插值
 * @author 智能驾驶决策规划团队
 * @date 2024-03-01
 * @version 1.0
 * @copyright Copyright (C) 2024 Shanghai Geometrical Perception and Learning Co., Ltd. All rights reserved.
 * @note 模块功能：
 * - 根据给定的 s 值和边界点列表进行线性插值，计算 l_left、l_right、left_type 和 right_type
 */

#include "decision_data/common_utils.h"

#include <cmath>

namespace gpal::pnc::planning {
namespace Decision {
/**
 * @brief 线性插值函数，根据 s 值计算 l_left、l_right、left_type 和 right_type
 * @param[in] s 用于插值计算的参考 s 值
 * @param[in] points 边界点列表，包含多个边界点信息
 * @par 输入参数说明:
 * - s: 类型 `double`，取值范围为 (-∞, +∞)
 * - points: 类型 `const BoundPointList&`，有效的边界点列表对象引用
 * @return std::tuple<double, double, BoundaryPointTypeInfo, BoundaryPointTypeInfo>
 *         包含插值计算得到的 l_left、l_right、left_type 和 right_type
 * @par 处理流程：
 * @startuml
 * start
 * if (points 列表为空) then (是)
 *   :返回 (0.0, 0.0, BoundaryPointTypeInfo::INVALID, BoundaryPointTypeInfo::INVALID);
 *   stop
 * else (否)
 *   :使用 std::lower_bound 找到最接近 s 的两个点;
 *   if (找到的迭代器指向列表开头) then (是)
 *     :返回列表第一个点的 l_left、l_right、left_type 和 right_type;
 *     stop
 *   else if (找到的迭代器指向列表末尾) then (是)
 *     :返回列表最后一个点的 l_left、l_right、left_type 和 right_type;
 *     stop
 *   else (其他情况)
 *     :获取最接近的两个点 p1 和 p2;
 *     :计算插值比例 ratio;
 *     :线性插值计算 l_left 和 l_right;
 *     :根据 ratio 选择最接近点的 left_type 和 right_type;
 *     :返回计算得到的 l_left、l_right、left_type 和 right_type;
 *     stop
 *   endif
 * endif
 * @enduml
 */
std::tuple<double, double, BoundaryPointTypeInfo, BoundaryPointTypeInfo> LateralBoundConsInterpolate(
    double s, const BoundPointList& points) {
  if (points.empty()) {
    return std::make_tuple(0.0, 0.0, BoundaryPointTypeInfo::INVALID, BoundaryPointTypeInfo::INVALID);
  }

  // 找到最接近的两个点
  auto it = std::lower_bound(points.begin(), points.end(), s, [](const BoundaryPoint& p, double s) { return p.s < s; });

  if (it == points.begin()) {
    return std::make_tuple(points.front().l_left, points.front().l_right, points.front().left_type,
                           points.front().right_type);
  }
  if (it == points.end()) {
    return std::make_tuple(points.back().l_left, points.back().l_right, points.back().left_type,
                           points.back().right_type);
  }

  const BoundaryPoint& p1 = *(it - 1);
  const BoundaryPoint& p2 = *it;

  // 计算插值比例
  double ratio = (s - p1.s) / (p2.s - p1.s);

  // 线性插值计算l_left和l_right
  double l_left = p1.l_left + ratio * (p2.l_left - p1.l_left);
  double l_right = p1.l_right + ratio * (p2.l_right - p1.l_right);

  // 对于类型，我们选择最接近的点的类型
  BoundaryPointTypeInfo left_type = ratio < 0.5 ? p1.left_type : p2.left_type;
  BoundaryPointTypeInfo right_type = ratio < 0.5 ? p1.right_type : p2.right_type;

  return std::make_tuple(l_left, l_right, left_type, right_type);
}

}  // namespace Decision
}  // namespace gpal::pnc::planning
