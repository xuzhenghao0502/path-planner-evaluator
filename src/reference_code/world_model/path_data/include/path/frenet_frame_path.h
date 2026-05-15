/**
 * @file frenet_frame_path.h
 * @brief Frenet坐标系路径类定义
 * @details 提供Frenet坐标系下的路径点序列及相关操作
 */

#pragma once

#include <utility>
#include <vector>

#include "proto/common/pnc_point.pb.h"
#include "proto/common/sl_boundary.pb.h"

namespace gpal::pnc::planning {

/**
 * @brief Frenet坐标系路径容器
 * @details 存储Frenet坐标系下的路径点序列，提供路径评估、最近点查询等操作
 *
 * @par 数据结构说明:
 * - 继承自std::vector<FrenetFramePoint>
 * - 路径点按s升序排列
 * - 支持基于s坐标的线性插值
 *
 * @startuml
 * class FrenetFramePath {
 *   +std::vector<FrenetFramePoint> frenet_points_
 *   +double length()
 *   +FrenetFramePoint evaluateByS(double s)
 *   +FrenetFramePoint getNearestPoint(const SLBoundary& sl)
 *   -- 比较函数实现二分查找...
 * }
 * @enduml
 */
class FrenetFramePath : public std::vector<gpal::pnc::FrenetFramePoint> {
 public:
  /**
   * @brief 默认构造函数
   * @details 创建空的Frenet坐标系路径对象
   */
  FrenetFramePath() = default;
  explicit FrenetFramePath(std::vector<gpal::pnc::FrenetFramePoint> points);

  /**
   * @brief 设置Frenet路径数据
   * @details 使用移动语义高效更新当前路径内容，替换原有数据
   *
   * @param[in] path 新的Frenet路径点集合
   * - 类型: std::vector<FrenetFramePoint>
   * - 要求: 必须按s坐标严格升序排列
   * - 所有权: 调用后原path变为未定义状态
   *
   * @par 流程图:
   * @startuml
   * start
   * :输入新路径数据;
   * stop
   * @enduml
   *
   * @note 特性说明:
   * 1. 使用std::move避免数据拷贝
   * 2. 操作复杂度为O(1)
   *
   * @warning 需确保:
   * 1. 新路径数据已正确排序
   */
  void setFrenetFramePath(std::vector<gpal::pnc::FrenetFramePoint> path) { *this = FrenetFramePath(std::move(path)); }

  /**
   * @brief 获取当前Frenet路径数据
   * @details 返回当前路径的不可变引用，用于访问路径点序列
   *
   * @return const std::vector<FrenetFramePoint>&
   * - 返回值类型: Frenet路径点集合的常量引用
   * - 数据约束: 路径点按s升序排列
   * - 生命周期: 与当前对象实例绑定
   *
   * @par 流程图:
   * @startuml
   * start
   * :返回当前对象引用;
   * stop
   * @enduml
   *
   * @note 使用场景:
   * 1. 路径可视化时读取数据
   * 2. 进行只读的路径分析操作
   *
   * @warning 重要说明:
   * 1. 禁止通过此引用修改内部数据
   * 2. 引用有效性依赖对象生命周期
   */
  const std::vector<gpal::pnc::FrenetFramePoint>& getFrenetFramePath() const { return *this; }

  /**
   * @brief 清空Frenet路径数据
   * @details 移除所有路径点，将容器恢复为空状态
   *
   * @par 操作效果:
   * - length()将返回0.0
   * - 所有迭代器失效
   * - 释放已分配的内存
   *
   * @par 流程图:
   * @startuml
   * start
   * :调用基类clear方法;
   * stop
   * @enduml
   *
   * @note 典型使用场景:
   * 1. 需要重置路径数据时
   * 2. 对象重用前的清理操作
   *
   * @warning 重要说明:
   * 1. 清空后原有数据不可恢复
   * 2. 操作时间复杂度为O(n)
   */
  void clearFrenetFramePath() { clear(); }

  double length() const;
  gpal::pnc::FrenetFramePoint evaluateByS(const double s) const;

  gpal::pnc::FrenetFramePoint getNearestPoint(const SLBoundary& sl) const;

 private:
  /**
   * @brief lower_bound比较函数
   * @details 用于std::lower_bound算法，判断路径点s坐标是否小于目标s值
   *
   * @param[in] p Frenet路径点
   * - 类型: FrenetFramePoint
   * - 必须包含有效s坐标
   * @param[in] s 目标s值
   * - 类型: double
   * - 单位: 米
   *
   * @par 流程图:
   * @startuml
   * start
   * :输入Frenet点p和s值;
   * if (p.s() < s) then (是)
   *   :返回true;
   * else (否)
   *   :返回false;
   * endif
   * stop
   * @enduml
   *
   * @note 使用场景:
   * 1. 在有序容器中查找第一个不小于s的路径点
   * 2. 需要二分查找s坐标时
   *
   * @warning 需确保:
   * 1. 路径点已按s升序排列
   * 2. 比较函数与排序规则一致
   */
  static bool lowerBoundComparator(const gpal::pnc::FrenetFramePoint& p, const double s) { return p.s() < s; }

  /**
   * @brief upper_bound比较函数
   * @details 用于std::upper_bound算法，判断目标s值是否小于路径点s坐标
   *
   * @param[in] s 目标s值
   * - 类型: double
   * - 单位: 米
   * @param[in] p Frenet路径点
   * - 类型: FrenetFramePoint
   * - 必须包含有效s坐标
   *
   * @par 流程图:
   * @startuml
   * start
   * :输入s值和Frenet点p;
   * if (s < p.s()) then (是)
   *   :返回true;
   * else (否)
   *   :返回false;
   * endif
   * stop
   * @enduml
   *
   * @note 使用场景:
   * 1. 在有序容器中查找第一个大于s的路径点
   * 2. 需要确定s值插入位置时
   *
   * @warning 需确保:
   * 1. 路径点已严格按s升序排列
   * 2. 比较函数与排序规则一致
   */
  static bool upperBoundComparator(const double s, const gpal::pnc::FrenetFramePoint& p) { return s < p.s(); }
};

}  // namespace gpal::pnc::planning
