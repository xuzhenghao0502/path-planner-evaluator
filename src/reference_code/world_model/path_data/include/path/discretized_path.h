/**
 * @file discretized_path.h
 * @brief 离散路径类的定义
 * @details 该文件定义了离散路径类DiscretizedPath，用于存储和管理离散路径点序列。
 * 该类提供了路径长度计算、路径点评估、最近点查询等功能。
 */

#pragma once

#include <utility>
#include <vector>

#include "point/path_pt.h"

// #include "proto/common/pnc_point.pb.h"

namespace gpal::pnc::planning {
/**
 * @brief 离散路径容器类
 * @details 用于存储和管理离散路径点序列，提供路径评估、最近点查询等操作
 *
 * @par 数据结构说明:
 * - 继承自std::vector<PathPt>
 * - 路径点按s升序排列
 * - 支持线性插值评估
 *
 * @startuml
 * class DiscretizedPath {
 *   +std::vector<PathPt> path_points_
 *   +double length()
 *   +PathPt evaluate(double s)
 *   +PathPt getNearestPoint(Vec3d pt)
 *   +iterator queryLowerBound(double s)
 *   -- 其他成员函数...
 * }
 * @enduml
 */
class DiscretizedPath : public std::vector<PathPt> {
 public:
  /**
   * @brief 默认构造函数
   * @details 创建一个空的离散路径对象
   */
  DiscretizedPath() = default;

  explicit DiscretizedPath(std::vector<PathPt> path_points);

  double length() const;

  PathPt evaluate(const double path_s) const;

  PathPt evaluateReverse(const double path_s) const;

  PathPt getNearestPoint(const gpal::pnc::planning::math::Vec3d& pt, double& min_dist) const;

  PathPt getNearestPoint(const gpal::pnc::planning::math::Vec3d& pt, double start_s, double end_s,
                         double& min_dist) const;

  void getPathPts(const double lower_s, const double upper_s, DiscretizedPath* path_pts) const;

 protected:
  std::vector<PathPt>::const_iterator queryLowerBound(const double path_s) const;
  std::vector<PathPt>::const_iterator queryUpperBound(const double path_s) const;
};

}  // namespace gpal::pnc::planning
