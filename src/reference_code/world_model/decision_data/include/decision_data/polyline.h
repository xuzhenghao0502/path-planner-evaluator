/**
 * @file polyline.h
 * @brief 定义折线和感知边界相关的类和枚举，用于处理边界信息
 * @author 智能驾驶决策规划团队
 * @date 2024-03-01
 * @version 1.0
 * @copyright Copyright (C) 2024 Shanghai Geometrical Perception and Learning Co., Ltd. All rights reserved.
 * @note 模块功能：
 * - 表示折线对象和感知边界对象，提供边界信息的存储和操作接口
 */

#pragma once
#include <vector>

#include "math/vec2d.h"
#include "util/base_struct.h"

namespace gpal {
namespace pnc {
namespace planning {
constexpr double kLaneWidth = 3.75;
constexpr double kWithinOneLaneThreshold = kLaneWidth;
constexpr double kWithinTwoLaneThreshold = 2.0 * kLaneWidth;

/// @brief 定义折线的类型枚举
enum class PolylineType {
  POLYLINE_TYPE_INVALID = 0,    /// @brief 类别无效
  POLYLINE_TYPE_CURB = 1,       /// @brief 低矮路沿
  POLYLINE_TYPE_BRUSHWOOD = 2,  /// @brief 草丛、灌木丛、绿化带等
  POLYLINE_TYPE_GUARDRAIL = 3,  /// @brief 护栏，包含金属，水泥护栏等
  POLYLINE_TYPE_WALL = 4,       /// @brief 墙壁，高度高于护栏，包含水泥砖墙壁、高速隔音墙等
  POLYLINE_TYPE_UNKNOWN = 99    /// @brief 类别未知
};

/// @brief 定义感知状态的枚举
enum class PerceptionStatus {
  POLYLINE_STATUS_INVALID = 0,                 /// @brief 状态无效
  POLYLINE_STATUS_CRUISE = 1,                  /// @brief 行车状态，边界精度低，顶点数少
  POLYLINE_STATUS_PARKING_SLOT_DETECTION = 2,  /// @brief 泊车寻库状态，边界精度中，顶点数中
  POLYLINE_STATUS_PARKING = 3                  /// @brief 泊车状态，边界精度高，顶点数多，可认为是原始栅格边界的散点
};

/// @brief 表示一个折线对象，由一系列的二维点和折线类型组成。
class Polyline {
 public:
  /**
   * @brief 默认构造函数
   */
  Polyline() = default;

  /**
   * @brief 带参数的构造函数
   * @param points 折线的点集
   * @param type 折线的类型，默认为无效类型
   */
  Polyline(std::vector<math::Vec3d> points, PolylineType type = PolylineType::POLYLINE_TYPE_INVALID)
      : points_(std::move(points)), type_(type) {}
  Polyline(const Polyline& polyline)
      : points_(polyline.points_),
        origin_points_(polyline.origin_points_),
        sl_points_(polyline.sl_points_),
        type_(polyline.type_) {}

  /**
   * @brief 获取折线的点集（常量引用）
   * @return 折线的点集的常量引用
   */
  const std::vector<math::Vec3d>& points() const { return points_; }
  const std::vector<math::Vec3d>& origin_points() const { return origin_points_; }

  /**
   * @brief 获取可修改的折线点集
   * @return 折线的点集的引用
   */
  std::vector<math::Vec3d>& mutable_points() { return points_; }
  void set_points(std::vector<math::Vec3d> points) { points_ = points; }
  std::vector<math::Vec3d>& mutable_origin_points() { return origin_points_; }
  void set_origin_points(std::vector<math::Vec3d> origin_points) { origin_points_ = origin_points; }

  /**
   * @brief 获取折线的类型（常量）
   * @return 折线的类型
   */
  const PolylineType type() const { return type_; }

  /**
   * @brief 获取可修改的折线类型
   * @return 折线的类型的引用
   */
  PolylineType& mutable_type() { return type_; }

  /**
   * @brief 判断折线是否为空（点数量少于 2 个）
   * @return 如果折线为空返回 true，否则返回 false
   */
  bool empty() const { return points_.size() < 2; }

  /**
   * @brief 获取折线的点数量
   * @return 折线的点数量
   */
  size_t size() const { return points_.size(); }

 private:
  std::vector<math::Vec3d> points_;
  std::vector<math::Vec3d> origin_points_;
  std::vector<gpal::pnc::SLPoint> sl_points_;
  PolylineType type_ = PolylineType::POLYLINE_TYPE_INVALID;
};

/**
 * @brief 表示一个感知边界对象，由多个折线和感知状态组成。
 */
class PolylinesPerceptionBoundary : public StampedBase {
 public:
  /**
   * @brief 默认构造函数
   */
  PolylinesPerceptionBoundary() = default;

  /**
   * @brief 带参数的构造函数
   * @param polylines 感知边界的折线集
   * @param status 感知状态，默认为无效状态
   */
  PolylinesPerceptionBoundary(std::vector<Polyline> polylines,
                              PerceptionStatus status = PerceptionStatus::POLYLINE_STATUS_INVALID)
      : polylines_(std::move(polylines)), status_(status) {}

  /**
   * @brief 获取感知边界的折线集（常量引用）
   * @return 感知边界的折线集的常量引用
   */
  const std::vector<Polyline>& polylines() const { return polylines_; }
  const std::vector<Polyline>& polylines_within_one_lane_width() const { return polylines_within_one_lane_width_; }
  const std::vector<Polyline>& polylines_within_two_lane_width() const { return polylines_within_two_lane_width_; }
  const std::vector<Polyline>& polylines_outside_two_lane_width() const { return polylines_outside_two_lane_width_; }

  /**
   * @brief 获取可修改的感知边界折线集
   * @return 感知边界的折线集的引用
   */
  std::vector<Polyline>& mutable_polylines() { return polylines_; }
  std::vector<Polyline>& mutable_polylines_within_one_lane_width() { return polylines_within_one_lane_width_; }
  std::vector<Polyline>& mutable_polylines_within_two_lane_width() { return polylines_within_two_lane_width_; }
  std::vector<Polyline>& mutable_polylines_outside_two_lane_width() { return polylines_outside_two_lane_width_; }

  /**
   * @brief 获取感知状态（常量）
   * @return 感知状态
   */
  const PerceptionStatus status() const { return status_; }

  /**
   * @brief 获取可修改的感知状态
   * @return 感知状态的引用
   */
  PerceptionStatus& mutable_status() { return status_; }

  /**
   * @brief 判断感知边界是否为空
   * @return 如果感知边界为空返回 true，否则返回 false
   */
  bool empty() const { return polylines_.empty(); }

  /**
   * @brief 获取感知边界的折线数量
   * @return 感知边界的折线数量
   */
  size_t size() const { return polylines_.size(); }

  void set_forward_range(double forward_range) { forward_range_ = forward_range; }
  void set_backward_range(double backward_range) { backward_range_ = backward_range; }
  void set_left_range(double left_range) { left_range_ = left_range; }
  void set_right_range(double right_range) { right_range_ = right_range; }
  void set_fpoint_resolution(double fpoint_resolution) { fpoint_resolution_ = fpoint_resolution; }
  void set_status(PerceptionStatus status) { status_ = status; }

  const double forward_range() const { return forward_range_; }
  const double backward_range() const { return backward_range_; }
  const double left_range() const { return left_range_; }
  const double right_range() const { return right_range_; }
  const double fpoint_resolution() const { return fpoint_resolution_; }
  const PerceptionStatus& get_status() const { return status_; }
  void set_base_relative_time(double base_relative_time) { base_relative_time_ = base_relative_time; }
  const double get_base_relative_time() const { return base_relative_time_; }

 private:
  double forward_range_ = 0.0;                                           /// @brief 边界输出前向范围 unit：m
  double backward_range_ = 0.0;                                          /// @brief 边界输出后向范围 unit：m
  double left_range_ = 0.0;                                              /// @brief 边界输出左侧范围 unit：m
  double right_range_ = 0.0;                                             /// @brief 边界输出右侧范围 unit：m
  double fpoint_resolution_ = 0.0;                                       /// @brief 边界算法中单个栅格作用范围 unit：m
  std::vector<Polyline> polylines_;                                      /// @brief/ 感知边界的折线集
  std::vector<Polyline> polylines_within_one_lane_width_;                /// @brief 感知边界在单车道宽度内的折线集
  std::vector<Polyline> polylines_within_two_lane_width_;                /// @brief 感知边界在双车道宽度内的折线集
  std::vector<Polyline> polylines_outside_two_lane_width_;               /// @brief 感知边界在双车道宽度外的折线集
  PerceptionStatus status_ = PerceptionStatus::POLYLINE_STATUS_INVALID;  /// @brief 感知状态，默认为无效状态
  double base_relative_time_ = 0.0;
};

}  // namespace planning
}  // namespace pnc
}  // namespace gpal
