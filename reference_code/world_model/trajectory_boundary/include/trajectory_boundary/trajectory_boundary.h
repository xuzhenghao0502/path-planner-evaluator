#pragma once

#include "trajectory_boundary/base_boundary.h"
#include "reference_line/reference_line.h"

namespace gpal::pnc::planning {

/**
 * @brief Traits for ReferencePoint
 */
template <>
struct Traits<ReferencePoint> {
  static double indice(const ReferencePoint& p) { return p.local_s(); }
};
/**
 * @brief 基于s的轨迹边界类
 * @details 管理基于s的横向边界和速度边界
 */
class TrajectoryBoundary : public BoundaryManagerBase<ReferencePoint, TrajectoryBoundary> {
 public:
  using BoundaryManagerBase<ReferencePoint, TrajectoryBoundary>::BoundaryManagerBase;
  TrajectoryBoundary() = default;

  const math::IntervalData<Boundary>& softLateralBound() const { return soft_lateral_bound_; }
  math::IntervalData<Boundary>& mutableSoftLateralBound() { return soft_lateral_bound_; }
  void setSoftLateralBound(const math::IntervalData<Boundary>& soft_lateral_bound) {
    soft_lateral_bound_ = soft_lateral_bound;
  }

  void updateSoftLateralBound(const std::vector<Boundary>& lateral_constraints);
  // 支持移动语义的重载
  void updateSoftLateralBound(std::vector<Boundary>&& lateral_constraints);


  const math::IntervalData<Boundary>& hardLateralBound() const { return hard_lateral_bound_; }
  math::IntervalData<Boundary>& mutableHardLateralBound() { return hard_lateral_bound_; }
  void setHardLateralBound(const math::IntervalData<Boundary>& hard_lateral_bound) {
    hard_lateral_bound_ = hard_lateral_bound;
  }

  void updateHardLateralBound(const std::vector<Boundary>& lateral_constraints);
  // 支持移动语义的重载
  void updateHardLateralBound(std::vector<Boundary>&& lateral_constraints);


  const math::IntervalData<Boundary>& softSpeedBound() const { return soft_speed_bound_; }
  math::IntervalData<Boundary>& mutableSoftSpeedBound() { return soft_speed_bound_; }
  void setSoftSpeedBound(const math::IntervalData<Boundary>& soft_speed_bound) { soft_speed_bound_ = soft_speed_bound; }

  void updateSoftSpeedBound(const std::vector<Boundary>& speed_constraints);
  // 支持移动语义的重载
  void updateSoftSpeedBound(std::vector<Boundary>&& speed_constraints);


  const math::IntervalData<Boundary>& hardSpeedBound() const { return hard_speed_bound_; }
  math::IntervalData<Boundary>& mutableHardSpeedBound() { return hard_speed_bound_; }
  void setHardSpeedBound(const math::IntervalData<Boundary>& hard_speed_bound) { hard_speed_bound_ = hard_speed_bound; }

  void updateHardSpeedBound(const std::vector<Boundary>& speed_constraints);
  // 支持移动语义的重载
  void updateHardSpeedBound(std::vector<Boundary>&& speed_constraints);


  const math::IntervalData<ReferencePoint>& referencePointsData() const { return reference_points_data_; }
  math::IntervalData<ReferencePoint>& mutableReferencePointsData() { return reference_points_data_; }

  double boundaryBlockS() const { return boundary_block_s_; }
  void setBoundaryBlockS(double s) { boundary_block_s_ = s; }

 private:
  friend class BoundaryManagerBase<ReferencePoint, TrajectoryBoundary>;

  void initializeImpl(const std::vector<ReferencePoint>& reference_points);
  void resetImpl();
  void createInterpolators();

  math::IntervalData<ReferencePoint> reference_points_data_;
  math::IntervalData<Boundary> soft_speed_bound_;
  math::IntervalData<Boundary> hard_speed_bound_;
  math::IntervalData<Boundary> soft_lateral_bound_;
  math::IntervalData<Boundary> hard_lateral_bound_;
  mutable double boundary_block_s_ = -1.0;  ///< 边界交叉的s位置
};

}  // namespace gpal::pnc::planning