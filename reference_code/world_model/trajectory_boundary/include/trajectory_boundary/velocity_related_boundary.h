#pragma once

#include "trajectory_boundary/base_boundary.h"
namespace gpal::pnc::planning {
/**
 * @brief 速度相关边界
 * @details 用于管理速度相关的参考点边界
 */
class VelocityRelatedBoundary : public BoundaryManagerBase<double, VelocityRelatedBoundary> {
 public:
  // 使用基类的构造函数
  using BoundaryManagerBase<double, VelocityRelatedBoundary>::BoundaryManagerBase;
  VelocityRelatedBoundary() = default;

  const math::IntervalData<Boundary>& softSteeringBound() const { return soft_steering_bound_; }
  math::IntervalData<Boundary>& mutableSoftSteeringBound() { return soft_steering_bound_; }
  void setSoftSteeringBound(const math::IntervalData<Boundary>& soft_steering_bound) {
    soft_steering_bound_ = soft_steering_bound;
  }

  void updateSoftSteeringBound(const std::vector<Boundary>& steering_constraints);
  // 支持移动语义的重载
  void updateSoftSteeringBound(std::vector<Boundary>&& steering_constraints);

  const math::IntervalData<Boundary>& hardSteeringBound() const { return hard_steering_bound_; }
  math::IntervalData<Boundary>& mutableHardSteeringBound() { return hard_steering_bound_; }
  void setHardSteeringBound(const math::IntervalData<Boundary>& hard_steering_bound) {
    hard_steering_bound_ = hard_steering_bound;
  }
  void updateHardSteeringBound(const std::vector<Boundary>& steering_constraints);
  void updateHardSteeringBound(std::vector<Boundary>&& steering_constraints);

  const math::IntervalData<Boundary>& softDsteeringBound() const { return soft_dsteering_bound_; }
  math::IntervalData<Boundary>& mutableSoftDsteeringBound() { return soft_dsteering_bound_; }
  void setSoftDsteeringBound(const math::IntervalData<Boundary>& soft_dsteering_bound) {
    soft_dsteering_bound_ = soft_dsteering_bound;
  }
  void updateSoftDsteeringBound(const std::vector<Boundary>& dsteering_constraints);
  void updateSoftDsteeringBound(std::vector<Boundary>&& dsteering_constraints);

  const math::IntervalData<Boundary>& hardDsteeringBound() const { return hard_dsteering_bound_; }
  math::IntervalData<Boundary>& mutableHardDsteeringBound() { return hard_dsteering_bound_; }
  void setHardDsteeringBound(const math::IntervalData<Boundary>& hard_dsteering_bound) {
    hard_dsteering_bound_ = hard_dsteering_bound;
  }
  void updateHardDsteeringBound(const std::vector<Boundary>& dsteering_constraints);
  void updateHardDsteeringBound(std::vector<Boundary>&& dsteering_constraints);

 private:
  friend class BoundaryManagerBase<double, VelocityRelatedBoundary>;

  void initializeImpl(const std::vector<double>& indices);
  void createInterpolators();
  void resetImpl();

  math::IntervalData<Boundary> soft_steering_bound_;
  math::IntervalData<Boundary> hard_steering_bound_;
  math::IntervalData<Boundary> soft_dsteering_bound_;
  math::IntervalData<Boundary> hard_dsteering_bound_;
};

}  // namespace gpal::pnc::planning