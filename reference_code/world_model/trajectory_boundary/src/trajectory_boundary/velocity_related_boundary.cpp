#include "trajectory_boundary/velocity_related_boundary.h"

namespace gpal::pnc::planning {

void VelocityRelatedBoundary::initializeImpl(const std::vector<double>& indices) {
  // 首次调用时创建插值器
  if (soft_steering_bound_.interpolator() == nullptr) {
    createInterpolators();
  }

  // 使用局部变量来构建约束数据
  std::vector<Boundary> soft_steering_constraints;
  std::vector<Boundary> hard_steering_constraints;
  std::vector<Boundary> soft_dsteering_constraints;
  std::vector<Boundary> hard_dsteering_constraints;

  // 预分配内存以提高效率
  soft_steering_constraints.reserve(size_);
  hard_steering_constraints.reserve(size_);
  soft_dsteering_constraints.reserve(size_);
  hard_dsteering_constraints.reserve(size_);

  // 定义此类特有的默认值
  constexpr double default_steering_limit = 1.0;
  constexpr double default_dsteering_limit = 1.0;
  for (const auto& s : indices) {
    soft_steering_constraints.emplace_back(s, -default_steering_limit, default_steering_limit);
    hard_steering_constraints.emplace_back(s, -default_steering_limit, default_steering_limit);
    soft_dsteering_constraints.emplace_back(s, -default_dsteering_limit, default_dsteering_limit);
    hard_dsteering_constraints.emplace_back(s, -default_dsteering_limit, default_dsteering_limit);
  }

  // 通过移动局部变量的数据所有权来更新 IntervalData，这非常高效
  soft_steering_bound_.update(start_s_, delta_s_, std::move(soft_steering_constraints));
  hard_steering_bound_.update(start_s_, delta_s_, std::move(hard_steering_constraints));
  soft_dsteering_bound_.update(start_s_, delta_s_, std::move(soft_dsteering_constraints));
  hard_dsteering_bound_.update(start_s_, delta_s_, std::move(hard_dsteering_constraints));
}

// 创建插值器
void VelocityRelatedBoundary::createInterpolators() {
  soft_steering_bound_.setInterpolator(createInterpolator<Boundary>());
  hard_steering_bound_.setInterpolator(createInterpolator<Boundary>());
  soft_dsteering_bound_.setInterpolator(createInterpolator<Boundary>());
  hard_dsteering_bound_.setInterpolator(createInterpolator<Boundary>());
}

// 重置实现
void VelocityRelatedBoundary::resetImpl() {
  soft_steering_bound_.clear();
  hard_steering_bound_.clear();
  soft_dsteering_bound_.clear();
  hard_dsteering_bound_.clear();
}

// 实现新的 update... 接口
void VelocityRelatedBoundary::updateSoftSteeringBound(const std::vector<Boundary>& steering_constraints) {
  soft_steering_bound_.update(start_s_, delta_s_, steering_constraints);
}
void VelocityRelatedBoundary::updateSoftSteeringBound(std::vector<Boundary>&& steering_constraints) {
  soft_steering_bound_.update(start_s_, delta_s_, std::move(steering_constraints));
}

void VelocityRelatedBoundary::updateHardSteeringBound(const std::vector<Boundary>& steering_constraints) {
  hard_steering_bound_.update(start_s_, delta_s_, steering_constraints);
}
void VelocityRelatedBoundary::updateHardSteeringBound(std::vector<Boundary>&& steering_constraints) {
  hard_steering_bound_.update(start_s_, delta_s_, std::move(steering_constraints));
}

void VelocityRelatedBoundary::updateSoftDsteeringBound(const std::vector<Boundary>& dsteering_constraints) {
  soft_dsteering_bound_.update(start_s_, delta_s_, dsteering_constraints);
}
void VelocityRelatedBoundary::updateSoftDsteeringBound(std::vector<Boundary>&& dsteering_constraints) {
  soft_dsteering_bound_.update(start_s_, delta_s_, std::move(dsteering_constraints));
}

void VelocityRelatedBoundary::updateHardDsteeringBound(const std::vector<Boundary>& dsteering_constraints) {
  hard_dsteering_bound_.update(start_s_, delta_s_, dsteering_constraints);
}
void VelocityRelatedBoundary::updateHardDsteeringBound(std::vector<Boundary>&& dsteering_constraints) {
  hard_dsteering_bound_.update(start_s_, delta_s_, std::move(dsteering_constraints));
}

}  // namespace gpal::pnc::planning