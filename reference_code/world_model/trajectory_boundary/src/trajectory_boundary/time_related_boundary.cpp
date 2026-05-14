#include "trajectory_boundary/time_related_boundary.h"

namespace gpal::pnc::planning {

void TimeRelatedBoundary::initializeImpl(const std::vector<double>& indices) {
  // 首次运行时创建插值器
  if (s_hard_bound_.interpolator() == nullptr) {
    createInterpolators();
  }

  // 使用局部变量来构建约束数据
  std::vector<Boundary> s_wall_constraints;
  std::vector<Boundary> s_static_obj_constraints;
  std::vector<Boundary> s_dynamic_obj_constraints;
  std::vector<Boundary> s_static_soft_constraints;
  std::vector<Boundary> s_static_hard_constraints;
  std::vector<Boundary> s_soft_constraints;
  std::vector<Boundary> s_hard_constraints;

  // 预分配内存以提高效率
  s_wall_constraints.reserve(size_);
  s_static_obj_constraints.reserve(size_);
  s_dynamic_obj_constraints.reserve(size_);
  s_static_soft_constraints.reserve(size_);
  s_static_hard_constraints.reserve(size_);
  s_soft_constraints.reserve(size_);
  s_hard_constraints.reserve(size_);

  constexpr double default_s_bound = 10000.0;
  for (const auto& s : indices) {
    s_wall_constraints.emplace_back(s, 0.0, default_s_bound);
    s_static_obj_constraints.emplace_back(s, 0.0, default_s_bound);
    s_dynamic_obj_constraints.emplace_back(s, 0.0, default_s_bound);
    s_static_soft_constraints.emplace_back(s, 0.0, default_s_bound);
    s_static_hard_constraints.emplace_back(s, 0.0, default_s_bound);
    s_soft_constraints.emplace_back(s, 0.0, default_s_bound);
    s_hard_constraints.emplace_back(s, 0.0, default_s_bound);
  }

  // 通过移动局部变量的数据所有权来更新 IntervalData，这非常高效
  s_wall_constraints_bound_.update(start_s_, delta_s_, std::move(s_wall_constraints));
  s_static_obj_bound_.update(start_s_, delta_s_, std::move(s_static_obj_constraints));
  s_dynamic_obj_bound_.update(start_s_, delta_s_, std::move(s_dynamic_obj_constraints));
  s_static_soft_bound_.update(start_s_, delta_s_, std::move(s_static_soft_constraints));
  s_static_hard_bound_.update(start_s_, delta_s_, std::move(s_static_hard_constraints));
  s_soft_bound_.update(start_s_, delta_s_, std::move(s_soft_constraints));
  s_hard_bound_.update(start_s_, delta_s_, std::move(s_hard_constraints));
}

// 创建插值器
void TimeRelatedBoundary::createInterpolators() {
  s_wall_constraints_bound_.setInterpolator(createInterpolator<Boundary>());
  s_static_obj_bound_.setInterpolator(createInterpolator<Boundary>());
  s_dynamic_obj_bound_.setInterpolator(createInterpolator<Boundary>());
  s_static_soft_bound_.setInterpolator(createInterpolator<Boundary>());
  s_static_hard_bound_.setInterpolator(createInterpolator<Boundary>());
  s_soft_bound_.setInterpolator(createInterpolator<Boundary>());
  s_hard_bound_.setInterpolator(createInterpolator<Boundary>());
}

// 重置实现
void TimeRelatedBoundary::resetImpl() {
  s_wall_constraints_bound_.clear();
  s_static_obj_bound_.clear();
  s_dynamic_obj_bound_.clear();
  s_static_soft_bound_.clear();
  s_static_hard_bound_.clear();
  s_soft_bound_.clear();
  s_hard_bound_.clear();
}

// 决策速度墙
void TimeRelatedBoundary::updateWallConstraintsBound(const std::vector<Boundary>& wall_constraints) {
  s_wall_constraints_bound_.update(start_s_, delta_s_, wall_constraints);
}
void TimeRelatedBoundary::updateWallConstraintsBound(std::vector<Boundary>&& wall_constraints) {
  s_wall_constraints_bound_.update(start_s_, delta_s_, std::move(wall_constraints));
}

// 静态障碍物
void TimeRelatedBoundary::updateStaticObjBound(const std::vector<Boundary>& obj_constraints) {
  s_static_obj_bound_.update(start_s_, delta_s_, obj_constraints);
}
void TimeRelatedBoundary::updateStaticObjBound(std::vector<Boundary>&& obj_constraints) {
  s_static_obj_bound_.update(start_s_, delta_s_, std::move(obj_constraints));
}

// 动态障碍物
void TimeRelatedBoundary::updateDynamicObjBound(const std::vector<Boundary>& obj_constraints) {
  s_dynamic_obj_bound_.update(start_s_, delta_s_, obj_constraints);
}
void TimeRelatedBoundary::updateDynamicObjBound(std::vector<Boundary>&& obj_constraints) {
  s_dynamic_obj_bound_.update(start_s_, delta_s_, std::move(obj_constraints));
}

// 综合静态边界
void TimeRelatedBoundary::updateSStaticSoftBound(const std::vector<Boundary>& s_constraints) {
  s_static_soft_bound_.update(start_s_, delta_s_, s_constraints);
}
void TimeRelatedBoundary::updateSStaticSoftBound(std::vector<Boundary>&& s_constraints) {
  s_static_soft_bound_.update(start_s_, delta_s_, std::move(s_constraints));
}

void TimeRelatedBoundary::updateSStaticHardBound(const std::vector<Boundary>& s_constraints) {
  s_static_hard_bound_.update(start_s_, delta_s_, s_constraints);
}
void TimeRelatedBoundary::updateSStaticHardBound(std::vector<Boundary>&& s_constraints) {
  s_static_hard_bound_.update(start_s_, delta_s_, std::move(s_constraints));
}

// 综合边界
void TimeRelatedBoundary::updateSSoftBound(const std::vector<Boundary>& s_constraints) {
  s_soft_bound_.update(start_s_, delta_s_, s_constraints);
}
void TimeRelatedBoundary::updateSSoftBound(std::vector<Boundary>&& s_constraints) {
  s_soft_bound_.update(start_s_, delta_s_, std::move(s_constraints));
}

void TimeRelatedBoundary::updateSHardBound(const std::vector<Boundary>& s_constraints) {
  s_hard_bound_.update(start_s_, delta_s_, s_constraints);
}
void TimeRelatedBoundary::updateSHardBound(std::vector<Boundary>&& s_constraints) {
  s_hard_bound_.update(start_s_, delta_s_, std::move(s_constraints));
}

}  // namespace gpal::pnc::planning