#include "trajectory_boundary/trajectory_boundary.h"

namespace gpal::pnc::planning {

void TrajectoryBoundary::initializeImpl(const std::vector<ReferencePoint>& reference_points) {
  // 使用局部变量来构建数据
  std::vector<Boundary> soft_lateral_constraints;
  std::vector<Boundary> hard_lateral_constraints;
  std::vector<Boundary> soft_speed_constraints;
  std::vector<Boundary> hard_speed_constraints;
  std::vector<ReferencePoint> local_reference_points;

  // 预分配内存以提高效率
  soft_lateral_constraints.reserve(size_);
  hard_lateral_constraints.reserve(size_);
  soft_speed_constraints.reserve(size_);
  hard_speed_constraints.reserve(size_);
  local_reference_points.reserve(size_);

  constexpr double default_lateral_bound = 10.0;  // 10 m
  constexpr double default_speed_limit = 40.0;    // 40 m/s
  for (const auto& ref_point : reference_points) {
    soft_lateral_constraints.emplace_back(ref_point.local_s(), -default_lateral_bound, default_lateral_bound);
    hard_lateral_constraints.emplace_back(ref_point.local_s(), -default_lateral_bound, default_lateral_bound);
    soft_speed_constraints.emplace_back(ref_point.local_s(), 0.0, default_speed_limit);
    hard_speed_constraints.emplace_back(ref_point.local_s(), 0.0, default_speed_limit);
    local_reference_points.emplace_back(ref_point);
  }

  // 如果 IntervalData 还未创建，先创建插值器
  if (reference_points_data_.interpolator() == nullptr) {
    createInterpolators();
  }

  // 使用 std::move 将局部变量的数据所有权转移给 IntervalData，避免深拷贝
  soft_lateral_bound_.update(start_s_, delta_s_, std::move(soft_lateral_constraints));
  hard_lateral_bound_.update(start_s_, delta_s_, std::move(hard_lateral_constraints));
  soft_speed_bound_.update(start_s_, delta_s_, std::move(soft_speed_constraints));
  hard_speed_bound_.update(start_s_, delta_s_, std::move(hard_speed_constraints));
  reference_points_data_.update(start_s_, delta_s_, std::move(local_reference_points));

  boundary_block_s_ = -1.0;
}

void TrajectoryBoundary::createInterpolators() {
  soft_lateral_bound_.setInterpolator(createInterpolator<Boundary>());
  hard_lateral_bound_.setInterpolator(createInterpolator<Boundary>());
  soft_speed_bound_.setInterpolator(createInterpolator<Boundary>());
  hard_speed_bound_.setInterpolator(createInterpolator<Boundary>());

  // 创建参考点插值器
  static auto ref_point_interpolator = [](const ReferencePoint& p1, const ReferencePoint& p2,
                                          double s) -> ReferencePoint {
    if (std::abs(p2.local_s() - p1.local_s()) < 1e-6) {
      return p1;
    }
    // 线性插值 s 参数
    double t = (s - p1.local_s()) / (p2.local_s() - p1.local_s());

    if (t < 0.0 || t > 1.0) {
      // 夹逼到 [0, 1]
      t = std::max(0.0, std::min(1.0, t));
    }

    ReferencePoint result;
    result.set_x(p1.x() * (1 - t) + p2.x() * t);
    result.set_y(p1.y() * (1 - t) + p2.y() * t);
    result.setHeading(math::slerp(p1.heading(), p1.local_s(), p2.heading(), p2.local_s(), s));
    result.setKappa(p1.kappa() * (1 - t) + p2.kappa() * t);
    result.setLocalS(s);

    return result;
  };
  reference_points_data_.setInterpolator(ref_point_interpolator);
  // 设置反向查找的 Finder
  auto high_precision_finder = [](const std::vector<ReferencePoint>& points,
                                  const ReferencePoint& target) -> std::optional<double> {
    if (points.size() < 2)
      return std::nullopt;

    const double query_x = target.x();
    const double query_y = target.y();
    double min_dist_sq = std::numeric_limits<double>::max();
    double best_s = 0.0;

    for (size_t i = 0; i < points.size() - 1; ++i) {
      const auto& p1 = points[i];
      const auto& p2 = points[i + 1];
      const double segment_dx = p2.x() - p1.x();
      const double segment_dy = p2.y() - p1.y();
      const double segment_len_sq = segment_dx * segment_dx + segment_dy * segment_dy;
      if (segment_len_sq < 1e-9)
        continue;
      const double query_dx = query_x - p1.x();
      const double query_dy = query_y - p1.y();
      double t = (query_dx * segment_dx + query_dy * segment_dy) / segment_len_sq;
      t = std::max(0.0, std::min(1.0, t));
      const double proj_x = p1.x() + t * segment_dx;
      const double proj_y = p1.y() + t * segment_dy;
      const double dist_sq = (query_x - proj_x) * (query_x - proj_x) + (query_y - proj_y) * (query_y - proj_y);
      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        best_s = p1.local_s() + t * (p2.local_s() - p1.local_s());
      }
    }
    return best_s;
  };
  reference_points_data_.setFinder(high_precision_finder);
}

void TrajectoryBoundary::resetImpl() {
  soft_speed_bound_.clear();
  hard_lateral_bound_.clear();
  soft_lateral_bound_.clear();
  hard_speed_bound_.clear();
  reference_points_data_.clear();
  boundary_block_s_ = -1.0;
}

// 实现 update... 接口
void TrajectoryBoundary::updateSoftLateralBound(const std::vector<Boundary>& lateral_constraints) {
  soft_lateral_bound_.update(start_s_, delta_s_, lateral_constraints);
}
void TrajectoryBoundary::updateSoftLateralBound(std::vector<Boundary>&& lateral_constraints) {
  soft_lateral_bound_.update(start_s_, delta_s_, std::move(lateral_constraints));
}

void TrajectoryBoundary::updateHardLateralBound(const std::vector<Boundary>& lateral_constraints) {
  hard_lateral_bound_.update(start_s_, delta_s_, lateral_constraints);
}
void TrajectoryBoundary::updateHardLateralBound(std::vector<Boundary>&& lateral_constraints) {
  hard_lateral_bound_.update(start_s_, delta_s_, std::move(lateral_constraints));
}

void TrajectoryBoundary::updateSoftSpeedBound(const std::vector<Boundary>& speed_constraints) {
  soft_speed_bound_.update(start_s_, delta_s_, speed_constraints);
}
void TrajectoryBoundary::updateSoftSpeedBound(std::vector<Boundary>&& speed_constraints) {
  soft_speed_bound_.update(start_s_, delta_s_, std::move(speed_constraints));
}

void TrajectoryBoundary::updateHardSpeedBound(const std::vector<Boundary>& speed_constraints) {
  hard_speed_bound_.update(start_s_, delta_s_, speed_constraints);
}
void TrajectoryBoundary::updateHardSpeedBound(std::vector<Boundary>&& speed_constraints) {
  hard_speed_bound_.update(start_s_, delta_s_, std::move(speed_constraints));
}

}  // namespace gpal::pnc::planning