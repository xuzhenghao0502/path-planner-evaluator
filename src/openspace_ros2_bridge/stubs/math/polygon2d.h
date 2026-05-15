#pragma once
#include <vector>
#include "math/vec2d.h"
#include "math/line_segment2d.h"

namespace math {

class Polygon2d {
 public:
  Polygon2d() = default;
  explicit Polygon2d(const std::vector<Vec2d>& points) : points_(points) {}

  const std::vector<Vec2d>& points() const { return points_; }
  size_t num_points() const { return points_.size(); }

  double DistanceTo(const Vec2d& point) const {
    double min_dist = 1e10;
    for (size_t i = 0; i < points_.size(); ++i) {
      const auto& a = points_[i];
      const auto& b = points_[(i + 1) % points_.size()];
      Vec2d ab = b - a;
      Vec2d ap = point - a;
      double t = ap.InnerProd(ab) / std::max(1e-12, ab.LengthSquare());
      t = std::max(0.0, std::min(1.0, t));
      Vec2d closest = a + ab * t;
      min_dist = std::min(min_dist, point.DistanceTo(closest));
    }
    return min_dist;
  }

  bool HasOverlap(const LineSegment2d& seg) const {
    for (size_t i = 0; i < points_.size(); ++i) {
      LineSegment2d edge(points_[i], points_[(i + 1) % points_.size()]);
      if (edge.HasOverlap(seg)) return true;
    }
    return false;
  }

  void GetOverlap(const LineSegment2d& seg, Vec2d* first, Vec2d* second) const {
    *first = seg.start();
    *second = seg.end();
    for (size_t i = 0; i < points_.size(); ++i) {
      LineSegment2d edge(points_[i], points_[(i + 1) % points_.size()]);
      if (edge.HasOverlap(seg)) {
        if (edge.IsPointIn(seg.start())) *first = seg.start();
        else *first = seg.end();
        *second = *first;
        return;
      }
    }
  }

 private:
  std::vector<Vec2d> points_;
};

}  // namespace math
