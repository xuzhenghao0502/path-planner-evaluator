#pragma once
#include <vector>
#include "math/vec2d.h"

namespace math {

class Polyline {
 public:
  Polyline() = default;
  explicit Polyline(const std::vector<Vec2d>& pts) : points_(pts) {}
  const std::vector<Vec2d>& points() const { return points_; }
  size_t size() const { return points_.size(); }

 private:
  std::vector<Vec2d> points_;
};

}  // namespace math
