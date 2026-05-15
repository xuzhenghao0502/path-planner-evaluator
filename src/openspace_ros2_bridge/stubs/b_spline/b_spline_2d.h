#pragma once
#include <vector>
#include "math/vec2d.h"

namespace b_spline {

class BSpline2d {
 public:
  BSpline2d() = default;
  BSpline2d(const std::vector<math::Vec2d>& pts, int order) {}
  math::Vec2d evaluate(double t) const { return math::Vec2d(); }
  double length() const { return 0; }
};

}  // namespace b_spline
