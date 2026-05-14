#pragma once
#include "math/vec2d.h"

namespace math {

class Circle {
 public:
  Circle() : center_(0, 0), radius_(0) {}
  Circle(const Vec2d& center, double radius) : center_(center), radius_(radius) {}

  const Vec2d& center() const { return center_; }
  double radius() const { return radius_; }
  void set_center(const Vec2d& c) { center_ = c; }
  void set_radius(double r) { radius_ = r; }

 private:
  Vec2d center_;
  double radius_;
};

}  // namespace math
