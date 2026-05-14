#pragma once
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace math {

class Vec2d {
 public:
  Vec2d() : x_(0), y_(0) {}
  Vec2d(double x, double y) : x_(x), y_(y) {}

  double x() const { return x_; }
  double y() const { return y_; }
  void set_x(double x) { x_ = x; }
  void set_y(double y) { y_ = y; }

  double Length() const { return std::sqrt(x_ * x_ + y_ * y_); }
  double LengthSquare() const { return x_ * x_ + y_ * y_; }
  double DistanceTo(const Vec2d& other) const {
    return (*this - other).Length();
  }

  double InnerProd(const Vec2d& other) const { return x_ * other.x_ + y_ * other.y_; }
  double CrossProd(const Vec2d& other) const { return x_ * other.y_ - y_ * other.x_; }

  Vec2d CreateUnitVec2d(double angle) const { return Vec2d(std::cos(angle), std::sin(angle)); }

  Vec2d operator+(const Vec2d& o) const { return Vec2d(x_ + o.x_, y_ + o.y_); }
  Vec2d operator-(const Vec2d& o) const { return Vec2d(x_ - o.x_, y_ - o.y_); }
  Vec2d operator*(double s) const { return Vec2d(x_ * s, y_ * s); }
  Vec2d operator/(double s) const { return Vec2d(x_ / s, y_ / s); }
  Vec2d& operator+=(const Vec2d& o) { x_ += o.x_; y_ += o.y_; return *this; }
  Vec2d& operator-=(const Vec2d& o) { x_ -= o.x_; y_ -= o.y_; return *this; }

  void Normalize() {
    double l = Length();
    if (l > 1e-10) { x_ /= l; y_ /= l; }
  }

  double Angle() const { return std::atan2(y_, x_); }

 private:
  double x_, y_;
};

inline Vec2d operator*(double s, const Vec2d& v) { return v * s; }

}  // namespace math
