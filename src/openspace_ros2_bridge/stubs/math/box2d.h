#pragma once
#include "math/vec2d.h"
#include "math/line_segment2d.h"
#include "point/path_pt.h"
#include <Eigen/Core>
#include <vector>
#include <cmath>

namespace math {

class Box2d {
 public:
  Box2d() : center_(0, 0), heading_(0), half_length_(0), half_width_(0),
            cos_heading_(1), sin_heading_(0) {}

  Box2d(const Vec2d& center, double heading, double length, double width)
      : center_(center), heading_(heading), half_length_(length / 2.0), half_width_(width / 2.0),
        cos_heading_(std::cos(heading)), sin_heading_(std::sin(heading)) {}

  Box2d(const PathPt& center, double heading, double length, double width)
      : Box2d(Vec2d(center.x(), center.y()), heading, length, width) {}

  const Vec2d& center() const { return center_; }
  double center_x() const { return center_.x(); }
  double center_y() const { return center_.y(); }
  double heading() const { return heading_; }
  double length() const { return half_length_ * 2.0; }
  double width() const { return half_width_ * 2.0; }
  double half_length() const { return half_length_; }
  double half_width() const { return half_width_; }
  double cos_heading() const { return cos_heading_; }
  double sin_heading() const { return sin_heading_; }

  std::vector<Vec2d> corners() const {
    std::vector<Vec2d> pts(4);
    double dx1 = cos_heading_ * half_length_;
    double dy1 = sin_heading_ * half_length_;
    double dx2 = sin_heading_ * half_width_;
    double dy2 = -cos_heading_ * half_width_;
    pts[0] = Vec2d(center_.x() + dx1 + dx2, center_.y() + dy1 + dy2);
    pts[1] = Vec2d(center_.x() + dx1 - dx2, center_.y() + dy1 - dy2);
    pts[2] = Vec2d(center_.x() - dx1 - dx2, center_.y() - dy1 - dy2);
    pts[3] = Vec2d(center_.x() - dx1 + dx2, center_.y() - dy1 + dy2);
    return pts;
  }

  bool IsPointIn(const Vec2d& pt) const {
    double dx = pt.x() - center_.x();
    double dy = pt.y() - center_.y();
    double proj_x = dx * cos_heading_ + dy * sin_heading_;
    double proj_y = -dx * sin_heading_ + dy * cos_heading_;
    return std::abs(proj_x) <= half_length_ && std::abs(proj_y) <= half_width_;
  }

  double DistanceTo(const Vec2d& pt) const {
    double dx = pt.x() - center_.x();
    double dy = pt.y() - center_.y();
    double proj_x = dx * cos_heading_ + dy * sin_heading_;
    double proj_y = -dx * sin_heading_ + dy * cos_heading_;
    double dx_abs = std::abs(proj_x) - half_length_;
    double dy_abs = std::abs(proj_y) - half_width_;
    if (dx_abs <= 0) return std::max(0.0, dy_abs);
    if (dy_abs <= 0) return std::max(0.0, dx_abs);
    return std::sqrt(dx_abs * dx_abs + dy_abs * dy_abs);
  }

  void Shift(const Vec2d& shift_vec) {
    center_ = center_ + shift_vec;
  }

  void LongitudinalExtend(double extend_length) {
    half_length_ += extend_length / 2.0;
  }

  bool HasOverlap(const Box2d& other) const {
    // Separating axis theorem simplified: check if corners overlap
    auto c1 = corners();
    auto c2 = other.corners();
    for (const auto& p : c1) if (other.IsPointIn(p)) return true;
    for (const auto& p : c2) if (IsPointIn(p)) return true;
    return false;
  }

  std::vector<Vec2d> GetAllCorners() const { return corners(); }

  double DistanceTo(const LineSegment2d& seg) const {
    Vec2d foot;
    return seg.GetPerpendicularFoot(center_, &foot);
  }

  bool HasOverlap(const LineSegment2d& seg) const {
    if (IsPointIn(seg.start()) || IsPointIn(seg.end())) return true;
    // Check segment against box edges
    auto c = corners();
    for (size_t i = 0; i < 4; ++i) {
      LineSegment2d edge(c[i], c[(i + 1) % 4]);
      if (edge.HasOverlap(seg)) return true;
    }
    return false;
  }

 private:
  Vec2d center_;
  double heading_;
  double half_length_, half_width_;
  double cos_heading_, sin_heading_;
};

}  // namespace math
