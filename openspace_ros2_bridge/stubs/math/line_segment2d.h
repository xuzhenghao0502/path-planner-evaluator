#pragma once
#include "math/vec2d.h"
#include "point/path_pt.h"
#include <utility>
#include <vector>

namespace math {

class LineSegment2d {
 public:
  LineSegment2d() : start_(0, 0), end_(0, 0) {}
  LineSegment2d(const Vec2d& start, const Vec2d& end) : start_(start), end_(end) {}
  LineSegment2d(const PathPt& start, const PathPt& end) : start_(start.x(), start.y()), end_(end.x(), end.y()) {}

  const Vec2d& start() const { return start_; }
  const Vec2d& end() const { return end_; }
  double length() const { return start_.DistanceTo(end_); }
  Vec2d unit_direction() const {
    Vec2d d = end_ - start_;
    double l = d.Length();
    return (l > 1e-10) ? d / l : Vec2d(1, 0);
  }
  double heading() const {
    Vec2d d = end_ - start_;
    return std::atan2(d.y(), d.x());
  }

  double GetPerpendicularFoot(const Vec2d& point, Vec2d* foot_point) const {
    Vec2d ab = end_ - start_;
    Vec2d ap = point - start_;
    double t = ap.InnerProd(ab) / std::max(1e-12, ab.LengthSquare());
    t = std::max(0.0, std::min(1.0, t));
    *foot_point = start_ + ab * t;
    return point.DistanceTo(*foot_point);
  }

  bool IsPointIn(const Vec2d& point) const {
    Vec2d foot;
    return GetPerpendicularFoot(point, &foot) < 1e-10;
  }

  double ProjectOntoUnit(const Vec2d& point) const {
    Vec2d d = unit_direction();
    return (point - start_).InnerProd(d);
  }

  bool HasOverlap(const LineSegment2d& other) const {
    // Simplified: check if any endpoint is close to the other segment
    return IsPointIn(other.start()) || IsPointIn(other.end()) ||
           other.IsPointIn(start_) || other.IsPointIn(end_);
  }

 private:
  Vec2d start_, end_;
};

}  // namespace math
