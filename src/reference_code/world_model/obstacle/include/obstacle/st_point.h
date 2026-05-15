#pragma once

#include <string>

#include "math/vec2d.h"

namespace gpal::pnc::planning {

class STPoint : public math::Vec2d {
 public:
  STPoint() = default;
  STPoint(const double s, const double t);
  explicit STPoint(const math::Vec2d& vec2d_point);

  double x() const = delete;
  double y() const = delete;

  double s() const;
  double t() const;
  void set_s(const double s);
  void set_t(const double t);
  std::string DebugString() const;
};

}  // namespace gpal::pnc::planning
