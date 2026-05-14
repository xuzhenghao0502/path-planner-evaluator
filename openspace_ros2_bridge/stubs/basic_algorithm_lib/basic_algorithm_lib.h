#pragma once
#include <fmt/format.h>
#include "math/vec2d.h"
#include "math/box2d.h"
#include "math/line_segment2d.h"
#include "math/circle.h"
#include "math/math_utils.h"
#include "math/linear_interpolation.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "base/log.h"
#include "basic_algorithm_lib/transfer.h"
#include "eka-rt/base/stamped_base.h"
#include "point/path_pt.h"
#include <vector>

namespace math {
using Vec3d = Eigen::Vector3d;
}

inline double Distance(double x, double y) { return std::sqrt(x * x + y * y); }
constexpr double kPostiveInfinity = 1e100;

inline PathPt CalculatePedalPoint(const PathPt& point, const PathPt& line_a, const PathPt& line_b, double factor) {
  double dx = line_b.x() - line_a.x();
  double dy = line_b.y() - line_a.y();
  double dist_sq = dx * dx + dy * dy;
  if (dist_sq < 1e-10) return line_a;
  double t = ((point.x() - line_a.x()) * dx + (point.y() - line_a.y()) * dy) / dist_sq;
  t = std::max(0.0, std::min(1.0, t));
  double x = line_a.x() + t * dx;
  double y = line_a.y() + t * dy;
  double theta = line_a.theta() + t * (line_b.theta() - line_a.theta());
  return PathPt(x, y, theta);
}

// Make math namespace available inside gpal::pnc::planning (reference code uses unqualified math::)
namespace gpal::pnc::planning {
  namespace math = ::math;
  using ::math::kMathEpsilon;
  using ::math::interpolateUsingLinearApproximation;
  using ::CalculatePedalPoint;
}

// Calculate cumulative arc length along a discretized path
inline void ReCalculateLineLength(std::vector<PathPt>& path) {
  if (path.empty()) return;
  path[0].set_s(0);
  for (size_t i = 1; i < path.size(); ++i) {
    double dx = path[i].x() - path[i-1].x();
    double dy = path[i].y() - path[i-1].y();
    path[i].set_s(path[i-1].s() + std::sqrt(dx*dx + dy*dy));
  }
}
