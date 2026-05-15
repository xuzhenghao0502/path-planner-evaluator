// include/common/CubicSplineSlopes.h
#ifndef _COMMON_CUBICSPLINESLOPES_H_
#define _COMMON_CUBICSPLINESLOPES_H_
#include <cmath>
#include <deque>
#include <memory>
#include <numeric>  // For std::inner_product
#include <vector>

#include "math/vec2d.h"
namespace gpal::pnc::planning {
namespace CubicSplineInterpolation {

double SafeDivide(const double nominator, const double denominator, const double threshold);
std::vector<double> CalcCubicSplineSlopes(const std::vector<math::Vec3d>& points);
double EvaluateCubicSpline(const std::vector<math::Vec3d>& points, const double x_values,
                           const std::vector<double>& slopes);
double GetAveragedHeading(const std::vector<math::Vec3d>& points, const uint32_t index);

}  // namespace CubicSplineInterpolation
}  // namespace gpal::pnc::planning

#endif  // COMMON_CUBICSPLINESLOPES_H