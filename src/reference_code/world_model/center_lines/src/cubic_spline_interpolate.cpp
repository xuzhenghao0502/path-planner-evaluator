#include "center_lines/cubic_spline_interpolate.h"
namespace gpal::pnc::planning {
namespace CubicSplineInterpolation {

double SafeDivide(const double nominator, const double denominator, const double threshold) {
  if (std::abs(denominator) < threshold) {
    return nominator / threshold;
  } else {
    return nominator / denominator;
  }
}

std::vector<double> CalcCubicSplineSlopes(const std::vector<math::Vec3d>& points) {
  std::vector<double> x = {};
  std::vector<double> y = {};
  std::vector<double> slopes = {};
  if (points.empty()) {
    return slopes;
  }
  for (size_t i = 0; i < points.size(); ++i) {
    // 将 x 和 y 坐标分别添加到对应的向量中
    const auto& point = points[i];
    x.push_back(point.x());
    y.push_back(point.y());
  }
  uint32_t capacity = x.size();
  if (capacity < 2) {
    slopes.push_back(0.0);
    return slopes;
  }
  std::vector<double> dx(capacity - 1);
  std::vector<double> dy(capacity - 1);
  // 计算差分
  for (size_t i = 0; i < dx.size(); ++i) {
    dx[i] = x[i + 1] - x[i];
    dy[i] = y[i + 1] - y[i];
  }
  if (capacity >= 2) {
    std::vector<std::vector<double>> sys;
    sys.resize(capacity);
    for (auto& row : sys) {
      row.resize(2, 0.0);  // 将每个内部向量初始化为2个元素，每个元素都是0.0f
    }
    slopes.resize(capacity, 0.0);
    // generate the two diagonals of the system
    sys[0][0] = 2.0 / dx[0];
    sys[0][1] = 1.0 / dx[0];
    slopes[0] = 3.0 * dy[0] / (dx[0] * dx[0]);
    for (size_t i = 1; i < (capacity - 1); ++i) {
      sys[i][0] = 2.0 * (1.0 / dx[i - 1] + 1.0f / dx[i]);
      sys[i][1] = 1.0 / dx[i];
      slopes[i] = 3.0 * (dy[i - 1] / (dx[i - 1] * dx[i - 1]) + dy[i] / (dx[i] * dx[i]));
    }
    sys[capacity - 1][0] = 2.0 / dx[capacity - 2];
    sys[capacity - 1][1] = 0.0;
    slopes[capacity - 1] = 3.0 * dy[capacity - 2] / (dx[capacity - 2] * dx[capacity - 2]);
    // Simple symmetric tridiagonal system => solve with gaussian elimination
    for (size_t i = 1; i < capacity; ++i) {
      float ratio = sys[i - 1][1] / sys[i - 1][0];
      sys[i][0] -= ratio * sys[i - 1][1];
      slopes[i] -= ratio * slopes[i - 1];
    }
    // Back substitution
    slopes[capacity - 1] = slopes[capacity - 1] / sys[capacity - 1][0];
    for (int i = (int)(capacity)-2; i >= 0; --i) {
      slopes[i] = (slopes[i] - slopes[i + 1] * sys[i][1]) / sys[i][0];
    }
  }
  return slopes;
}

double EvaluateCubicSpline(const std::vector<math::Vec3d>& points, const double x_values,
                           const std::vector<double>& slopes) {
  double y_values;
  std::vector<double> x;
  std::vector<double> y;
  x.clear();
  y.clear();
  // 遍历 point 中的所有点
  for (size_t i = 0; i < points.size(); ++i) {
    x.push_back(points[i].x());
    y.push_back(points[i].y());
  }
  uint32_t capacity = x.size();
  if (capacity == 0) {
    y_values = 0;
    return y_values;
  }
  if (x_values <= x[0]) {
    // Extrapolate the first segment with zero second derivative
    y_values = y[0] + (x_values - x[0]) * slopes[0];
  } else if (x_values >= x[capacity - 1]) {
    // Extrapolate the last segment with zero second derivative
    y_values = y[capacity - 1] + (x_values - x[capacity - 1]) * slopes[capacity - 1];
  } else {
    size_t segment = 0;
    while (segment < (capacity - 2) && x[segment + 1] <= x_values) {
      ++segment;
    }
    double x1 = x[segment];
    double x2 = x[segment + 1];
    double k1 = slopes[segment];
    double k2 = slopes[segment + 1];
    double y1 = y[segment];
    double y2 = y[segment + 1];
    double t = (x_values - x1) / (x2 - x1);
    double a = k1 * (x2 - x1) - (y2 - y1);
    double b = -k2 * (x2 - x1) + (y2 - y1);

    y_values = (1.0 - t) * y1 + t * y2 + t * (1.0 - t) * (a * (1.0 - t) + b * t);
  }
  return y_values;
}

double GetAveragedHeading(const std::vector<math::Vec3d>& points, const uint32_t index) {
  uint32_t point_num = points.size();
  if (point_num < 2) {
    return 0.0;
  } else {
    double denominator = 0.0;
    double nominator = 0.0;
    uint32_t startIndex = index - 2;
    if (startIndex < 1) {
      startIndex = 1;
    }
    uint32_t endIndex = index + 2;
    if (endIndex > (point_num - 1)) {
      endIndex = (point_num - 1);
    }
    for (uint32_t k = startIndex; k < endIndex; ++k) {
      const auto& current = points[k];
      const auto& preview = points[k - 1];
      double deltaX = current.x() - preview.x();
      double deltaY = current.y() - preview.y();
      nominator += CubicSplineInterpolation::SafeDivide(deltaY, deltaX, 1e-4);
      denominator += 1.0;
    }
    return std::atan2(nominator, denominator);
  }
}

}  // namespace CubicSplineInterpolation
}  // namespace gpal::pnc::planning
