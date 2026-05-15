#pragma once
#include <vector>
#include <cmath>

namespace math {

inline double Lerp(double a, double b, double t) { return a + t * (b - a); }

template <typename T>
inline T Interpolate1d(double x, const std::vector<double>& xs, const std::vector<T>& ys) {
  if (xs.size() < 2 || ys.size() < 2) return T();
  if (x <= xs.front()) return ys.front();
  if (x >= xs.back()) return ys.back();
  for (size_t i = 0; i < xs.size() - 1; ++i) {
    if (x >= xs[i] && x <= xs[i + 1]) {
      double t = (x - xs[i]) / (xs[i + 1] - xs[i]);
      return T(ys[i] + t * (ys[i + 1] - ys[i]));
    }
  }
  return ys.back();
}

template <typename T1, typename T2>
inline auto interpolateUsingLinearApproximation(const T1& a, const T2& b, double s) -> T1 {
  double dx = b.x() - a.x();
  double dy = b.y() - a.y();
  double dist = std::sqrt(dx * dx + dy * dy);
  if (dist < 1e-10) return a;
  double ratio = (s - a.s()) / dist;
  double x = a.x() + dx * ratio;
  double y = a.y() + dy * ratio;
  double theta = a.theta() + (b.theta() - a.theta()) * ratio;
  return T1(x, y, theta);
}

}  // namespace math
