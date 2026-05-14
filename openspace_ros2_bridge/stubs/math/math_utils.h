#pragma once
#include <cmath>
#include "math/vec2d.h"

namespace math {

constexpr double kMathEpsilon = 1e-10;

inline double CrossProd(const Vec2d& a, const Vec2d& b) { return a.CrossProd(b); }
inline double InnerProd(const Vec2d& a, const Vec2d& b) { return a.InnerProd(b); }
inline double NormalizeAngle(double angle) {
  double a = std::fmod(angle, 2.0 * M_PI);
  if (a > M_PI) a -= 2.0 * M_PI;
  if (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
inline double WrapAngle(double angle) { return NormalizeAngle(angle); }

inline double Sqr(double x) { return x * x; }
inline double RadToDeg(double rad) { return rad * 180.0 / M_PI; }
inline double DegToRad(double deg) { return deg * M_PI / 180.0; }

}  // namespace math
