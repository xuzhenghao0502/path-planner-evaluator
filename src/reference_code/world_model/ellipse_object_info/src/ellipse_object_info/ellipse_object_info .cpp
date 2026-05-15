#include "ellipse_object_info/ellipse_object_info.h"
#include <cmath>
#include <algorithm>
#include <Eigen/Eigenvalues>

namespace gpal::pnc::planning {

ObjectInfo::ObjectInfo(const double& x, const double& y, const double& theta, const double& local_s,
                       const double& length, const double& width, const double& v, const double& acc,
                       const double& long_buffer, const double& lateral_buffer, const double& ego_buffer)
    : x(x), y(y), theta(theta), local_s(local_s), length(length), width(width), v(v), acc(acc) {

  double ell_long_buf = std::max(0.0, long_buffer - 0.415 * length / 2.0);
  double ell_lat_buf = std::max(0.0, lateral_buffer - 0.415 * width / 2.0);

  a = 1.415 * length / 2.0 + ell_long_buf + ego_buffer;
  b = 1.415 * width / 2.0 + ell_lat_buf + ego_buffer;
  a_square = a * a;
  b_square = b * b;
  cos_theta = std::cos(theta);
  sin_theta = std::sin(theta);
}

ObjectInfo::ObjectInfo(const double& x, const double& y, const double& theta, const double& local_s,
                       const double& length, const double& width, const double& v, const double& acc,
                       const double& long_buffer, const double& lateral_buffer, const double& ego_buffer,
                       const double& time_horizon)
    : x(x), y(y), theta(theta), local_s(local_s), length(length), width(width), v(v), acc(acc) {

  a = 1.415 * length / 2.0 + long_buffer + v * time_horizon + 0.5 * acc * time_horizon * time_horizon
      + 1.415 * ego_buffer;
  b = 1.415 * width / 2.0 + lateral_buffer + 1.415 * ego_buffer;
  a_square = a * a;
  b_square = b * b;
  cos_theta = std::cos(theta);
  sin_theta = std::sin(theta);
}

ObjectInfo::ObjectInfo(const double& x, const double& y, const double& theta, const double& local_s,
                       const double& length, const double& width, const double& v, const double& acc,
                       const double& ego_buffer, const Eigen::Matrix2d& covariance, const double& ell_angle,
                       const BufferCalculationFunction& buffer_func, const double& n_std)
    : x(x), y(y), theta(theta), local_s(local_s), length(length), width(width), v(v), acc(acc),
      ego_buffer(ego_buffer), covariance_matrix(covariance), ellipse_angle(ell_angle),
      buffer_calculator_(std::make_shared<BufferCalculationFunction>(buffer_func)) {

  extractEllipseFromCovariance(covariance, n_std);
}

void ObjectInfo::extractEllipseFromCovariance(const Eigen::Matrix2d& cov_matrix, double n_std) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov_matrix);
  Eigen::Vector2d eigenvalues = solver.eigenvalues();

  base_a = n_std * std::sqrt(std::abs(eigenvalues(1)));
  base_b = n_std * std::sqrt(std::abs(eigenvalues(0)));
  a = base_a;
  b = base_b;
  a_square = a * a;
  b_square = b * b;

  cos_theta = std::cos(ellipse_angle);
  sin_theta = std::sin(ellipse_angle);
}

double ObjectInfo::computeMinimalScaleFactor(const std::vector<std::array<double, 2>>& rect_corners,
                                            double a_val, double b_val, double ell_angle,
                                            const std::array<double, 2>& center) {
  if (rect_corners.empty() || a_val < 1e-6 || b_val < 1e-6) return 1.0;

  double cos_ang = std::cos(ell_angle);
  double sin_ang = std::sin(ell_angle);
  double max_val = 0.0;

  for (const auto& corner : rect_corners) {
    double dx = corner[0] - center[0];
    double dy = corner[1] - center[1];
    double qx = dx * cos_ang + dy * sin_ang;
    double qy = -dx * sin_ang + dy * cos_ang;
    double val = (qx * qx) / (a_val * a_val) + (qy * qy) / (b_val * b_val);
    max_val = std::max(max_val, val);
  }
  return std::sqrt(std::max(max_val, 0.0));
}

void ObjectInfo::updateDynamicBuffers(double curr_v, double curr_a, int k) {
  if (!buffer_calculator_) return;
  auto [long_buffer, lat_buffer] = (*buffer_calculator_)(*this, curr_v, curr_a, k);
  ellipse_long_buffer = std::max(0.0, long_buffer - 0.415 * length / 2.0);
  ellipse_lat_buffer = std::max(0.0, lat_buffer - 0.415 * width / 2.0);
}

void ObjectInfo::adjust(double ego_heading, double ang_a_min_factor, double ang_a_max_factor,
                        double ang_b_min_factor, double ang_b_max_factor) {
  double s_ang = computeAngleFactor(ego_heading, ellipse_angle);
  double a_adj = base_a * ((1.0 - s_ang) * ang_a_max_factor + s_ang * ang_a_min_factor);
  double b_adj = base_b * ((1.0 - s_ang) * ang_b_min_factor + s_ang * ang_b_max_factor);

  auto rect_corners = computeRectangleCorners();
  double k = computeMinimalScaleFactor(rect_corners, a_adj, b_adj, ellipse_angle, {x, y});

  a = k * a_adj + ego_buffer + ellipse_long_buffer;
  b = k * b_adj + ego_buffer + ellipse_lat_buffer;
  a_square = a * a;
  b_square = b * b;
}

std::vector<std::array<double, 2>> ObjectInfo::computeRectangleCorners() const {
  double hl = length / 2.0;
  double hw = width / 2.0;
  std::array<std::array<double, 2>, 4> locals = {{{hl, hw}, {hl, -hw}, {-hl, -hw}, {-hl, hw}}};
  double ct = std::cos(theta);
  double st = std::sin(theta);

  std::vector<std::array<double, 2>> worlds;
  worlds.reserve(4);
  for (const auto& pt : locals) {
    worlds.push_back({x + pt[0] * ct - pt[1] * st, y + pt[0] * st + pt[1] * ct});
  }
  return worlds;
}

double ObjectInfo::computeAngleFactor(double ego_heading, double obs_heading) const {
  double diff = std::abs(ego_heading - obs_heading);
  diff = std::fmod(diff + M_PI, 2.0 * M_PI) - M_PI;
  return std::sin(std::abs(diff));
}

double ObjectInfo::ComputeMahalanobisDistance(double px, double py) const {
  double dx = px - x;
  double dy = py - y;
  double rx = dx * cos_theta + dy * sin_theta;
  double ry = -dx * sin_theta + dy * cos_theta;
  return std::sqrt((rx * rx) / a_square + (ry * ry) / b_square);
}

double ObjectInfo::ComputePerceptionMahalanobisDistance(double px, double py, double pt,
                                                       double ps, double pt_time) const {
  double s_obs = v * pt_time + 0.5 * acc * pt_time * pt_time;
  double obs_x = x + s_obs * std::cos(theta);
  double obs_y = y + s_obs * std::sin(theta);

  double ego_x = px + ps * std::cos(pt) * pt_time;
  double ego_y = py + ps * std::sin(pt) * pt_time;

  double dx = ego_x - obs_x;
  double dy = ego_y - obs_y;
  double rx = dx * cos_theta + dy * sin_theta;
  double ry = -dx * sin_theta + dy * cos_theta;
  return std::sqrt((rx * rx) / a_square + (ry * ry) / b_square);
}

double ObjectInfo::ComputeHeadingRisk(double px, double py, double pt) const {
  double dx = x - px;
  double dy = y - py;
  double rx = dx * std::cos(pt) + dy * std::sin(pt);
  double ry = -dx * std::sin(pt) + dy * std::cos(pt);
  return (1.0 + std::cos(std::atan2(ry, rx))) / 2.0 + 1.0;
}

bool ObjectInfo::CalculateRiskySpeed(double px, double py, double pt, double ps, double& risky_speed) const {
  if (longitudinal_od_tag != gpal::pnc::planning::Decision::LongitudinalOdTag::RISKY && longitudinal_od_tag != gpal::pnc::planning::Decision::LongitudinalOdTag::YIELD) {
    return false;
  }

  double dx = x - px;
  double dy = y - py;
  double rx = dx * std::cos(pt) + dy * std::sin(pt);
  double ry = -dx * std::sin(pt) + dy * std::cos(pt);

  if ((1.0 - std::cos(std::atan2(ry, rx))) / 2.0 > 0.5) return false;

  double d1 = ComputeMahalanobisDistance(px, py);
  double d2 = ComputePerceptionMahalanobisDistance(px, py, pt, ps, 1.0);
  double delta = d1 - d2;

  if (delta <= 1e-5) return false;

  double ttc = d1 / delta;
  double proj_v = v * std::cos(theta - pt);

  constexpr double kMinTTC = 1.0;
  constexpr double kSafeTTC = 3.0;
  double ttc_f = std::clamp(1.0 - (kSafeTTC - ttc) / (kSafeTTC - kMinTTC), 0.0, 1.0);

  risky_speed = std::min(risky_speed, proj_v * ttc_f);
  return true;
}

} // namespace gpal::pnc::planning
