#include "openspace_path_planner/core/generator/rs_path.h"

// #include <glog/logging.h>

namespace gpal::pnc::planning {
const RSPath::RSPathSegmentType RSPath::RS_path_segment_type[18][5] = {
    {L, R, L, N, N}, {R, L, R, N, N}, {L, R, L, R, N}, {R, L, R, L, N}, {L, R, S, L, N}, {R, L, S, R, N},
    {L, S, R, L, N}, {R, S, L, R, N}, {L, R, S, R, N}, {R, L, S, L, N}, {R, S, R, L, N}, {L, S, L, R, N},
    {L, S, R, N, N}, {R, S, L, N, N}, {L, S, L, N, N}, {R, S, R, N, N}, {L, R, S, L, R}, {R, L, S, R, L}};

RSPath::RSPath(double turning_radius) : turning_radius_(turning_radius) {}

double RSPath::Mod2Pi(double x) {
  double v = fmod(x, 2 * M_PI);

  if (v < -M_PI) {
    v += 2.0 * M_PI;
  } else if (v > M_PI) {
    v -= 2.0 * M_PI;
  }

  return v;
}

void RSPath::Polar(double x, double y, double& r, double& theta) {
  r = std::sqrt(x * x + y * y);
  theta = std::atan2(y, x);
}

void RSPath::TauOmega(double u, double v, double xi, double eta, double phi, double& tau, double& omega) {
  double delta = Mod2Pi(u - v);
  double A = std::sin(u) - std::sin(delta);
  double B = std::cos(u) - std::cos(delta) - 1.0;
  double t1 = std::atan2(eta * A - xi * B, xi * A + eta * B);
  double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;
  tau = (t2 < 0.0) ? Mod2Pi(t1 + M_PI) : Mod2Pi(t1);
  omega = Mod2Pi(tau - u + v - phi);
}

bool RSPath::LpSpLp(double x, double y, double phi, double& t, double& u, double& v) {
  Polar(x - std::sin(phi), y - 1.0 + std::cos(phi), u, t);

  if (t >= 0.0) {
    v = Mod2Pi(phi - t);
    if (v >= 0.0) {
      return true;
    }
  }

  return false;
}

bool RSPath::LpSpRp(double x, double y, double phi, double& t, double& u, double& v) {
  double t1, u1;
  Polar(x + std::sin(phi), y - 1 - std::cos(phi), u1, t1);
  u1 = std::pow(u1, 2);

  if (u1 < 4.0) {
    return false;
  }

  double theta;
  u = std::sqrt(u1 - 4.0);
  theta = std::atan2(2.0, u);
  t = Mod2Pi(t1 + theta);
  v = Mod2Pi(t - phi);

  return true;
}

RSPath::RSPathData RSPath::GetRSPath(const double x_0, const double y_0, const double yaw_0, const double x_1,
                                     const double y_1, const double yaw_1) {
  // translation
  double dx = x_1 - x_0;
  double dy = y_1 - y_0;

  // rotate
  double c = std::cos(yaw_0);  // 2d rotation matrix
  double s = std::sin(yaw_0);
  double x = c * dx + s * dy;
  double y = -s * dx + c * dy;
  double phi = yaw_1 - yaw_0;
  return GetRSPath(x / turning_radius_, y / turning_radius_, phi);
}

RSPath::RSPathData RSPath::GetRSPath(const double x, const double y, const double phi) {
  RSPathData path;
  CSC(x, y, phi, path);
  CCC(x, y, phi, path);
  CCCC(x, y, phi, path);
  CCSC(x, y, phi, path);
  CCSCC(x, y, phi, path);

  return path;
}

double RSPath::Distance(const double x_0, const double y_0, const double yaw_0, const double x_1, const double y_1,
                        const double yaw_1) {
  return turning_radius_ * GetRSPath(x_0, y_0, yaw_0, x_1, y_1, yaw_1).Length();
}

void RSPath::CSC(double x, double y, double phi, RSPathData& path) {
  double t, u, v, length_min = path.Length(), L;

  if (LpSpLp(x, y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[14], t, u, v);
    length_min = L;
  }

  if (LpSpLp(-x, y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[14], -t, -u, -v);
    length_min = L;
  }

  if (LpSpLp(x, -y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[15], t, u, v);
    length_min = L;
  }

  if (LpSpLp(-x, -y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[15], -t, -u, -v);
    length_min = L;
  }

  if (LpSpRp(x, y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[12], t, u, v);
    length_min = L;
  }

  if (LpSpRp(-x, y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[12], -t, -u, -v);
    length_min = L;
  }

  if (LpSpRp(x, -y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[13], t, u, v);
    length_min = L;
  }

  if (LpSpRp(-x, -y, phi, t, u, v) && length_min > std::fabs(t) + std::fabs(u) + std::fabs(v)) {
    path = RSPathData(RS_path_segment_type[13], -t, -u, -v);
  }
}

bool RSPath::LpRmL(double x, double y, double phi, double& t, double& u, double& v) {
  double xi = x - std::sin(phi);
  double eta = y - 1.0 + std::cos(phi);
  double u1, theta;
  Polar(xi, eta, u1, theta);

  if (u1 > 4.0) {
    return false;
  }

  u = -2.0 * std::asin(0.25 * u1);
  t = Mod2Pi(theta + 0.5 * u + M_PI);
  v = Mod2Pi(phi - t + u);

  return true;
}

void RSPath::CCC(double x, double y, double phi, RSPathData& path) {
  double t, u, v, L;
  double length_min = path.Length();

  if (LpRmL(x, y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[0], t, u, v);
    length_min = L;
  }

  if (LpRmL(-x, y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[0], -t, -u, -v);
    length_min = L;
  }

  if (LpRmL(x, -y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[1], t, u, v);
    length_min = L;
  }

  if (LpRmL(-x, -y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[1], -t, -u, -v);
    length_min = L;
  }

  double xb = x * std::cos(phi) + y * std::sin(phi);
  double yb = x * std::sin(phi) - y * std::cos(phi);
  if (LpRmL(xb, yb, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[0], v, u, t);
    length_min = L;
  }

  if (LpRmL(-xb, yb, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[0], -v, -u, -t);
    length_min = L;
  }

  if (LpRmL(xb, -yb, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[1], v, u, t);
    length_min = L;
  }

  if (LpRmL(-xb, -yb, phi, t, u, v) && length_min > std::fabs(t) + std::fabs(u) + std::fabs(v)) {
    path = RSPathData(RS_path_segment_type[1], -v, -u, -t);
  }
}

bool RSPath::LpRupLumRm(double x, double y, double phi, double& t, double& u, double& v) {
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  double rho = 0.25 * (2.0 + std::sqrt(xi * xi + eta * eta));

  if (rho > 1.0) {
    return false;
  }

  u = std::acos(rho);
  TauOmega(u, -u, xi, eta, phi, t, v);

  return true;
}

bool RSPath::LpRumLumRp(double x, double y, double phi, double& t, double& u, double& v) {
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  double rho = (20.0 - xi * xi - eta * eta) / 16.0;

  if (rho >= 0.0 && rho <= 1.0) {
    u = -std::acos(rho);
    if (u >= -M_PI_2) {
      TauOmega(u, u, xi, eta, phi, t, v);
      return true;
    }
  }

  return false;
}

void RSPath::CCCC(double x, double y, double phi, RSPathData& path) {
  double t, u, v, L;
  double length_min = path.Length();

  if (LpRupLumRm(x, y, phi, t, u, v) && length_min > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[2], t, u, -u, v);
    length_min = L;
  }

  if (LpRupLumRm(-x, y, -phi, t, u, v) && length_min > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[2], -t, -u, u, -v);
    length_min = L;
  }

  if (LpRupLumRm(x, -y, -phi, t, u, v) && length_min > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[3], t, u, -u, v);
    length_min = L;
  }

  if (LpRupLumRm(-x, -y, phi, t, u, v) && length_min > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[3], -t, -u, u, -v);
    length_min = L;
  }

  if (LpRumLumRp(x, y, phi, t, u, v) && length_min > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[2], t, u, u, v);
    length_min = L;
  }

  if (LpRumLumRp(-x, y, -phi, t, u, v) && length_min > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[2], -t, -u, -u, -v);
    length_min = L;
  }

  if (LpRumLumRp(x, -y, -phi, t, u, v) && length_min > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[3], t, u, u, v);
    length_min = L;
  }

  if (LpRumLumRp(-x, -y, phi, t, u, v) && length_min > std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v)) {
    path = RSPathData(RS_path_segment_type[3], -t, -u, -u, -v);
  }
}

bool RSPath::LpRmSmLm(double x, double y, double phi, double& t, double& u, double& v) {
  double xi = x - std::sin(phi);
  double eta = y - 1.0 + std::cos(phi);
  double rho, theta;

  Polar(xi, eta, rho, theta);

  if (rho < 2.0) {
    return false;
  }

  double r = std::sqrt(rho * rho - 4.0);
  u = 2.0 - r;
  t = Mod2Pi(theta + std::atan2(r, -2.0));
  v = Mod2Pi(phi - M_PI_2 - t);

  return true;
}

bool RSPath::LpRmSmRm(double x, double y, double phi, double& t, double& u, double& v) {
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  double rho, theta;

  Polar(-eta, xi, rho, theta);

  if (rho < 2.0) {
    return false;
  }

  t = theta;
  u = 2.0 - rho;
  v = Mod2Pi(t + M_PI_2 - phi);

  return true;
}

void RSPath::CCSC(double x, double y, double phi, RSPathData& path) {
  double t, u, v, L;
  double length_min = path.Length() - M_PI_2;

  if (LpRmSmLm(x, y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[4], t, -M_PI_2, u, v);
    length_min = L;
  }

  if (LpRmSmLm(-x, y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[4], -t, M_PI_2, -u, -v);
    length_min = L;
  }

  if (LpRmSmLm(x, -y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[5], t, -M_PI_2, u, v);
    length_min = L;
  }

  if (LpRmSmLm(-x, -y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[5], -t, M_PI_2, -u, -v);
    length_min = L;
  }

  if (LpRmSmRm(x, y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[8], t, -M_PI_2, u, v);
    length_min = L;
  }

  if (LpRmSmRm(-x, y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[8], -t, M_PI_2, -u, -v);
    length_min = L;
  }

  if (LpRmSmRm(x, -y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[9], t, -M_PI_2, u, v);
    length_min = L;
  }

  if (LpRmSmRm(-x, -y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[9], -t, M_PI_2, -u, -v);
    length_min = L;
  }

  double xb = x * std::cos(phi) + y * std::sin(phi);
  double yb = x * std::sin(phi) - y * std::cos(phi);
  if (LpRmSmLm(xb, yb, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[6], v, u, -M_PI_2, t);
    length_min = L;
  }

  if (LpRmSmLm(-xb, yb, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[6], -v, -u, M_PI_2, -t);
    length_min = L;
  }

  if (LpRmSmLm(xb, -yb, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[7], v, u, -M_PI_2, t);
    length_min = L;
  }

  if (LpRmSmLm(-xb, -yb, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[7], -v, -u, M_PI_2, -t);
    length_min = L;
  }

  if (LpRmSmRm(xb, yb, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[10], v, u, -M_PI_2, t);
    length_min = L;
  }

  if (LpRmSmRm(-xb, yb, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[10], -v, -u, M_PI_2, -t);
    length_min = L;
  }

  if (LpRmSmRm(xb, -yb, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[11], v, u, -M_PI_2, t);
    length_min = L;
  }

  if (LpRmSmRm(-xb, -yb, phi, t, u, v) && length_min > std::fabs(t) + std::fabs(u) + std::fabs(v)) {
    path = RSPathData(RS_path_segment_type[11], -v, -u, M_PI_2, -t);
  }
}

bool RSPath::LpRmSLmRp(double x, double y, double phi, double& t, double& u, double& v) {
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  double rho, theta;

  Polar(xi, eta, rho, theta);

  if (rho >= 2.0) {
    u = 4.0 - std::sqrt(rho * rho - 4.0);

    if (u <= 0.0) {
      t = Mod2Pi(std::atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
      v = Mod2Pi(t - phi);

      return true;
    }
  }

  return false;
}

void RSPath::CCSCC(double x, double y, double phi, RSPathData& path) {
  double t, u, v, L;
  double length_min = path.Length() - M_PI;
  if (LpRmSLmRp(x, y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[16], t, -M_PI_2, u, -M_PI_2, v);
    length_min = L;
  }

  if (LpRmSLmRp(-x, y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[16], -t, M_PI_2, -u, M_PI_2, -v);
    length_min = L;
  }

  if (LpRmSLmRp(x, -y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = RSPathData(RS_path_segment_type[17], t, -M_PI_2, u, -M_PI_2, v);
    length_min = L;
  }

  if (LpRmSLmRp(-x, -y, phi, t, u, v) && length_min > std::fabs(t) + std::fabs(u) + std::fabs(v)) {
    path = RSPathData(RS_path_segment_type[17], -t, M_PI_2, -u, M_PI_2, -v);
  }
}

std::vector<Eigen::Vector3d> RSPath::GetRSPath(const Eigen::Vector3d& start_state, const Eigen::Vector3d& goal_state,
                                               const double step_size, double& length) {
  RSPathData rs_path =
      GetRSPath(start_state.x(), start_state.y(), start_state.z(), goal_state.x(), goal_state.y(), goal_state.z());

  length = rs_path.Length() * turning_radius_;

  const double path_length = rs_path.Length() * turning_radius_;
  const auto interpolation_number = static_cast<unsigned int>(path_length / step_size);

  double phi;

  std::vector<Eigen::Vector3d> path_poses;

  for (unsigned int i = 0; i <= interpolation_number; ++i) {
    double v;
    double t = i * 1.0 / interpolation_number;
    double seg = t * rs_path.Length();

    Eigen::Vector3d temp_pose(0.0, 0.0, start_state.z());
    for (unsigned int j = 0; j < 5u && seg > 0; ++j) {
      if (rs_path.length_[j] < 0.0) {
        v = std::max(-seg, rs_path.length_[j]);
        seg += v;
      } else {
        v = std::min(seg, rs_path.length_[j]);
        seg -= v;
      }

      phi = temp_pose.z();
      switch (rs_path.type_[j]) {
        case L:
          temp_pose.x() = std::sin(phi + v) - std::sin(phi) + temp_pose.x();
          temp_pose.y() = -std::cos(phi + v) + std::cos(phi) + temp_pose.y();
          temp_pose.z() = phi + v;
          break;
        case R:
          temp_pose.x() = -std::sin(phi - v) + std::sin(phi) + temp_pose.x();
          temp_pose.y() = std::cos(phi - v) - std::cos(phi) + temp_pose.y();
          temp_pose.z() = phi - v;
          break;
        case S:
          temp_pose.x() = v * std::cos(phi) + temp_pose.x();
          temp_pose.y() = v * std::sin(phi) + temp_pose.y();
          temp_pose.z() = phi;
          break;
        case N:
          break;
      }
    }
    Eigen::Vector3d pose(0.0, 0.0, 0.0);
    pose.block<2, 1>(0, 0) = temp_pose.block<2, 1>(0, 0) * turning_radius_ + start_state.block<2, 1>(0, 0);
    pose.z() = temp_pose.z();
    path_poses.emplace_back(pose);
  }

  return path_poses;
}

std::vector<std::vector<PathPt>> RSPath::GetRSPath(const Eigen::Vector3d& start_state,
                                                   const Eigen::Vector3d& goal_state, const double step_size,
                                                   double& length, int& num, bool s_enable) {
  // 1. 获取Reeds-Shepp路径数据
  RSPathData rs_path =
      GetRSPath(start_state.x(), start_state.y(), start_state.z(), goal_state.x(), goal_state.y(), goal_state.z());

  std::vector<std::vector<PathPt>> rs_res_paths;
  // 结果校验S形轨迹
  if (!s_enable) {
    bool last_is_forward = rs_path.length_[0] > 0.0;
    bool has_left = false;
    bool has_right = false;
    for (int i = 0; i < 5; ++i) {
      bool cur_is_forward = rs_path.length_[i] > 0.0;
      if (last_is_forward != cur_is_forward) {
        has_left = false;
        has_right = false;
      }
      if (rs_path.type_[i] == L) {
        has_left = true;
      } else if (rs_path.type_[i] == R) {
        has_right = true;
      }

      if (has_left && has_right) {
        return rs_res_paths;
      }
      last_is_forward = cur_is_forward;
    }
  }

  // 2. 初始化变量
  Eigen::Vector3d current_pose = start_state;
  double phi = start_state.z();  // 当前航向角

  PathPt::Direction current_direction = PathPt::Direction::FORWARD;  // 初始方向
  double current_kappa = 0.0;                                        // 当前曲率

  // 3. 遍历每一段路径（最多5段）
  for (int i = 0; i < 5; ++i) {
    if (std::abs(rs_path.length_[i]) < 1e-6)
      break;  // 跳过零长度段
    // 4. 确定当前段的方向和曲率
    if (rs_path.type_[i] != N) {
      current_direction = (rs_path.length_[i] < 0.0) ? PathPt::Direction::BACKWARD : PathPt::Direction::FORWARD;
      switch (rs_path.type_[i]) {
        case L:
          current_kappa = 1.0 / turning_radius_;
          break;
        case R:
          current_kappa = -1.0 / turning_radius_;
          break;
        case S:
          current_kappa = 0.0;
          break;
      }
    }

    // 5. 计算当前段的插值点数
    double segment_length = std::abs(rs_path.length_[i]) * turning_radius_;
    int interpolation_number = std::max(1, static_cast<int>(segment_length / step_size));

    // 6. 生成当前段的轨迹点
    std::vector<PathPt> current_segment;
    for (int j = 0; j <= interpolation_number; ++j) {
      double t = static_cast<double>(j) / interpolation_number;
      double v = t * rs_path.length_[i];  // 归一化长度（带方向）
      Eigen::Vector3d temp_pose = current_pose;
      switch (rs_path.type_[i]) {
        case L:  // 左转圆弧
          temp_pose.x() += turning_radius_ * (std::sin(phi + v) - std::sin(phi));
          temp_pose.y() += turning_radius_ * (-std::cos(phi + v) + std::cos(phi));
          temp_pose.z() = phi + v;
          break;
        case R:  // 右转圆弧
          temp_pose.x() += turning_radius_ * (-std::sin(phi - v) + std::sin(phi));
          temp_pose.y() += turning_radius_ * (std::cos(phi - v) - std::cos(phi));
          temp_pose.z() = phi - v;
          break;
        case S:  // 直线
          temp_pose.x() += turning_radius_ * v * std::cos(phi);
          temp_pose.y() += turning_radius_ * v * std::sin(phi);
          break;
        case N:  // 无运动
          break;
      }
      // 创建路径点 (PathPt ctor: x, y, theta, kappa, s)
      PathPt pt(temp_pose.x(), temp_pose.y(), temp_pose.z(), current_kappa, 0.0);
      pt.set_direction(current_direction);
      current_segment.push_back(pt);
    }

    ReCalculateLineLength(current_segment);
    // 7. 更新当前位姿（使用当前段的最后一个点）
    if (!current_segment.empty()) {
      current_pose.x() = current_segment.back().x();
      current_pose.y() = current_segment.back().y();
      current_pose.z() = current_segment.back().theta();
      phi = current_pose.z();
    }

    // 8. 合并或新建轨迹段
    if (!rs_res_paths.empty() && rs_res_paths.back().front().direction() == current_segment.front().direction()) {
      // 方向相同，合并到前一段
      rs_res_paths.back().insert(rs_res_paths.back().end(), current_segment.begin(), current_segment.end());
    } else {
      // 方向不同，新建一段
      rs_res_paths.push_back(current_segment);
    }
  }

  // 9. 设置输出参数
  num = rs_res_paths.size();
  length = rs_path.Length() * turning_radius_;

  // 10. 调试输出
  // OPENSPACE_LOG(D, "++++++++++++++++++++++++++++++");
  // OPENSPACE_LOG(D, "start_state: ", start_state.x(), " ", start_state.y());
  // OPENSPACE_LOG(D, "goal_state: ", goal_state.x(), " ", goal_state.y());
  // for (auto dis : rs_path.length_) {
  //   OPENSPACE_LOG(D, "dis: ", dis * turning_radius_);
  // }
  // for (auto path : rs_res_paths) {
  //   for (auto pt : path) {
  //     OPENSPACE_LOG(D, "rs_res_paths: ", pt.x(), " ", pt.y(), " ", pt.theta(), " ", pt.kappa(), " ",
  //               static_cast<int>(pt.direction()));
  //   }
  //   OPENSPACE_LOG(D, "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
  // }
  // OPENSPACE_LOG(D, "++++++++++++++++++++++++++++++");

  return rs_res_paths;
}

}  // namespace gpal::pnc::planning