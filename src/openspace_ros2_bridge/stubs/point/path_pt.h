#pragma once
#include <cstdint>
#include <cmath>

class PathPt {
 public:
  enum class Direction : uint8_t { FORWARD = 0, BACKWARD = 1 };

  PathPt() = default;
  PathPt(double x, double y, double theta)
      : x_(x), y_(y), theta_(theta), kappa_(0), s_(0), dkappa_(0), ddkappa_(0), direction_(Direction::FORWARD) {}
  PathPt(double x, double y, double theta, double kappa, double s)
      : x_(x), y_(y), theta_(theta), kappa_(kappa), s_(s), dkappa_(0), ddkappa_(0), direction_(Direction::FORWARD) {}
  PathPt(double x, double y, double theta, double kappa, double s, double dkappa, double ddkappa, uint8_t dir)
      : x_(x), y_(y), theta_(theta), kappa_(kappa), s_(s), dkappa_(dkappa), ddkappa_(ddkappa),
        direction_(static_cast<Direction>(dir)) {}

  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }
  double theta() const { return theta_; }
  double kappa() const { return kappa_; }
  double s() const { return s_; }
  double dkappa() const { return dkappa_; }
  double ddkappa() const { return ddkappa_; }
  double front_steer() const { return front_steer_; }
  double rear_steer() const { return rear_steer_; }
  Direction direction() const { return direction_; }

  void set_x(double v) { x_ = v; }
  void set_y(double v) { y_ = v; }
  void set_z(double v) { z_ = v; }
  void set_theta(double v) { theta_ = v; }
  void set_kappa(double v) { kappa_ = v; }
  void set_s(double v) { s_ = v; }
  void set_dkappa(double v) { dkappa_ = v; }
  void set_ddkappa(double v) { ddkappa_ = v; }
  void set_front_steer(double v) { front_steer_ = v; }
  void set_rear_steer(double v) { rear_steer_ = v; }
  void set_direction(Direction d) { direction_ = d; }

  double DistanceTo(const PathPt& other) const {
    double dx = x_ - other.x_;
    double dy = y_ - other.y_;
    return std::sqrt(dx * dx + dy * dy);
  }

  PathPt path_pt() const { return *this; }

 private:
  double x_ = 0, y_ = 0, z_ = 0, theta_ = 0, kappa_ = 0, s_ = 0;
  double dkappa_ = 0, ddkappa_ = 0;
  double front_steer_ = 0, rear_steer_ = 0;
  Direction direction_ = Direction::FORWARD;
};
