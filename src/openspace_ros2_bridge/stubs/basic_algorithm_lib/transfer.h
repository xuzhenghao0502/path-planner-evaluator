#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include "point/path_pt.h"

namespace transfer {

inline void transformPoint(const Eigen::Matrix4d& tf, PathPt* pt) {
  double x = pt->x(), y = pt->y();
  double new_x = tf(0, 0) * x + tf(0, 1) * y + tf(0, 3);
  double new_y = tf(1, 0) * x + tf(1, 1) * y + tf(1, 3);
  pt->set_x(new_x);
  pt->set_y(new_y);
}

inline void transformRPY(const Eigen::Matrix4d& tf, Eigen::Vector3d* rpy) {
  double yaw = std::atan2(tf(1, 0), tf(0, 0));
  rpy->z() += yaw;
}

inline void transformPoint(const Eigen::Matrix4d& tf, Eigen::Vector3d* pt) {
  Eigen::Vector4d h(pt->x(), pt->y(), pt->z(), 1.0);
  h = tf * h;
  pt->x() = h.x();
  pt->y() = h.y();
  pt->z() = h.z();
}

}  // namespace transfer
