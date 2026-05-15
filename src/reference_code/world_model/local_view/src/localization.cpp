#include "local_view/localization.h"

namespace gpal::pnc::planning {

Localization::Localization() { clear(); }

Localization::~Localization() {}

void Localization::clear() {
  StampedBase::reset();
  is_odometry_loc_ = false;
  is_last_odometry_loc_ = false;
}

// ego -> map
Eigen::Matrix4d Localization::getTfEgo2Map() const {
  Eigen::Matrix4d t_matrix = Eigen::Matrix4d::Identity();
  Eigen::Vector3d position(vehicle_align_pose_point_.x(), vehicle_align_pose_point_.y(), vehicle_align_pose_point_.z());
  Eigen::Quaterniond quaternion;
  quaternion = transfer::QuatFromEuler(vehicle_align_pose_point_.yaw(), vehicle_align_pose_point_.pitch(),
                                       vehicle_align_pose_point_.roll());
  Eigen::Matrix3d rotmat;
  transfer::RotmatFromQuat(quaternion, rotmat);
  t_matrix.block<3, 1>(0, 3) = position;
  t_matrix.block<3, 3>(0, 0) = rotmat;

  return t_matrix;
}

Eigen::Matrix4d Localization::getTfEgo2Map(const MapPoint& vehicle_pose) const {
  Eigen::Matrix4d t_matrix = Eigen::Matrix4d::Identity();
  Eigen::Vector3d position(vehicle_pose.x(), vehicle_pose.y(), vehicle_pose.z());
  Eigen::Quaterniond quaternion;
  quaternion = transfer::QuatFromEuler(vehicle_pose.yaw(), vehicle_pose.pitch(), vehicle_pose.roll());
  Eigen::Matrix3d rotmat;
  transfer::RotmatFromQuat(quaternion, rotmat);
  t_matrix.block<3, 1>(0, 3) = position;
  t_matrix.block<3, 3>(0, 0) = rotmat;

  return t_matrix;
}

// map -> ego
Eigen::Matrix4d Localization::getTfMap2Ego() const {
  Eigen::Matrix4d t_matrix = Eigen::Matrix4d::Identity();
  Eigen::Vector3d position(vehicle_align_pose_point_.x(), vehicle_align_pose_point_.y(), vehicle_align_pose_point_.z());
  Eigen::Quaterniond quaternion;
  quaternion = transfer::QuatFromEuler(vehicle_align_pose_point_.yaw(), vehicle_align_pose_point_.pitch(),
                                       vehicle_align_pose_point_.roll());
  Eigen::Quaterniond quaternion_inv;
  quaternion_inv = quaternion.inverse();
  Eigen::Matrix3d rotmat;
  transfer::RotmatFromQuat(quaternion_inv, rotmat);
  t_matrix.block<3, 1>(0, 3) = quaternion_inv * position * (-1.0);
  t_matrix.block<3, 3>(0, 0) = rotmat;

  return t_matrix;
}

Eigen::Matrix4d Localization::getTfMap2Ego(const MapPoint& vehicle_pose) const {
  Eigen::Matrix4d t_matrix = Eigen::Matrix4d::Identity();
  Eigen::Vector3d position(vehicle_pose.x(), vehicle_pose.y(), vehicle_pose.z());
  Eigen::Quaterniond quaternion;
  quaternion = transfer::QuatFromEuler(vehicle_pose.yaw(), vehicle_pose.pitch(), vehicle_pose.roll());
  Eigen::Quaterniond quaternion_inv;
  quaternion_inv = quaternion.inverse();
  Eigen::Matrix3d rotmat;
  transfer::RotmatFromQuat(quaternion_inv, rotmat);
  t_matrix.block<3, 1>(0, 3) = quaternion_inv * position * (-1.0);
  t_matrix.block<3, 3>(0, 0) = rotmat;

  return t_matrix;
}

}  // namespace gpal::pnc::planning
