#include "local_view/locator_compensator.h"

#include "base/log.h"
#include "math/linear_interpolation.h"

namespace gpal::pnc::planning {

// ego -> map
std::pair<bool, Eigen::Matrix4d> VehiclePoseCompensator::getTransformEgo2Map() const {
  std::pair<bool, Eigen::Matrix4d> res(false, Eigen::Matrix4d::Identity());
  if (ptr_cache_ != nullptr) {
    res.first = true;
    res.second = getTfEgo2Map(ptr_cache_);
  }
  return res;
}

// map -> ego
std::pair<bool, Eigen::Matrix4d> VehiclePoseCompensator::getTransformMap2Ego() const {
  std::pair<bool, Eigen::Matrix4d> res(false, Eigen::Matrix4d::Identity());
  if (ptr_cache_ != nullptr) {
    res.first = true;
    res.second = getTfMap2Ego(ptr_cache_);
  }
  return res;
}

bool VehiclePoseCompensator::hasPoseBuffer() const {
  std::lock_guard<std::mutex> lck(buffer_mutex_);
  return !pose_buffer_.empty();
}

void VehiclePoseCompensator::appendPoseBuffer(std::shared_ptr<gpal::proto::VehiclePose> ptr,
                                              const std::chrono::milliseconds timeout, const size_t size_max) {
  std::lock_guard<std::mutex> lck(buffer_mutex_);
  if (!ptr->is_valid()) {
    last_pose_is_valid_ = false;
    return;
  }
  if (isBufferTimeout(ptr, timeout) || !last_pose_is_valid_) {
    pose_buffer_.clear();
  }
  // ERT_PLOG_I << "append pose stamp: " << std::chrono::nanoseconds(ptr->header.stamp.sec * 1000000000ULL +
  // ptr->header.stamp.nanosec).count() ;
  last_pose_is_valid_ = true;
  pose_buffer_.emplace_back(ptr);
  while (pose_buffer_.size() > size_max) {
    pose_buffer_.pop_front();
  }
}

void VehiclePoseCompensator::appendPoseBuffer(std::shared_ptr<gpal::proto::VehiclePose> ptr, const MapPoint origin,
                                              const std::chrono::milliseconds timeout, const size_t size_max) {
  std::lock_guard<std::mutex> lck(buffer_mutex_);
  if (!ptr->is_valid()) {
    last_pose_is_valid_ = false;
    return;
  }
  auto ptr_deviation = std::make_shared<gpal::proto::VehiclePose>(*ptr);
  ptr_deviation->mutable_position()->set_x(ptr_deviation->position().x() - origin.x());
  ptr_deviation->mutable_position()->set_y(ptr_deviation->position().y() - origin.y());

  if (isBufferTimeout(ptr_deviation, timeout) || !last_pose_is_valid_) {
    pose_buffer_.clear();
  }
  // ERT_PLOG_I << "append deviation pose stamp: " << std::chrono::nanoseconds(ptr_deviation->header.stamp.sec *
  // 1000000000ULL + ptr_deviation->header.stamp.nanosec).count() ;
  last_pose_is_valid_ = true;
  pose_buffer_.emplace_back(ptr_deviation);
  while (pose_buffer_.size() > size_max) {
    pose_buffer_.pop_front();
  }
}

bool VehiclePoseCompensator::isBufferTimeout(std::shared_ptr<gpal::proto::VehiclePose> ptr,
                                             const std::chrono::milliseconds timeout) const {
  return !pose_buffer_.empty() && (std::chrono::nanoseconds(ptr->header().time_meas()) <
                                       std::chrono::nanoseconds(pose_buffer_.back()->header().time_meas()) ||
                                   std::chrono::duration_cast<std::chrono::milliseconds>(
                                       std::chrono::nanoseconds(ptr->header().time_meas()) -
                                       std::chrono::nanoseconds(pose_buffer_.back()->header().time_meas())) > timeout);
}

// bool VehiclePoseCompensator::isBufferLowConfidence(const int low_conf_thrd) const {
//   std::lock_guard<std::mutex> lck(buffer_mutex_);
//   int size = static_cast<int>(pose_buffer_.size());
//   int cnt = 0;
//   for (int i = size - 1; i >= std::max(0, size - low_conf_thrd); i--) {
//     if (pose_buffer_[i]->align_status() < uto::gpal::proto::VehiclePose::POOR) {
//       cnt++;
//       if (cnt == low_conf_thrd) {
//         return true;
//       }
//     } else {
//       return false;
//     }
//   }
//   return false;
// }

Eigen::Matrix4d VehiclePoseCompensator::getTfEgo2Map(std::shared_ptr<gpal::proto::VehiclePose> ptr) const {
  CHECK_NOTNULL(ptr);
  Eigen::Matrix4d t_matrix = Eigen::Matrix4d::Identity();
  Eigen::Vector3d position(ptr->position().x(), ptr->position().y(), ptr->position().z());
  Eigen::Quaterniond quaternion;
  quaternion = transfer::QuatFromEuler(ptr->euler_angles().z(), ptr->euler_angles().y(), ptr->euler_angles().x());
  Eigen::Matrix3d rotmat;
  transfer::RotmatFromQuat(quaternion, rotmat);
  t_matrix.block<3, 1>(0, 3) = position;
  t_matrix.block<3, 3>(0, 0) = rotmat;

  return t_matrix;
}

Eigen::Matrix4d VehiclePoseCompensator::getTfMap2Ego(std::shared_ptr<gpal::proto::VehiclePose> ptr) const {
  Eigen::Matrix4d t_matrix = Eigen::Matrix4d::Identity();
  Eigen::Vector3d position(ptr->position().x(), ptr->position().y(), ptr->position().z());
  Eigen::Quaterniond quaternion;
  quaternion = transfer::QuatFromEuler(ptr->euler_angles().z(), ptr->euler_angles().y(), ptr->euler_angles().x());
  Eigen::Quaterniond quaternion_inv;
  quaternion_inv = quaternion.inverse();
  Eigen::Matrix3d rotmat;
  transfer::RotmatFromQuat(quaternion_inv, rotmat);
  t_matrix.block<3, 1>(0, 3) = quaternion_inv * position * (-1.0);
  t_matrix.block<3, 3>(0, 0) = rotmat;

  return t_matrix;
}
}  // namespace gpal::pnc::planning
