#pragma once

#include <chrono>
#include <deque>
#include <utility>

#include "Eigen/Geometry"
#include "base/log.h"
#include "basic_algorithm_lib/position_conversion.h"
#include "gpal-interface/localization/vehicle_pose.pb.h"
#include "math/linear_interpolation.h"
#include "point/point.h"

namespace gpal::pnc::planning {
class VehiclePoseCompensator {
 public:
  // static VehiclePoseCompensator& instance() {
  //   static VehiclePoseCompensator instance;
  //   return instance;
  // }
  VehiclePoseCompensator() = default;

 protected:
  using BaseStamp = std::chrono::duration<double, std::milli>;

 public:
  ~VehiclePoseCompensator() = default;
  void setReverse(bool is_reverse = true) { is_reverse_ = is_reverse; };
  bool hasBaseVehiclePose() const { return ptr_cache_ != nullptr; }
  void setBaseVehiclePose(const gpal::proto::VehiclePose& pose) {
    if (ptr_cache_ == nullptr) {
      ptr_cache_ = std::make_shared<gpal::proto::VehiclePose>(pose);
    } else {
      *ptr_cache_ = pose;
    }
  }
  void setBaseVehiclePose(std::shared_ptr<gpal::proto::VehiclePose> ptr) { ptr_cache_ = ptr; }
  void setBaseVehiclePose(std::shared_ptr<gpal::proto::VehiclePose> ptr, const MapPoint& origin) {
    ptr_cache_ = std::make_shared<gpal::proto::VehiclePose>(*ptr);
    ptr_cache_->mutable_position()->set_x(ptr_cache_->position().x() - origin.x());
    ptr_cache_->mutable_position()->set_y(ptr_cache_->position().y() - origin.y());
  }
  const gpal::proto::VehiclePose getBaseVehiclePose() const {
    if (ptr_cache_ == nullptr) {
      gpal::proto::VehiclePose pose;
      return pose;
    } else {
      return *ptr_cache_;
    }
  }
  template <typename _Rep, typename _Period>
  double getRelativeTime(const std::chrono::duration<_Rep, _Period>& stamp);
  template <typename _Rep, typename _Period>
  std::pair<bool, gpal::proto::VehiclePose> getVehiclePoseStamp(
      const std::chrono::duration<_Rep, _Period>& stamp,
      const std::chrono::milliseconds tol = std::chrono::milliseconds(10)) const;
  std::pair<bool, Eigen::Matrix4d> getTransformEgo2Map() const;
  template <typename _Rep, typename _Period>
  std::pair<bool, Eigen::Matrix4d> getTransformEgo2Map(
      const std::chrono::duration<_Rep, _Period>& stamp,
      const std::chrono::milliseconds = std::chrono::milliseconds(10)) const;
  std::pair<bool, Eigen::Matrix4d> getTransformMap2Ego() const;
  template <typename _Rep, typename _Period>
  std::pair<bool, Eigen::Matrix4d> getTransformEgoCompensate(
      const std::chrono::duration<_Rep, _Period>& stamp,
      const std::chrono::milliseconds = std::chrono::milliseconds(10)) const;
  // bool isBufferLowConfidence(const int low_conf_thrd) const;

 public:  // thread safe
  bool hasPoseBuffer() const;
  void setPoseBuffer(const std::deque<std::shared_ptr<gpal::proto::VehiclePose>>& pose_buffer) {
    std::lock_guard<std::mutex> lck(buffer_mutex_);  // 加锁
    pose_buffer_ = pose_buffer;
  }
  const std::deque<std::shared_ptr<gpal::proto::VehiclePose>> getPoseBuffer() const {
    std::lock_guard<std::mutex> lck(buffer_mutex_);  // 加锁
    return pose_buffer_;
  }
  void appendPoseBuffer(std::shared_ptr<gpal::proto::VehiclePose> ptr,
                        const std::chrono::milliseconds timeout = std::chrono::milliseconds(1000),
                        const size_t size_max = 100);
  void appendPoseBuffer(std::shared_ptr<gpal::proto::VehiclePose> ptr, const MapPoint origin,
                        const std::chrono::milliseconds timeout = std::chrono::milliseconds(1000),
                        const size_t size_max = 100);

 protected:  // not thread safe
  bool isBufferTimeout(std::shared_ptr<gpal::proto::VehiclePose>, const std::chrono::milliseconds) const;
  Eigen::Matrix4d getTfEgo2Map(std::shared_ptr<gpal::proto::VehiclePose>) const;
  Eigen::Matrix4d getTfMap2Ego(std::shared_ptr<gpal::proto::VehiclePose>) const;

 private:  // not thread safe
  bool is_reverse_{false};
  bool last_pose_is_valid_{true};
  std::shared_ptr<gpal::proto::VehiclePose> ptr_cache_{nullptr};

 private:  // thread safe
  mutable std::mutex buffer_mutex_;
  std::deque<std::shared_ptr<gpal::proto::VehiclePose>> pose_buffer_;
};

template <typename _Rep, typename _Period>
double VehiclePoseCompensator::getRelativeTime(const std::chrono::duration<_Rep, _Period>& stamp) {
  double relative_time = 0.0;
  if (ptr_cache_ != nullptr) {
    relative_time = std::chrono::duration_cast<std::chrono::duration<double>>(
                        stamp - std::chrono::nanoseconds(ptr_cache_->header().time_meas()))
                        .count();
  }
  return relative_time;
}

template <typename _Rep, typename _Period>
std::pair<bool, gpal::proto::VehiclePose> VehiclePoseCompensator::getVehiclePoseStamp(
    const std::chrono::duration<_Rep, _Period>& stamp, const std::chrono::milliseconds tol) const {
  std::pair<bool, gpal::proto::VehiclePose> res(false, gpal::proto::VehiclePose());
  std::lock_guard<std::mutex> lck(buffer_mutex_);
  if (!pose_buffer_.empty()) {
    res.first = true;
    if (pose_buffer_.size() < 2) {
      res.second = *pose_buffer_.back();
      // ERT_PLOG_I << "compensate stamp with unique res" ;
    } else {
      auto iter_upper = std::upper_bound(
          pose_buffer_.begin(), pose_buffer_.end(), stamp,
          [](const std::chrono::duration<_Rep, _Period>& stamp, std::shared_ptr<gpal::proto::VehiclePose> element) {
            // return stamp < headerTimeStamp(element->header);
            return stamp < std::chrono::nanoseconds(element->header().time_meas());
          });
      if (iter_upper == pose_buffer_.begin()) {
        res.second = *pose_buffer_.front();
        // ERT_PLOG_I << "compensate stamp " << stamp.count() << " with front " <<
        // std::chrono::nanoseconds(pose_buffer_.front()->header.stamp.sec * 1000000000ULL +
        // pose_buffer_.front()->header.stamp.nanosec).count() ;
      } else if (iter_upper == pose_buffer_.end()) {
        res.second = *pose_buffer_.back();
        // ERT_PLOG_I << "compensate stamp " << stamp.count() << " with back " <<
        // std::chrono::nanoseconds(pose_buffer_.front()->header.stamp.sec * 1000000000ULL +
        // pose_buffer_.back()->header.stamp.nanosec).count() ;
      } else {
        auto err_upper = std::chrono::duration_cast<BaseStamp>(
            std::chrono::nanoseconds((*iter_upper)->header().time_meas()) - stamp);
        if (err_upper < tol) {
          res.second = *(*iter_upper);
          // ERT_PLOG_I << "compensate stamp " << stamp.count() << " with upper " <<
          // std::chrono::nanoseconds((*iter_upper)->header.stamp.sec * 1000000000ULL +
          // (*iter_upper)->header.stamp.nanosec).count() << " within tol " << err_upper.count() ;
        } else {
          // linear interpolate
          auto iter_lower = iter_upper - 1;
          auto stamp_accurate = std::chrono::duration_cast<BaseStamp>(stamp);
          auto stamp_lower =
              std::chrono::duration_cast<BaseStamp>(std::chrono::nanoseconds((*iter_lower)->header().time_meas()));
          auto stamp_upper =
              std::chrono::duration_cast<BaseStamp>(std::chrono::nanoseconds((*iter_upper)->header().time_meas()));
          // ERT_PLOG_I << "compensate stamp " << stamp_accurate.count() << " with lower " << stamp_lower.count() << "
          // and upper " << stamp_upper.count() ;
          res.second.set_is_valid((*iter_upper)->is_valid());
          res.second.mutable_position()->set_x(math::lerp((*iter_lower)->position().x(), stamp_lower.count(),
                                                          (*iter_upper)->position().x(), stamp_upper.count(),
                                                          stamp_accurate.count()));
          res.second.mutable_position()->set_y(math::lerp((*iter_lower)->position().y(), stamp_lower.count(),
                                                          (*iter_upper)->position().y(), stamp_upper.count(),
                                                          stamp_accurate.count()));
          res.second.mutable_position()->set_z(math::lerp((*iter_lower)->position().z(), stamp_lower.count(),
                                                          (*iter_upper)->position().z(), stamp_upper.count(),
                                                          stamp_accurate.count()));
          res.second.mutable_euler_angles()->set_x(math::slerp((*iter_lower)->euler_angles().x(), stamp_lower.count(),
                                                               (*iter_upper)->euler_angles().x(), stamp_upper.count(),
                                                               stamp_accurate.count()));
          res.second.mutable_euler_angles()->set_y(math::slerp((*iter_lower)->euler_angles().y(), stamp_lower.count(),
                                                               (*iter_upper)->euler_angles().y(), stamp_upper.count(),
                                                               stamp_accurate.count()));
          res.second.mutable_euler_angles()->set_z(math::slerp((*iter_lower)->euler_angles().z(), stamp_lower.count(),
                                                               (*iter_upper)->euler_angles().z(), stamp_upper.count(),
                                                               stamp_accurate.count()));
        }
      }
    }
  }
  return res;
}

template <typename _Rep, typename _Period>
std::pair<bool, Eigen::Matrix4d> VehiclePoseCompensator::getTransformEgo2Map(
    const std::chrono::duration<_Rep, _Period>& stamp, const std::chrono::milliseconds tol) const {
  std::pair<bool, Eigen::Matrix4d> res(false, Eigen::Matrix4d::Identity());
  std::lock_guard<std::mutex> lck(buffer_mutex_);
  if (!pose_buffer_.empty()) {
    res.first = true;
    if (pose_buffer_.size() < 2) {
      res.second = getTfEgo2Map(pose_buffer_.back());
      // ERT_PLOG_I << "compensate stamp with unique res" ;
    } else {
      auto iter_upper = std::upper_bound(
          pose_buffer_.begin(), pose_buffer_.end(), stamp,
          [](const std::chrono::duration<_Rep, _Period>& stamp, std::shared_ptr<gpal::proto::VehiclePose> element) {
            return stamp < std::chrono::nanoseconds(element->header().time_meas());
          });
      if (iter_upper == pose_buffer_.begin()) {
        res.second = getTfEgo2Map(*iter_upper);
        // ERT_PLOG_I << "compensate stamp " << stamp.count() << " with front " <<
        // std::chrono::nanoseconds((*iter_upper)->header.stamp.sec * 1000000000ULL +
        // (*iter_upper)->header.stamp.nanosec).count() ;
      } else if (iter_upper == pose_buffer_.end()) {
        res.second = getTfEgo2Map(pose_buffer_.back());
        // ERT_PLOG_I << "compensate stamp " << stamp.count() << " with back " <<
        // std::chrono::nanoseconds(pose_buffer_.back()->header.stamp.sec * 1000000000ULL +
        // pose_buffer_.back()->header.stamp.nanosec).count() ;
      } else {
        auto err_upper = std::chrono::duration_cast<BaseStamp>(
            std::chrono::nanoseconds(std::chrono::nanoseconds((*iter_upper)->header().time_meas()) - stamp));
        if (err_upper < tol) {
          res.second = getTfEgo2Map(*iter_upper);
          // ERT_PLOG_I << "compensate stamp " << stamp.count() << " with upper " <<
          // std::chrono::nanoseconds((*iter_upper)->header.stamp.sec * 1000000000ULL +
          // (*iter_upper)->header.stamp.nanosec).count() << " within tol " << err_upper.count() ;
        } else {
          // linear interpolate
          auto iter_lower = iter_upper - 1;
          auto stamp_accurate = std::chrono::duration_cast<BaseStamp>(stamp);
          auto stamp_lower =
              std::chrono::duration_cast<BaseStamp>(std::chrono::nanoseconds((*iter_lower)->header().time_meas()));
          auto stamp_upper =
              std::chrono::duration_cast<BaseStamp>(std::chrono::nanoseconds((*iter_upper)->header().time_meas()));
          // ERT_PLOG_I << "compensate stamp " << stamp_accurate.count() << " with lower " << stamp_lower.count() << "
          // and upper " << stamp_upper.count() ;
          double x_lerp = math::lerp((*iter_lower)->position().x(), stamp_lower.count(), (*iter_upper)->position().x(),
                                     stamp_upper.count(), stamp_accurate.count());
          double y_lerp = math::lerp((*iter_lower)->position().y(), stamp_lower.count(), (*iter_upper)->position().y(),
                                     stamp_upper.count(), stamp_accurate.count());
          double z_lerp = math::lerp((*iter_lower)->position().z(), stamp_lower.count(), (*iter_upper)->position().z(),
                                     stamp_upper.count(), stamp_accurate.count());
          double roll_lerp =
              math::slerp((*iter_lower)->euler_angles().x(), stamp_lower.count(), (*iter_upper)->euler_angles().x(),
                          stamp_upper.count(), stamp_accurate.count());
          double pitch_lerp =
              math::slerp((*iter_lower)->euler_angles().y(), stamp_lower.count(), (*iter_upper)->euler_angles().y(),
                          stamp_upper.count(), stamp_accurate.count());
          double yaw_lerp = math::slerp((*iter_lower)->euler_angles().z(), stamp_lower.count(),
                                        (*iter_upper)->euler_angles().z(), stamp_upper.count(), stamp_accurate.count());

          Eigen::Vector3d position(x_lerp, y_lerp, z_lerp);
          Eigen::Quaterniond quaternion;
          quaternion = transfer::QuatFromEuler(yaw_lerp, pitch_lerp, roll_lerp);
          Eigen::Matrix3d rotmat;
          transfer::RotmatFromQuat(quaternion, rotmat);
          res.second.block<3, 1>(0, 3) = position;
          res.second.block<3, 3>(0, 0) = rotmat;
        }
      }
    }
  }
  return res;
}

template <typename _Rep, typename _Period>
std::pair<bool, Eigen::Matrix4d> VehiclePoseCompensator::getTransformEgoCompensate(
    const std::chrono::duration<_Rep, _Period>& stamp, const std::chrono::milliseconds tol) const {
  std::pair<bool, Eigen::Matrix4d> res(false, Eigen::Matrix4d::Identity());
  auto [flag_stamp, tf_ego_2_map_stamp] = getTransformEgo2Map(stamp, tol);
  if (flag_stamp) {
    auto [flag, tf_map_2_ego] = getTransformMap2Ego();
    if (flag) {
      res.second = tf_map_2_ego * tf_ego_2_map_stamp;
      res.first = true;
    }
  }

  return res;
}

}  // namespace gpal::pnc::planning
