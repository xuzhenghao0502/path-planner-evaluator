#pragma once

/**
 * @file
 * @brief Defines the Localization class.
 */

#include "basic_algorithm_lib/basic_algorithm_lib.h"

namespace gpal::pnc::planning {
/**
 * @class Localization
 *
 * @brief Implements a class of Localization Data.
 */

using namespace std;
using gpal::pnc::planning::Status;

class Localization : public StampedBase {
 public:
  friend class VehiclePoseAdapter;
  friend class OdometryAdapter;
  explicit Localization();
  ~Localization();

  void clear();

  const bool& isOdometryLocalization() const { return is_odometry_loc_; }
  void setIsOdometryLocalization(const bool& is_odometry_loc) { is_odometry_loc_ = is_odometry_loc; }

  const bool& isLastOdometryLocalization() const { return is_last_odometry_loc_; }
  void setIsLastOdometryLocalization(const bool& is_last_odometry_loc) { is_last_odometry_loc_ = is_last_odometry_loc; }

  MapPoint* mutablePncOriginPoint() { return &pnc_origin_point_; }
  const MapPoint& pncOriginPoint() const { return pnc_origin_point_; }

  MapPoint* mutableVehicleAlignPosePoint() { return &vehicle_align_pose_point_; }
  const MapPoint& vehicleAlignPosePoint() const { return vehicle_align_pose_point_; }

  Eigen::Matrix4d getTfEgo2Map() const;
  Eigen::Matrix4d getTfEgo2Map(const MapPoint& vehicle_pose) const;
  Eigen::Matrix4d getTfMap2Ego() const;
  Eigen::Matrix4d getTfMap2Ego(const MapPoint& vehicle_pose) const;

 private:
  bool is_odometry_loc_ = false;
  bool is_last_odometry_loc_ = false;
  MapPoint pnc_origin_point_;
  MapPoint vehicle_align_pose_point_;
};

}  // namespace gpal::pnc::planning
