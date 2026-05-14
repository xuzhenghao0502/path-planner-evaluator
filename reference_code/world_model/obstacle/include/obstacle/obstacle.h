#pragma once

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "base/indexed_list.h"
#include "config/vehicle_model/vehicle_config.pb.h"
#include "gpal-interface/perception/perception_obstacle.pb.h"
#include "gpal-interface/planning/planning_visualization.pb.h"
#include "gpal-interface/planning/trajectory.pb.h"
#include "math/box2d.h"
#include "math/interval_data.h"
#include "math/polygon2d.h"
#include "math/vec2d.h"
#include "point/path_pt.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/sl_boundary.pb.h"
#include "proto/obstacle_box.pb.h"
#include "reference_line/reference_line.h"
#include "st_boundary.h"

namespace gpal::pnc::planning {

class Obstacle {
 public:
  Obstacle() = default;
  ~Obstacle() = default;
  explicit Obstacle(const std::string& id, math::Polygon2d polygon);
  explicit Obstacle(const std::string& id, const gpal::proto::PerceptionObstacle& perception_obstacle,
                    const std::vector<ObstacleBox>& obstacle_boxes, const double base_relative_time = 0.0);
  explicit Obstacle(const std::string& id, const int sub_index,
                    const gpal::proto::PerceptionObstacle& perception_obstacle,
                    const std::vector<proto::TrajectoryPoint>& predicted_trajectory,
                    const double base_relative_time = 0.0);

  const std::string& id() const { return id_; }
  void setId(const std::string& id) { id_ = id; }
  const double& timeDelay() const { return time_delay_; }
  void setTimeDelay(const double& time_delay) { time_delay_ = time_delay; }
  const int32_t perception_id() const { return perception_id_; }
  void setPerceptionId(const int32_t& perception_id) { perception_id_ = perception_id; }
  const double& age() const { return age_; }
  void setAge(const double& age) { age_ = age; }
  const double& exist_confidence() const { return exist_confidence_; }
  void setExistConfidence(const double& exist_confidence) { exist_confidence_ = exist_confidence; }
  bool isVirtual() const { return is_virtual_; }
  const double speed() const { return speed_; }
  void setSpeed(const double& speed) { speed_ = speed; }
  bool isMultiSubObstacle() const { return is_multi_sub_obstacle_; }
  const std::vector<pnc::ObstacleBox>& obstacle_boxes() const { return obstacle_boxes_; }
  std::vector<pnc::ObstacleBox>& mutable_obstacle_boxes() { return obstacle_boxes_; }
  void setVirtual(const bool is_virtual) { is_virtual_ = is_virtual; }

  const gpal::proto::PerceptionObstacle_ObstacleType& type() const { return type_; };
  const gpal::proto::PerceptionObstacle_ObstacleMotionStatus& motion_status() const { return motion_status_; }
  const gpal::proto::VehicleLightStatus& vehicle_light_status() const { return vehicle_light_status_; }
  void set_type(const gpal::proto::PerceptionObstacle_ObstacleType& type) { type_ = type; }
  void set_motion_status(const gpal::proto::PerceptionObstacle_ObstacleMotionStatus& motion_status) {
    motion_status_ = motion_status;
  }
  void set_vehicle_light_status(const gpal::proto::VehicleLightStatus& vehicle_light_status) {
    vehicle_light_status_ = vehicle_light_status;
  }
  const math::Box2d& perceptionBoundingBox() const { return perception_bounding_box_; }
  const math::Polygon2d& perceptionPolygon() const { return perception_polygon_; }

  const std::string debugString() const;

 public:
  static Obstacle createObstacle(const gpal::proto::PerceptionObstacle& perception_obstacle,
                                               const std::vector<ObstacleBox>& obstacle_boxes,
                                               const double base_relative_time = 0.0);
  static std::unique_ptr<Obstacle> createStaticVirtualObstacle(const int32_t& id, const math::Box2d& box2d);

 private:
  bool invadePath(const double start_l, const double end_l, const double left_boundary,
                  const double right_boundary) const {
    return !(start_l > left_boundary || end_l < right_boundary);
  }

 private:
  std::string id_ = "";
  int32_t perception_id_{-1};
  double age_{0.0};
  double exist_confidence_{0.0};
  double time_delay_{0.0};
  bool is_virtual_{false};
  double speed_{0.0};
  bool is_multi_sub_obstacle_{false};

  std::vector<pnc::ObstacleBox> obstacle_boxes_;
  gpal::proto::PerceptionObstacle_ObstacleType type_{gpal::proto::PerceptionObstacle_ObstacleType_kTypeInvalid};
  gpal::proto::PerceptionObstacle_ObstacleMotionStatus motion_status_{
      gpal::proto::PerceptionObstacle_ObstacleMotionStatus_kMotionInvalid};
  gpal::proto::VehicleLightStatus vehicle_light_status_;
  math::Box2d perception_bounding_box_;
  math::Polygon2d perception_polygon_;
};
static constexpr double kEpsilon = 1e-6f;
constexpr int kObstacleKeepFrameCount = 1;
constexpr float kObstacleMinWidth = 0.2;
typedef IndexedList<Obstacle, kObstacleKeepFrameCount> IndexedObstacles;
typedef ThreadSafeIndexedList<Obstacle> ThreadSafeIndexedObstacles;

}  // namespace gpal::pnc::planning
