#include "obstacle/obstacle.h"

#include <algorithm>
#include <utility>

#include "config_manager/config_manager.h"
#include "math/linear_interpolation.h"
#include "util/string_util.h"
#include "util/util.h"

namespace gpal::pnc::planning {
namespace {

const double kStBoundaryDeltaS = 0.2;           // meters
const double kStBoundarySparseDeltaS = 1.0;     // meters
const double kStBoundaryDeltaT = 0.05;          // seconds
const double MaxStaticObstacleSpeed = 5 / 3.6;  // km/h
}  // namespace

Obstacle::Obstacle(const std::string& id, math::Polygon2d polygon)
    : id_(id),
      perception_polygon_(std::move(polygon)),
      type_{gpal::proto::PerceptionObstacle_ObstacleType_kTypeInvalid} {
  perception_bounding_box_ = perception_polygon_.MinAreaBoundingBox();
  pnc::ObstacleBox obstacle_box;
  obstacle_box.mutable_position()->set_x(perception_bounding_box_.center().x());
  obstacle_box.mutable_position()->set_y(perception_bounding_box_.center().y());
  obstacle_box.mutable_position()->set_z(0.0);
  obstacle_box.mutable_velocity()->set_x(0.0);
  obstacle_box.mutable_velocity()->set_y(0.0);
  obstacle_box.mutable_acceleration()->set_x(0.0);
  obstacle_box.mutable_acceleration()->set_y(0.0);
  obstacle_box.mutable_angle()->set_z(perception_bounding_box_.heading());
  obstacle_box.mutable_angular_rate()->set_z(0.0);
  obstacle_box.set_width(perception_bounding_box_.width());
  obstacle_box.set_length(perception_bounding_box_.length());
  obstacle_box.set_height(0.0);
  obstacle_boxes_.emplace_back(std::move(obstacle_box));
  is_virtual_ = true;
  speed_ = 0;
}

Obstacle::Obstacle(const std::string& id, const gpal::proto::PerceptionObstacle& perception_obstacle,
                   const std::vector<ObstacleBox>& obstacle_boxes, const double base_relative_time)
    : id_(id),
      perception_id_(perception_obstacle.id()),
      age_(perception_obstacle.age()),
      exist_confidence_(perception_obstacle.exist_confidence()),
      type_(perception_obstacle.obstacle_type()),
      motion_status_(perception_obstacle.motion_status()),
      vehicle_light_status_(perception_obstacle.vehicle_light_status()),
      time_delay_(base_relative_time),
      perception_bounding_box_({perception_obstacle.sub_obstacles(0).position().x(),
                                perception_obstacle.sub_obstacles(0).position().y()},
                               perception_obstacle.sub_obstacles(0).heading_angle(),
                               std::max(kObstacleMinWidth, perception_obstacle.sub_obstacles(0).length()),
                               std::max(kObstacleMinWidth, perception_obstacle.sub_obstacles(0).width())) {
  const auto& polygon_points = perception_bounding_box_.GetAllCorners();
  bool is_convex_hull = math::Polygon2d::ComputeConvexHull(polygon_points, &perception_polygon_);
  if (!is_convex_hull) {
    ERT_PLOG_I << "object[" << id << "] polygon is not a valid convex hull.";
    assert(is_convex_hull);
  }
  obstacle_boxes_ = obstacle_boxes;
  is_virtual_ = (perception_obstacle.id() < 0);
  speed_ = std::hypot(obstacle_boxes_.at(0).velocity().x(),
                      obstacle_boxes_.at(0).velocity().y());
}

const std::string Obstacle::debugString() const {
  std::stringstream ss;
  ss << "Obstacle id: " << id_;
  return ss.str();
}

Obstacle Obstacle::createObstacle(const gpal::proto::PerceptionObstacle& perception_obstacle,
                                                const std::vector<ObstacleBox>& obstacle_boxes,
                                                const double base_relative_time) {
  const auto perception_id = std::to_string(perception_obstacle.id());
  return Obstacle(perception_id, perception_obstacle, obstacle_boxes, base_relative_time);
}

std::unique_ptr<Obstacle> Obstacle::createStaticVirtualObstacle(const int32_t& id, const math::Box2d& box2d) {
  // create a "virtual" perception_obstacle
  gpal::proto::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_id(id);

  ObstacleBox obstacle_box;
  obstacle_box.mutable_position()->set_x(box2d.center().x());
  obstacle_box.mutable_position()->set_y(box2d.center().y());
  obstacle_box.mutable_position()->set_z(0.0);
  obstacle_box.mutable_velocity()->set_x(0.0);
  obstacle_box.mutable_velocity()->set_y(0.0);
  obstacle_box.mutable_acceleration()->set_x(0.0);
  obstacle_box.mutable_acceleration()->set_y(0.0);
  obstacle_box.mutable_angle()->set_z(box2d.heading());
  obstacle_box.mutable_angular_rate()->set_z(0.0);
  obstacle_box.set_width(box2d.width());
  obstacle_box.set_length(box2d.length());
  obstacle_box.set_height(5.0);

  auto* obstacle = new Obstacle(std::to_string(id), perception_obstacle, {obstacle_box});
  obstacle->is_virtual_ = true;

  return std::unique_ptr<Obstacle>(obstacle);
}

}  // namespace gpal::pnc::planning
