#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <memory>
#include <any>
#include <mutex>
#include <thread>
#include <chrono>
#include <string>
#include <sstream>

#include "openspace_path_planner/core/generator/openspace_path_provider.h"
#include "openspace_path_planner/core/generator/arc_model.h"
#include "openspace_path_planner/core/manager/openspace_search_data.h"
#include "local_view/Freespace.h"
#include "config/openspace/openspace_search.pb.h"
#include "config/vehicle_config.pb.h"
#include "config_manager/config_manager.h"
#include "openspace_ros2_bridge/freespace_adapter.hpp"

using namespace gpal::pnc::planning;

class PlannerBridgeNode : public rclcpp::Node {
 public:
  PlannerBridgeNode() : Node("planner_bridge_node") {
    // Publishers
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/trajectory_path", 10);
    box_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vehicle_boxes", 10);
    result_pub_ = this->create_publisher<std_msgs::msg::String>("/planning_result", 10);
    start_goal_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/start_goal_viz", 10);

    // Subscriber for start pose (via RViz 2D Pose Estimate → /initialpose)
    start_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10,
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
          auto pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
          pose->header = msg->header;
          pose->pose = msg->pose.pose;
          start_ = pose;
          double yaw_deg = std::atan2(2.0*(pose->pose.orientation.w*pose->pose.orientation.z + pose->pose.orientation.x*pose->pose.orientation.y),
                                      1.0-2.0*(pose->pose.orientation.y*pose->pose.orientation.y + pose->pose.orientation.z*pose->pose.orientation.z))*180.0/M_PI;
          RCLCPP_INFO(this->get_logger(), "Start pose: (%.2f, %.2f, %.1f deg)",
                      pose->pose.position.x, pose->pose.position.y, yaw_deg);
          publishStartGoalViz();
        });

    // Subscriber for goal pose (via RViz 2D Goal Pose tool)
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
          goal_ = msg;
          RCLCPP_INFO(this->get_logger(), "Goal pose: (%.2f, %.2f)",
                      msg->pose.position.x, msg->pose.position.y);
          publishStartGoalViz();
        });

    // Subscriber for obstacle polygons (from Map Editor)
    // Uses MarkerArray where each marker represents a polygon region
    polygon_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/obstacle_polygons", 10,
        [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(polygons_mutex_);
          polygons_ = *msg;
          RCLCPP_INFO(this->get_logger(), "Received %zu obstacle polygons", msg->markers.size());
        });

    // Planning service: std_srvs::Trigger to run plan with current start/goal/polygons
    plan_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/plan_path",
        std::bind(&PlannerBridgeNode::onPlanPath, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Declare and load parameters
    declare_parameters();
    loadConfigs();

    RCLCPP_INFO(this->get_logger(), "Planner Bridge Node ready. "
                "Start: RViz '2D Pose Estimate' (/initialpose). "
                "Goal: RViz '2D Goal Pose' (/goal_pose). "
                "Call ~/plan_path to trigger planning.");
  }

 private:
  // ============================================================
  // Parameter declarations
  // ============================================================
  void declare_parameters() {
    this->declare_parameter("vehicle.length", 12.0);
    this->declare_parameter("vehicle.width", 2.6);
    this->declare_parameter("vehicle.wheel_base", 2.8);
    this->declare_parameter("vehicle.rear_overhang", 0.9);
    this->declare_parameter("vehicle.rear_edge_to_ego", 3.5);
    this->declare_parameter("vehicle.width_without_rearview_mirror", 2.6);

    this->declare_parameter("search.step_size", 0.3);
    this->declare_parameter("search.steering_angle_discrete_num", 1);
    this->declare_parameter("search.steering_radian", 0.4);
    this->declare_parameter("search.search_time_limit", 10.0);
    this->declare_parameter("search.shot_distance", 8.0);
    this->declare_parameter("search.angular_upper", 300);
    this->declare_parameter("search.map_grid_resolution", 1.0);
    this->declare_parameter("search.min_path_length_limit", 1.0);
    this->declare_parameter("search.steering_penalty", 1.0);
    this->declare_parameter("search.changing_gear_penalty", 10.0);
    this->declare_parameter("search.width_collision_buff", 0.1);
    this->declare_parameter("search.length_collision_buff", 0.1);

    this->declare_parameter("map.resolution", 0.1);
    this->declare_parameter("map.size_x", 55.0);
    this->declare_parameter("map.size_y", 55.0);
    this->declare_parameter("map.origin_x", 5.0);
    this->declare_parameter("map.origin_y", 50.0);

    RCLCPP_INFO(this->get_logger(), "Parameters declared (use --ros-args -p <param>:=<value> to override)");
  }

  // ============================================================
  // Load configs from ROS2 params into ConfigManager
  // ============================================================
  void loadConfigs() {
    VehicleConfig vc;
    vc.mutable_vehicle_param()->set_length(this->get_parameter("vehicle.length").as_double());
    vc.mutable_vehicle_param()->set_width(this->get_parameter("vehicle.width").as_double());
    vc.mutable_vehicle_param()->set_wheel_base(this->get_parameter("vehicle.wheel_base").as_double());
    vc.mutable_vehicle_param()->set_rear_overhang(this->get_parameter("vehicle.rear_overhang").as_double());
    vc.mutable_vehicle_param()->set_rear_edge_to_ego(this->get_parameter("vehicle.rear_edge_to_ego").as_double());
    vc.mutable_vehicle_param()->set_width_without_rearview_mirror(
        this->get_parameter("vehicle.width_without_rearview_mirror").as_double());
    Singleton<ConfigManager>::get_instance()->setConfig("VehicleConfig", vc);
    vehicle_config_ = vc;
    RCLCPP_INFO(this->get_logger(), "Vehicle: %.1fx%.1fm, wheelbase=%.1fm",
                vc.vehicle_param().length(), vc.vehicle_param().width(),
                vc.vehicle_param().wheel_base());

    OpenspaceSearchConfig sc;
    ArcModelConfig amc;
    amc.move_step_ = this->get_parameter("search.step_size").as_double();
    amc.steering_angle_discrete_num_ = this->get_parameter("search.steering_angle_discrete_num").as_int();
    amc.steering_radian_ = this->get_parameter("search.steering_radian").as_double();
    amc.search_time_limit_ = this->get_parameter("search.search_time_limit").as_double();
    amc.shot_distance_ = this->get_parameter("search.shot_distance").as_double();
    amc.angular_upper_ = this->get_parameter("search.angular_upper").as_int();
    amc.map_grid_resolution_ = this->get_parameter("search.map_grid_resolution").as_double();
    amc.min_path_length_limit_ = this->get_parameter("search.min_path_length_limit").as_double();
    amc.steering_penalty_ = this->get_parameter("search.steering_penalty").as_double();
    amc.changing_gear_penalty_ = this->get_parameter("search.changing_gear_penalty").as_double();
    amc.width_collision_buff_ = this->get_parameter("search.width_collision_buff").as_double();
    amc.length_collision_buff_ = this->get_parameter("search.length_collision_buff").as_double();
    amc.move_nums_ = 5;
    sc.arc_model_configs_["regular"] = amc;
    Singleton<ConfigManager>::get_instance()->setConfig("OpenspaceSearch", sc);
    RCLCPP_INFO(this->get_logger(), "Search: step=%.2fm, steer=%.2frad, grid=%.2fm, timeout=%.1fs",
                amc.move_step_, amc.steering_radian_, amc.map_grid_resolution_,
                amc.search_time_limit_);

    freespace_builder_.init(
        this->get_parameter("map.resolution").as_double(),
        this->get_parameter("map.size_x").as_double(),
        this->get_parameter("map.size_y").as_double(),
        this->get_parameter("map.origin_x").as_double(),
        this->get_parameter("map.origin_y").as_double());
    RCLCPP_INFO(this->get_logger(), "Map: %dx%d @ %.2fm resolution",
                static_cast<int>(this->get_parameter("map.size_x").as_double()),
                static_cast<int>(this->get_parameter("map.size_y").as_double()),
                this->get_parameter("map.resolution").as_double());
  }

  // ============================================================
  // Service callback: run planning with current start/goal/polygons
  // ============================================================
  void onPlanPath(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (!start_ || !goal_) {
      response->success = false;
      response->message = "Start or goal pose not set. Use RViz 2D Pose Estimate and 2D Goal Pose tools.";
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Planning: (%.2f,%.2f) -> (%.2f,%.2f)",
                start_->pose.position.x, start_->pose.position.y,
                goal_->pose.position.x, goal_->pose.position.y);

    // Build freespace from current polygons
    auto freespace = buildFreespace();

    // Convert quaternion to yaw
    auto quatToYaw = [](const geometry_msgs::msg::Quaternion& q) -> double {
      return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    };

    // Setup search data
    auto search_data = std::make_shared<OpenspaceSearchData>();
    search_data->freespace_ptr_ = freespace;
    search_data->start_pose_ = PathPt(start_->pose.position.x,
                                       start_->pose.position.y,
                                       quatToYaw(start_->pose.orientation));
    search_data->end_pose_ = PathPt(goal_->pose.position.x,
                                     goal_->pose.position.y,
                                     quatToYaw(goal_->pose.orientation));
    search_data->search_config_name_ = "regular";
    search_data->scenario_tags_ = {};

    // ROI boundary
    double margin = 20.0;
    double cx = (start_->pose.position.x + goal_->pose.position.x) / 2.0;
    double cy = (start_->pose.position.y + goal_->pose.position.y) / 2.0;
    double half_range = std::max(std::abs(start_->pose.position.x - goal_->pose.position.x),
                                  std::abs(start_->pose.position.y - goal_->pose.position.y)) / 2.0 + margin;
    search_data->roi_.roi_xy_boundary = {
        static_cast<float>(cx - half_range), static_cast<float>(cx + half_range),
        static_cast<float>(cy - half_range), static_cast<float>(cy + half_range)};

    // Validate start/goal poses are collision-free
    auto veh = vehicle_config_.vehicle_param();
    math::Box2d start_box(math::Vec2d(start_->pose.position.x, start_->pose.position.y),
                          quatToYaw(start_->pose.orientation), veh.length(), veh.width());
    math::Box2d goal_box(math::Vec2d(goal_->pose.position.x, goal_->pose.position.y),
                         quatToYaw(goal_->pose.orientation), veh.length(), veh.width());

    auto checkPose = [&](const math::Box2d& box, const char* name) -> std::string {
      if (freespace->isOutOfMap(box.center().x(), box.center().y())) {
        return std::string(name) + " is out of map bounds";
      }
      math::Vec2d hit;
      if (freespace->isOccupied(box, &hit)) {
        return std::string(name) + " is in collision with obstacle";
      }
      return "";
    };

    std::string err = checkPose(start_box, "Start");
    if (!err.empty()) {
      response->success = false;
      response->message = err;
      RCLCPP_WARN(this->get_logger(), "%s", err.c_str());
      return;
    }
    err = checkPose(goal_box, "Goal");
    if (!err.empty()) {
      response->success = false;
      response->message = err;
      RCLCPP_WARN(this->get_logger(), "%s", err.c_str());
      return;
    }

    // Run search (A* runs in background thread, WAITING means still in progress)
    PathProviderBaseHAStar<ArcModel, ArcPathNodeKeyHash> searcher;
    std::any data = search_data;
    auto status = searcher.run(data);
    while (status == BaseOpenspacePathPlanner::WAITING) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      status = searcher.run(data);
    }

    // Build response
    if (status == BaseOpenspacePathPlanner::FINISH) {
      const auto& path = search_data->search_path_;
      response->success = true;
      response->message = "Path found: " + std::to_string(path.size()) + " points";

      // Print trajectory points
      RCLCPP_INFO(this->get_logger(), "=== Trajectory (%zu points) ===", path.size());
      for (size_t i = 0; i < path.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "  [%zu] (%.2f, %.2f, %.1f deg)",
                    i, path[i].x(), path[i].y(), path[i].theta() * 180.0 / M_PI);
      }

      publishPath(path);
      publishVehicleBoxes(path);

      RCLCPP_INFO(this->get_logger(), "Planning succeeded: %zu points", path.size());
    } else {
      const char* status_str = (status == BaseOpenspacePathPlanner::FAILED) ? "SEARCH_FAILED" : "UNKNOWN";
      response->success = false;
      response->message = std::string("Planning failed: ") + status_str;

      RCLCPP_WARN(this->get_logger(), "Planning failed: %s (status=%d)",
                  status_str, static_cast<int>(status));
    }

    auto result_msg = std_msgs::msg::String();
    result_msg.data = response->success ? "SUCCESS" : "FAILED";
    result_pub_->publish(result_msg);
  }

  // ============================================================
  // Build Freespace from received MarkerArray polygons
  // ============================================================
  std::shared_ptr<Freespace> buildFreespace() {
    std::lock_guard<std::mutex> lock(polygons_mutex_);

    freespace_builder_.clear();
    freespace_builder_.init(
        this->get_parameter("map.resolution").as_double(),
        this->get_parameter("map.size_x").as_double(),
        this->get_parameter("map.size_y").as_double(),
        this->get_parameter("map.origin_x").as_double(),
        this->get_parameter("map.origin_y").as_double());

    for (const auto& marker : polygons_.markers) {
      if (marker.points.empty()) continue;
      openspace_ros2_bridge::PolygonRegion region;
      for (const auto& pt : marker.points) {
        region.vertices.emplace_back(pt.x, pt.y);
      }
      // red marker = obstacle, green = free space
      region.is_occupied = (marker.color.r > 0.5);
      freespace_builder_.addPolygon(region);
    }

    RCLCPP_DEBUG(this->get_logger(), "Building freespace with %zu regions", polygons_.markers.size());
    return freespace_builder_.build();
  }

  // ============================================================
  // Publish helpers
  // ============================================================
  void publishPath(const std::vector<PathPt>& path) {
    nav_msgs::msg::Path msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    for (const auto& pt : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position.x = pt.x();
      pose.pose.position.y = pt.y();
      pose.pose.position.z = 0;
      double theta = pt.theta();
      pose.pose.orientation.z = std::sin(theta / 2.0);
      pose.pose.orientation.w = std::cos(theta / 2.0);
      msg.poses.push_back(pose);
    }
    path_pub_->publish(msg);
  }

  void publishVehicleBoxes(const std::vector<PathPt>& path) {
    visualization_msgs::msg::MarkerArray arr;
    double length = vehicle_config_.vehicle_param().length();
    double width = vehicle_config_.vehicle_param().width();

    for (size_t i = 0; i < path.size(); ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = this->now();
      marker.header.frame_id = "map";
      marker.ns = "vehicle_box";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = path[i].x();
      marker.pose.position.y = path[i].y();
      marker.pose.position.z = 0.15;
      double theta = path[i].theta();
      marker.pose.orientation.z = std::sin(theta / 2.0);
      marker.pose.orientation.w = std::cos(theta / 2.0);
      marker.scale.x = length;
      marker.scale.y = width;
      marker.scale.z = 0.3;
      marker.color.r = 0.0;
      marker.color.g = 0.8;
      marker.color.b = 0.0;
      marker.color.a = 0.5;
      arr.markers.push_back(marker);
    }
    box_pub_->publish(arr);
  }

  // ============================================================
  // Publish start/goal visualization markers
  // ============================================================
  void publishStartGoalViz() {
    visualization_msgs::msg::MarkerArray arr;
    int id = 0;

    auto makeArrow = [&](const geometry_msgs::msg::Pose& pose, float r, float g, float b) {
      visualization_msgs::msg::Marker m;
      m.header.stamp = this->now();
      m.header.frame_id = "map";
      m.ns = "start_goal";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose = pose;
      m.scale.x = 0.8;   // shaft length
      m.scale.y = 0.15;  // shaft diameter
      m.scale.z = 0.3;   // head diameter
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.color.a = 1.0;
      return m;
    };

    if (start_) {
      arr.markers.push_back(makeArrow(start_->pose, 0.2, 0.6, 1.0));  // blue
    }
    if (goal_) {
      arr.markers.push_back(makeArrow(goal_->pose, 1.0, 0.2, 0.2));  // red
    }

    start_goal_viz_pub_->publish(arr);
  }

  // ============================================================
  // Members
  // ============================================================
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr box_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr start_goal_viz_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr polygon_sub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_srv_;

  geometry_msgs::msg::PoseStamped::ConstSharedPtr start_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr goal_;

  visualization_msgs::msg::MarkerArray polygons_;
  std::mutex polygons_mutex_;

  openspace_ros2_bridge::FreespaceBuilder freespace_builder_;
  VehicleConfig vehicle_config_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
