#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <any>

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
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/trajectory_path", 10);
    box_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vehicle_boxes", 10);
    result_pub_ = this->create_publisher<std_msgs::msg::String>("/planning_result", 10);

    RCLCPP_INFO(this->get_logger(), "Planner Bridge Node started");

    // Run a hardcoded test
    testPlan();
  }

 private:
  void testPlan() {
    // 1. Setup vehicle config
    VehicleConfig vc;
    vc.mutable_vehicle_param()->set_length(4.8);
    vc.mutable_vehicle_param()->set_width(1.9);
    vc.mutable_vehicle_param()->set_wheel_base(2.8);
    vc.mutable_vehicle_param()->set_rear_overhang(0.9);
    vc.mutable_vehicle_param()->set_rear_edge_to_ego(3.5);

    Singleton<ConfigManager>::get_instance()->setConfig("VehicleConfig", vc);

    // 2. Setup search config
    OpenspaceSearchConfig sc;
    ArcModelConfig amc;
    amc.move_step_ = 0.3;
    amc.move_nums_ = 5;
    amc.steering_radian_ = 0.4;
    amc.steering_angle_discrete_num_ = 1;
    amc.steering_penalty_ = 1.0;
    amc.changing_gear_penalty_ = 10.0;
    amc.shot_distance_ = 8.0;
    amc.search_time_limit_ = 10.0;
    amc.min_path_length_limit_ = 1.0;
    amc.width_collision_buff_ = 0.1;
    amc.length_collision_buff_ = 0.1;
    amc.angular_upper_ = 300;
    amc.map_grid_resolution_ = 1.0;
    sc.arc_model_configs_["regular"] = amc;
    Singleton<ConfigManager>::get_instance()->setConfig("OpenspaceSearch", sc);

    // 3. Build a simple freespace
    openspace_ros2_bridge::FreespaceBuilder builder;
    builder.init(0.1, 40.0, 40.0, 20.0, 20.0);  // 40x40m map
    auto freespace = builder.build();

    // 4. Setup search data
    auto search_data = std::make_shared<OpenspaceSearchData>();
    search_data->freespace_ptr_ = freespace;
    search_data->start_pose_ = PathPt(0, 0, 0);
    search_data->end_pose_ = PathPt(10, 0, 0);
    search_data->search_config_name_ = "regular";
    search_data->scenario_tags_ = {};

    // ROI boundary
    search_data->roi_.roi_xy_boundary = {-20, 20, -20, 20};

    // 5. Run search
    PathProviderBaseHAStar<ArcModel, ArcPathNodeKeyHash> searcher;
    std::any data = search_data;
    auto status = searcher.run(data);

    if (status == BaseOpenspacePathPlanner::FINISH) {
      RCLCPP_INFO(this->get_logger(), "Search succeeded! Path points: %zu",
                  search_data->search_path_.size());

      publishPath(search_data->search_path_);
      publishVehicleBoxes(search_data->search_path_, vc);

      auto msg = std_msgs::msg::String();
      msg.data = "SUCCESS";
      result_pub_->publish(msg);
    } else {
      RCLCPP_WARN(this->get_logger(), "Search failed! Status: %d",
                  static_cast<int>(status));
      auto msg = std_msgs::msg::String();
      msg.data = "FAILED";
      result_pub_->publish(msg);
    }
  }

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
      // theta as quaternion (yaw only)
      double theta = pt.theta();
      pose.pose.orientation.z = std::sin(theta / 2.0);
      pose.pose.orientation.w = std::cos(theta / 2.0);
      msg.poses.push_back(pose);
    }

    path_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published path with %zu points", path.size());
  }

  void publishVehicleBoxes(const std::vector<PathPt>& path, const VehicleConfig& vc) {
    visualization_msgs::msg::MarkerArray arr;
    double length = vc.vehicle_param().length();
    double width = vc.vehicle_param().width();

    for (size_t i = 0; i < path.size(); ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = this->now();
      marker.header.frame_id = "map";
      marker.ns = "vehicle_box";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = path[i].x();
      marker.pose.position.y = path[i].y();
      marker.pose.position.z = 0;
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

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr box_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
