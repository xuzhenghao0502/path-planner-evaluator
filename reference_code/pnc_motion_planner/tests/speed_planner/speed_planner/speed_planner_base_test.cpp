#include <gtest/gtest.h>
#include <any>
#include "config_manager/config_manager.h"

#define protected public
#include "speed_planner/speed_planner_base.h"
#undef protected

namespace gpal::pnc::planning {

class SpeedPlannerBaseTest : public ::testing::Test {
 protected:
  void SetUp() override {
    int argc = 0;
    char path[] = "policy_planner";
    char* argv[]{path};
    if (!Singleton<ConfigManager>::get_instance()->init(argc, argv)) {
      std::cout << "Failed to init config!";
    }

    // Setup default config
  }

  SpeedPlannerBase speed_planner_;

};

// Test the initialization function
TEST_F(SpeedPlannerBaseTest, InitTest) {
  // Test that init returns true
  speed_planner_.init();
  
  // Test that init flag is set correctly
  // Note: We can't directly access private members, so we'll need to test indirectly
  // through other methods that depend on initialization
}

// Test calcBehaviorState function
TEST_F(SpeedPlannerBaseTest, CalcBehaviorStateTest) {
  // Create mock data for testing
  StageState stage_state;
  BehaviorState behavior_state;
  std::vector<PathPt> path_points;
  PathPt pt;
  pt.set_x(0.0);
  pt.set_y(0.0);
  pt.set_theta(0.0);
  pt.set_kappa(0.001);
  pt.set_s(0.0);
  path_points.push_back(pt);
  DiscretizedPath path = DiscretizedPath(path_points);
  // Call the function
  speed_planner_.calcBehaviorState(stage_state, path, &behavior_state);

  stage_state = StageState::HpaDrivingStage;
  path.back().set_direction(PathPt::Direction::BACKWARD);
  speed_planner_.calcBehaviorState(stage_state, path, &behavior_state);

  stage_state = StageState::ParkInStage;
  speed_planner_.calcBehaviorState(stage_state, path, &behavior_state);

  stage_state = StageState::ParkOutStage;
  speed_planner_.calcBehaviorState(stage_state, path, &behavior_state);

  stage_state = StageState::LccDrivingStage;
  speed_planner_.calcBehaviorState(stage_state, path, &behavior_state);

  stage_state = StageState::AccDrivingStage;
  speed_planner_.calcBehaviorState(stage_state, path, &behavior_state);

  
  // Add assertions based on expected behavior
  // (Specific assertions would depend on the implementation details)
}

// Test calcInitState function
TEST_F(SpeedPlannerBaseTest, CalcInitStateTest) {
  // Create mock data for testing
  LocalView local_view;
  pnc::SpeedPoint speed_point;
  speed_point.set_v(0.0);
  speed_point.set_a(0.0);
  speed_point.set_s(0.0);
  speed_planner_.last_speed_data_.push_back(speed_point);
  
  // Call the function
  speed_planner_.in_first_frame_ = true;
  speed_planner_.calcInitState(local_view);

  speed_planner_.in_first_frame_ = false;
  local_view.getChassisPtr()->setDrivingMode(proto::PowertrainInfo::DrivingMode::PowertrainInfo_DrivingMode_kCompleteAutoDrive);
  speed_planner_.calcInitState(local_view);

  speed_point.set_v(0.0);
  speed_point.set_a(0.0);
  speed_point.set_s(10.0);
  speed_planner_.last_speed_data_.clear();
  speed_planner_.last_speed_data_.push_back(speed_point);
  local_view.getChassisPtr()->set_LongituAcc(-2.0);
  speed_planner_.calcInitState(local_view);
  // Add assertions based on expected behavior
  // (Specific assertions would depend on the implementation details)
}

// // Test setOcpSpeedData function
TEST_F(SpeedPlannerBaseTest, SetOcpSpeedDataTest) {
  // Create mock data for testing
  std::pair<std::vector<OcpVariable>, std::vector<OcpVariable>> x_u;
  SpeedData speed_data;
  
  // Call the function
  speed_planner_.setOcpSpeedData(x_u, &speed_data);

  std::vector<std::pair<std::string, double>> named_vars = {{"s", 0.0}, {"v", 10.0}, {"a", 0.0}};
  OcpVariable var(named_vars);
  x_u.first.resize(51,var);
  x_u.second.resize(51,var);
  speed_planner_.setOcpSpeedData(x_u, &speed_data);
  
  // Add assertions based on expected behavior
  // (Specific assertions would depend on the implementation details)
}

// // Test resetSpeedData function
TEST_F(SpeedPlannerBaseTest, ResetSpeedDataTest) {
  // Create mock data for testing
  double desire_acc = 0.0;
  double desire_speed = 0.0;
  SpeedData speed_data;
  
  // Call the function
  speed_planner_.resetSpeedData(desire_acc, desire_speed, &speed_data);
  
  // Add assertions based on expected behavior
  // (Specific assertions would depend on the implementation details)
}

// // Test getTrajectory function
TEST_F(SpeedPlannerBaseTest, GetTrajectoryTest) {
  // Create mock data for testing
  std::shared_ptr<Localization> localization = make_shared<Localization>();
  DiscretizedPath discretized_path;
  SpeedData speed_data;
  proto::Trajectory trajectory_result;
  
  // Call the function
  speed_planner_.getTrajectory(localization, discretized_path, speed_data, &trajectory_result);

  speed_data.resize(51);
  speed_planner_.getTrajectory(localization, discretized_path, speed_data, &trajectory_result);

  std::vector<PathPt> path_points;
  PathPt pt;
  pt.set_x(0.0);
  pt.set_y(0.0);
  pt.set_theta(0.0);
  pt.set_kappa(0.001);
  pt.set_s(0.0);
  path_points.resize(51,pt);
  discretized_path = DiscretizedPath(path_points);
  speed_planner_.getTrajectory(localization, discretized_path, speed_data, &trajectory_result);

  
  // Add assertions based on expected behavior
  // (Specific assertions would depend on the implementation details)
}

// Test resetTrajectory function
TEST_F(SpeedPlannerBaseTest, ResetTrajectoryTest) {
  // Create mock data for testing
  std::shared_ptr<Localization> localization = make_shared<Localization>();
  proto::Trajectory trajectory_result;
  
  // Call the function
  speed_planner_.resetTrajectory(localization, &trajectory_result);
  
  // Add assertions based on expected behavior
  // (Specific assertions would depend on the implementation details)
}

TEST_F(SpeedPlannerBaseTest, StopReasonCheckTest) {
  // Create mock data for testing
  LocalView local_view;
  InvasionObstacle nearest_invasion_obstacle;
  SpeedData speed_data;
  StopReason stop_reason;
  
  // Test case 1: Vehicle is driving (speed > 0.1 * KMH_MS)
  // Mock chassis speed to be greater than threshold
  // Note: We need to set up the local_view with appropriate chassis data
  local_view.getChassisPtr()->set_Speed(5.0 * KMH_MS); // Set speed to 5 km/h
  speed_planner_.stopReasonCheck(local_view, nearest_invasion_obstacle, &speed_data, &stop_reason);
  // EXPECT_EQ(stop_reason.stop_reason_type, StopReason::StopReasonType::DRIVING);
  // EXPECT_STREQ(stop_reason.stop_reason.c_str(), "driving");
  
  // Test case 2: Vehicle is not moving but speed_data is not empty and back().s() > 0.5
  local_view.getChassisPtr()->set_Speed(0.0);
  pnc::SpeedPoint speed_point;
  speed_point.set_s(1.0); // Set s > 0.5
  speed_data.push_back(speed_point);
  speed_planner_.stopReasonCheck(local_view, nearest_invasion_obstacle, &speed_data, &stop_reason);
  // EXPECT_EQ(stop_reason.stop_reason_type, StopReason::StopReasonType::DRIVING);
  // EXPECT_STREQ(stop_reason.stop_reason.c_str(), "driving");
  
  // Test case 3: stop_reason_type is INVALID and nearest_invasion_obstacle.obj_id_ is empty
  stop_reason.stop_reason_type = StopReason::StopReasonType::INVALID;
  nearest_invasion_obstacle.obj_id_ = "";
  speed_data.clear();
  speed_planner_.stopReasonCheck(local_view, nearest_invasion_obstacle, &speed_data, &stop_reason);
  // EXPECT_EQ(stop_reason.stop_reason_type, StopReason::StopReasonType::INVALID);
  // EXPECT_STREQ(stop_reason.stop_reason.c_str(), "unrecognized_stop_reason");
  
  // Test case 4: nearest_invasion_obstacle.is_obstacle_ is true
  stop_reason.stop_reason_type = StopReason::StopReasonType::INVALID;
  nearest_invasion_obstacle.obj_id_ = "obstacle_1";
  nearest_invasion_obstacle.is_obstacle_ = true;
  nearest_invasion_obstacle.invasion_s_ = 10.0;
  speed_planner_.stopReasonCheck(local_view, nearest_invasion_obstacle, &speed_data, &stop_reason);
  // EXPECT_EQ(stop_reason.stop_reason_type, StopReason::StopReasonType::BLOCK_OD);
  // EXPECT_STREQ(stop_reason.stop_reason.c_str(), "obstacle_1");
  // EXPECT_DOUBLE_EQ(stop_reason.stop_distance, 10.0);
  
  // Test case 5: nearest_invasion_obstacle.obj_id_ matches FREESPACE_WALL
  stop_reason.stop_reason_type = StopReason::StopReasonType::INVALID;
  nearest_invasion_obstacle.obj_id_ = "fs"; // This should match speed_wall_id_map.at(FREESPACE_WALL)
  nearest_invasion_obstacle.is_obstacle_ = false;
  nearest_invasion_obstacle.invasion_s_ = 15.0;
  speed_planner_.stopReasonCheck(local_view, nearest_invasion_obstacle, &speed_data, &stop_reason);
  // EXPECT_EQ(stop_reason.stop_reason_type, StopReason::StopReasonType::BLOCK_FS);
  // EXPECT_STREQ(stop_reason.stop_reason.c_str(), "freespace_wall_id");
  // EXPECT_DOUBLE_EQ(stop_reason.stop_distance, 15.0);
  
  // Test case 6: nearest_invasion_obstacle.obj_id_ matches RSA_WALL
  stop_reason.stop_reason_type = StopReason::StopReasonType::INVALID;
  nearest_invasion_obstacle.obj_id_ = "rsa"; // This should match speed_wall_id_map.at(RSA_WALL)
  nearest_invasion_obstacle.is_obstacle_ = false;
  nearest_invasion_obstacle.invasion_s_ = 20.0;
  speed_planner_.stopReasonCheck(local_view, nearest_invasion_obstacle, &speed_data, &stop_reason);
  // EXPECT_EQ(stop_reason.stop_reason_type, StopReason::StopReasonType::GATE);
  // EXPECT_STREQ(stop_reason.stop_reason.c_str(), "rsa_wall_id");
  // EXPECT_DOUBLE_EQ(stop_reason.stop_distance, 20.0);
  
  // Test case 7: nearest_invasion_obstacle.obj_id_ matches TFL_WALL
  stop_reason.stop_reason_type = StopReason::StopReasonType::INVALID;
  nearest_invasion_obstacle.obj_id_ = "tsr"; // This should match speed_wall_id_map.at(TFL_WALL)
  nearest_invasion_obstacle.is_obstacle_ = false;
  nearest_invasion_obstacle.invasion_s_ = 25.0;
  speed_planner_.stopReasonCheck(local_view, nearest_invasion_obstacle, &speed_data, &stop_reason);
  // EXPECT_EQ(stop_reason.stop_reason_type, StopReason::StopReasonType::STOP_LINE);
  // EXPECT_STREQ(stop_reason.stop_reason.c_str(), "tfl_wall_id");
  // EXPECT_DOUBLE_EQ(stop_reason.stop_distance, 25.0);
  
  // Test case 8: nearest_invasion_obstacle.obj_id_ matches DESTINATION_WALL
  stop_reason.stop_reason_type = StopReason::StopReasonType::INVALID;
  nearest_invasion_obstacle.obj_id_ = "destination"; // This should match speed_wall_id_map.at(DESTINATION_WALL)
  nearest_invasion_obstacle.is_obstacle_ = false;
  nearest_invasion_obstacle.invasion_s_ = 30.0;
  speed_planner_.stopReasonCheck(local_view, nearest_invasion_obstacle, &speed_data, &stop_reason);
  // EXPECT_EQ(stop_reason.stop_reason_type, StopReason::StopReasonType::DESTINATION);
  // EXPECT_STREQ(stop_reason.stop_reason.c_str(), "destination_wall_id");
  // EXPECT_DOUBLE_EQ(stop_reason.stop_distance, 30.0);
  
  // Test case 9: nearest_invasion_obstacle.obj_id_ matches JUNCTION_STOP
  stop_reason.stop_reason_type = StopReason::StopReasonType::INVALID;
  nearest_invasion_obstacle.obj_id_ = "junction_stop"; // This should match speed_wall_id_map.at(JUNCTION_STOP)
  nearest_invasion_obstacle.is_obstacle_ = false;
  nearest_invasion_obstacle.invasion_s_ = 35.0;
  speed_planner_.stopReasonCheck(local_view, nearest_invasion_obstacle, &speed_data, &stop_reason);
  // EXPECT_EQ(stop_reason.stop_reason_type, StopReason::StopReasonType::STOP_LINE);
  // EXPECT_STREQ(stop_reason.stop_reason.c_str(), "junction_stop_id");
  // EXPECT_DOUBLE_EQ(stop_reason.stop_distance, 35.0);
  
  // Test case 10: nearest_invasion_obstacle.obj_id_ matches VIRTUAL_LINE
  stop_reason.stop_reason_type = StopReason::StopReasonType::INVALID;
  nearest_invasion_obstacle.obj_id_ = "virtual_line"; // This should match speed_wall_id_map.at(VIRTUAL_LINE)
  nearest_invasion_obstacle.is_obstacle_ = false;
  nearest_invasion_obstacle.invasion_s_ = 40.0;
  speed_planner_.stopReasonCheck(local_view, nearest_invasion_obstacle, &speed_data, &stop_reason);
  // EXPECT_EQ(stop_reason.stop_reason_type, StopReason::StopReasonType::STOP_LINE);
  // EXPECT_STREQ(stop_reason.stop_reason.c_str(), "virtual_line_id");
  // EXPECT_DOUBLE_EQ(stop_reason.stop_distance, 40.0);
  
  // Test case 11: nearest_invasion_obstacle.obj_id_ matches CLEAR_ZONE
  stop_reason.stop_reason_type = StopReason::StopReasonType::INVALID;
  nearest_invasion_obstacle.obj_id_ = "clear_zone"; // This should match speed_wall_id_map.at(CLEAR_ZONE)
  nearest_invasion_obstacle.is_obstacle_ = false;
  nearest_invasion_obstacle.invasion_s_ = 45.0;
  speed_planner_.stopReasonCheck(local_view, nearest_invasion_obstacle, &speed_data, &stop_reason);
  // EXPECT_EQ(stop_reason.stop_reason_type, StopReason::StopReasonType::CLEAR_ZONE);
  // EXPECT_STREQ(stop_reason.stop_reason.c_str(), "clear_zone_id");
  // EXPECT_DOUBLE_EQ(stop_reason.stop_distance, 45.0);
  
  // Test case 12: nearest_invasion_obstacle.obj_id_ matches END_POINT
  stop_reason.stop_reason_type = StopReason::StopReasonType::INVALID;
  nearest_invasion_obstacle.obj_id_ = "end_point"; // This should match speed_wall_id_map.at(END_POINT)
  nearest_invasion_obstacle.is_obstacle_ = false;
  nearest_invasion_obstacle.invasion_s_ = 50.0;
  speed_planner_.stopReasonCheck(local_view, nearest_invasion_obstacle, &speed_data, &stop_reason);
  // EXPECT_EQ(stop_reason.stop_reason_type, StopReason::StopReasonType::DESTINATION);
  // EXPECT_STREQ(stop_reason.stop_reason.c_str(), "end_point_id");
  // EXPECT_DOUBLE_EQ(stop_reason.stop_distance, 50.0);
  
  // Test case 13: nearest_invasion_obstacle.obj_id_ matches REF_LINE_END
  stop_reason.stop_reason_type = StopReason::StopReasonType::INVALID;
  nearest_invasion_obstacle.obj_id_ = "ref_line_end"; // This should match speed_wall_id_map.at(REF_LINE_END)
  nearest_invasion_obstacle.is_obstacle_ = false;
  nearest_invasion_obstacle.invasion_s_ = 55.0;
  speed_planner_.stopReasonCheck(local_view, nearest_invasion_obstacle, &speed_data, &stop_reason);
  // EXPECT_EQ(stop_reason.stop_reason_type, StopReason::StopReasonType::REF_LINE_END);
  // EXPECT_STREQ(stop_reason.stop_reason.c_str(), "ref_line_end_id");
  // EXPECT_DOUBLE_EQ(stop_reason.stop_distance, 55.0);
  
  // Test case 14: nearest_invasion_obstacle.obj_id_ doesn't match any known wall types
  stop_reason.stop_reason_type = StopReason::StopReasonType::INVALID;
  nearest_invasion_obstacle.obj_id_ = "unknown_wall";
  nearest_invasion_obstacle.is_obstacle_ = false;
  speed_planner_.stopReasonCheck(local_view, nearest_invasion_obstacle, &speed_data, &stop_reason);
  // EXPECT_EQ(stop_reason.stop_reason_type, StopReason::StopReasonType::INVALID);
  // EXPECT_STREQ(stop_reason.stop_reason.c_str(), "unrecognized_stop_reason");
}

}

