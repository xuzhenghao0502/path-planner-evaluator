#include <gtest/gtest.h>
#include <any>
#include "config_manager/config_manager.h"

#define protected public
#define private public
#include "speed_planner/ocp_qp_speed_planner.h"
#undef protected

namespace gpal::pnc::planning {

class OcpQpSpeedPlannerTest : public ::testing::Test {
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

  void addReferenceLineInfo() {
    // 设置有效的reference line和停止线
    vector<ReferencePoint> ref_points;
    for (int i = 0; i < 50; i++) {
      ref_points.emplace_back(math::Vec3d(i * 1.0, 0.0, 0.0), 0.0, 0.0, 0.0001, 0.0);
      ref_points.back().local_s_ = i * 1.0;
    }
    ReferenceLine ref_line(ref_points);
    ref_line.speed_limits_.emplace_back(0.0, 30.0, 10.0, 0.0, math::Vec3d(), math::Vec3d());

    reference_line_info_.ref_line_ = ref_line;

    reference_line_info_.vehicle_state_.set_x(0.0);
    reference_line_info_.vehicle_state_.set_y(0.0);
  }

  void addPath() {
    // 设置有效的reference line和停止线
    vector<PathPt> path_points;
    for (int i = 0; i < 50; i++) {
      PathPt pt;
      pt.set_x(i * 1.0);
      pt.set_y(0.0);
      pt.set_theta(0.0);
      pt.set_kappa(0.2);
      pt.set_s(i * 1.0);
      path_points.push_back(pt);
    }
    path_data_.discretized_path_ = DiscretizedPath(path_points);
    path_data_.local_path_ = DiscretizedPath(path_points);
  }

  void addDynamicObstacle(string id, double x, double y, LongitudinalOdTag tag, Decision::ObjectGameType game_type, double v) {
    Obstacle obs(id, math::Polygon2d(math::Box2d(math::Vec2d(x, y), 0.0, v, 2.0)));

    std::vector<std::shared_ptr<Decision::RawSinglePrediction>> raw_predictions;
    std::vector<Decision::RawSinglePrediction> raw_prediction_in;
    Decision::RawPredictionTrajectory raw_prediction_traj;
    for (int i = 0; i < 50; i++) {
      proto::TrajectoryPoint pt;
      pt.mutable_path_point()->set_x(x + i * 0.1 * 5.0);
      pt.mutable_path_point()->set_y(y);
      pt.mutable_path_point()->set_theta(0.0);
      pt.mutable_path_point()->set_kappa(0.001);
      pt.mutable_path_point()->set_s(i * 0.1 * 5.0);
      pt.set_relative_time(i * 0.1);
      raw_prediction_traj.push_back(pt);
    }
    std::shared_ptr<Decision::RawSinglePrediction> raw_prediction = std::make_shared<Decision::RawSinglePrediction>();
    raw_prediction.traj = raw_prediction_traj;
    raw_predictions.push_back(raw_prediction);

    Decision::DecisionObject decision_obj(id, 5.0, 2.0, x, y, 0.0, 5.0, false, raw_prediction_in);
    decision_obj.long_od_tag = tag;
    decision_obj.game_type = game_type;
    decision_obj.raw_predictions =
        std::make_shared<std::vector<std::shared_ptr<Decision::RawSinglePrediction>>>(raw_predictions);

    obstacle_ =
        SpeedPlannerObstacle(std::make_shared<Obstacle>(obs), std::make_shared<Decision::DecisionObject>(decision_obj));
    obstacle_set_.emplace(obstacle_.id(), std::make_shared<SpeedPlannerObstacle>(obstacle_));
  }

  std::shared_ptr<SpeedResult> speed_result_;
  LocalView local_view_;
  ReferenceLineInfo reference_line_info_;
  PathData path_data_;
  DecisionResult decision_result_;
  SpeedState init_speed_state_;
  StageState stage_state_;
  ObstacleSet obstacle_set_;
  SpeedPlannerObstacle obstacle_;

  OcpQpSpeedPlanner ocp_qp_speed_planner_;
};

// Test the runOnce function
TEST_F(OcpQpSpeedPlannerTest, RunOnceTest) {
  // Create mock data for testing
  int64_t time_stamp = 0;
  speed_result_ = make_shared<SpeedResult>();
  ocp_qp_speed_planner_.init();
  // Test case 1: Valid input data
  // Set up valid data for all input parameters
  // This would require creating mock objects for all the dependencies
  // For now, we'll just call the function with empty objects
  Status status = ocp_qp_speed_planner_.runOnce(
      local_view_, stage_state_, make_shared<ReferenceLineInfo>(reference_line_info_),
      make_shared<DecisionResult>(decision_result_), make_shared<PathData>(path_data_), time_stamp, speed_result_);
  // Add assertions based on expected behavior
  // EXPECT_TRUE(status.ok());
  addReferenceLineInfo();
  vector<PathPt> path_points;
  PathPt pt;
  pt.set_x(1.0);
  pt.set_y(0.0);
  pt.set_theta(0.0);
  pt.set_kappa(0.2);
  pt.set_s(0.0);
  path_points.push_back(pt);
  path_data_.discretized_path_ = DiscretizedPath(path_points);
  path_data_.local_path_ = DiscretizedPath(path_points);
  status = ocp_qp_speed_planner_.runOnce(
      local_view_, stage_state_, make_shared<ReferenceLineInfo>(reference_line_info_),
      make_shared<DecisionResult>(decision_result_), make_shared<PathData>(path_data_), time_stamp, speed_result_);

  addReferenceLineInfo();
  addPath();
  status = ocp_qp_speed_planner_.runOnce(
      local_view_, stage_state_, make_shared<ReferenceLineInfo>(reference_line_info_),
      make_shared<DecisionResult>(decision_result_), make_shared<PathData>(path_data_), time_stamp, speed_result_);

  ocp_qp_speed_planner_.speed_planner_config_.set_use_risk_speed_planner(false);
  status = ocp_qp_speed_planner_.runOnce(
      local_view_, stage_state_, make_shared<ReferenceLineInfo>(reference_line_info_),
      make_shared<DecisionResult>(decision_result_), make_shared<PathData>(path_data_), time_stamp, speed_result_);

}

// Test speedDataPostProcess function
TEST_F(OcpQpSpeedPlannerTest, SpeedDataPostProcessTest) {
  // Create mock data for testing
  LocalView local_view;
  ObstacleSet obstacle_map;
  InvasionObstacle nearest_invasion_obstacle;
  std::pair<std::vector<OcpVariable>, std::vector<OcpVariable>> x_u;
  SpeedData speed_data;
  StopReason stop_reason;

  // Test with empty data
  ocp_qp_speed_planner_.speedDataPostProcess(local_view, obstacle_map, nearest_invasion_obstacle, x_u, &speed_data,
                                             &stop_reason);

  // Add assertions based on expected behavior
  // (Specific assertions would depend on the implementation details)
}

// Test decideStopBasedOnNearestObstacle function
TEST_F(OcpQpSpeedPlannerTest, DecideStopBasedOnNearestObstacleTest) {
  // Create mock data for testing
  LocalView local_view;
  ObstacleSet obstacle_map;
  InvasionObstacle nearest_invasion_obstacle;
  SpeedData speed_data;

  // Test with empty data
  // Test case 1: nearest_invasion_obstacle.obj_id_ is "None"
  nearest_invasion_obstacle.obj_id_ = "None";
  ocp_qp_speed_planner_.decideStopBasedOnNearestObstacle(local_view, obstacle_map, nearest_invasion_obstacle, &speed_data);
  // Expect no change to speed_data as the obstacle is "None"
  
  // Test case 2: nearest_invasion_obstacle.is_obstacle_ is false
  nearest_invasion_obstacle.obj_id_ = "obstacle_1";
  nearest_invasion_obstacle.is_obstacle_ = false;
  nearest_invasion_obstacle.invasion_s_ = 10.0;
  ocp_qp_speed_planner_.decideStopBasedOnNearestObstacle(local_view, obstacle_map, nearest_invasion_obstacle, &speed_data);
  // Expect speed_data to be reset as the obstacle is not a real obstacle
  
  // Test case 3: nearest_invasion_obstacle is valid and static (obstacle speed <= 1.0 * KMH_MS)
  nearest_invasion_obstacle.obj_id_ = "obstacle_2";
  nearest_invasion_obstacle.is_obstacle_ = true;
  nearest_invasion_obstacle.invasion_s_ = 0.3; // Less than stop distance for default object type
  
  // Create a mock obstacle with low speed
  addDynamicObstacle("obstacle_2", 0.0, 0.0, LongitudinalOdTag::FOLLOW, ObjectGameType::MERGE_GAME, 0.0);
  // Set ego speed to less than threshold
  local_view.getChassisPtr()->set_Speed(0.3); // Less than 0.5 m/s threshold
  ocp_qp_speed_planner_.decideStopBasedOnNearestObstacle(local_view, obstacle_set_, nearest_invasion_obstacle, &speed_data);
  // Expect speed_data to be reset as the obstacle is static and within stop distance
  
  // Test case 4: nearest_invasion_obstacle is valid but not static (obstacle speed > 2.0 * KMH_MS)
  // mock_obstacle->set_speed(3.0 * KMH_MS); // Greater than 2.0 * KMH_MS
  obstacle_set_.clear();
  addDynamicObstacle("obstacle_2", 0.0, 0.0, LongitudinalOdTag::FOLLOW, ObjectGameType::MERGE_GAME, 3.0 * KMH_MS);
  ocp_qp_speed_planner_.decideStopBasedOnNearestObstacle(local_view, obstacle_set_, nearest_invasion_obstacle, &speed_data);
  // Expect speed_data not to be reset as the obstacle is not static
  
  // Test case 5: nearest_invasion_obstacle is valid and static but outside stop distance
  nearest_invasion_obstacle.invasion_s_ = 10.0; // Greater than stop distance for default object type
    obstacle_set_.clear();
  addDynamicObstacle("obstacle_2", 0.0, 0.0, LongitudinalOdTag::FOLLOW, ObjectGameType::MERGE_GAME, 0.5 * KMH_MS);
  ocp_qp_speed_planner_.decideStopBasedOnNearestObstacle(local_view, obstacle_set_, nearest_invasion_obstacle, &speed_data);
  // Expect speed_data not to be reset as the obstacle is static but outside stop distance
  
  // Test case 6: nearest_invasion_obstacle is "destination" and within stop distance
  nearest_invasion_obstacle.obj_id_ = "destination";
  nearest_invasion_obstacle.is_obstacle_ = true;
  nearest_invasion_obstacle.invasion_s_ = 0.3; // Less than 0.5 stop distance for destination
  ocp_qp_speed_planner_.decideStopBasedOnNearestObstacle(local_view, obstacle_map, nearest_invasion_obstacle, &speed_data);
  // Expect speed_data to be reset as the destination obstacle is static and within stop distance
  
  // Test case 7: nearest_invasion_obstacle is "tsr" and within stop distance
  nearest_invasion_obstacle.obj_id_ = "tsr";
  nearest_invasion_obstacle.is_obstacle_ = true;
  nearest_invasion_obstacle.invasion_s_ = 0.5; // Less than 1.0 stop distance for tsr
  ocp_qp_speed_planner_.decideStopBasedOnNearestObstacle(local_view, obstacle_map, nearest_invasion_obstacle, &speed_data);
  // Expect speed_data to be reset as the tsr obstacle is static and within stop distance
  // Add assertions based on expected behavior
  // (Specific assertions would depend on the implementation details)
}

TEST_F(OcpQpSpeedPlannerTest, DecideStopBasedOnNearestObstacleDynamicTest) {
  LocalView local_view;
  SpeedData speed_data;
  InvasionObstacle nearest_invasion_obstacle;
  nearest_invasion_obstacle.obj_id_ = "obstacle_2";
  nearest_invasion_obstacle.is_obstacle_ = true;
  nearest_invasion_obstacle.invasion_s_ = 0.3; // Less than stop distance for default object type
  addDynamicObstacle("obstacle_2", 0.0, 0.0, LongitudinalOdTag::FOLLOW, ObjectGameType::MERGE_GAME, 3.0 * KMH_MS);
  obstacle_set_["obstacle_2"]->is_static_ = false;
  obstacle_set_["obstacle_2"]->raw_obstacle_ptr_->speed_ = 10;
  // Set ego speed to less than threshold
  local_view.getChassisPtr()->set_Speed(0.3); // Less than 0.5 m/s threshold
  for(int i = 0; i < 10; i++){
    ocp_qp_speed_planner_.decideStopBasedOnNearestObstacle(local_view, obstacle_set_, nearest_invasion_obstacle, &speed_data);
  }
}


// Test determineStopDistance function
TEST_F(OcpQpSpeedPlannerTest, DetermineStopDistanceTest) {
  // Test case 1: destination
  double distance = ocp_qp_speed_planner_.determineStopDistance("destination");
  EXPECT_DOUBLE_EQ(distance, 0.5);

  // Test case 2: tsr
  distance = ocp_qp_speed_planner_.determineStopDistance("tsr");
  EXPECT_DOUBLE_EQ(distance, 1.0);

  // Test case 3: junction_stop
  distance = ocp_qp_speed_planner_.determineStopDistance("junction_stop");
  EXPECT_DOUBLE_EQ(distance, 1.0);

  // Test case 4: other object
  distance = ocp_qp_speed_planner_.determineStopDistance("other");
  EXPECT_DOUBLE_EQ(distance, 5.0);
}

// Test determineStayStaticDistanceBuffer function
TEST_F(OcpQpSpeedPlannerTest, DetermineStayStaticDistanceBufferTest) {
  // Test case 1: destination
  double buffer = ocp_qp_speed_planner_.determineStayStaticDistanceBuffer("destination");
  EXPECT_DOUBLE_EQ(buffer, 0.0);

  // Test case 2: tsr
  buffer = ocp_qp_speed_planner_.determineStayStaticDistanceBuffer("tsr");
  EXPECT_DOUBLE_EQ(buffer, 1.0);

  // Test case 3: junction_stop
  buffer = ocp_qp_speed_planner_.determineStayStaticDistanceBuffer("junction_stop");
  EXPECT_DOUBLE_EQ(buffer, 1.0);

  // Test case 4: other object
  buffer = ocp_qp_speed_planner_.determineStayStaticDistanceBuffer("other");
  EXPECT_DOUBLE_EQ(buffer, 3.0);
}

// Test getDestinationStopFlag function
TEST_F(OcpQpSpeedPlannerTest, GetDestinationStopFlagTest) {
  // Create mock data for testing
  LocalView local_view;
  shared_ptr<PathData> path_data = make_shared<PathData>();

  // Test case 1: Invalid remain distance info
  // Set up path_data with invalid remain distance info
  bool flag = ocp_qp_speed_planner_.getDestinationStopFlag(local_view, path_data);
  EXPECT_FALSE(flag);

  // Test case 2: Valid remain distance info but not meeting stop conditions
  // Set up path_data with valid remain distance info but not meeting stop conditions
  path_data->remain_dis_info_ = std::make_pair(true, 10.0);
  flag = ocp_qp_speed_planner_.getDestinationStopFlag(local_view, path_data);
  // EXPECT_FALSE(flag);

  // Test case 3: Valid remain distance info and meeting stop conditions
  // Set up path_data with valid remain distance info and meeting stop conditions
  path_data->remain_dis_info_ = std::make_pair(true, 0.0);
  flag = ocp_qp_speed_planner_.getDestinationStopFlag(local_view, path_data);
  // EXPECT_TRUE(flag);
}

}  // namespace gpal::pnc::planning