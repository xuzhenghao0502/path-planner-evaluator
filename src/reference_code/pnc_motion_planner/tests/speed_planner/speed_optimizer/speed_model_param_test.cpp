#include <gtest/gtest.h>
#include <any>
#include "config_manager/config_manager.h"
#include "config/speed_planner/st_graph_processor.pb.h"

#define private public
#include "speed_optimizer/speed_model_param.h"


namespace gpal::pnc::planning {

class SpeedModelParamTest : public ::testing::Test {
 protected:
  void SetUp() override {
    int argc = 0;
    // char path[] = "/work/eka_program/eka/prebuilt/linux-x86_64/bin/policy_planner";
    char path[] = "policy_planner";
    char* argv[]{path};
    // constexpr std::chrono::seconds vehicle_parameters_timeout(0);
    // auto vehicle_parameters_callback =
    //     std::make_shared<std::packaged_task<proto::VehicleParameters(std::shared_ptr<proto::VehicleParameters>)>>(
    //         [](std::shared_ptr<proto::VehicleParameters> ptr_msg) { return *ptr_msg; });
    if (!Singleton<ConfigManager>::get_instance()->init(argc, argv)) {
      std::cout << "Failed to init config!";
    }
    // Setup default config
  }

  void addPathGroup() {
    vector<PathPt> path_points;
    for (int i = 0; i < 150; i++) {
      PathPt pt;
      pt.set_x(i * 1.0);
      pt.set_y(0.0);
      pt.set_theta(0.0);
      pt.set_kappa(0.001);
      pt.set_s(i * 1.0);
      path_points.push_back(pt);
    }
    auto path_ = DiscretizedPath(path_points);
    path_group_.origin_path_ = path_;
    path_group_.extand_interval_path_ = path_;
    path_group_.path_points_for_segmentation_after_extand_ = path_;
    path_group_.path_points_for_segmentation_ = path_;
  }
  void addDynamicObstacle(string id, double x, double y, LongitudinalOdTag tag, Decision::ObjectGameType game_type) {
    Obstacle obs(id, math::Polygon2d(math::Box2d(math::Vec2d(x, y), 0.0, 5.0, 2.0)));

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
    decision_obj.safe_hwt_threshold_lane_change = 1.5;

    auto obstacle_ =
        SpeedPlannerObstacle(std::make_shared<Obstacle>(obs), std::make_shared<Decision::DecisionObject>(decision_obj));
    obstacle_map_.emplace(obstacle_.id(), std::make_shared<SpeedPlannerObstacle>(obstacle_));
  }

  void addStBoundary() {
    for (int i = 0; i < time_grid_.size(); i++) {
      STDrivableBoundary st_bound;
      st_bound.t = time_grid_[i];
      speed_result->st_graph_.st_drivable_boundaries_.push_back(st_bound);
    }
  }

  void addSpeedWall(SpeedWallType type) {
    SpeedWall speed_wall;
    speed_wall.type = type;
    speed_wall.stop_distance = 2.0;
    speed_wall.st_wall = {{0.0, 10.0}, {5.0, 10.0}};
    speed_walls_.push_back(speed_wall);
    speed_result->speed_walls_.push_back(speed_wall);
  }

  void addStBoundary(string id, STBoundary::BoundaryType type) {
    std::vector<std::pair<STPoint, STPoint>> st_bound;
    std::vector<double> lateral_s;
    for (int i = 0; i < 51; i++) {
      STPoint pt1(20.0, i * 0.1);
      STPoint pt2(20.0 + 5.0, i * 0.1);
      st_bound.emplace_back(pt1, pt2);
      lateral_s.push_back(0.1);
    }
    STBoundary* st_boundary = new STBoundary(st_bound);
    st_boundary->id_ = id;
    st_boundary->setBoundaryType(type);
    st_boundary->set_lateral_signed_distances(lateral_s);
    if(obstacle_map_.find(id) != obstacle_map_.end()){
      obstacle_map_.at(id)->setPathStBoundary(*st_boundary);
      obstacle_map_.at(id)->setRiskStBoundary(*st_boundary);
    }
    speed_result->st_graph_.st_boundaries_.push_back(st_boundary);
  }

  void addSpeedLimitResult() {
    std::vector<proto::TrajectoryPoint> path_v_t;
    for (int i = 0; i < 51; i++) {
      proto::TrajectoryPoint pt;
      pt.set_relative_time(i * 0.1);
      pt.set_v(10.0);
      path_v_t.push_back(pt);
    }
    speed_result->speed_limit_result_.path_v_t_ = path_v_t;
  }

  const std::vector<double> time_grid_ = {0.0, 0.1, 0.2, 0.4, 0.6, 1.0, 1.4, 1.8,
                                          2.2, 2.6, 3.0, 3.4, 3.8, 4.2, 4.6, 5.0};
  const double time_resolution_{0.1};
  const double time_horizon_{5.0};
  SpeedModelParam processor_ = SpeedModelParam(time_grid_, time_resolution_, time_horizon_);
  SpeedState speed_init_state_;
  LocalView local_view_;
  DecisionResult decision_result_;
  ObstacleSet obstacle_map_;
  PathGroup path_group_;
  std::shared_ptr<SpeedResult> speed_result = std::make_shared<SpeedResult>();

  std::vector<SpeedWall> speed_walls_;
};

TEST_F(SpeedModelParamTest, calculateSpeedModelParamNormal) {
  addPathGroup();
  addStBoundary();
  addSpeedLimitResult();
  processor_.calculateSpeedModelParam(local_view_, decision_result_, obstacle_map_, path_group_, speed_init_state_,
                                      speed_result);
}
TEST_F(SpeedModelParamTest, calculateSpeedModelParamNormal2) {
  addPathGroup();
  addStBoundary();
  addSpeedLimitResult();
  processor_.speed_ocp_qp_optimizer_config_.set_enable_merge_special_case(false);
  processor_.speed_planner_config_.set_enable_local_path_od_check(true);
  // init函数导致配置文件加载失效
  processor_.calculateSpeedModelParam(local_view_, decision_result_, obstacle_map_, path_group_, speed_init_state_,
                                      speed_result);
}

TEST_F(SpeedModelParamTest, calculateSpeedModelParamNormal3) {
  addPathGroup();
  addStBoundary();
  addSpeedLimitResult();
  addDynamicObstacle("1001", 20.0, 0, LongitudinalOdTag::FOLLOW, Decision::ObjectGameType::MERGE_GAME);
  addDynamicObstacle("1002", 20.0, 0, LongitudinalOdTag::YIELD, Decision::ObjectGameType::MERGE_GAME);
  addDynamicObstacle("1003", 20.0, 0, LongitudinalOdTag::OVERTAKE, Decision::ObjectGameType::MERGE_GAME);
  addSpeedWall(SpeedWallType::TFL_WALL);
  addStBoundary("1001", STBoundary::BoundaryType::FOLLOW);
  addStBoundary("1002", STBoundary::BoundaryType::YIELD);
  addStBoundary("1003", STBoundary::BoundaryType::OVERTAKE);
  processor_.calculateSpeedModelParam(local_view_, decision_result_, obstacle_map_, path_group_, speed_init_state_,
                                      speed_result);
}
TEST_F(SpeedModelParamTest, calculateSpeedModelParamNormal4) {
  addPathGroup();
  addStBoundary();
  addSpeedLimitResult();
  addDynamicObstacle("1001", 20.0, 0, LongitudinalOdTag::FOLLOW, Decision::ObjectGameType::MERGE_GAME);
  addDynamicObstacle("1002", 20.0, 0, LongitudinalOdTag::YIELD, Decision::ObjectGameType::MERGE_GAME);
  addDynamicObstacle("1003", 20.0, 0, LongitudinalOdTag::OVERTAKE, Decision::ObjectGameType::MERGE_GAME);
  addSpeedWall(SpeedWallType::TFL_WALL);
  addStBoundary("1001", STBoundary::BoundaryType::FOLLOW);
  addStBoundary("1002", STBoundary::BoundaryType::YIELD);
  addStBoundary("1003", STBoundary::BoundaryType::OVERTAKE);
  local_view_.chassis_->set_Speed(5.0);

  obstacle_map_.find("1001")->second->follow_params_.follow_params_valid_ = true;
  speed_result->st_graph_.st_drivable_boundaries_.at(0).has_front_yield_obj = true;
  speed_result->st_graph_.st_drivable_boundaries_.at(0).upper_obj_id = "1001";
  speed_result->st_graph_.st_drivable_boundaries_.at(1).upper_obj_id = "tsr";
  speed_result->st_graph_.st_drivable_boundaries_.at(2).upper_obj_id = "junction_stop";
  speed_result->st_graph_.st_drivable_boundaries_.at(3).upper_obj_id = "1009";

  processor_.calculateSpeedModelParam(local_view_, decision_result_, obstacle_map_, path_group_, speed_init_state_,
                                      speed_result);
}

TEST_F(SpeedModelParamTest, investigateNearestObjNormal) {
  addDynamicObstacle("1001", 20.0, 0, LongitudinalOdTag::FOLLOW, Decision::ObjectGameType::MERGE_GAME);
  addDynamicObstacle("1002", 20.0, 0, LongitudinalOdTag::YIELD, Decision::ObjectGameType::MERGE_GAME);
  addDynamicObstacle("1003", 20.0, 0, LongitudinalOdTag::OVERTAKE, Decision::ObjectGameType::MERGE_GAME);
  addSpeedWall(SpeedWallType::TFL_WALL);
  addStBoundary("1001", STBoundary::BoundaryType::FOLLOW);
  addStBoundary("1002", STBoundary::BoundaryType::YIELD);
  addStBoundary("1003", STBoundary::BoundaryType::OVERTAKE);
  InvasionObstacle obs;
  processor_.investigateNearestObj(obstacle_map_, speed_result->st_graph_, speed_walls_, &obs);
}

TEST_F(SpeedModelParamTest, calcEmpiricalAccelRange) {
  double min_a = 0, max_a = 0;
  processor_.has_front_yield_obj_ = true;
  processor_.calcEmpiricalAccelRange(min_a, max_a);
  processor_.has_front_yield_obj_ = false;
  processor_.calcEmpiricalAccelRange(min_a, max_a);
}

TEST_F(SpeedModelParamTest, calcCompleteSSoftUpperBoundsNormal) {
  addDynamicObstacle("1001", 20.0, 0, LongitudinalOdTag::FOLLOW, Decision::ObjectGameType::MERGE_GAME);
  addDynamicObstacle("1002", 20.0, 0, LongitudinalOdTag::YIELD, Decision::ObjectGameType::MERGE_GAME);
  addDynamicObstacle("1003", 20.0, 0, LongitudinalOdTag::OVERTAKE, Decision::ObjectGameType::MERGE_GAME);
  addSpeedWall(SpeedWallType::TFL_WALL);
  addStBoundary("1001", STBoundary::BoundaryType::FOLLOW);
  addStBoundary("1002", STBoundary::BoundaryType::YIELD);
  addStBoundary("1003", STBoundary::BoundaryType::OVERTAKE);
  processor_.calcCompleteSSoftUpperBounds(speed_result->st_graph_, speed_walls_);
}

TEST_F(SpeedModelParamTest, updateDecisionCoarseTrajNormal) {
  processor_.init();
  std::vector<Decision::RefTrajPoint> traj;
  for (int i = 0; i < 30; i++) {
    Decision::RefTrajPoint pt;
    pt.t = i * 0.1;
    pt.x = i;
    pt.y = 0.0;
    pt.v = 10.0;
    pt.a = 0.0;
    traj.emplace_back(pt);
  }
  decision_result_.ref_traj_info_->traj_points = traj;
  processor_.updateDecisionCoarseTraj(decision_result_);

  traj.clear();
  for (int i = 10; i < 30; i++) {
    Decision::RefTrajPoint pt;
    pt.t = i * 0.1;
    pt.x = i;
    pt.y = 0.0;
    pt.v = 10.0;
    pt.a = 0.0;
    traj.emplace_back(pt);
  }
  decision_result_.ref_traj_info_->traj_points = traj;
  processor_.updateDecisionCoarseTraj(decision_result_);

  traj.clear();
  Decision::RefTrajPoint pt;
  pt.t = 0.1;
  pt.x = 1;
  pt.y = 0.0;
  pt.v = 10.0;
  pt.a = 0.0;
  traj.emplace_back(pt);
  decision_result_.ref_traj_info_->traj_points = traj;
  processor_.updateDecisionCoarseTraj(decision_result_);
}

TEST_F(SpeedModelParamTest,specialCaseOfVehicleStartNormal){
  double w = 0.0;
  processor_.specialCaseOfVehicleStart(w);

  processor_.nearest_front_obj_invasion_v_ = 30.0;
  processor_.specialCaseOfVehicleStart(w);

}

TEST_F(SpeedModelParamTest, setContraintParamFail) {
  processor_.time_grid_.emplace_back(0.5);
  processor_.setContraintParam(local_view_, speed_result->st_graph_, 30.0);
}

TEST_F(SpeedModelParamTest, specialCaseForLaneChangeNoraml) {
  processor_.init();

  decision_result_.curr_fsm_state_ = FsmState::LEFT_CHANGE;
  addDynamicObstacle("1001", 20.0, 0, LongitudinalOdTag::FOLLOW, Decision::ObjectGameType::MERGE_GAME);
  addDynamicObstacle("1002", 20.0, 0, LongitudinalOdTag::YIELD, Decision::ObjectGameType::MERGE_GAME);
  obstacle_map_.at("1002")->obstacle_decision_hwt_ = 0.1;
  addStBoundary();
  speed_result->st_graph_.st_drivable_boundaries_.at(0).has_front_yield_obj = true;
  speed_result->st_graph_.st_drivable_boundaries_.at(0).upper_obj_id = "1001";
  speed_result->st_graph_.st_drivable_boundaries_.at(1).has_front_yield_obj = true;
  speed_result->st_graph_.st_drivable_boundaries_.at(1).upper_obj_id = "1003";
  speed_result->st_graph_.st_drivable_boundaries_.at(2).has_front_yield_obj = true;
  speed_result->st_graph_.st_drivable_boundaries_.at(2).upper_obj_id = "1002";
  processor_.specialCaseForLaneChange(decision_result_, obstacle_map_, speed_result->st_graph_);

  for(int i = 0; i <= 18; i++){
    decision_result_.curr_fsm_state_ = static_cast<FsmState>(i);
    processor_.specialCaseForLaneChange(decision_result_, obstacle_map_, speed_result->st_graph_);
  }
}

TEST_F(SpeedModelParamTest, specialCaseForLaneChange2) {
  processor_.init();
  addDynamicObstacle("1001", 20.0, 0, LongitudinalOdTag::FOLLOW, Decision::ObjectGameType::MERGE_GAME);
  addDynamicObstacle("1003", 20.0, 0, LongitudinalOdTag::OVERTAKE, Decision::ObjectGameType::MERGE_GAME);
  addStBoundary();
  for(int index = 0; index < speed_result->st_graph_.st_drivable_boundaries().size(); index++){
    speed_result->st_graph_.st_drivable_boundaries_.at(index).has_front_yield_obj = true;
    speed_result->st_graph_.st_drivable_boundaries_.at(index).upper_obj_id = "1001";
  }
  size_t complete_size = speed_result->st_graph_.st_drivable_boundaries().size();
  processor_.history_obj_hwt_.resize(complete_size, make_pair("1001", 1.0));
  processor_.specialCaseForLaneChange(decision_result_, obstacle_map_, speed_result->st_graph_);
}


TEST_F(SpeedModelParamTest, specialCaseForObstacleStopNormal) {
  processor_.init();

  processor_.nearest_front_obj_obstacle_ = true;
  processor_.nearest_front_obj_id_ = "1001";
  processor_.specialCaseForObstacleStop();

  processor_.nearest_front_obj_obstacle_ = true;
  processor_.nearest_front_obj_id_ = "1001";
  addDynamicObstacle("1001", 20.0, 0, LongitudinalOdTag::RISKY, Decision::ObjectGameType::MERGE_GAME);
  addStBoundary("1001", STBoundary::BoundaryType::RISKY);
  processor_.obstacle_map_ = obstacle_map_;
  processor_.specialCaseForObstacleStop();
  
  processor_.nearest_front_obj_obstacle_ = true;
  processor_.nearest_front_obj_id_ = "1002";
  processor_.nearest_front_obj_invasion_s_ = 20;
  processor_.nearest_front_obj_invasion_t_ = 0;
  processor_.nearest_front_obj_invasion_v_ = 5;
  addDynamicObstacle("1002", 20.0, 0, LongitudinalOdTag::FOLLOW, Decision::ObjectGameType::MERGE_GAME);
  addStBoundary("1002", STBoundary::BoundaryType::FOLLOW);
  processor_.obstacle_map_ = obstacle_map_;
  processor_.specialCaseForObstacleStop();
  processor_.obstacle_map_.at("1002")->path_st_boundary_.boundary_type_ = STBoundary::BoundaryType::YIELD;
  processor_.specialCaseForObstacleStop();

  processor_.nearest_front_obj_obstacle_ = false;
  processor_.nearest_front_obj_id_ = "tsr";
  processor_.nearest_front_obj_invasion_s_ = 20;
  processor_.nearest_front_obj_invasion_t_ = 0;
  processor_.nearest_front_obj_invasion_v_ = 5;
  processor_.specialCaseForObstacleStop();

  processor_.nearest_front_obj_id_ = "destination";
  processor_.specialCaseForObstacleStop();

  processor_.nearest_front_obj_id_ = "junction_stop";
  processor_.specialCaseForObstacleStop();
}


TEST_F(SpeedModelParamTest, specialCaseForMergeObstacleNormal) {
  processor_.init();
  addDynamicObstacle("1001", 20.0, 0, LongitudinalOdTag::FOLLOW, Decision::ObjectGameType::MERGE_GAME);
  addPathGroup();
  std::vector<std::pair<STPoint, STPoint>> st_bound;
  std::vector<double> lateral_s;
  for (int i = 0; i < 51; i++) {
    STPoint pt1(20.0, i * 0.1);
    STPoint pt2(20.0 + 5.0, i * 0.1);
    st_bound.emplace_back(pt1, pt2);
    lateral_s.push_back(0.1);
  }
  STBoundary* st_boundary = new STBoundary(st_bound);
  st_boundary->id_ = "1001";
  st_boundary->setBoundaryType(STBoundary::BoundaryType::FOLLOW);
  st_boundary->set_lateral_signed_distances(lateral_s);
  obstacle_map_.at("1001")->risk_st_boundary_ = *st_boundary;
  obstacle_map_.at("1001")->is_merge_obstacle_ = true;

  processor_.specialCaseForMergeObstacle(path_group_, obstacle_map_);
}


TEST_F(SpeedModelParamTest, specialCaseForOvertakeObstacleNormal) {
  processor_.init();
  addStBoundary();
  speed_result->st_graph_.st_drivable_boundaries_.at(0).has_back_take_over_obj = true;
  speed_result->st_graph_.st_drivable_boundaries_.at(0).lower_obj_id = "1001";
  processor_.specialCaseForOvertakeObstacle(decision_result_, obstacle_map_, speed_result->st_graph_);


  addDynamicObstacle("1001", 20.0, 0, LongitudinalOdTag::FOLLOW, Decision::ObjectGameType::MERGE_GAME);
  addStBoundary();
  speed_result->st_graph_.st_drivable_boundaries_.at(2).has_back_take_over_obj = true;
  speed_result->st_graph_.st_drivable_boundaries_.at(2).lower_obj_id = "1001";
  speed_result->st_graph_.st_drivable_boundaries_.at(2).s_lower_bound = 0.0;
  processor_.specialCaseForOvertakeObstacle(decision_result_, obstacle_map_, speed_result->st_graph_);
}

TEST_F(SpeedModelParamTest, localPathRiskyConsiderationNormal) {
  processor_.init();
  processor_.localPathRiskyConsideration(local_view_, speed_result);

  std::vector<std::pair<STPoint, std::string>> local_path_boundary;
  local_path_boundary.emplace_back(STPoint(10.0, 0.0), "1001");
  local_path_boundary.emplace_back(STPoint(10.0, 5.0), "1001");
  speed_result->local_path_lower_boundary = local_path_boundary;
  processor_.localPathRiskyConsideration(local_view_, speed_result);
  
  local_path_boundary.clear();
  local_path_boundary.emplace_back(STPoint(3.0, 0.0), "1001");
  local_path_boundary.emplace_back(STPoint(3.0, 5.0), "1001");
  speed_result->local_path_lower_boundary = local_path_boundary;
  processor_.localPathRiskyConsideration(local_view_, speed_result);
}

TEST_F(SpeedModelParamTest, updateSingleFrameSpeedLimit) {
  std::vector<SpatialSpeedLimit> input_speed_limit_vec;
  std::vector<SpatialSpeedLimit> res_speed_limit_vec;
  addPathGroup();
  for (int i = 0; i < path_group_.origin_path_.size(); i++) {
    input_speed_limit_vec.emplace_back(SpatialSpeedLimit(i, 10.0, "map"));
    res_speed_limit_vec.emplace_back(SpatialSpeedLimit(i, 20.0, "default"));
  }
  processor_.updateSingleFrameSpeedLimit(path_group_, input_speed_limit_vec, res_speed_limit_vec);

  input_speed_limit_vec.clear();
  res_speed_limit_vec.clear();
  for (int i = 0; i < path_group_.origin_path_.size(); i++) {
    input_speed_limit_vec.emplace_back(SpatialSpeedLimit(i, 20.0, "map"));
    res_speed_limit_vec.emplace_back(SpatialSpeedLimit(i, 10.0, "default"));
  }
  processor_.updateSingleFrameSpeedLimit(path_group_, input_speed_limit_vec, res_speed_limit_vec);

  input_speed_limit_vec.clear();
  res_speed_limit_vec.clear();
  processor_.updateSingleFrameSpeedLimit(path_group_, input_speed_limit_vec, res_speed_limit_vec);
}

TEST_F(SpeedModelParamTest, boundValidProtect) {
  double hard_upper = 10.0;
  double hard_lower = 0.0;
  double soft_upper = 5.0;
  double soft_lower = 5.0;
  processor_.boundValidProtect(hard_upper, hard_lower, soft_upper, soft_lower);
}

TEST_F(SpeedModelParamTest, getSpeedWallIdThroughType) {
  processor_.getSpeedWallIdThroughType(SpeedWallType::TFL_WALL);
  processor_.getSpeedWallIdThroughType(static_cast<SpeedWallType>(99));
}

TEST_F(SpeedModelParamTest, getStaticVSoftBound) {
  vector<proto::TrajectoryPoint> path;
  proto::TrajectoryPoint pt;
  pt.set_relative_time(0.0);
  pt.set_v(10.0);
  path.push_back(pt);
  processor_.getStaticVSoftBound(path, 0.0);

  pt.set_relative_time(1.0);
  pt.set_v(10.0);
  path.push_back(pt);
  processor_.getStaticVSoftBound(path, -1.0);
  processor_.getStaticVSoftBound(path, 2.0);
}

TEST_F(SpeedModelParamTest, findObstacle) {
  addDynamicObstacle("1001", 20.0, 0, LongitudinalOdTag::FOLLOW, Decision::ObjectGameType::MERGE_GAME);
  addDynamicObstacle("1002", 20.0, 0, LongitudinalOdTag::YIELD, Decision::ObjectGameType::MERGE_GAME);
  addDynamicObstacle("1003", 20.0, 0, LongitudinalOdTag::OVERTAKE, Decision::ObjectGameType::MERGE_GAME);
  processor_.obstacle_map_ = obstacle_map_;
  processor_.findObstacle("1001");
  processor_.findObstacle("1005");
}

TEST_F(SpeedModelParamTest, getAccThw) {
  processor_.getAccThw(FollowingDistanceLevel::Invalid);
  processor_.getAccThw(FollowingDistanceLevel::Min);
  processor_.getAccThw(FollowingDistanceLevel::Mid);
  processor_.getAccThw(FollowingDistanceLevel::Max);
  processor_.getAccThw(FollowingDistanceLevel::Low);
  processor_.getAccThw(FollowingDistanceLevel::High);
  processor_.getAccThw(static_cast<FollowingDistanceLevel>(999));
}

}  // namespace gpal::pnc::planning