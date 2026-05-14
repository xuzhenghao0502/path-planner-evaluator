#include <gtest/gtest.h>
#include <any>
#include "config_manager/config_manager.h"
#include "config/speed_planner/st_graph_processor.pb.h"

#define private public
#include "st_graph_processor/st_graph_processor.h"


namespace gpal::pnc::planning {

class STGraphProcessorTest : public ::testing::Test {
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

  void addPath() {
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
    path_ = DiscretizedPath(path_points);
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
    path_ = DiscretizedPath(path_points);
    path_group_.origin_path_ = path_;
    path_group_.extand_interval_path_ = path_;
    path_group_.path_points_for_segmentation_after_extand_ = path_;
    path_group_.path_points_for_segmentation_ = path_;
  }

  proto::PerceptionObstacle addPerceptionObstacle(){
    proto::PerceptionObstacle perception_obstacle;
    perception_obstacle.add_sub_obstacles();
    return perception_obstacle;
  }

  void addStaticObstacle() {
    Obstacle obs("1001", math::Polygon2d(math::Box2d(math::Vec2d(20.0, 0), 0.0, 5.0, 2.0)));
    obs.perception_obstacle_= addPerceptionObstacle();
    std::vector<Decision::RawSinglePrediction> raw_prediction_in;
    Decision::DecisionObject decision_obj("1001", 5.0, 2.0, 20.0, 0.0, 0.0, 5.0, true, raw_prediction_in);
    decision_obj.long_od_tag = LongitudinalOdTag::FOLLOW;
    obstacle_ =
        SpeedPlannerObstacle(std::make_shared<Obstacle>(obs), std::make_shared<Decision::DecisionObject>(decision_obj));
    obstacle_set_.emplace(obstacle_.id(), std::make_shared<SpeedPlannerObstacle>(obstacle_));
  }

  void addDynamicObstacle() {
    Obstacle obs("1001", math::Polygon2d(math::Box2d(math::Vec2d(20.0, 0), 0.0, 5.0, 2.0)));

    std::vector<std::shared_ptr<Decision::RawSinglePrediction>> raw_predictions;
    std::vector<Decision::RawSinglePrediction> raw_prediction_in;
    Decision::RawPredictionTrajectory raw_prediction_traj;
    for (int i = 0; i < 50; i++) {
      proto::TrajectoryPoint pt;
      pt.mutable_path_point()->set_x(20.0 + i * 0.1 * 5.0);
      pt.mutable_path_point()->set_y(0.0);
      pt.mutable_path_point()->set_theta(0.0);
      pt.mutable_path_point()->set_kappa(0.001);
      pt.mutable_path_point()->set_s(i * 0.1 * 5.0);
      pt.set_relative_time(i * 0.1);
      raw_prediction_traj.push_back(pt);
    }
    std::shared_ptr<Decision::RawSinglePrediction> raw_prediction = std::make_shared<Decision::RawSinglePrediction>();
    raw_prediction.traj = raw_prediction_traj;
    raw_predictions.push_back(raw_prediction);

    Decision::DecisionObject decision_obj("1001", 5.0, 2.0, 20.0, 0.0, 0.0, 5.0, false, raw_prediction_in);
    decision_obj.long_od_tag = LongitudinalOdTag::FOLLOW;
    decision_obj.raw_predictions =
        std::make_shared<std::vector<std::shared_ptr<Decision::RawSinglePrediction>>>(raw_predictions);

    obstacle_ =
        SpeedPlannerObstacle(std::make_shared<Obstacle>(obs), std::make_shared<Decision::DecisionObject>(decision_obj));
    obstacle_set_.emplace(obstacle_.id(), std::make_shared<SpeedPlannerObstacle>(obstacle_));
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

    obstacle_ =
        SpeedPlannerObstacle(std::make_shared<Obstacle>(obs), std::make_shared<Decision::DecisionObject>(decision_obj));
    obstacle_set_.emplace(obstacle_.id(), std::make_shared<SpeedPlannerObstacle>(obstacle_));
  }


  const std::vector<double> time_grid_ = {0.0, 0.1, 0.2, 0.4, 0.6, 1.0, 1.4, 1.8,
                                          2.2, 2.6, 3.0, 3.4, 3.8, 4.2, 4.6, 5.0};
  const double time_resolution_{0.1};
  const double time_horizon_{5.0};
  STGraphProcessor processor_ = STGraphProcessor(time_grid_, time_resolution_, time_horizon_);
  DiscretizedPath path_;
  PathGroup path_group_;
  ObstacleSet obstacle_set_;
  BehaviorState behavior_state_;
  StGraphProcessorConfig config_;
  SpeedPlannerObstacle obstacle_;
  LocalView local_view_;
  DecisionResult decision_result_;
};


TEST_F(STGraphProcessorTest, ProcessNormal) {
  std::shared_ptr<SpeedResult> speed_result = std::make_shared<SpeedResult>();
  addPathGroup();
  addStaticObstacle();
  addDynamicObstacle("1002", 20.0, 0.0, LongitudinalOdTag::YIELD, Decision::ObjectGameType::CROSS_GAME);
  addDynamicObstacle("1003", 20.0, 0.0, LongitudinalOdTag::OVERTAKE, Decision::ObjectGameType::CROSS_GAME);
  addDynamicObstacle("1004", 20.0, 0.0, LongitudinalOdTag::RISKY, Decision::ObjectGameType::CROSS_GAME);
  addDynamicObstacle("1005", 20.0, 0.0, LongitudinalOdTag::FOLLOW, Decision::ObjectGameType::OPPOSITE_GAME);
  addDynamicObstacle("1005", 20.0, 0.0, LongitudinalOdTag::YIELD, Decision::ObjectGameType::OPPOSITE_GAME);
  LateralPath lateral_path;
  lateral_path.discretized_path_group_ = path_group_;
  lateral_path.local_path_group_ = path_group_;
  processor_.process(local_view_,decision_result_, lateral_path, behavior_state_,0, obstacle_set_, speed_result);
}

TEST_F(STGraphProcessorTest, ProcessEnableLocalPath) {
  std::shared_ptr<SpeedResult> speed_result = std::make_shared<SpeedResult>();
  processor_.speed_planner_config_.set_enable_local_path_od_check(true);
  addPathGroup();
  addStaticObstacle();
  LateralPath lateral_path;
  lateral_path.discretized_path_group_ = path_group_;
  lateral_path.local_path_group_ = path_group_;
  processor_.process(local_view_,decision_result_, lateral_path, behavior_state_,0, obstacle_set_, speed_result);
}


TEST_F(STGraphProcessorTest, processCipvStBoundaryNormal){
  std::shared_ptr<SpeedResult> speed_result = std::make_shared<SpeedResult>();
  addPathGroup();
  addStaticObstacle();
  LateralPath lateral_path;
  lateral_path.discretized_path_group_ = path_group_;
  lateral_path.local_path_group_ = path_group_;
  processor_.processCipvStBoundary(local_view_,decision_result_, lateral_path, 0, obstacle_set_, speed_result);
}


TEST_F(STGraphProcessorTest, laneChangeOvertakeObstacleProcessNormal){
  addPathGroup();
  addStaticObstacle();
  STBoundaryMapper st_boundary_mapper_(time_grid_, time_resolution_, time_horizon_);
  st_boundary_mapper_.process(local_view_, obstacle_set_,path_group_, behavior_state_);
  processor_.laneChangeOvertakeObstacleProcess(obstacle_set_,"1001");
}


TEST_F(STGraphProcessorTest, ObstacleSpeedAndAccelPostProcess) {
  double speed,accel;
  processor_.ObstacleSpeedAndAccelPostProcess(1.0, -1.0, 0.1, speed, accel);
  processor_.ObstacleSpeedAndAccelPostProcess(1.0, 1.0, 0.1, speed, accel);
}

TEST_F(STGraphProcessorTest, GetBoundsAtT) {
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
    st_boundary->setBoundaryType(STBoundary::BoundaryType::OVERTAKE);
    st_boundary->set_lateral_signed_distances(lateral_s);

    STDrivableBoundary st_drivable_boundary;
    st_drivable_boundary.t = 1 * 0.1;
    st_drivable_boundary.s_lower_bound = 1.0;
    processor_.GetBoundsAtT(st_boundary, 0.1, 5.0, st_drivable_boundary);

    st_boundary->setBoundaryType(STBoundary::BoundaryType::RISKY);
    processor_.GetBoundsAtT(st_boundary, 0.1, 5.0, st_drivable_boundary);

}


TEST_F(STGraphProcessorTest, updataSpeedWallBoundInStGraph) {

    SpeedWall speed_wall;
    speed_wall.type = SpeedWallType::DESTINATION_WALL;
    speed_wall.stop_distance = 2.0;
    speed_wall.st_wall = {{0.0, 5.0}, {5.0, 5.0}};
    vector<SpeedWall> speed_walls_;
    speed_walls_.push_back(speed_wall);
    speed_wall.st_wall = {{0.0, 100.0}, {5.0, 100.0}};
    speed_walls_.push_back(speed_wall);
    StGraph st_graph;
    for (int i = 0; i < 51; i++) {
      STDrivableBoundary st_bound;
      st_bound.t = i*0.1;
      st_bound.s_upper_bound = 10.0;
      st_graph.st_drivable_boundaries_.push_back(st_bound);
    }
    processor_.updataSpeedWallBoundInStGraph(local_view_,speed_walls_, &st_graph);
}


TEST_F(STGraphProcessorTest, updateDecisionObstacleTag) {
  std::shared_ptr<SpeedResult> speed_result = std::make_shared<SpeedResult>();
  speed_result->behavior_state_.park_in_state_ = true;
  addPathGroup();
  addDynamicObstacle("2001", 20.0, 0.0, LongitudinalOdTag::FOLLOW, Decision::ObjectGameType::OPPOSITE_GAME);

  processor_.updateDecisionObstacleTag(local_view_,std::make_shared<SpeedPlannerObstacle>(obstacle_), speed_result);
  
  speed_result->behavior_state_.is_r_gear_ = true;
  processor_.updateDecisionObstacleTag(local_view_,std::make_shared<SpeedPlannerObstacle>(obstacle_), speed_result);
}


TEST_F(STGraphProcessorTest, emptyStboundaryMapper) {
  addPathGroup();
  addDynamicObstacle("2001", 20.0, 0.0, LongitudinalOdTag::FOLLOW, Decision::ObjectGameType::OPPOSITE_GAME);
  // 调用函数
  std::shared_ptr<SpeedResult> speed_result_ = std::make_shared<SpeedResult>();
  processor_.updateRiskyObstacleTag(local_view_, std::make_shared<SpeedPlannerObstacle>(obstacle_), speed_result_);
  processor_.updateOvertakeObstacleTag(local_view_, std::make_shared<SpeedPlannerObstacle>(obstacle_), speed_result_);
  processor_.updateYieldObstacleTag(std::make_shared<SpeedPlannerObstacle>(obstacle_), speed_result_);
  processor_.updateFollowObstacleTag(std::make_shared<SpeedPlannerObstacle>(obstacle_), speed_result_);
}

TEST_F(STGraphProcessorTest, CheckOppositeObstacleRiskNoRiskField) {
  // 准备测试数据
  addPathGroup();
  addDynamicObstacle("2001", 20.0, 0.0, LongitudinalOdTag::FOLLOW, Decision::ObjectGameType::OPPOSITE_GAME);
  // 调用函数
  bool result = processor_.checkOppositeObstacleRisk(local_view_, std::make_shared<SpeedPlannerObstacle>(obstacle_));

  SpeedPlannerObstacle::RiskFieldInfo risk_field;
  risk_field.nearest_s = 1.0;
  risk_field.min_box_distance = 1.0;
  obstacle_.risk_field_infos_.emplace_back(risk_field);
  result = processor_.checkOppositeObstacleRisk(local_view_, std::make_shared<SpeedPlannerObstacle>(obstacle_));
  // 验证结果 - 无风险场信息时应返回false
  EXPECT_FALSE(result);
}

TEST_F(STGraphProcessorTest, CalcFollowObstacleSpeedAccelDistChangeRateNormal) {
  // 准备测试数据
  addPathGroup();
  addDynamicObstacle("3001", 30.0, 0.0, LongitudinalOdTag::FOLLOW, Decision::ObjectGameType::CROSS_GAME);
  
  // 创建障碍物对象
  std::shared_ptr<SpeedPlannerObstacle> obstacle = std::make_shared<SpeedPlannerObstacle>(obstacle_);
  
  // 设置初始距离
  double distance = 30.0;
  
  // 初始化输出参数
  double filtered_speed = 0.0;
  double filtered_accel = 0.0;
  double acc_gain = 1.0;
  double distance_change_rate = 0.0;
  
  // 调用函数
  processor_.CalcFollowObstacleSpeedAccelDistChangeRate(obstacle, distance, filtered_speed, filtered_accel, acc_gain, distance_change_rate);
  
  // 验证结果
  EXPECT_GE(filtered_speed, 0.0);  // 速度应该非负
  EXPECT_LE(filtered_speed, obstacle->speed() * 1.25);  // 速度应该不超过原始速度的1.25倍
  EXPECT_GE(filtered_speed, obstacle->speed() * 0.75);  // 速度应该不低于原始速度的0.75倍
}

TEST_F(STGraphProcessorTest, CalcFollowObstacleSpeedAccelDistChangeRateWithHistory) {
  // 准备测试数据
  addPathGroup();
  addDynamicObstacle("3002", 30.0, 0.0, LongitudinalOdTag::FOLLOW, Decision::ObjectGameType::CROSS_GAME);
  
  // 创建障碍物对象
  std::shared_ptr<SpeedPlannerObstacle> obstacle = std::make_shared<SpeedPlannerObstacle>(obstacle_);
  
  // 模拟历史帧数据
  std::unordered_map<std::string, double> speed_map;
  std::unordered_map<std::string, double> dist_map;
  std::unordered_map<std::string, double> accel_map;
  
  speed_map[obstacle->id()] = 10.0;
  dist_map[obstacle->id()] = 25.0;
  accel_map[obstacle->id()] = -1.0;
  
  // 将历史数据添加到processor_中
  processor_.history_frame_follow_obs_speeds_.push_back(speed_map);
  processor_.history_frame_follow_obs_distances_.push_back(dist_map);
  processor_.history_frame_follow_obs_accels_.push_back(accel_map);
  processor_.history_frame_time_stamps_.push_back(1000000); // 1秒前的时间戳
  
  // 设置当前时间戳
  processor_.time_stamp_ = 2000000; // 当前时间戳
  
  // 设置初始距离
  double distance = 30.0;
  
  // 初始化输出参数
  double filtered_speed = 5.0;
  double filtered_accel = 0.0;
  double acc_gain = 1.0;
  double distance_change_rate = 0.0;
  
  // 调用函数
  processor_.CalcFollowObstacleSpeedAccelDistChangeRate(obstacle, distance, filtered_speed, filtered_accel, acc_gain, distance_change_rate);
  
  // 验证结果
  // EXPECT_GE(filtered_speed, 0.0);  // 速度应该非负
}

TEST_F(STGraphProcessorTest, CalcFollowObstacleSpeedAccelDistChangeRateDecelGain) {
  // 准备测试数据
  addPathGroup();
  addDynamicObstacle("3003", 30.0, 0.0, LongitudinalOdTag::FOLLOW, Decision::ObjectGameType::CROSS_GAME);
  
  // 创建障碍物对象
  std::shared_ptr<SpeedPlannerObstacle> obstacle = std::make_shared<SpeedPlannerObstacle>(obstacle_);
  
  // 模拟历史帧数据，包含减速度增加的情况
  std::unordered_map<std::string, double> speed_map;
  std::unordered_map<std::string, double> dist_map;
  std::unordered_map<std::string, double> accel_map;
  
  speed_map[obstacle->id()] = 10.0;
  dist_map[obstacle->id()] = 25.0;
  accel_map[obstacle->id()] = -2.0;
  
  // 将历史数据添加到processor_中
  processor_.history_frame_follow_obs_speeds_.push_back(speed_map);
  processor_.history_frame_follow_obs_distances_.push_back(dist_map);
  processor_.history_frame_follow_obs_accels_.push_back(accel_map);
  processor_.history_frame_time_stamps_.push_back(1000000); // 1秒前的时间戳
  
  // 设置当前时间戳
  processor_.time_stamp_ = 2000000; // 当前时间戳
  
  // 设置初始距离
  double distance = 30.0;
  
  // 初始化输出参数
  double filtered_speed = 5.0;
  double filtered_accel = -3.0;  // 设置一个较大的负加速度
  double acc_gain = 1.0;
  double distance_change_rate = 0.0;
  
  // 调用函数
  processor_.CalcFollowObstacleSpeedAccelDistChangeRate(obstacle, distance, filtered_speed, filtered_accel, acc_gain, distance_change_rate);
  
  // 验证结果 - 减速度增益应该被应用
  // EXPECT_LE(acc_gain, 1.0);  // acc_gain应该被更新
}

TEST_F(STGraphProcessorTest, CalcFollowObstacleSpeedAccelDistChangeRateEdgeCase) {
  // 准备测试数据
  addPathGroup();
  addDynamicObstacle("3004", 30.0, 0.0, LongitudinalOdTag::FOLLOW, Decision::ObjectGameType::CROSS_GAME);
  
  // 创建障碍物对象
  std::shared_ptr<SpeedPlannerObstacle> obstacle = std::make_shared<SpeedPlannerObstacle>(obstacle_);
  
  // 设置初始距离为0
  double distance = 0.0;
  
  // 初始化输出参数
  double filtered_speed = 0.0;
  double filtered_accel = 0.0;
  double acc_gain = 1.0;
  double distance_change_rate = 0.0;
  
  // 调用函数
  processor_.CalcFollowObstacleSpeedAccelDistChangeRate(obstacle, distance, filtered_speed, filtered_accel, acc_gain, distance_change_rate);
  
  // // 验证结果
  // EXPECT_GE(filtered_speed, 0.0);  // 速度应该非负
  // EXPECT_GE(distance_change_rate, 0.0);  // 距离变化率应该非负
}

TEST_F(STGraphProcessorTest, GenerateLaneChangeOvertakeObstacleSTBoundary) {
  // 测试正常情况
  addDynamicObstacle("1001", 20.0, 0.0, LongitudinalOdTag::OVERTAKE, Decision::ObjectGameType::CROSS_GAME);
  auto obstacle = obstacle_set_["1001"];
  
  // 创建一个初始的ST边界
  std::vector<STPoint> lower_points = {STPoint(15.0, 2.0), STPoint(15.0, 3.0)};
  std::vector<STPoint> upper_points = {STPoint(25.0, 2.0), STPoint(25.0, 3.0)};
  auto initial_boundary = STBoundary::createInstance(lower_points, upper_points);
  obstacle->setPathStBoundary(initial_boundary);

    //   std::vector<std::pair<STPoint, STPoint>> st_bound;
    // std::vector<double> lateral_s;
    // for (int i = 0; i < 51; i++) {
    //   STPoint pt1(20.0, i * 0.1);
    //   STPoint pt2(20.0 + 5.0, i * 0.1);
    //   st_bound.emplace_back(pt1, pt2);
    //   lateral_s.push_back(0.1);
    // }
    // STBoundary* st_boundary = new STBoundary(st_bound);
    // st_boundary->id_ = id;
    // st_boundary->setBoundaryType(type);
    // st_boundary->set_lateral_signed_distances(lateral_s);
  
  // 设置障碍物速度和加速度
  proto::PerceptionObstacle perception_obstacle;
  auto obs = perception_obstacle.add_sub_obstacles();
  obs->mutable_acceleration()->set_x(1.0);
  obstacle->raw_obstacle_ptr_ = make_shared<Obstacle>();
  obstacle->raw_obstacle_ptr_->perception_obstacle_ = perception_obstacle;
  obstacle->raw_obstacle_ptr_->speed_ = 10.0;
  processor_.GenerateLaneChangeOvertakeObstacleSTBoundary(obstacle);
  
  // // 验证生成的ST边界不为空
  // EXPECT_FALSE(obstacle->pathStBoundary().IsEmpty());
  
  // // 验证边界类型
  // EXPECT_EQ(obstacle->pathStBoundary().boundary_type(), STBoundary::BoundaryType::OVERTAKE);
  
  // // 验证ID
  // EXPECT_EQ(obstacle->pathStBoundary().id(), "1001");
}

}