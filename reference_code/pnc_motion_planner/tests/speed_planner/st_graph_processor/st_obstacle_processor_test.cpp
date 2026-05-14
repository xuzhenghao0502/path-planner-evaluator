#include <gtest/gtest.h>
#include <any>

#include "config_manager/config_manager.h"
#include "config/speed_planner/st_graph_processor.pb.h"

#define private public
#include "st_graph_processor/st_obstacle_processor.h"


namespace gpal::pnc::planning {
  
class StObstacleProcessorTest : public ::testing::Test {
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

    processor_.param_ = config_;
  }


  void addPath() {
    // 设置有效的reference line和停止线
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

  void addStaticObstacle(){
    Obstacle obs("1001", math::Polygon2d(math::Box2d(math::Vec2d(20.0, 0), 0.0, 5.0, 2.0)));

    std::vector<Decision::RawSinglePrediction> raw_prediction_in;

    // Decision::RawPredictionTrajectory raw_prediction_traj;
    // for(int i = 0; i < 50;i++){
    //   proto::TrajectoryPoint pt;
    //   pt.mutable_path_point()->set_x(20.0 + i* 0.1* 5.0)
      
    // }

    Decision::DecisionObject decision_obj("1001", 5.0, 2.0, 20.0, 0.0, 0.0, 5.0, true, raw_prediction_in);
    decision_obj.long_od_tag = LongitudinalOdTag::FOLLOW;

    obstacle_ = SpeedPlannerObstacle(std::make_shared<Obstacle>(obs),std::make_shared<Decision::DecisionObject>(decision_obj));
  }

    void addDynamicObstacle(){
    Obstacle obs("1001", math::Polygon2d(math::Box2d(math::Vec2d(20.0, 0), 0.0, 5.0, 2.0)));

    std::vector<std::shared_ptr<Decision::RawSinglePrediction>> raw_predictions;
     std::vector<Decision::RawSinglePrediction> raw_prediction_in;
    Decision::RawPredictionTrajectory raw_prediction_traj;
    for(int i = 0; i < 50;i++){
      proto::TrajectoryPoint pt;
      pt.mutable_path_point()->set_x(20.0 + i* 0.1* 5.0);
      pt.mutable_path_point()->set_y(0.0);
      pt.mutable_path_point()->set_theta(0.0);
      pt.mutable_path_point()->set_kappa(0.001);
      pt.mutable_path_point()->set_s( i* 0.1* 5.0);
      pt.set_relative_time(i*0.1);
      raw_prediction_traj.push_back(pt);
    }
    std::shared_ptr<Decision::RawSinglePrediction> raw_prediction = std::make_shared<Decision::RawSinglePrediction>();
    raw_prediction.traj = raw_prediction_traj;
    raw_predictions.push_back(raw_prediction);

    Decision::DecisionObject decision_obj("1001", 5.0, 2.0, 20.0, 0.0, 0.0, 5.0, false, raw_prediction_in);
    decision_obj.long_od_tag = LongitudinalOdTag::FOLLOW;
    decision_obj.raw_predictions = std::make_shared<std::vector<std::shared_ptr<Decision::RawSinglePrediction>>>(raw_predictions);

    obstacle_ = SpeedPlannerObstacle(std::make_shared<Obstacle>(obs),std::make_shared<Decision::DecisionObject>(decision_obj));
  }

  const std::vector<double> time_grid_ = {0.0, 0.1, 0.2, 0.4, 0.6, 1.0, 1.4, 1.8,
                                          2.2, 2.6, 3.0, 3.4, 3.8, 4.2, 4.6, 5.0};
  const double time_resolution_{0.1};
  const double time_horizon_{5.0};
  STObstacleProcessor processor_ = STObstacleProcessor(time_grid_,time_resolution_,time_horizon_);
  DiscretizedPath path_;
  BehaviorState behavior_state_;
  StGraphProcessorConfig config_;
  SpeedPlannerObstacle obstacle_;
};


TEST_F(StObstacleProcessorTest, ComputeSTBoundaryNormal){
  addPath();
  addStaticObstacle();
  processor_.ComputeSTBoundary(std::make_shared<SpeedPlannerObstacle>(obstacle_),path_,behavior_state_);
}

TEST_F(StObstacleProcessorTest, ComputeSTBoundaryDynamic){
  addPath();
  addDynamicObstacle();
  cout<<"  obstacle trajectory size = "<<obstacle_.predictedTrajectory().size()<<endl;
  processor_.ComputeSTBoundary(std::make_shared<SpeedPlannerObstacle>(obstacle_),path_,behavior_state_);
}


TEST_F(StObstacleProcessorTest, ComputeLocalSTBoundaryNormal){
  addPath();
  addStaticObstacle();
  size_t t_index_min = 0;
  size_t t_index_max = obstacle_.predictedTrajectory().size();
  std::vector<BoxProjectInfo> box_project_info;
  STBoundary boundary;
  processor_.ComputeSTBoundaryLocalPath(std::make_shared<SpeedPlannerObstacle>(obstacle_),path_,behavior_state_,box_project_info,t_index_min,t_index_max,boundary);
}

TEST_F(StObstacleProcessorTest, ComputeLocalSTBoundaryDynamic){
  addPath();
  addDynamicObstacle();
  size_t t_index_min = 0;
  size_t t_index_max = obstacle_.predictedTrajectory().size();
  std::vector<BoxProjectInfo> box_project_info;
  STBoundary boundary;
  processor_.ComputeSTBoundaryLocalPath(std::make_shared<SpeedPlannerObstacle>(obstacle_),path_,behavior_state_,box_project_info,t_index_min,t_index_max,boundary);
}

TEST_F(StObstacleProcessorTest, getSpeedLimit){
  processor_.getSpeedLimit(0.5, 10, proto::PerceptionObstacle_ObstacleType_kTypeCones);
  processor_.getSpeedLimit(0.5, 10, proto::PerceptionObstacle_ObstacleType_kTypeImpassible);
  processor_.getSpeedLimit(0.5, 10, proto::PerceptionObstacle_ObstacleType_kTypePedestrianNonStanding);
  processor_.getSpeedLimit(0.5, 10, proto::PerceptionObstacle_ObstacleType_kTypePedestrian);
  processor_.getSpeedLimit(0.5, 10, proto::PerceptionObstacle_ObstacleType_kTypeBicycle);
  processor_.getSpeedLimit(0.5, 10, proto::PerceptionObstacle_ObstacleType_kTypeCar);
}

TEST_F(StObstacleProcessorTest, calcAlphaBetweenBox){
  BoxInfo box1(0.0, 0.0, 0.0, 5.0, 2.0);
  BoxInfo box2(0.0, 0.0, 0.0, 5.0, 2.0);
  math::Vec2d nearest_point;
  double alpha = processor_.calcAlphaBetweenBox(box1, box2, true, &nearest_point);
}


}
