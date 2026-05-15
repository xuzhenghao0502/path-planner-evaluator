#include <gtest/gtest.h>
#include <any>

#include "config_manager/config_manager.h"
#include "config/speed_planner/st_graph_processor.pb.h"

#define private public
#include "st_graph_processor/st_boundary_mapper.h"


namespace gpal::pnc::planning {

class StBoundaryMapperTest : public ::testing::Test {
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

  void addStaticObstacle() {
    Obstacle obs("1001", math::Polygon2d(math::Box2d(math::Vec2d(20.0, 0), 0.0, 5.0, 2.0)));
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

  const std::vector<double> time_grid_ = {0.0, 0.1, 0.2, 0.4, 0.6, 1.0, 1.4, 1.8,
                                          2.2, 2.6, 3.0, 3.4, 3.8, 4.2, 4.6, 5.0};
  const double time_resolution_{0.1};
  const double time_horizon_{5.0};
  STBoundaryMapper processor_ = STBoundaryMapper(time_grid_, time_resolution_, time_horizon_);
  DiscretizedPath path_;
  PathGroup path_group_;
  ObstacleSet obstacle_set_;
  BehaviorState behavior_state_;
  StGraphProcessorConfig config_;
  SpeedPlannerObstacle obstacle_;
  LocalView local_view_;
};

TEST_F(StBoundaryMapperTest, ProcessNormal) {
  addPathGroup();
  addStaticObstacle();
  processor_.process(local_view_, obstacle_set_, path_group_, behavior_state_);
  obstacle_set_["1001"]->predicted_trajectory_.clear();
  processor_.process(local_view_, obstacle_set_, path_group_, behavior_state_);

}

TEST_F(StBoundaryMapperTest, CaculateLocalPathLowerBoundaryNormal) {
  std::vector<std::pair<STPoint, std::string>> local_path_lower_boundary;
  addPathGroup();
  addStaticObstacle();
  processor_.CaculateLocalPathLowerBoundary(local_view_, obstacle_set_, path_group_, behavior_state_, local_path_lower_boundary);
  obstacle_set_["1001"]->predicted_trajectory_.clear();
  processor_.CaculateLocalPathLowerBoundary(local_view_, obstacle_set_, path_group_, behavior_state_, local_path_lower_boundary);

}

TEST_F(StBoundaryMapperTest, isEgoAwayFromDiscretizedPath) {
  addPathGroup();
  bool is_ego_away = processor_.isEgoAwayFromDiscretizedPath(local_view_,  path_group_.origin_path_);
  EXPECT_FALSE(is_ego_away);
}

TEST_F(StBoundaryMapperTest, calcSLProjection) {
  addPath();
  processor_.calcSLProjection(math::Vec2d(10, 0), path_, 30, 10);

  processor_.calcSLProjection(math::Vec2d(10, 0), path_, 20, 20);
}

}  // namespace gpal::pnc::planning