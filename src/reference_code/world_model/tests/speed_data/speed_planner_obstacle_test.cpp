#include <base/singleton.h>
#include <fmt/chrono.h>
#include <gtest/gtest.h>

#include <any>

#include "config_manager/config_manager.h"

#define private public
#include "speed/speed_planner_obstacle.h"


namespace gpal::pnc::planning {

TEST(SpeedPlannerObstacleTest, TestSpeedPlannerObstacle) {
  proto::PerceptionObstacle perception_obstacle;
  perception_obstacle.add_sub_obstacles();
  perception_obstacle.set_obstacle_type(proto::PerceptionObstacle_ObstacleType_kTypeCar);
  Obstacle obs("1001", 0, perception_obstacle);

  std::vector<Decision::RawSinglePrediction> raw_prediction_in;
  Decision::DecisionObject decision_obj("1001", 5.0, 2.0, 20.0, 0.0, 0.0, 5.0, false, raw_prediction_in);

  std::vector<gpal::proto::TrajectoryPoint> predicted_trajectory;

  SpeedPlannerObstacle speed_planner_obstacle1;
  SpeedPlannerObstacle speed_planner_obstacle2(std::make_shared<Obstacle>(obs));
  SpeedPlannerObstacle speed_planner_obstacle3(std::make_shared<Obstacle>(obs),
                                               std::make_shared<Decision::DecisionObject>(decision_obj));
  SpeedPlannerObstacle speed_planner_obstacle4(nullptr);
  SpeedPlannerObstacle speed_planner_obstacle5(nullptr, std::make_shared<Decision::DecisionObject>(decision_obj));
  SpeedPlannerObstacle speed_planner_obstacle6(std::make_shared<Obstacle>(obs), nullptr);
  SpeedPlannerObstacle speed_planner_obstacle7(nullptr, predicted_trajectory);
  SpeedPlannerObstacle speed_planner_obstacle8(std::make_shared<Obstacle>(obs), predicted_trajectory);
  gpal::proto::TrajectoryPoint trajectory_point;
  predicted_trajectory.emplace_back(trajectory_point);
  SpeedPlannerObstacle speed_planner_obstacle9(std::make_shared<Obstacle>(obs), predicted_trajectory);
}

TEST(SpeedPlannerObstacleTest, TestSpeedPlannerObstaclePrediction) {
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
  raw_prediction->traj = raw_prediction_traj;
  raw_predictions.push_back(raw_prediction);

  Decision::DecisionObject decision_obj("1001", 5.0, 2.0, 20.0, 0.0, 0.0, 5.0, false, raw_prediction_in);
  decision_obj.long_od_tag = LongitudinalOdTag::FOLLOW;
  decision_obj.raw_predictions =
      std::make_shared<std::vector<std::shared_ptr<Decision::RawSinglePrediction>>>(raw_predictions);

  SpeedPlannerObstacle speed_planner_obstacle(std::make_shared<Obstacle>(obs),
                                              std::make_shared<Decision::DecisionObject>(decision_obj));

  speed_planner_obstacle.getPointAtTime(-1.0);
  speed_planner_obstacle.getPointAtTime(10.0);
  speed_planner_obstacle.getPointAtTime(3.0);

  speed_planner_obstacle.getBoundingBoxAtTime(-1.0);
  speed_planner_obstacle.getBoundingBoxAtTime(10.0);
  speed_planner_obstacle.getBoundingBoxAtTime(3.0);
}

TEST(SpeedPlannerObstacleTest, TestSpeedPlannerObstaclePrediction2) {
  Obstacle obs("1001", math::Polygon2d(math::Box2d(math::Vec2d(20.0, 0), 0.0, 5.0, 2.0)));

  std::vector<std::shared_ptr<Decision::RawSinglePrediction>> raw_predictions;
  std::vector<Decision::RawSinglePrediction> raw_prediction_in;
  Decision::RawPredictionTrajectory raw_prediction_traj;
  int i = 1;
  proto::TrajectoryPoint pt;
  pt.mutable_path_point()->set_x(20.0 + i * 0.1 * 5.0);
  pt.mutable_path_point()->set_y(0.0);
  pt.mutable_path_point()->set_theta(0.0);
  pt.mutable_path_point()->set_kappa(0.001);
  pt.mutable_path_point()->set_s(i * 0.1 * 5.0);
  pt.set_relative_time(i * 0.1);
  raw_prediction_traj.push_back(pt);

  std::shared_ptr<Decision::RawSinglePrediction> raw_prediction = std::make_shared<Decision::RawSinglePrediction>();
  raw_prediction->traj = raw_prediction_traj;
  raw_predictions.push_back(raw_prediction);

  Decision::DecisionObject decision_obj("1001", 5.0, 2.0, 20.0, 0.0, 0.0, 5.0, false, raw_prediction_in);
  decision_obj.long_od_tag = LongitudinalOdTag::FOLLOW;
  decision_obj.raw_predictions =
      std::make_shared<std::vector<std::shared_ptr<Decision::RawSinglePrediction>>>(raw_predictions);

  SpeedPlannerObstacle speed_planner_obstacle(std::make_shared<Obstacle>(obs),
                                              std::make_shared<Decision::DecisionObject>(decision_obj));

  speed_planner_obstacle.getPointAtTime(3.0);
}

TEST(SpeedPlannerObstacleTest, TestSpeedPlannerObstaclePrediction3) {
  Obstacle obs("1001", math::Polygon2d(math::Box2d(math::Vec2d(20.0, 0), 0.0, 5.0, 2.0)));

  std::vector<std::shared_ptr<Decision::RawSinglePrediction>> raw_predictions;
  std::vector<Decision::RawSinglePrediction> raw_prediction_in;
  Decision::RawPredictionTrajectory raw_prediction_traj;
  int i = 1;
  proto::TrajectoryPoint pt;
  pt.mutable_path_point()->set_x(20.0 + i * 0.1 * 5.0);
  pt.mutable_path_point()->set_y(0.0);
  pt.mutable_path_point()->set_theta(0.0);
  pt.mutable_path_point()->set_kappa(0.001);
  pt.mutable_path_point()->set_s(i * 0.1 * 5.0);
  pt.set_relative_time(-i * 0.1);
  raw_prediction_traj.push_back(pt);

  std::shared_ptr<Decision::RawSinglePrediction> raw_prediction = std::make_shared<Decision::RawSinglePrediction>();
  raw_prediction->traj = raw_prediction_traj;
  raw_predictions.push_back(raw_prediction);

  Decision::DecisionObject decision_obj("1001", 5.0, 2.0, 20.0, 0.0, 0.0, 5.0, false, raw_prediction_in);
  decision_obj.long_od_tag = LongitudinalOdTag::FOLLOW;
  decision_obj.raw_predictions =
      std::make_shared<std::vector<std::shared_ptr<Decision::RawSinglePrediction>>>(raw_predictions);

  SpeedPlannerObstacle speed_planner_obstacle(std::make_shared<Obstacle>(obs),
                                              std::make_shared<Decision::DecisionObject>(decision_obj));

  speed_planner_obstacle.getPointAtTime(3.0);
}

TEST(SpeedPlannerObstacleTest, RiskInfo) {
  SpeedPlannerObstacle::RiskFieldInfo risk_field_info;

  std::vector<std::pair<double, double>> box_dis_vector;
  box_dis_vector.emplace_back(0.0, 2.0);
  box_dis_vector.emplace_back(1.0, 2.0);
  box_dis_vector.emplace_back(2.0, 1.0);
  box_dis_vector.emplace_back(3.0, 0.5);
  box_dis_vector.emplace_back(4.0, 0.5);
  box_dis_vector.emplace_back(5.0, 1.0);
  box_dis_vector.emplace_back(6.0, 2.0);

  math::IntervalData<std::pair<double, double>> box_distance(
      box_dis_vector.front().first, 1.0, box_dis_vector,
      [](const std::pair<double, double>& p0, const std::pair<double, double>& p1, const double x) {
        return std::make_pair(x, p0.second + (p1.second - p0.second) * (x - p0.first) / (p1.first - p0.first));
      });
  box_distance.setLeftExtrapolationFunction(
      [](const std::pair<double, double>& p0, const double x) { return std::make_pair(x, p0.second); });
  box_distance.setRightExtrapolationFunction(
      [](const std::pair<double, double>& p0, const double x) { return std::make_pair(x, p0.second); });
  risk_field_info.box_distance = box_distance;
  risk_field_info.min_box_distance = 0.5;

  double bound;
  risk_field_info.s_invade_in(1.0, &bound);
  risk_field_info.s_invade_out(1.0, &bound);

  risk_field_info.s_invade_in(3.0, &bound);
  risk_field_info.s_invade_out(3.0, &bound);

  risk_field_info.s_invade_in(0.1, &bound);
  risk_field_info.s_invade_out(0.1, &bound);
}

}  // namespace gpal::pnc::planning