#include <gtest/gtest.h>
#include <any>
#include "config_manager/config_manager.h"
#include "config/speed_planner/speed_preprocessor.pb.h"

#define private public
#include "speed_common/speed_common.h"


namespace gpal::pnc::planning {

TEST(IntervalPathTest, EmptyPath) {
  DiscretizedPath empty_path;
  double step = 1.0;
  DiscretizedPath result = intervalPath(empty_path, step);
  EXPECT_TRUE(result.empty());
}

TEST(IntervalPathTest, SinglePointPath) {
  std::vector<PathPt> path_points;
  PathPt pt;
  pt.set_x(0.0);
  pt.set_y(0.0);
  pt.set_theta(0.0);
  pt.set_kappa(0.001);
  pt.set_s(0.0);
  path_points.push_back(pt);
  DiscretizedPath path = DiscretizedPath(path_points);
  double step = 1.0;
  DiscretizedPath result = intervalPath(path, step);
  EXPECT_EQ(result.size(), 1);
  EXPECT_DOUBLE_EQ(result[0].x(), 0.0);
  EXPECT_DOUBLE_EQ(result[0].y(), 0.0);
}

TEST(IntervalPathTest, StepLargerThanPath) {
  std::vector<PathPt> path_points;
  PathPt pt;
  pt.set_x(0.0);
  pt.set_y(0.0);
  pt.set_theta(0.0);
  pt.set_kappa(0.001);
  pt.set_s(0.0);
  path_points.push_back(pt);
  pt.set_x(0.0);
  pt.set_y(1.0);
  pt.set_theta(0.0);
  pt.set_kappa(0.001);
  pt.set_s(1.0);
  path_points.push_back(pt);
  DiscretizedPath path = DiscretizedPath(path_points);
  double step = 10.0;
  DiscretizedPath result = intervalPath(path, step);
  EXPECT_EQ(result.size(), 2);
  EXPECT_DOUBLE_EQ(result[0].x(), 0.0);
  EXPECT_DOUBLE_EQ(result[0].y(), 0.0);
}

TEST(IntervalPathTest, InvalidLength) {
  std::vector<PathPt> path_points;
  PathPt pt;
  pt.set_x(0.0);
  pt.set_y(0.0);
  pt.set_theta(0.0);
  pt.set_kappa(0.001);
  pt.set_s(0.0);
  path_points.push_back(pt);
  pt.set_x(0.0);
  pt.set_y(0.0);
  pt.set_theta(0.0);
  pt.set_kappa(0.001);
  pt.set_s(0.0);
  path_points.push_back(pt);
  DiscretizedPath path = DiscretizedPath(path_points);
  double step = 10.0;
  DiscretizedPath result = intervalPath(path, step);
  EXPECT_EQ(result.size(), 1);
  EXPECT_DOUBLE_EQ(result[0].x(), 0.0);
  EXPECT_DOUBLE_EQ(result[0].y(), 0.0);
}

TEST(IntervalPathTest, ReversePath) {
  std::vector<PathPt> path_points;
  PathPt pt;
  pt.set_x(0.0);
  pt.set_y(1.0);
  pt.set_theta(0.0);
  pt.set_kappa(0.001);
  pt.set_s(1.0);
  path_points.push_back(pt);
  pt.set_x(0.0);
  pt.set_y(0.0);
  pt.set_theta(0.0);
  pt.set_kappa(0.001);
  pt.set_s(0.0);
  path_points.push_back(pt);
  DiscretizedPath path = DiscretizedPath(path_points);
  double step = 1.0;
  DiscretizedPath result = intervalPath(path, step);
  EXPECT_EQ(result.size(), 2);
}

TEST(IntervalPathTest, NormalPath) {
  std::vector<PathPt> path_points;
  for (int i = 0; i < 50; i++) {
    PathPt pt;
    pt.set_x(i * 1.0);
    pt.set_y(0.0);
    pt.set_theta(0.0);
    pt.set_kappa(0.001);
    pt.set_s(i * 1.0);
    path_points.push_back(pt);
  }
  DiscretizedPath path = DiscretizedPath(path_points);
  double step = 1.0;
  DiscretizedPath result = intervalPath(path, step);
  EXPECT_EQ(result.size(), path_points.size());
}

TEST(EvaluatePathTest, EmptyPath) {
  DiscretizedPath empty_path;
  auto result = evaluatePath(empty_path, 0, 1.0);
  EXPECT_EQ(result.first, 0);
}

TEST(EvaluatePathTest, OnePointPath) {
  std::vector<PathPt> path_points;
  PathPt pt;
  pt.set_x(0.0);
  pt.set_y(0.0);
  pt.set_theta(0.0);
  pt.set_kappa(0.001);
  pt.set_s(0.0);
  path_points.push_back(pt);
  DiscretizedPath path = DiscretizedPath(path_points);
  auto result = evaluatePath(path, 0, 1.0);
  // EXPECT_EQ(result.first, 0);
}

TEST(EvaluatePathTest, TwoPointPath) {
  std::vector<PathPt> path_points;
  PathPt pt;
  pt.set_x(0.0);
  pt.set_y(0.0);
  pt.set_theta(0.0);
  pt.set_kappa(0.001);
  pt.set_s(0.0);
  path_points.push_back(pt);
  path_points.push_back(pt);
  DiscretizedPath path = DiscretizedPath(path_points);
  auto result = evaluatePath(path, 0, 1.0);
  // EXPECT_EQ(result.first, 0);
}

TEST(EvaluatePathTest, ForwardPath) {
  std::vector<PathPt> path_points;
  for (int i = 0; i < 50; i++) {
    PathPt pt;
    pt.set_x(i * 1.0);
    pt.set_y(0.0);
    pt.set_theta(0.0);
    pt.set_kappa(0.001);
    pt.set_s(i * 1.0);
    path_points.push_back(pt);
  }
  DiscretizedPath path = DiscretizedPath(path_points);

  for (double s = -2.0; s < 52.0; s += 1.0) {
    auto result = evaluatePath(path, 0, s);
  }
}

TEST(EvaluatePathTest, ReversePath) {
  std::vector<PathPt> path_points;
  for (int i = 50; i > 0; i--) {
    PathPt pt;
    pt.set_x(i * 1.0);
    pt.set_y(0.0);
    pt.set_theta(0.0);
    pt.set_kappa(0.001);
    pt.set_s(i * 1.0);
    path_points.push_back(pt);
  }
  DiscretizedPath path = DiscretizedPath(path_points);
  for (double s = -2.0; s < 52.0; s += 1.0) {
    auto result = evaluatePath(path, 0, s);
  }
}

TEST(EvaluatePathTest, PositiveStartIndex) {
  std::vector<PathPt> path_points;
  for (int i = 0; i < 50; i++) {
    PathPt pt;
    pt.set_x(i * 1.0);
    pt.set_y(0.0);
    pt.set_theta(0.0);
    pt.set_kappa(0.001);
    pt.set_s(i * 1.0);
    path_points.push_back(pt);
  }
  DiscretizedPath path = DiscretizedPath(path_points);
  auto result = evaluatePath(path, 5, 2.0);
}
TEST(EvaluatePathTest, NegativeStartIndex) {
  std::vector<PathPt> path_points;
  for (int i = 0; i < 50; i++) {
    PathPt pt;
    pt.set_x(i * 1.0);
    pt.set_y(0.0);
    pt.set_theta(0.0);
    pt.set_kappa(0.001);
    pt.set_s(i * 1.0);
    path_points.push_back(pt);
  }
  DiscretizedPath path = DiscretizedPath(path_points);
  auto result = evaluatePath(path, -5, 100.0);
}

TEST(IsObstacleStaticTest, StaticType) {
  Decision::RawPredictionTrajectory traj;
  for (int i = 0; i < 50; i++) {
    proto::TrajectoryPoint pt;
    pt.mutable_path_point()->set_x(0 + i * 0.1 * 5);
    pt.mutable_path_point()->set_y(0);
    pt.mutable_path_point()->set_theta(0);
    pt.set_relative_time(i * 0.1);
    traj.push_back(pt);
  }
  auto type_ped = proto::PerceptionObstacle_ObstacleType_kTypePedestrian;
  EXPECT_FALSE(isObstacleStatic(type_ped, traj, true));
  auto type_bicycle = proto::PerceptionObstacle_ObstacleType_kTypeBicycle;
  EXPECT_FALSE(isObstacleStatic(type_bicycle, traj, true));
}

TEST(IsObstacleStaticTest, EmptyTraj) {
  Decision::RawPredictionTrajectory traj;
  auto type = proto::PerceptionObstacle_ObstacleType_kTypeCar;
  EXPECT_TRUE(isObstacleStatic(type, traj, true));
}

TEST(IsObstacleStaticTest, Normal1) {
  Decision::RawPredictionTrajectory traj;
  for (int i = 0; i < 50; i++) {
    proto::TrajectoryPoint pt;
    pt.mutable_path_point()->set_x(0 + i * 0.1 * 5);
    pt.mutable_path_point()->set_y(0);
    pt.mutable_path_point()->set_theta(0);
    pt.mutable_path_point()->set_s(i * 0.1 * 5);
    pt.set_relative_time(i * 0.1);
    traj.push_back(pt);
  }
  auto type = proto::PerceptionObstacle_ObstacleType_kTypeCar;
  EXPECT_FALSE(isObstacleStatic(type, traj, true));
}

TEST(IsObstacleStaticTest, Normal2) {
  Decision::RawPredictionTrajectory traj;
  for (int i = 0; i < 50; i++) {
    proto::TrajectoryPoint pt;
    pt.mutable_path_point()->set_x(0 + i * 0.1 * 0.0001);
    pt.mutable_path_point()->set_y(0);
    pt.mutable_path_point()->set_theta(0);
    pt.mutable_path_point()->set_s(i * 0.1 * 0.0001);
    pt.set_relative_time(i * 0.1);
    traj.push_back(pt);
  }
  auto type = proto::PerceptionObstacle_ObstacleType_kTypeCar;
  EXPECT_TRUE(isObstacleStatic(type, traj, true));
}
TEST(IsObstacleStaticTest, Normal3) {
  Decision::RawPredictionTrajectory traj;
  proto::TrajectoryPoint pt;
  pt.mutable_path_point()->set_s(0.0);
  traj.push_back(pt);
  pt.mutable_path_point()->set_s(0.005);
  traj.push_back(pt);
  pt.mutable_path_point()->set_s(0.011);
  traj.push_back(pt);
  auto type = proto::PerceptionObstacle_ObstacleType_kTypeCar;
  EXPECT_FALSE(isObstacleStatic(type, traj, true));
}

}