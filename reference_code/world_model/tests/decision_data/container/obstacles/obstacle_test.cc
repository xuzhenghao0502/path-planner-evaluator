#include <chrono>
#include <gtest/gtest.h>
#include <any>
#define private public
#include "common/test_utils.hpp"
#include "decision_data/container/obstacles/obstacle.h"

namespace gpal {
namespace pnc {
namespace prediction {

bool IsReverseAngle(double angle);

class ObstacleTest : public ::testing::Test {
 protected:
  void SetUp() override {
    planning::InitConfigManager();
    obstacle_.reset(new Obstacle());

    planning_obstacles_ = std::move(planning::MockPlanningObstacle());
    obstacle_ = Obstacle::Create(planning_obstacles_[0], planning_obstacles_[0].timeDelay());
  }

  std::unique_ptr<Obstacle> obstacle_ = nullptr;
  std::vector<planning::Obstacle> planning_obstacles_;
};

TEST_F(ObstacleTest, InsertFailure) {
  auto& obstacles = planning_obstacles_[0];
  EXPECT_FALSE(obstacle_->Insert(obstacles, 999.0));

  obstacles.perception_obstacle_.Clear();
  EXPECT_FALSE(obstacle_->Insert(obstacles, 999.0));

  obstacles.perception_obstacle_.set_obstacle_type(proto::PerceptionObstacle::kTypeBicycle);
  EXPECT_FALSE(obstacle_->Insert(obstacles, 999.0));
}

TEST_F(ObstacleTest, TypeCheck) {
  EXPECT_FALSE(obstacle_->IsVehicle());

  EXPECT_FALSE(obstacle_->IsHeavyVehicle());
  EXPECT_FALSE(obstacle_->IsPedestrian());

  EXPECT_TRUE(obstacle_->IsVru());
  EXPECT_TRUE(obstacle_->IsNonMotorizedVehicle());
}

TEST_F(ObstacleTest, IsPredictable) {
  EXPECT_TRUE(obstacle_->IsPredictable());
}

TEST_F(ObstacleTest, IsOnLane) {
  auto* feature = obstacle_->mutable_latest_feature();
  feature->mutable_lane_feature_set()->set_lane_search_radius(1.0f);
  EXPECT_FALSE(obstacle_->IsOnLane());

  feature->mutable_lane_feature_set()->mutable_current_lane_feature()->set_id("1");
  EXPECT_TRUE(obstacle_->IsOnLane());
}

TEST_F(ObstacleTest, IsInJunction) {
  auto* feature = obstacle_->mutable_latest_feature();
  feature->mutable_junction_feature()->set_id("1");
  EXPECT_FALSE(obstacle_->IsInJunction());

  feature->mutable_junction_feature()->add_exits()->set_lane_id("1");
  EXPECT_TRUE(obstacle_->IsInJunction());
}

TEST_F(ObstacleTest, IsDowngraded) {
  EXPECT_FALSE(obstacle_->IsDowngraded());

  obstacle_->SetDowngrade();
  EXPECT_TRUE(obstacle_->IsDowngraded());
}

TEST_F(ObstacleTest, IsFallback) {
  EXPECT_FALSE(obstacle_->IsFallback());

  obstacle_->SetFallback();
  EXPECT_TRUE(obstacle_->IsFallback());
}

TEST_F(ObstacleTest, IsRetrogradeDriving) {
  EXPECT_FALSE(obstacle_->IsRetrogradeDriving());
}

TEST_F(ObstacleTest, DiscardOutdatedHistory) {
  auto& obstacles = planning_obstacles_[0];
  obstacle_->Insert(obstacles, 9999.0);
  obstacle_->DiscardOutdatedHistory();

  obstacle_->feature_history_.clear();
  obstacle_->DiscardOutdatedHistory();
}

TEST_F(ObstacleTest, SetTheta) {
  auto sub_obstacle = proto::SubObstacle();
  obstacle_->SetTheta(sub_obstacle, obstacle_->mutable_latest_feature());

  sub_obstacle.set_heading_angle(std::numeric_limits<float>::quiet_NaN());
  obstacle_->SetTheta(sub_obstacle, obstacle_->mutable_latest_feature());
}

TEST_F(ObstacleTest, SetYawRate) {
  auto sub_obstacle = proto::SubObstacle();
  sub_obstacle.set_heading_rate(std::numeric_limits<float>::quiet_NaN());
  obstacle_->SetYawRate(sub_obstacle, obstacle_->mutable_latest_feature());

  sub_obstacle.set_heading_rate(0.0f);
  obstacle_->SetYawRate(sub_obstacle, obstacle_->mutable_latest_feature());
}

TEST_F(ObstacleTest, SetVelocity) {
  auto sub_obstacle = proto::SubObstacle();
  obstacle_->SetVelocity(sub_obstacle, obstacle_->mutable_latest_feature());

  auto* velocity = sub_obstacle.mutable_velocity();
  velocity->set_x(std::numeric_limits<float>::quiet_NaN());
  velocity->set_y(std::numeric_limits<float>::quiet_NaN());
  obstacle_->SetVelocity(sub_obstacle, obstacle_->mutable_latest_feature());
}

TEST_F(ObstacleTest, SetAcceleration) {
  auto sub_obstacle = proto::SubObstacle();
  auto* acceleration = sub_obstacle.mutable_acceleration();
  acceleration->set_x(std::numeric_limits<float>::quiet_NaN());
  acceleration->set_y(std::numeric_limits<float>::quiet_NaN());
  obstacle_->SetAcceleration(sub_obstacle, obstacle_->mutable_latest_feature());

  acceleration->set_x(1.0);
  acceleration->set_y(2.0);
  obstacle_->SetAcceleration(sub_obstacle, obstacle_->mutable_latest_feature());
}

TEST_F(ObstacleTest, SetMotionStatus) {
  auto obstacle = proto::PerceptionObstacle();
  auto* feature = obstacle_->mutable_latest_feature();
  obstacle_->SetMotionStatus(obstacle, feature);

  obstacle.set_obstacle_type(proto::PerceptionObstacle::kTypeCar);
  obstacle.set_motion_status(proto::PerceptionObstacle::kMotionDynamic);
  feature->set_speed(0.0f);
  obstacle_->SetMotionStatus(obstacle, feature);
}

TEST_F(ObstacleTest, SetSpeedRatio) {
  obstacle_->mutable_latest_feature()->Clear();
  obstacle_->SetSpeedRatio(obstacle_->latest_feature());
}

TEST_F(ObstacleTest, Priority) {
  auto* feature = obstacle_->mutable_latest_feature();

  EXPECT_TRUE(obstacle_->IsIgnore());
  EXPECT_FALSE(obstacle_->IsNormal());
  EXPECT_FALSE(obstacle_->IsCaution());
}

TEST_F(ObstacleTest, SetEvaluatorType) {
  obstacle_->SetEvaluatorType(ObstacleConfig::EMPTY_EVALUATOR);
}

TEST_F(ObstacleTest, SetPredictorType) {
  obstacle_->SetPredictorType(ObstacleConfig::EMPTY_PREDICTOR);
}

TEST_F(ObstacleTest, SetDelayTime) {
  obstacle_->SetDelayTime(1.0f);
}

TEST(IsReverseAngleTest, IsReverseAngleTest) {
  EXPECT_FALSE(IsReverseAngle(1.0));
  EXPECT_TRUE(IsReverseAngle(M_PI));
}

TEST_F(ObstacleTest, GetStillSpeedThreshold) {
  obstacle_->type_ = proto::PerceptionObstacle::kTypePedestrian;
  obstacle_->GetStillSpeedThreshold();
}

}  // namespace prediction
}  // namespace pnc
}  // namespace gpal