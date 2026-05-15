
#include <any>
#include <gtest/gtest.h>

#define private public
#include "common/test_utils.hpp"
#include "decision_data/container/obstacles/obstacles_container.h"
#include "obstacle/obstacle.h"

namespace gpal {
namespace pnc {
namespace prediction {

class ObstaclesContainerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    planning::InitConfigManager();

    // invalid test
    double invalid_t = -1.0;
    container_.SetTimestamp(invalid_t, delay_t_);
    auto planning_obstacle_tmp = std::move(planning::MockPlanningObstacle(3, timestamp_)[0]);
    frame_obstacles_.add(planning_obstacle_tmp);
    container_.Insert(frame_obstacles_);
    frame_obstacles_.reset();

    // normal test
    planning_obstacle_ = std::move(planning::MockPlanningObstacle(id_, timestamp_)[0]);
    auto planning_obstacle_1 = std::move(planning::MockPlanningObstacle(2, timestamp_)[0]);
    planning_obstacle_1.perception_obstacle_.set_obstacle_type(proto::PerceptionObstacle::kTypeCones);
    auto planning_obstacle_2 = std::move(planning::MockPlanningObstacle(3, timestamp_)[0]);
    planning_obstacle_2.perception_obstacle_.set_obstacle_type(proto::PerceptionObstacle::kTypeCar);
    auto planning_obstacle_3 = std::move(planning::MockPlanningObstacle(4, timestamp_)[0]);
    planning_obstacle_3.perception_obstacle_.set_obstacle_type(proto::PerceptionObstacle::kTypeBicycle);

    container_.SetTimestamp(timestamp_, delay_t_);
    container_.InsertPlanningObstacle(planning_obstacle_, timestamp_);

    frame_obstacles_.add(planning_obstacle_1);
    frame_obstacles_.add(planning_obstacle_2);
    frame_obstacles_.add(planning_obstacle_3);
    container_.Insert(frame_obstacles_);
  }

  int id_ = 0;
  double timestamp_ = 1.0;
  double delay_t_ = -0.1;

  PredictionConfig config_;
  ObstaclesContainer container_;
  planning::Obstacle planning_obstacle_;
  planning::IndexedObstacles frame_obstacles_;
};

TEST_F(ObstaclesContainerTest, InsertWithEmptyEgo) {
  container_.ptr_obstacles_.Clear();
  EXPECT_FALSE(container_.Insert(frame_obstacles_));
}

TEST_F(ObstaclesContainerTest, InsertWithInvalidDistance) {
  auto planning_obstacle = std::move(planning::MockPlanningObstacle(3, timestamp_)[0]);
  auto* sub_obstacle = planning_obstacle.perception_obstacle_.mutable_sub_obstacles(0);
  auto* position = sub_obstacle->mutable_position();
  position->set_x(1000.0);
  position->set_y(1000.0);
  frame_obstacles_.add(planning_obstacle);

  container_.Insert(frame_obstacles_);
}

TEST_F(ObstaclesContainerTest, InsertAbnormalInput) {
  container_.timestamp_ = 1.0;
  EXPECT_FALSE(container_.Insert(1.0));
}

TEST_F(ObstaclesContainerTest, InsertPlanningObstacle) {
  auto planning_obstacle = std::move(planning::MockPlanningObstacle(5, timestamp_)[0]);

  double invalid_t = -1.0;
  container_.InsertPlanningObstacle(planning_obstacle, invalid_t);

  planning_obstacle.perception_obstacle_.mutable_sub_obstacles()->Clear();
  container_.InsertPlanningObstacle(planning_obstacle, timestamp_);

  auto planning_obstacle_1 = std::move(planning::MockPlanningObstacle(3, 2.0)[0]);
  planning_obstacle_1.perception_obstacle_.set_obstacle_type(proto::PerceptionObstacle::kTypeCar);
  container_.InsertPlanningObstacle(planning_obstacle_1, 1.1);
  container_.InsertPlanningObstacle(planning_obstacle_1, 1.2);
}

TEST_F(ObstaclesContainerTest, InsertOutdatedPlanningObstacle) {
  container_.timestamp_ = timestamp_;
  auto planning_obstacle = std::move(planning::MockPlanningObstacle(5, timestamp_)[0]);
  planning_obstacle.perception_obstacle_.clear_obstacle_type();
  container_.InsertPlanningObstacle(planning_obstacle, timestamp_);
}

TEST_F(ObstaclesContainerTest, GetObstacleWithLRUUpdate) {
  container_.GetObstacleWithLRUUpdate(id_);
}

TEST_F(ObstaclesContainerTest, UpdateDelayTime) {
  container_.UpdateDelayTime(-2.0);
}

TEST_F(ObstaclesContainerTest, SetConsideredObstacleIds) {
  for (const int id : container_.frame_obstacle_ids_) {
    auto* obstacle = container_.GetObstacle(id);
    if (obstacle != nullptr) {
      obstacle->mutable_latest_feature()->set_priority(ObstaclePriority::CAUTION);
    }
  }
  container_.SetConsideredObstacleIds();
}

}  // namespace prediction
}  // namespace pnc
}  // namespace gpal