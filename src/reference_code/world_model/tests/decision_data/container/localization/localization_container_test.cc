
#include <gtest/gtest.h>
#include <any>
#include "common/test_utils.hpp"
#include "decision_data/container/container.h"
#define private public
#include "decision_data/container/localization/localization_container.h"

namespace gpal {
namespace pnc {
namespace prediction {
using namespace planning;

class LocalizationContainerTest : public ::testing::Test {
  void SetUp() override {
    planning::InitConfigManager();

    localization_ = std::move(planning::MockLocalization());

    EXPECT_TRUE(container_.Insert(localization_));
  }

 protected:
  LocalizationContainer container_;
  planning::Localization localization_;
};

TEST_F(LocalizationContainerTest, InsertEmptyLocalization) {
  planning::Localization localization;
  EXPECT_FALSE(container_.Insert(localization));
}

TEST_F(LocalizationContainerTest, InsertInvalidType) {
  const std::string msg = "invalid message type";
  EXPECT_FALSE(container_.Insert(msg));
}

TEST_F(LocalizationContainerTest, InsertValidLocalization) {
  EXPECT_TRUE(container_.Insert(localization_));
}

TEST_F(LocalizationContainerTest, Clear) {
  container_.Clear();
  EXPECT_DOUBLE_EQ(container_.timestamp(), -1.0);
  EXPECT_EQ(container_.obstacle_ptr_, nullptr);
}

TEST_F(LocalizationContainerTest, UpdateChassisSuccess) {
  planning::Chassis chassis;
  chassis.set_Speed(10.0);
  chassis.set_LateralAcc(1.0);
  chassis.set_LongituAcc(2.0);
  chassis.set_YawRate(0.1);

  EXPECT_TRUE(container_.Update(chassis));
}

TEST_F(LocalizationContainerTest, UpdateChassisFailure) {
  container_.Clear();
  EXPECT_FALSE(container_.Update(planning::Chassis()));
}

TEST_F(LocalizationContainerTest, GetPlanningObstacleSuccess) {
  planning::Obstacle ego_obstacle;
  EXPECT_TRUE(container_.GetPlanningObstacle(&ego_obstacle));
}

TEST_F(LocalizationContainerTest, GetPlanningObstacleFailure) {
  container_.Clear();
  planning::Obstacle ego_obstacle;
  EXPECT_FALSE(container_.GetPlanningObstacle(&ego_obstacle));
}
}  // namespace prediction
}  // namespace pnc
}  // namespace gpal