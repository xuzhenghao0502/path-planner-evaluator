#include <gtest/gtest.h>

#define private public
#include "navigation_data/perception_road_marking.h"


namespace gpal::pnc::planning {

class PerceptionRoadMarkingTest : public ::testing::Test {
 protected:
  void SetUp() override { road_marking_ = new PerceptionRoadMarking; }
  void TearDown() override { delete road_marking_; }

  PerceptionRoadMarking* road_marking_;
};

TEST_F(PerceptionRoadMarkingTest, initTest) {
  road_marking_->clear();
  EXPECT_TRUE(true);
}

}  // namespace gpal::pnc::planning
