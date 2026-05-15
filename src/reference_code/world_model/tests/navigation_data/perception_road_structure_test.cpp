#include <gtest/gtest.h>

#define private public
#include "navigation_data/perception_road_structure.h"


namespace gpal::pnc::planning {

class PerceptionRoadStructureTest : public ::testing::Test {
 protected:
  void SetUp() override { road_structure_ = new PerceptionRoadStructure; }
  void TearDown() override { delete road_structure_; }

  PerceptionRoadStructure* road_structure_;
};

TEST_F(PerceptionRoadStructureTest, basicTest) {
  road_structure_->clear();
  EXPECT_TRUE(true);
}

}  // namespace gpal::pnc::planning
