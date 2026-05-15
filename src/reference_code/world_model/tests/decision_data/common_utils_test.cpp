#include "decision_data/common_utils.h"

#include <gtest/gtest.h>

namespace gpal::pnc::planning::Decision {

class CommonUtilsTest : public ::testing::Test {
 protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(CommonUtilsTest, LateralBoundConsInterpolate_1) {
  double s = 0.0;
  BoundPointList points;
  Decision::LateralBoundConsInterpolate(s, points);
  EXPECT_TRUE(true);
}

TEST_F(CommonUtilsTest, LateralBoundConsInterpolate_2) {
  double s = 0.0;
  BoundaryPoint p1;
  p1.s = 1.0;
  BoundaryPoint p2;
  p2.s = 10.0;
  BoundPointList points = {p1, p2};
  Decision::LateralBoundConsInterpolate(s, points);
  EXPECT_TRUE(true);
}

TEST_F(CommonUtilsTest, LateralBoundConsInterpolate_3) {
  double s = 20.0;
  BoundaryPoint p1;
  p1.s = 1.0;
  BoundaryPoint p2;
  p2.s = 10.0;
  BoundPointList points = {p1, p2};
  Decision::LateralBoundConsInterpolate(s, points);
  EXPECT_TRUE(true);
}

TEST_F(CommonUtilsTest, LateralBoundConsInterpolate_4) {
  double s = 5.0;
  BoundaryPoint p1;
  p1.s = 1.0;
  BoundaryPoint p2;
  p2.s = 10.0;
  BoundPointList points = {p1, p2};
  Decision::LateralBoundConsInterpolate(s, points);
  EXPECT_TRUE(true);
}

}  // namespace gpal::pnc::planning::Decision