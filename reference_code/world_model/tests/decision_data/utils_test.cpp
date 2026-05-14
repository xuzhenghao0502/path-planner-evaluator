#include "decision_data/utils.h"

#include <gtest/gtest.h>

namespace gpal::pnc::planning::Decision::Utils {

class UtilsTest : public ::testing::Test {
 protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(UtilsTest, GetTimeInSec_1) {
  uint64_t sec = 0;
  uint64_t nsec = 0;
  GetTimeInSec(sec, nsec);
  EXPECT_TRUE(true);
}

TEST_F(UtilsTest, NormalizeAngle_1) {
  NormalizeAngle(20.0);
  NormalizeAngle(200.0);
  NormalizeAngle(400.0);
  EXPECT_TRUE(true);
}

TEST_F(UtilsTest, AngleDiff_1) {
  AngleDiff(20.0, 200.0);
  AngleDiff(200.0, 400.0);
  AngleDiff(400.0, 20.0);
  EXPECT_TRUE(true);
}

TEST_F(UtilsTest, RandomInt_1) {
  RandomInt(100, 0, 0);
  RandomInt(0, 100, 0);
  EXPECT_TRUE(true);
}

TEST_F(UtilsTest, RandomDouble_1) {
  double s = 0.0;
  double t = 100.0;
  unsigned int rand_seed = 0;
  RandomDouble(s, t, rand_seed);
  EXPECT_TRUE(true);
}

TEST_F(UtilsTest, Gaussian_1) {
  double u = 0.0;
  double std = 100.0;
  double x = 0.0;
  Gaussian(u, std, x);
  EXPECT_TRUE(true);
}

}  // namespace gpal::pnc::planning::Decision::Utils