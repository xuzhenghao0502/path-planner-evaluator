#include <gtest/gtest.h>

#define private public
#include "navigation_data/navigation.h"


namespace gpal::pnc::planning {

class NavigationTest : public ::testing::Test {
 protected:
  void SetUp() override { navigation_ = new Navigation; }
  void TearDown() override { delete navigation_; }

  Navigation* navigation_;
};

TEST_F(NavigationTest, basicTest) {
  navigation_->clear();
  EXPECT_TRUE(true);
}

}  // namespace gpal::pnc::planning
