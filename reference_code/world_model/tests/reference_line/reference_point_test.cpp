#include <base/singleton.h>
#include <fmt/chrono.h>
#include <gtest/gtest.h>

#include "base/log.h"

#include "config_manager/config_manager.h"

#define private public
#include "reference_line/reference_point.h"


namespace gpal::pnc::planning {

class ReferencePointTest : public ::testing::Test {
 protected:
  void SetUp() override {
    int argc = 0;
    char path[] = "policy_planner";
    char* argv[]{path};
    if (!Singleton<ConfigManager>::get_instance()->init(argc, argv)) {
      ERT_PLOG_I << "Failed to init config!";
    }
  }
};

TEST_F(ReferencePointTest, CreatReferencePointTest) {
  ReferencePoint reference_point_1(math::Vec3d(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ReferencePoint reference_point_2(math::Vec3d(), 0.0, 0.0, 0.0, 0.0, 0.0);
  ReferencePoint reference_point_3(math::Vec3d(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ReferencePoint reference_point_4(math::Vec3d(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  auto pt = reference_point_1.ToPathPoint(0.0);
  auto ref_pt = reference_point_1.lateralShift(0.2);
  ref_pt = reference_point_1.longitudinalShift(0.2);
  auto str = reference_point_1.DebugString();
  EXPECT_TRUE(true);
}
}  // namespace gpal::pnc::planning