#include "navigation_data/common_typedefs.h"

#include <base/singleton.h>
#include <fmt/chrono.h>
#include <gtest/gtest.h>


#include "config_manager/config_manager.h"

namespace gpal::pnc::planning {

TEST(PerceptionMergeForkPointTest, basicTest) {
  PerceptionMergeForkPoint::RelatedLaneMarking marking;
  marking.id = "1";
  marking.related_point = math::Vec3d(1.0, 2.0, 0.0);
  EXPECT_EQ(marking.id, "1");
  EXPECT_EQ(marking.related_point.x(), 1.0);
  marking.clear();
  EXPECT_EQ(marking.id, "");

  PerceptionMergeForkPoint point;
  point.related_lane_markings.emplace_back(std::move(marking));
  point.clear();
  EXPECT_EQ(point.related_lane_markings.size(), 0);
}

TEST(PerceptionGateTest, basicTest) {
  PerceptionGate gate;
  gate.clear();
  EXPECT_TRUE(true);
}

TEST(PerceptionAreaTest, basicTest) {
  PerceptionArea area;
  area.clear();
  EXPECT_TRUE(true);
}

TEST(PerceptionLaneMarkingGroupTest, basicTest) {
  PerceptionLaneMarkingGroup group;
  group.clear();
  EXPECT_TRUE(true);
}

}  // namespace gpal::pnc::planning
