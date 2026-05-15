#include "path/path_decision.h"

#include "gtest/gtest.h"

namespace gpal::pnc::planning {

class PathDecisionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // 初始化测试数据
    IgnoreRangeInfo ignore_range;
    ignore_range.type = IgnoreRangeInfo::IgnoreRangeType::GATE;
    ignore_range.start_s = 10.0;
    ignore_range.end_s = 20.0;
    path_decision_.mutableIgnoreRange()->push_back(ignore_range);

    RangeInfo range_info;
    range_info.type = RangeInfo::RangeType::LEFT;
    range_info.start_s = 5.0;
    range_info.end_s = 15.0;
    range_info.contract_coff = 0.8;

    PathBoundInfo bound_info;
    bound_info.s = 0.0;
    bound_info.left_bound = -2.0;
    bound_info.right_bound = 2.0;
    path_decision_.mutable_max_allowed_bounds()->second.push_back(bound_info);

    // 添加测试障碍物
    Obstacle static_obs("static_1", math::Polygon2d(std::vector<math::Vec2d>{math::Vec2d(0, 0), math::Vec2d(1, 0),
                                                                             math::Vec2d(1, 1), math::Vec2d(0, 1)}));
    path_decision_.mutable_static_obstacles()->add(static_obs);

    Obstacle dynamic_obs("dynamic_1", math::Polygon2d(std::vector<math::Vec2d>{math::Vec2d(0, 0), math::Vec2d(1, 0),
                                                                               math::Vec2d(1, 1), math::Vec2d(0, 1)}));
    path_decision_.mutable_dynamic_obstacles()->add(dynamic_obs);

    Obstacle risky_obs("risky_1", math::Polygon2d(std::vector<math::Vec2d>{math::Vec2d(0, 0), math::Vec2d(1, 0),
                                                                           math::Vec2d(1, 1), math::Vec2d(0, 1)}));
    path_decision_.mutable_risky_obstacles()->add(risky_obs);
  }

  PathDecision path_decision_;
};

TEST_F(PathDecisionTest, IgnoreRangeFunctions) {
  // 测试 isInRange 函数
  EXPECT_TRUE(isInRange(path_decision_.ignoreRange(), 15.0));
  EXPECT_FALSE(isInRange(path_decision_.ignoreRange(), 25.0));
  EXPECT_TRUE(isInRange(path_decision_.ignoreRange(), 15.0, 25.0));
  EXPECT_FALSE(isInRange(path_decision_.ignoreRange(), 25.0, 30.0));

  // ranges为空情况
  std::vector<IgnoreRangeInfo> empty_ranges;
  EXPECT_FALSE(isInRange(empty_ranges, 10.0));
  EXPECT_FALSE(isInRange(empty_ranges, 25.0, 30.0));
}

TEST_F(PathDecisionTest, RangeInfoFunctions) {
  std::vector<RangeInfo> ranges;

  // ranges为空情况
  EXPECT_FALSE(isInRange(ranges, 10.0));
  EXPECT_FALSE(isInRange(ranges, 20.0));

  // 测试 isInRange 函数
  RangeInfo range;
  range.type = RangeInfo::RangeType::LEFT;
  range.start_s = 5.0;
  range.end_s = 15.0;
  ranges.push_back(range);

  EXPECT_TRUE(isInRange(ranges, 10.0));
  EXPECT_FALSE(isInRange(ranges, 20.0));
}

TEST_F(PathDecisionTest, ObstacleContainers) {
  // 测试障碍物容器访问
  EXPECT_TRUE(path_decision_.static_obstacles().has("static_1"));
  EXPECT_TRUE(path_decision_.dynamic_obstacles().has("dynamic_1"));
  EXPECT_TRUE(path_decision_.risky_obstacles().has("risky_1"));

  // 测试可修改接口
  EXPECT_NE(nullptr, path_decision_.mutable_static_obstacles());
  EXPECT_NE(nullptr, path_decision_.mutable_dynamic_obstacles());
  EXPECT_NE(nullptr, path_decision_.mutable_risky_obstacles());
}

TEST_F(PathDecisionTest, PathBoundInfo) {
  // 测试路径边界信息
  EXPECT_FALSE(path_decision_.max_allowed_bounds().first);
  EXPECT_EQ(1, path_decision_.max_allowed_bounds().second.size());

  // 测试可修改接口
  auto* bounds = path_decision_.mutable_max_allowed_bounds();
  bounds->first = true;
  EXPECT_TRUE(bounds->first);
}

TEST_F(PathDecisionTest, BehaviorTypeOperations) {
  // 测试行为类型设置和获取
  path_decision_.set_behavior_type(PathDecision::BehaviorType::PULL_OVER_LONGI);
  EXPECT_EQ(PathDecision::BehaviorType::PULL_OVER_LONGI, path_decision_.get_behavior_type());
}

TEST_F(PathDecisionTest, ReferenceLineOperations) {
  // 测试参考线设置
  path_decision_.setLaneKeepStartS(50.0);
  EXPECT_DOUBLE_EQ(50.0, path_decision_.lane_keep_start_s());

  path_decision_.setStopReferenceLineS(100.0);
  EXPECT_DOUBLE_EQ(100.0, path_decision_.stop_reference_line_s());
}

TEST_F(PathDecisionTest, RefOffsetsOperations) {
  // 测试路径偏移量
  EXPECT_TRUE(path_decision_.ref_offsets().empty());

  auto* offsets = path_decision_.mutable_ref_offsets();
  offsets->push_back({0.0, 0.0});
  EXPECT_EQ(1, path_decision_.ref_offsets().size());
}

TEST_F(PathDecisionTest, SpecialFlags) {
  // 测试特殊标志位
  path_decision_.set_pull_over_stop(true);
  EXPECT_TRUE(path_decision_.pull_over_stop());

  path_decision_.set_enable_backward_planning(true);
  EXPECT_TRUE(path_decision_.enable_backward_planning());
}

TEST_F(PathDecisionTest, DebugInfo) {
  // 测试调试信息
  path_decision_.debug_info = "test debug info";
  EXPECT_EQ("test debug info", path_decision_.debug_info);
}

}  // namespace gpal::pnc::planning